import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import RPi.GPIO as GPIO

import time

class Dist_sensor(Node):
    def __init__(self):
        super().__init__('dist_sensor_node')
        self.record_publisher = self.create_publisher(String, 'record', 10)

        self.trig = 17  # GPIO pin connected to the trigger pin of HC-SR04
        self.echo = 18  # GPIO pin connected to the echo pin of HC-SR04
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

        self.distance_threshold = 15  # Set the distance threshold in centimeters
        self.rate = 1  # sleep in sec
        self.wait_time = 5  # Waiting period before publishing in seconds

    def pub(self):
        '''
        pos_sig is the number of seconds the target (aka the dog) has been close enough to the dispenser
        This way, the dispenser will wait "wait_time" seconds before dispensing any food
        This avoids the dispension of food when the dog/human is only walking by the dispenser
        pos_sig also behaves like a buffer to not immediately stop the camera recording if the dog isnt
        detected by the sensor.
        '''
        pos_sig = 0
        record_msg = String()
        record_msg.data = "false"
        
        while rclpy.ok():

            distance = self.measure_distance()
            
            if distance < self.distance_threshold:
                if pos_sig >= self.wait_time:
                    record_msg = String()
                    record_msg.data = "true"  # Publish "true" to indicate recording

                else:
                    pos_sig +=1
                    record_msg = String()
                    record_msg.data = "false"  # Publish "false" to indicate not recording

            else:
                if (pos_sig > 0) and (record_msg.data == "true"):
                    pos_sig -= 1
                    record_msg = String()
                    record_msg.data = "true"  # Publish "false" to indicate not recording

                else:
                    pos_sig = 0
                    record_msg = String()
                    record_msg.data = "false"  # Publish "false" to indicate not recording

            self.record_publisher.publish(record_msg)
            rclpy.sleep(self.rate)

    def measure_distance(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        
        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(self.echo) == 0:
            start_time = time.time()
        while GPIO.input(self.echo) == 1:
            stop_time = time.time()

        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound in cm/s
        return distance

def main(args=None):
    rclpy.init(args=args)
    dist_sensor_node = Dist_sensor()
    dist_sensor_node.pub()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
