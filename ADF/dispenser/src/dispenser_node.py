import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO

class Dispenser(Node):
    def __init__(self):
        super().__init__('dispenser_node')
        
        ENA_PIN = 25  # GPIO pin connected to the EN1 pin
        IN1_PIN = 8  # GPIO pin connected to the IN1 pin 
        IN2_PIN = 7  # GPIO pin connected to the IN2 pin 

        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENA_PIN, GPIO.OUT)
        GPIO.setup(IN1_PIN, GPIO.OUT)
        GPIO.setup(IN2_PIN, GPIO.OUT)

        # Set ENA_PIN to HIGH to enable the actuator
        GPIO.output(ENA_PIN, GPIO.HIGH)

        self.record_subscriber = self.create_subscription(String, 'record', self.record_callback, 10)
        self.last_disp_time = 0

        # Minimum dispense interval, setting default to 15 minutes (in seconds)
        # I think this is appropriate to avoid wasting food/water
        self.max_disp_int = 900  

        '''
        Note: I think the proper way of doing this would be to introduce a weight sensor under the water and dry food
        containers in the dispenser. Create a node for the sensors such that they constantly publish the weight of the 
        of the water or dry food available to a topic "weight" (e.g. "100, 200" where the first number represent the weight of water 
        and second number represents the weight of dry food already available to the dog) and only dispense more if its
        below a certain threshhold. We would have an additional subscriber to the topic "weight" in this node to check 
        if the containers are empty. I dont think implementing it would demonstrate my abilities any further as it's very
        similar to the other 3 nodes I've written. Hence, we'll stick to the simple version of dispensing at most once 
        every "max_disp_int" time.
        '''

    def record_callback(self, msg):
        if msg.data == "true":
            current_time = time.time()
            if current_time - self.last_dispense_time >= self.min_dispense_interval:
                self.dispense_food()
                self.last_dispense_time = current_time

    def dispense_food(self):
        '''
        There is a linear actuator connected to a h bridge, this function controls the h bridge
        that will allow the actuator to extend or retract

        The sleep rates need to be adjusted based on the design of the ADF to allow an appropriate
        amount of food/water to be dispensed
        '''

        # Extend the actuator
        GPIO.output(IN1_PIN, GPIO.HIGH)
        GPIO.output(IN2_PIN, GPIO.LOW)
        rclpy.sleep(1)

        # Stop the actuator
        GPIO.output(IN1_PIN, GPIO.HIGH)
        GPIO.output(IN2_PIN, GPIO.HIGH)
        rclpy.sleep(1)

        # Retract the actuator
        GPIO.output(IN1_PIN, GPIO.LOW)
        GPIO.output(IN2_PIN, GPIO.HIGH)
        rclpy.sleep(1.1)

        # Stop the actuator
        GPIO.output(IN1_PIN, GPIO.HIGH)
        GPIO.output(IN2_PIN, GPIO.HIGH)

        print("Dispensing food/water")

def main(args=None):
    rclpy.init(args=args)
    dispenser_node = Dispenser()
    rclpy.spin(dispenser_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
