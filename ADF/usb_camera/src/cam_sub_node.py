#!/usr/bin/env python3

# Copyright 2021 Evan Flynn, Lucas Walter, Shayan Azizi
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Evan Flynn nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import cv2
import numpy as np
import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# This is a modified/extended version of the script given in the https://github.com/ros-drivers/usb_cam/tree/ros2 since a usb camera is used.  

class Camera_cap(Node):

    def __init__(self):
        super().__init__('cam_cap')

        # Subscribe to the usb camera driver
        self.img_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            100)
        
        self.mat = None
        self.video_out = None

        # Below can be changed based on needs/camera properties
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Specify the codec
        self.fps = 30  # Specify the frames per second

    
        # Subscribe to topic that controls recording
        self.record_sub = self.create_subscription(
            String,
            'record', 
            self.record_callback,
            10)
        
        self.record_now = False

    def image_callback(self, msg):

        if self.record_now:

            sz = (msg.height, msg.width)

            if False:
                print('{encoding} {width} {height} {step} {data_size}'.format(
                    encoding=msg.encoding, width=msg.width, height=msg.height,
                    step=msg.step, data_size=len(msg.data)))
                
            if msg.step * msg.height != len(msg.data):
                print('bad step/height/data size')
                return

            # Assume coloured recording (black and white videos of dogs arent as interesting)
            if msg.encoding == 'rgb8':
                dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                        msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                        self.mat.shape[2] != 3)
                if dirty:
                    self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
                self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
                self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
                self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)

            else:
                print('unsupported encoding {}'.format(msg.encoding))
                return

            # dt_now will let the user know when the image was captured
            # It also stop replacing the previous video files due to having the same name
            dt_now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            if self.video_out is None:
                self.video_out = cv2.VideoWriter(f'output_{dt_now}.avi', self.fourcc, self.fps, (msg.width, msg.height))

            # Write the current frame to the video
            self.video_out.write(self.mat)

    def record_callback(self, msg):
        # Callback function to handle messages indicating whether to start or stop recording
        # Note that bool msg type can also be used, I am more comfortable with strings and find them to be more versatile
        if msg.data == "true":
            self.record_now = True
            if self.video_out is not None:
                self.get_logger().info('Recording started.')

        elif msg.data == "false":
            self.record_now = False
            if self.video_out is not None:
                self.video_out.release()
                self.video_out = None
                self.get_logger().info('Recording stopped.')

        else:
            self.get_logger().info('Invalid command')



def main(args=None):

    rclpy.init(args=args)
    cam_cap = Camera_cap()

    # Would benefit from an appropriate exception. i.e. A button on the ADF that when pressed destroys the node and shuts down ROS2
    try:
        rclpy.spin(cam_cap)
    except KeyboardInterrupt:
        pass

    cam_cap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()