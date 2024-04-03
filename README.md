# Automatic Dog Feeder

The ADF contains 3 files:

- us_dist_sensor.py: Uses a supersonic sensor to detect objects infront of it, if within a range for a specificed about of time, it will publish to the "record" topic to let other nodes know that the doggo is waiting to be fed!
  
- cam_sub_node.py: Listens to the "record" topic and starts recording once it receives "true" and saves the video onto disk. It captures the frames by subscribing to the usb camera driver and stitches the frames together using opencv.
  
- dispenser_node.py: Listen to the "record" topic and activates the linear actuators through a H brdige once it reveives "true". After a small amount of time has passed, the linear actuator is retracted back.

### Testing:
- Create a package for each of the above nodes (colcon build), launch the nodes and then place objects infront of the ultrasonic sensor at variying varying distances and for varying time intervals and see how the system behaves.
- Further testing is required to optimise the time which the linear actuator extends, remains stationary and retracts depending on the geometrical properties of the ADF.
- Final stages of testing will benefit from using a real dog interacting with the setup to finalise any required improvements.

### Assumptions and considerations:
The assumptions and considerations have been appropriatey pointed out within the code and system diagram. Further considerations and assumptions include:
- I didnt have access to a ROS2 environment at the time of writing this code.
- I think there is room for improvement in several aspects of the code. Notably, I'd prefer to detect whether the dog is waiting to be fed or not using a CNN that is trained on dog faces, upon detecting the dog's face using the usb camera at a certain distance, food is dispensed. However, this might run into power supply or privacy problems.
- I've adhered to the limited budget available for this setup.
- It's assumed that the linear actuator moves an attachment that blocks food and water from leaving the supply container.


