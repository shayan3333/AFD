The ADF contains 3 files:

dispenser_node.py
us_dist_sensor.py
cam_sub_node.py

us_dist_sensor.py uses a supersonic sensor to detect objects infront of it, if within a range for a specificed about of time, it will publish to the "record" topic to let other nodes know that the doggo is waiting to be fed!

cam_sub_node.py listens to the "record" topic and starts recording once it receives "true" and saves the video onto disk. It captures the frames by subscribing to the usb camera driver and stitches the frames together using opencv

dispenser_node.py listen to the "record" topic and activates the linear actuators through a H brdige once it reveives "true". After a small amount of time has passed, the linear actuator is retracted back.


Testing:
Create a package for each of the above nodes, launch the nodes and then place objects infront of the distance sensor for variying time intervals and see how the system behaves.
