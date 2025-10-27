In @teleop_node.py we introduced a state machine to control the arm in @arm_controller.py

One thing we do is we have an intermediate state to ensure the robot stays within a competition set extension limit 

this creates a fairly jerky intermediate state 

One thing the moteus trajectory generation allows us to do is set a target velocity once we reach the end point.  I think it might be interesting to modify our code to allow using this for the intermediate points.  

basically for most movements the end velocity should be zero like it is now, but for the intermediate points we should have a constant velocity (such as half the maximum velocity, it should be specified by something we can modify in teleop)

can you update the state machine configuration, messages being sent, and robot code to handle this? 