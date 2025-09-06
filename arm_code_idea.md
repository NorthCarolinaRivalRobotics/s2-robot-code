we are adding a double jointed arm to our robot 

this will add two motors.  These should be controlled by robot-code/standalone_drivetrain.py which will act as our moteus entrypoint 

Our bus and Id mapping changes a bit as we will add motors 5 (elbow) and 6 (shoulder)
motors 1 and 2 are on bus 1
motor 6 is on bus 2 
motors 5 3 and 4 are on bus 4 

1,2,3,4 remain to control the drivetrain the way they do now and should functionally be the same

the arm is a double jointed arm where there is a virtual 4 bar like mechanism which means that the rotation of the shoulder and elbow are independent meaning we do not need to compensate in the elbow for when the shoulder moves

the motors have 10:1 gear reductions on them.  We should initialize their starting position to zero.  

For testing we should use very conservative torque and velocity limits 
- 0.25 nm to start 
- 0.5 rotations per second to start on velocity 

we should publish the positions of both joints 

for now we should zero the position on startup 

driver station changes: 
For control we should have it setup so the d pad slowly increments the position of the motors, left and right do the shoulder and up and down will do the elbow.  This should be debounced ideally and if it is properly debounced it should move a small but noticable amount.

we should implement a node for handling gamepad state at this point.  Preserve all existing functionality.  

See example_moteus_swerve.py to see how to use the moteus position api

Let me know if there are any conflicting goals 

First start by researching the existing code then plan and finally implement the changes.


