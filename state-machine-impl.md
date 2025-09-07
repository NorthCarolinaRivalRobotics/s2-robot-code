We wish to create a state machine for managing the motion of the arm in the robot.  

See @state_machine_diagram.png for reference 


In order to create this we will need a wrist subsystem.  This is currently not physically on the robot.  This subsystem will have a wrist servo and a claw servo.  We will have some estimate of when the servo arrives in position based on time perhaps.  This should be a node in the robot-code that is easy to adapt to our servo claw / wrist code later.  this should be in robot-code


In driver-station:

we also need some stuff in driver station to orchestrate the arm, we have some primitives in place for sending now but we need a state machine.  We should have an arm state variable which represents the position of the arm and the claw open / close state 

The state machine needs to work as follows:

init state --> instantly transitions to stow 

stow can transition to place state (head on) with triangle 
stow can transition to place state (side) with horizontal

both place states transition to claw open with circle

claw open transitions to silo intaking after cross

In silo intaking circle goes to ground intaking 

in ground intaking circle goes to silo intaking 

in either silo or ground intaking cross goes to stow 

in stow circle goes to ground intaking 

BUTTONS MUST BE DEBOUNCED 


We should use the Autonomous move to position when the triggers are pressed > 0.5 

when the left trigger is pressed move to a pose labeled SILO position

when the right trigger is pressed move to a pose labeled GOAL position

Just use all zero values for the pose and arm states above, I will tune these later.  










