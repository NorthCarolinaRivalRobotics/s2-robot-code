# For install on robot
run `uv sync --extra robot` 

# For running on PC
run `uv sync --extra laptop`


Instructions to run:

- on the driver station computer navigate to driver-station and run `uv run tide up` and it will start the teleop tide node 

- on the robot: first ssh into `ssh ncrival@ncrival.local` using the password ncrival, then navigate to `cd s2-robot-code/robot-code/` and run `sudo uv run tide up`.  Due to the permissions needed for motor control the sudo is very important. 