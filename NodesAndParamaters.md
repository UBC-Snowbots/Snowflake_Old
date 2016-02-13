#   Nodes
A list of all possible nodes in all packagesthe repo, with all possible paramaters for each (for use in rosLaunch files)
---------------------------
##  Drivers
---------------------------
- elsa_driver
  - **Parameters**
    - *max_turn_rate* -  (default = 40) The maximum that the turn rate will go to
    (ie. if max_turn_rate = 40, the max command sent to the apm would be 125 +/- 40)
    - *turn_rate_sensitivity* - (default = 160) How sensitive the robot is to a given turn command, 
    a higher value means that smaller commands will have a greater effect
    (ie. if turn_rate_sensitivity = 10, the apm command will be 125 + (turn_rate_sensitivity * given command))
    - *move_rate_sensitivity* - (default = 25) How sensitive the robot is to a given movement (forward/backward) command, 
    a higher value means that smaller commands will have a greater effect 
    (ie. if move_rate_sensitivity = 10, the apm command will be 125 + (move_rate_sensitivity * given command))
    - *port* - (default = "/dev/ttyACM") The USB port where the driver will look for the APM

##  Decision
---------------------------
- sb_waypoint_creator
  - **Parameters**
    - *path* - A list of x,y coordinates that the robot should go to, in the form [x, y, x, y, ...]
