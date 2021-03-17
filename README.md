Prerequisites:
dynamixel_workbench
https://github.com/ROBOTIS-GIT/dynamixel-workbench

dynamixel_workbench_msgs
https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs

To launch:
roslaunch separator_end_effector separator_ee.launch

Service "tool_service" is used for controlling the end-effector. Requests are:
"open1" - start motors for opening side 1
"open2" - start motors for opening side 2
"close1" - start motors for closing side 1
"close2" - start motors for closing side 2
"open_both" - start motors for opening both sides
"close_both" - start motors for closing both sides
"stop" - stops both motors

response is current motor state of both motors
