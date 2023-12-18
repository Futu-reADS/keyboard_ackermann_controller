g# keybard_ackermann_controller

A keyboard-based manual controller for Ackermann model vehicle

2023-12-13 H.Miyagi (hideyasu.miyagi.ni@futu-re.co.jp)

## Abstract

This program allows users to manually move a virtual vehicle in autoware-compatible simulators such as AWSIM.
Also the program is expected to work like 'teleop_keyboard' for a vehicle with Ackermann model.

## Usage

~~~
# run AWSIM (either on local PC or on remote PC)

$ ros2 node list   # To check if the simulator is alive
  (you should see /AWSIM)

# Run this controller 
$ ros2 run keyboard_ackermann_controller keyboard_ackermann_controller --ros-args -p "use_report:=yes" -p "use_sim_time:=true"

~~~
# run on real machine (IFB board)

$ ros2 node list   # To check if the simulator is alive
  (you should see node controlling IFB board)

# Run this controller
$ ros2 run keyboard_ackermann_controller keyboard_ackermann_controller


Operation Keys
--------------

~~~
    i     <--- increase forward speed (or decrease backward speed)
  j k l   <--- 'k' to stop altogether
    ,     <--- increase backward speed (or decrease forward speed)

  ^   ^
  |   |
  |   +--- "decrease" steer angle (rotate steering wheel to the right)
  | 
  +------- "increase" steer angle (rotate steering wheel to the left)


~~~~

Press `Ctrl-C` to stop.

[END OF TEXT]



