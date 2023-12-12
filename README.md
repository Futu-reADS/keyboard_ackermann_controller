# keybard_ackermann_controller

A keyboard-based manual controller for Ackermann model vehicle

2023-12-13 H.Miyagi (hideyasu.miyagi.ni@futu-re.co.jp)

## Abstract

This program allows users to manually move a virtual vehicle in autoware-compatible simulators such as AWSIM.
Also the program is expected to work like 'teleop_keyboard' for a vehicle with Ackermann model.

## Usage

~~~
$ ros2 node list   # To check if the simulator is alive
  (you should see /AWSIM)

$ ros2 run keyboard_ackermann_controller keyboard_ackermann_controller   # Run this controller
~~~

Operation Keys
--------------

~~~
  u i o   <--- increase forward speed (or decrease backward speed)
  j k l   <--- make target speed closer to 0
  m , .   <--- increase backward speed (or decrease forward speed)

  ^ ^ ^
  | | |
  | | +--- decrease steer angle (steer to the right)
  | +----- set steer angle to 0
  +------- increase steer angle (steer to the left)

~~~~

`Ctlr-C` to stop the node.


