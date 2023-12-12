# Keyboard Ackermann manual controller for Autoware

2023-12-13 H.Miyagi (hideyasu.miyagi.ni@futu-re.co.jp)

## Abstract

This node allows user to manually move virtual vehicle in AWSIM simulator.
The program is expected to work like 'teleop_keyboard' for a vehicle with Ackermann model

## Usage

~~~
$ ros2 node list
  (you should see /AWSIM)

$ ros2 run keyboard_ackermann_controller keyboard_ackermann_controller
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

