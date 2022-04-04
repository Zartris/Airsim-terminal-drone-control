# Airsim terminal drone control

Commands:
- arm
- disarm
- takeoff
- home
- stop
- kc = Keyboard Control
- move (x y z vel)
- moveOnPath (x1 y1 z1 ... xn yn zn vel)

Capital letters does not matter.

## Install
pip install msgpack-rpc-python

pip install airsim

pip install pynput

We are using pynput over keyboard since linux users have to run as sudo for keyboard to function.

Since the pynput didn't have all the functionality I wanted, a wrapper is created to handle multiple key presses.


## Keyboard Control
To enter keyboard control type kc in the terminal.

Remember to arm and takeoff before entering the kc mode.

Currently we are only supporting world space controlling, so the control will be independend on the direction the drone is facing.
The drone space controlling will be supportet later.

Control:
- W = + x-axis 
- S = - x-axis
- D = + y-axis
- A = - y-axis
- X = + z-axis (down)
- Z = - z-axis (up)
- E = turn right
- Q = turn left
- T = Terminate kc mode
