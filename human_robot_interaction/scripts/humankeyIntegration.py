#!/usr/bin/env python3
import time
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# moveBindings["w"] === (12,20)
# moveBindings["w"][0] === 12
# moveBindings["w"][1] === 20

moveBindings = {
        'w':(0,1),
        'a':(-1,0),
        's':(0,-1),
        'd':(1,0)
    }

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)
def main():
    key_timeout = 0.5
    settings = saveTerminalSettings()
        
    while True:
        key = getKey(settings, key_timeout)
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            cmd_vel.target_x = current_x + x
            cmd_vel.target_y = current_y + y
            print(key)
            print("x: %s" %x)
            print("y: %s" %y)
main()
