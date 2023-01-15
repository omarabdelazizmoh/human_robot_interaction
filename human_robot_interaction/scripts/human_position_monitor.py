#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from worldsim.msg import Act_msg
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

current_x = 0
current_y = 0

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

def callback_fn(data):
    global current_x
    global current_y

    current_model = data.name
    #rospy.loginfo(data.pose[1])

    # Actor 1 Position
    current_x = data.pose[-2].position.x
    current_y = data.pose[-2].position.y
    # current_x = data.pose[-1].position.x
    # current_y = data.pose[-1].position.y

   
    # current_x1 = data.pose[-2].position.x
    # current_y1 = data.pose[-2].position.y

    # linear_vel  = data.twist[-1].linear.x
    # angular_vel = data.twist[-1].angular.z
    #rospy.loginfo("This is my first Node")
    # rospy.loginfo("Actor 1 Position, X: {:0.2f}, Y:{:0.2f}".format(current_x,current_y))
    # rospy.loginfo("Actor 2 Position, X: {:0.2f}, Y:{:0.2f}".format(current_x1,current_y1))
    # rospy.loginfo("Actor 1 Velocity, X: {:0.2f}, Y:{:0.2f}".format(linear_vel,angular_vel))


def main():
    global current_x
    global current_y
    key_timeout = 0.5
    settings = saveTerminalSettings()
    rospy.init_node('location_monitor')
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback_fn)
    pub = rospy.Publisher('actor1/vel_cmd', Act_msg, queue_size=10)

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        key = getKey(settings, key_timeout)
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            # Publisher 
            cmd_vel = Act_msg()
            cmd_vel.target_x = current_x + x
            cmd_vel.target_y = current_y + y
            cmd_vel.velocity = 1.0

            pub.publish(cmd_vel)
        rate.sleep()
    

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

