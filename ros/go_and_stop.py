import rospy, time, sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import UInt16

from time import sleep

SIGNAL_GO = 2309


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def autopilot():
    signal_go = rospy.Publisher('/signal/go', UInt16, queue_size=1,latch=True)
    r = rospy.Rate(10)  # 10hz

    signal_go.publish(0)

    while(1):

        key = getKey()

        if key == 'g':  # GO!
            signal_go.publish(SIGNAL_GO)
        elif key == 's':  # STOP
            signal_go.publish(0)

    r.sleep()


if __name__ == '__main__': 
    rospy.init_node('eStop_node', anonymous=True)
    settings = termios.tcgetattr(sys.stdin)
    try:
        autopilot()
    except rospy.ROSInterruptException:
        pass
