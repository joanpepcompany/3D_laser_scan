#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

pub_pan = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=10)
pub_lift = rospy.Publisher('/arm_shoulder_lift_joint/command', Float64, queue_size=10)


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        nb = raw_input('Introduce a key: ')
        print "Key introduced = %s" % nb
        if (nb == 's'):
            startPan360Rotation()

        if (nb == 'r'):
            movePanToPos(0)
            moveLiftToPos(0)

        if (nb == 'rp'):
            movePanToPos(0)

        if (nb == 'rl'):
            moveLiftToPos(0)

        if (nb == '0'):
            movePanToPos(0)
            moveLiftToPos(0)

        if (nb == '10'):
            movePanToPos(float(nb))

        else:
            rospy.loginfo('Help Introduce an "s" to start a 360 rotation')
            rospy.loginfo('"r" to move the servo to zero')


       
        rate.sleep()

def movePanToPos(pose):
    pub_pan.publish(pose*3.14/360)

def moveLiftToPos(pose):
    pub_lift.publish(pose*3.14/360)

def startPan360Rotation():

    rate = rospy.Rate(40) # 10hz
    for x in range(0, 1300):
        # pub_pan.publish(x*3.14/720)
        pub_pan.publish(x*0.0025)

        rate.sleep()
        if rospy.is_shutdown():
            break
    rospy.loginfo('360 rotation has finished')


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass