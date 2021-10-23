#!/usr/bin/env python

# Send 0 as a constant command to G29 steering wheel
# to achive self centering

import rospy
from g29_force_feedback.msg import ForceFeedback
from std_msgs.msg import Header


if __name__ == "__main__":
    rospy.init_node('g29_self_centering')
    angle_pub = rospy.Publisher('/target_angle', ForceFeedback, queue_size=1)
    r = rospy.Rate(20)

    # send angle of 0 to steering wheel
    centering_cmd = ForceFeedback()
    centering_cmd.header = Header()
    centering_cmd.header.seq = 0
    centering_cmd.header.frame_id = 'force_feedback_target'
    centering_cmd.angle = 0.0
    centering_cmd.force = 0.0
    centering_cmd.pid_mode = True


    # rospy.spin()
    while not rospy.is_shutdown():
        # Update header
        centering_cmd.header.stamp = rospy.Time.now()
        
        # Publish command
        angle_pub.publish(centering_cmd)
        r.sleep()
    