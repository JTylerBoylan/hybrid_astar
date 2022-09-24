import rospy
from geometry_msgs.msg import PointStamped

def goal_point():
    
    # Initialize ROS node
    rospy.init_node('goal_point', anonymous=True)

    # Get parameters
    frm = rospy.get_param("~frame_id")
    tpc = rospy.get_param("~goal_topic")
    px = rospy.get_param("~x")
    py = rospy.get_param("~y")
    pz = rospy.get_param("~z")

    goal = PointStamped()

    goal.header.frame_id = frm

    goal.point.x = px
    goal.point.y = py
    goal.point.z = pz


    pub = rospy.Publisher(tpc, PointStamped, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rate.sleep()


if __name__ == '__main__':
    try:
        goal_point()
    except rospy.ROSInterruptException:
        pass