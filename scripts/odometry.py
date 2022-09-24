import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

def odometry():
    
    # Initialize ROS node
    rospy.init_node('odometry', anonymous=True)

    # Get parameters
    frm = rospy.get_param("~frame_id")
    tpc = rospy.get_param("~odom_topic")
    px = rospy.get_param("~px")
    py = rospy.get_param("~py")
    pz = rospy.get_param("~pz")
    ox = rospy.get_param("~ox")
    oy = rospy.get_param("~oy")
    oz = rospy.get_param("~oz")
    ow = rospy.get_param("~ow")
    v = rospy.get_param("~v")
    u = rospy.get_param("~u")

    odom = Odometry()

    odom.header.frame_id = frm

    odom.child_frame_id = frm

    posecv = PoseWithCovariance()
    posecv.pose.position.x = px
    posecv.pose.position.y = px
    posecv.pose.position.z = px
    posecv.pose.orientation.x = ox
    posecv.pose.orientation.y = oy
    posecv.pose.orientation.z = oz
    posecv.pose.orientation.w = ow

    odom.pose = posecv

    twistcv = TwistWithCovariance()
    twistcv.twist.linear.x = v
    twistcv.twist.angular.z = u

    odom.twist = twistcv

    pub = rospy.Publisher(tpc, Odometry, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        pub.publish(odom)
        rate.sleep()


if __name__ == '__main__':
    try:
        odometry()
    except rospy.ROSInterruptException:
        pass