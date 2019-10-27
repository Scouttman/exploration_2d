#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher ('/my_odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='turtlebot3_burger'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    #print(result.pose)
    x = result.pose.position.x
    y = result.pose.position.y

    rx = result.pose.orientation.x
    ry = result.pose.orientation.y
    rz = result.pose.orientation.z
    rw = result.pose.orientation.w

    odom_broadcaster.sendTransform(
        (x,y,0),
        (rx,ry,rz,rw),
        current_time,
        "odom",
        "map"
    )

    r.sleep()