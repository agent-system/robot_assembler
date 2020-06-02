import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    ## see descripton
    ## http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
    rospy.loginfo('min %f -(%f)-> max %f'%(
        msg.angle_min, msg.angle_increment, msg.angle_max))
    #msg.range_min
    #msg.range_max
    #msg.ranges

if __name__ == '__main__':
    rospy.init_node('range_listener', anonymous=True)

    rospy.Subscriber("range_sensor", LaserScan, callback)

    rospy.spin()
