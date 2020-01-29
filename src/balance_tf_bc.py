#!/usr/bin/python


# general stuff
import sys

# import ros stuff
import rospy
import tf.msg
from geometry_msgs.msg import TransformStamped, Twist
from tf.transformations import quaternion_from_euler

# reconfigure
# from dynamic_reconfigure.server import Server
# from testbench_zz.cfg import TFAdjustConfig

import math
# from collections import deque
# from threading import Thread, Lock


class BalancerTFBC:

    def __init__(self, name="balancer_tf_broadcast", from_frame=None, to_frame=None, pub_rate=100):

        self.node_name = name
        self.seq_count  = 0
        self.rate       = rospy.Rate(pub_rate)

        self.pub_tf = rospy.Publisher("/tf",tf.msg.tfMessage, queue_size=10)

        self.tf_msg                         = TransformStamped()
        self.tf_msg.header.seq              = self.seq_count
        self.tf_msg.header.frame_id         = from_frame
        self.tf_msg.child_frame_id          = to_frame
        self.tf_msg.header.stamp            = rospy.Time.now()
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.x    = 0.0
        self.tf_msg.transform.rotation.y    = 0.0
        self.tf_msg.transform.rotation.z    = 0.0
        self.tf_msg.transform.rotation.w    = 1.0

        self.tfm = tf.msg.tfMessage([self.tf_msg])

        self.pub_tf.publish(self.tfm)       # pub once at init

        rospy.loginfo("{}: init completed.".format(self.node_name))

    # helper functions
    def _to_rad(self,angles):
        return [ ((angle/180) * math.pi) for angle in angles ]

    # global functions
    def run_pub(self):
        rospy.loginfo("{}: starting publisher & keep node running.".format(self.node_name))

        while not rospy.is_shutdown():
            self.tf_msg.header.stamp = rospy.Time.now()     # Very Important!
            self.tfm = tf.msg.tfMessage([self.tf_msg])
            self.pub_tf.publish(self.tfm)                   # publish at rate
            self.rate.sleep()

    def register_sub(self, topic, msgtype):
        rospy.loginfo('{}: registering subscriber to: {}'.format(self.node_name, topic))
        self.balance_sub = rospy.Subscriber(topic, msgtype, self.__balance_cb)

    # callback functions
    def __balance_cb(self, msg):
        roll, pitch = msg.angular.x, msg.angular.y
        rad_angles = self._to_rad(angles=[roll, pitch])
        q = quaternion_from_euler(rad_angles[0], rad_angles[1], 0)
        self.tf_msg.transform.rotation.x    = q[0]
        self.tf_msg.transform.rotation.y    = q[1]
        self.tf_msg.transform.rotation.z    = q[2]
        self.tf_msg.transform.rotation.w    = q[3]


if __name__ == '__main__':

    node_name = "testbench_dynamic_tf"
    default_rate    = 100

    subscriber_topic = '/balance_transforms'
    subscriber_msgtype = Twist

    node = rospy.init_node(node_name)
    rospy.loginfo("{}: initializing node.".format(node_name))

    if len(sys.argv) >= 2:
        try:
            init_rate = sys.argv[1]
            init_rate = int(init_rate)
            rospy.loginfo("{}: publish rate set via input: {}".format(node_name, init_rate))
        except:
            rospy.logwarn("{}: except thrown; failed to cast string rate input to int".format(node_name))
            init_rate = default_rate
            rospy.loginfo("{}: publish rate using default: {}".format(node_name, init_rate))
    else:
        init_rate = default_rate
        rospy.loginfo("{}: publish rate using default: {}".format(node_name, init_rate))

    balance_tf = BalancerTFBC(name=node_name, from_frame='map', to_frame='velodyne', pub_rate=init_rate)

    try:
        balance_tf.register_sub(topic=subscriber_topic, msgtype=subscriber_msgtype)
        balance_tf.run_pub()        # do last --> is blocking
    except rospy.ROSInterruptException as e:
        rospy.logerr('{}: {}'.format(balance_tf.node_name, e))

