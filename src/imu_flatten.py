#!/usr/bin/python

# ROS imports
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# general imports
from collections import deque
import math as m


class ImuCalc:
    
    def __init__(self, name='imu_calcer', out_topic=None):

        self.name = name
        rospy.loginfo('{}: initiating class...'.format(self.name))

        self.init_info = True
        self.values = []
        self.disp_count = 0
        self.flatten_threshold = 50.0
        self.d = deque(self.values)

        self.pub_rate = 10.0
        self.rate = rospy.Rate(self.pub_rate)
        self.balance_pub = rospy.Publisher(out_topic, Twist, queue_size=10)
        self.balance_msg = Twist()
        rospy.loginfo('{}: balance publisher registered.'.format(self.name))

        rospy.loginfo('init done.')

    # global function
    def register_imu_sub(self, topic):
        rospy.Subscriber(topic, Imu, self.__imu_cb)
        rospy.loginfo('{}: subscriber registered.'.format(self.name))
        # rospy.spin()

    def pub_balance(self):
        rospy.loginfo('{}: start publishing balancer values'.format(self.name))
        while not rospy.is_shutdown():
            self.balance_pub.publish(self.balance_msg)
            self.rate.sleep()

    # helper functions
    def _RPcalc(self,x,y,z):
        roll    = m.atan2(y,z) * 57.3
        pitch   = m.atan2( (-x), m.sqrt(y*y+z*z) ) * 57.3
        return round(roll,2), round(pitch,2)

    def _get_average_from_list(self,list):
        x, y, z = [], [], []
        for xyz in list:
            x.append(xyz[0])
            y.append(xyz[1])
            z.append(xyz[2])
        x_av = round((sum(x) / self.flatten_threshold),2)
        y_av = round((sum(y) / self.flatten_threshold),2)
        z_av = round((sum(z) / self.flatten_threshold),2)
        return x_av, y_av, z_av

    # subscriber callbacks / publisher functions
    def __imu_cb(self, data):
        imu = data.linear_acceleration
        l = [imu.x, imu.y, imu.z]
        d_len = len(self.d)

        self.d.pop() if d_len >= self.flatten_threshold else 0
        self.d.appendleft(l)

        if d_len >= self.flatten_threshold:
            x_av, y_av, z_av = self._get_average_from_list(list=list(self.d))   # calc mittelwert
            if self.disp_count == self.flatten_threshold:
                r, p = self._RPcalc(x_av,y_av,z_av) # roll & pitch, yaw = 0
                self.balance_msg.angular.x = r
                self.balance_msg.angular.y = p
                # self.balance_msg.angular.z = 0
                # rospy.loginfo('flattened vals:   x: {},   y: {},   z: {}'.format(x_av,y_av,z_av))
                rospy.loginfo('{}: flattened angular:r: {},   p: {}'.format(self.name, r,p))
                self.disp_count = 0
            else:
                self.disp_count += 1


if __name__ == "__main__":

    print('running from inside imu_flatten.py')

    # register rosnode
    name = 'imu_calc'
    in_topic = '/imu_raw'
    out_topic = '/balance_transforms'
    rospy.init_node(name)
    rospy.loginfo('{}: node initiated.'.format(name))

    ic = ImuCalc(name=name, out_topic=out_topic)

    try:
        ic.register_imu_sub(topic="/imu_raw")
        ic.pub_balance()
    except rospy.ROSInterruptException as e:
        rospy.logfatal('{}: something went wrong here: {}'.format(ic.name, e))
