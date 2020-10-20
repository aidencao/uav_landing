#!/usr/bin/env python
import rospy
import math
import thread
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import String


class listener_tags:
    def __init__(self):
        self.tags_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tags_callback)
        self.com_sub = rospy.Subscriber(
            "landing_recv", String, self.com_callback)
        self.com_pub = rospy.Publisher("landing_send", String,
                                       queue_size=1)

    def getYaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        yaw = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        # angleYaw = yaw*180/math.pi
        return yaw

    def tags_callback(self, data):
        global g_msg, lock, g_count

        positionArray = data.detections

        lock.acquire()
        msg_77 = '0 ' + '0 ' + '0 ' + '0 ' + '0'
        msg_78 = '0 ' + '0 ' + '0 ' + '0 ' + '0'

        for i in range(len(positionArray)):
            # id == 78
            if(positionArray[i].id == (78,)):
                positionInfo = positionArray[i].pose.pose.pose
                offset = positionInfo.position
                quaternion = positionInfo.orientation
                yaw = self.getYaw(quaternion)
                msg_78 = '1 ' + str(float('%.5f' % offset.x)) + ' ' + str(float('%.5f' % offset.y)) + \
                    ' ' + str(float('%.5f' % offset.z)) + \
                    ' ' + str(float('%.5f' % yaw))

            # id == (77,)
            if(positionArray[i].id == (77,)):
                positionInfo = positionArray[i].pose.pose.pose
                offset = positionInfo.position
                quaternion = positionInfo.orientation
                yaw = self.getYaw(quaternion)
                msg_77 = '1 ' + str(float('%.5f' % offset.x)) + ' ' + str(float('%.5f' % offset.y)) + \
                    ' ' + str(float('%.5f' % offset.z)) + \
                    ' ' + str(float('%.5f' % yaw))
        '''
        if(msg_77[0:1] is '0' and msg_78[0:1] is '0' and g_count < 10):
            g_count = g_count+1
        else:
            g_count = 0
            g_msg = msg_77 + ' ' + msg_78
        '''
        g_msg = msg_77 + ' ' + msg_78
        # print(g_msg)
        lock.release()

    def com_callback(self, data):
        global g_msg, lock, g_count
        try:
            #msg = data.data
            #print(msg)
            lock.acquire()
            self.com_pub.publish(g_msg)
            print(g_msg)
            '''
            g_msg = '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0'
            g_count = 0
            '''
            lock.release()
        except Exception as e:
            print("get_msg error")
            print(e)
            if lock.locked():
                lock.release()


if __name__ == '__main__':
    try:
        lock = thread.allocate_lock()
        g_msg = '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0 ' + '0'
        g_count = 0
        rospy.init_node("listener_node")
        rospy.loginfo("Starting listener node")
        listener_tags()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down listener node.")
