#!/usr/bin/env python

#import roslib
#roslib.load_manifest('camera_tf')
import rospy

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
        rospy.init_node('camera_tf2_broadcaster')
        pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        #br = tf2_ros.StaticTransformBroadcaster()
        rate=rospy.Rate(10)

        while not rospy.is_shutdown():
                static_transformStamped = geometry_msgs.msg.TransformStamped()
                
                static_transformStamped.header.stamp = rospy.Time.now()
                static_transformStamped.header.frame_id = ""
                static_transformStamped.child_frame_id = "map"
                
                static_transformStamped.transform.translation.x = 0.0
                static_transformStamped.transform.translation.y = 0.0
                static_transformStamped.transform.translation.z = 0.0
                
                #quat = tf.transformations.quaternion_from_euler(float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
                static_transformStamped.transform.rotation.x = 0.0
                static_transformStamped.transform.rotation.y = 0.0
                static_transformStamped.transform.rotation.z = 0.0
                static_transformStamped.transform.rotation.w = 1.0
                
                #br.sendTransform(static_transformStamped)
                tfm=tf2_msgs.msg.TFMessage([static_transformStamped])
                pub_tf.publish(tfm)
                rate.sleep()
