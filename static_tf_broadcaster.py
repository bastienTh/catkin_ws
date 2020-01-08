#!/usr/bin/env python  
import roslib
import rospy
import tf

if __name__ == '__main__':
   rospy.init_node('static_tf_broadcaster')
   name = "static_tf_broadcaster"
   br = tf.TransformBroadcaster()
   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():
      x=-0.23
      y=-0.40
      z=-0.125
      br.sendTransform((x + 0.34/2, y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        name,
                        "base")
      rate.sleep()