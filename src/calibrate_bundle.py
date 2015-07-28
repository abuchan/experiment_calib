#!/usr/bin/python

import rospy
import rosbag
import tf
import sys
from std_msgs.msg import String
from experiment_calib.srv import *
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, TransformStamped
from avg_bag_tf import numpy_to_tf_msg, tf_msg_to_numpy

TIME_THRESH = rospy.Duration(1.0)

class CalibrateBundle():
  def __init__(self, main_marker = 'ar_marker_0',bundle_markers=['ar_marker_1']):
    self.main_marker = main_marker
    self.bundle_markers = bundle_markers
    self.known_poses = {}
    self.next_poses = {}
  
  def commit_next_poses(self, req):
    self.known_poses.update(self.next_poses)
    return CommitNextPosesResponse()

  def save_known_poses(self, req):
    print self.known_poses.values()
    with rosbag.Bag('bundle.bag','w') as bag:
      tf_msg = TFMessage()
      tf_msg.transforms = self.known_poses.values()
      bag.write('/tf',tf_msg,rospy.Time.now())
      bag.close()

    return SaveKnownPosesResponse()

  def run(self):
    rospy.init_node('calibrate_bundle')
    next_pub = rospy.Publisher('next_poses', String, queue_size = 1)
    tf_b = tf.TransformBroadcaster()
    tf_l = tf.TransformListener()
    
    rate = rospy.Rate(10)
    
    rospy.Service('commit_next_poses', CommitNextPoses, self.commit_next_poses )
    rospy.Service('save_known_poses', SaveKnownPoses, self.save_known_poses )

    main_frame_avg = self.main_marker + '_avg'
    main_frame_bundle = self.main_marker + '_bundle'
    
    while not rospy.is_shutdown():
      self.next_poses = {}
      for bundle_marker in self.bundle_markers:
        marker_avg_frame = bundle_marker + '_avg'
        marker_bundle_frame = bundle_marker + '_bundle'
        if marker_bundle_frame not in self.known_poses.keys():
          try:
            now = rospy.Time.now()
            if (now - tf_l.getLatestCommonTime(main_frame_avg, marker_avg_frame)) < TIME_THRESH:
              T,R = tf_l.lookupTransform(main_frame_avg, marker_avg_frame, rospy.Time(0))
              tfs = TransformStamped()
              tfs.header.stamp = now
              tfs.header.frame_id = main_frame_bundle
              tfs.child_frame_id = marker_bundle_frame
              tfs.transform = numpy_to_tf_msg(T,R)
              self.next_poses[tfs.child_frame_id] = tfs

          except (
            tf.Exception, tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException) as e :
            continue

      tf_b.sendTransform([0,0,0],[0,0,0,1],now, main_frame_bundle, main_frame_avg)
      
      for kp in self.known_poses.values():
        base_frame = kp.header.frame_id
        child_frame = kp.child_frame_id
        T,R = tf_msg_to_numpy(kp)
        tf_b.sendTransform(T,R,now,child_frame,base_frame)
      
      next_pub.publish(str(self.next_poses.keys()))
      
      rate.sleep()
      
if __name__ == '__main__':
  cb = CalibrateBundle(sys.argv[1],sys.argv[2:])
  cb.run()
