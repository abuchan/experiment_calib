#!/usr/bin/python

import rosbag
import rospy
import rospkg
import tf
from tf.transformations import *
import numpy
from avg_bag_tf import tf_msg_to_numpy
import sys

rospack = rospkg.RosPack()
DEFAULT_BAGFILE = rospack.get_path('experiment_calib') + '/data/bundle.bag'

TIME_THRESH = rospy.Duration(1.0)
POS_THRESH = 0.01
ORI_THRESH = 0.1

def transform_mean(Hs):
  Qs = numpy.array([quaternion_from_matrix(H) for H in Hs])
  for Q0,Q1 in zip(Qs[:-1],Qs[1:]):
    if Q0.dot(Q1) < 0:
      Q1 *= -1.0
  Ts = numpy.array([H[0:3,3] for H in Hs])
  avg_Q = Qs.mean(axis=0)
  avg_Q[3] = (1.0 - (avg_Q[:3]**2).sum())**0.5
  avg_T = Ts.mean(axis=0)
  return avg_T, avg_Q

def transform_diff(H0,H1):
  diff_T = ((H1[0:3,3] - H0[0:3,3])**2).sum()**0.5
  Q0,Q1 = quaternion_from_matrix(H0), quaternion_from_matrix(H1)
  diff_Q = ((Q1-Q0)**2).sum()**0.5
  return diff_T, diff_Q

class TrackBundle():
  def __init__(self, base_frame='usb_cam', bundle_bagfile=DEFAULT_BAGFILE):
    with rosbag.Bag(bundle_bagfile) as bundle_bag:
      tf_msg = [msg for _,msg,_ in bundle_bag.read_messages()][0]

    self.base_frame = base_frame

    # build dictionary mapping marker_filt to inverse transform necessary to get to main marker
    self.bundle_tfs = {}
    for bundle_tf in tf_msg.transforms:
      marker_name = bundle_tf.child_frame_id[:-7] + '_filt'
      T,R = tf_msg_to_numpy(bundle_tf)
      H = quaternion_matrix(R)
      H[0:3,3] = T
      self.bundle_tfs[marker_name] = inverse_matrix(H)
    
    self.main_marker = tf_msg.transforms[0].header.frame_id[:-7]
    self.bundle_tfs[self.main_marker + '_filt'] = numpy.eye(4)

  def run(self):
    rospy.init_node('track_bundle')
    tf_l = tf.TransformListener()
    tf_b = tf.TransformBroadcaster()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
      visible_recons = {}
      now = rospy.Time.now()
      for child_frame in self.bundle_tfs.keys():
        try:
          tf_time = tf_l.getLatestCommonTime(self.base_frame, child_frame)
          if (now - tf_time) < TIME_THRESH:
            T,R = tf_l.lookupTransform(self.base_frame, child_frame, rospy.Time(0))
            H = quaternion_matrix(R)
            H[0:3,3] = T
            visible_recons[child_frame] = H.dot(self.bundle_tfs[child_frame])
        
        except (
          tf.Exception, tf.LookupException, 
          tf.ConnectivityException, tf.ExtrapolationException) as e :
          continue
     
      if len(visible_recons) is not 0:
        avg_T, avg_Q = transform_mean(visible_recons.values())
        tf_b.sendTransform(avg_T, avg_Q, now, self.main_marker + '_bundle', self.base_frame)
        for frame_name, trans in self.bundle_tfs.iteritems():
          H = inverse_matrix(trans)
          T,R = H[0:3,3],quaternion_from_matrix(H)
          tf_b.sendTransform(T, R, now, frame_name + '_bundle', self.main_marker + '_bundle')

      rate.sleep()
      
if __name__ == '__main__':
  tb = TrackBundle(sys.argv[1],sys.argv[2])
  tb.run()
