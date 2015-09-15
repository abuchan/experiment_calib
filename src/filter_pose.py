#!/usr/bin/python

import rospy
import tf
import sys
import numpy

# Number of frames to use for average
N_FRAMES = 10

# Oldest transform allowed
TIME_THRESH = rospy.Duration(N_FRAMES * 0.1)

# Translation threshold (should be in meters)
TRANS_THRESH = 0.05

# Orientation threshold (quaternion norm)
ORI_THRESH = 0.3

def avg_pose(poses):
  Ts = numpy.array([T for _,T,_ in poses])
  Rs = numpy.array([R for _,_,R in poses])
  return Ts.mean(axis=0), Rs.mean(axis=0)
  
if __name__ == '__main__':
  base_frame = sys.argv[1]
  child_frames = sys.argv[2:]
  rospy.init_node('pose_filter')

  rate = rospy.Rate(10)
  listener = tf.TransformListener()
  broadcaster = tf.TransformBroadcaster()

  hists = [[]]*len(child_frames)

  while not rospy.is_shutdown():
    n = rospy.Time.now()
    
    # remove old frames
    for hist in hists:
      for i in range(len(hist)):
        if n - (hist[i][0]) > TIME_THRESH:
          hist = hist[:(i-1)]
          break

    for child_frame,hist in zip(child_frames,hists):
      try:
        tf_time = listener.getLatestCommonTime(base_frame, child_frame)
        if (n - tf_time) < TIME_THRESH: 
          # get current pose
          T,R = listener.lookupTransform(base_frame, child_frame, rospy.Time(0))
        
          # add current pose to history, truncating to N_FRAMES
          hist = [(tf_time,T,R)] + hist
          hist = hist[:N_FRAMES]

          # calculate average pose over history window
          if len(hist) > 0:
            T_n, R_n = numpy.array(T), numpy.array(R)
            T_avg, R_avg = avg_pose(hist)
            broadcaster.sendTransform(T_avg, R_avg, n, child_frame + '_avg', base_frame)
        
            T_diff = ((T_n-T_avg)**2).sum()**0.5
            R_diff = ((R_n-R_avg)**2).sum()**0.5
            
            print T_diff, R_diff

            # if current pose is within threshold of average pose
            if T_diff < TRANS_THRESH and R_diff < ORI_THRESH:
              broadcaster.sendTransform(T, R, n, child_frame + '_filt', base_frame)

      except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e :
        continue

      rate.sleep()

