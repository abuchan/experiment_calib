#!/usr/bin/python

import numpy
import sys
import rosbag

from avg_bag_tf import *

from tf.transformations import *
from copy import deepcopy

frame_to_ar_tf = TransformStamped()
frame_to_ar_tf.header.frame_id = 'calib_frame'
frame_to_ar_tf.child_frame_id = 'ar_marker_0'
frame_to_ar_tf.transform = numpy_to_tf_msg(
  [0.07, 0.07, 0.0],[0.0,0.0,0.0,1.0]
)

exp_ar = deepcopy(frame_to_ar_tf)
exp_ar.child_frame_id = 'exp_ar0'

goal_tf = ['camera_rig','usb_cam']
goal_path = ['camera_rig','map','calib_frame','ar_marker_0','usb_cam']

def tf_to_static_launch(tfs):
  fmt_str = '<node name=\"%s\" pkg=\"tf\" type=\"static_transform_publisher\" args=\"%f %f %f %f %f %f %f %s %s 100\" />'
  src = tfs.header.frame_id
  dest = tfs.child_frame_id
  name = src + '_to_' + dest + '_pub'
  return fmt_str % (
    name,
    tfs.transform.translation.x,
    tfs.transform.translation.y,
    tfs.transform.translation.z,
    tfs.transform.rotation.x,
    tfs.transform.rotation.y,
    tfs.transform.rotation.z,
    tfs.transform.rotation.w,
    src,
    dest
  )
  
def H_to_tf_msg(H):
  trans = H[0:3,3]
  rot = quaternion_from_matrix(H)
  return numpy_to_tf_msg(trans,rot)

if __name__ == '__main__':
  with rosbag.Bag(sys.argv[1]) as bag:
    for topic,msg,t in bag.read_messages():
      if topic == '/tf':
        tf_msg = msg

    tf_msg.transforms.append(frame_to_ar_tf)

    srcs = []
    dests = []
    for t in tf_msg.transforms:
      srcs.append(t.header.frame_id)
      dests.append(t.child_frame_id)
   
    tf_idxs = []
    for (src,dest) in zip(goal_path[:-1], goal_path[1:]):
      for i in range(len(srcs)):
        t_s, t_d = srcs[i],dests[i]
        if src == t_s and dest == t_d:
          tf_idxs.append((i,False))
        elif src == t_d and dest == t_s:
          tf_idxs.append((i,True))

    print tf_idxs

    tf_chain = []
    for tfi,flip in tf_idxs:
      trans,rot = tf_msg_to_numpy(tf_msg.transforms[tfi])
      if flip:
        trans = -trans
        rot = quaternion_inverse(rot)

      H = quaternion_matrix(rot)
      H[0:3,3] = H[0:3,0:3].dot(trans)
      tf_chain.append(H)

    goal_H = reduce(numpy.dot, tf_chain)

    goal_msg = TransformStamped()
    goal_msg.header.frame_id = goal_tf[0]
    goal_msg.child_frame_id = goal_tf[1]
    goal_msg.transform = H_to_tf_msg(goal_H)
    
    for i in range(1,len(tf_chain)):
      inter_tf = deepcopy(goal_msg)
      inter_tf.child_frame_id = 'int_' + goal_path[i]
      inter_tf.transform = H_to_tf_msg(reduce(numpy.dot, tf_chain[:i]))
      print tf_to_static_launch(inter_tf)

    print tf_to_static_launch(exp_ar)
    print tf_to_static_launch(goal_msg)
