#!/usr/bin/python

import rosbag
import sys
import numpy
from geometry_msgs.msg import Transform, TransformStamped
from tf2_msgs.msg import TFMessage
import rospy
import time

# Tools for finding the average transforms between tf frames in a bagfile
# See end of file for usage instructions

def get_tf_groups(bag_name):
  tf_groups = []
  with rosbag.Bag(bag_name) as bag:
    tf_msgs = [
      message for topic, message, ts in bag.read_messages() if topic == '/tf'
    ]

    tf_groups.append([tf_msgs[0].transforms[0]])
    
    for message in tf_msgs:
      for transform in message.transforms:
        added = False
        for tf_group in tf_groups:
          if (transform.header.frame_id == tf_group[0].header.frame_id and 
            transform.child_frame_id == tf_group[0].child_frame_id
          ):
            tf_group.append(transform)
            added = True
            break
        
        if not added:
          tf_groups.append([transform])

  return tf_groups

def tf_msg_to_numpy(tf_msg):
  t = tf_msg.transform.translation
  trans = numpy.array([t.x, t.y, t.z])
  r = tf_msg.transform.rotation
  rots = numpy.array([r.x, r.y, r.z, r.w])
  return trans,rots

def numpy_to_tf_msg(tran, rot):
  t = Transform()
  t.translation.x = tran[0]
  t.translation.y = tran[1]
  t.translation.z = tran[2]
  t.rotation.x = rot[0]
  t.rotation.y = rot[1]
  t.rotation.z = rot[2]
  t.rotation.w = rot[3]
  return t

# Find the average tranforms of a list of transform groups
def avg_transforms(tf_groups):
  avgs = []
  for tf_group in tf_groups:
    trans = numpy.empty((len(tf_group),3))
    rots = numpy.empty((len(tf_group),4))
    for i in range(len(tf_group)):
      trans[i,:], rots[i,:] = tf_msg_to_numpy(tf_group[i])
  
    avg_trans = trans.mean(axis=0)
    avg_rot = rots.mean(axis=0)
    # Assuming quaternions are close, averaging and normalizing should be close
    # to the desired average quaternion
    avg_rot[3] = (1.0-(avg_rot[:3]**2).sum())**0.5

    avg_tf = TransformStamped()
    avg_tf.header.frame_id = tf_group[0].header.frame_id
    avg_tf.child_frame_id = tf_group[0].child_frame_id
    avg_tf.transform = numpy_to_tf_msg(avg_trans, avg_rot)

    avgs.append(avg_tf)
  
  return avgs

# Pass bagfile name as first argument
if __name__ == '__main__':
  tf_groups = get_tf_groups(sys.argv[1])
  avgs = avg_transforms(tf_groups)
  
  """
  for tfg, avg in zip(tf_groups,avgs):
    print '%d transforms from \'%s\' to \'%s\'' % (
      len(tfg), tfg[0].header.frame_id, tfg[0].child_frame_id
    )
    print 'Average Transform:'
    print avg.transform
  """
  
  if len(sys.argv) > 2:
    with rosbag.Bag(sys.argv[2],'w') as outbag:
      tf_msg = TFMessage()
      tf_msg.transforms = avgs
      outbag.write("/tf", tf_msg, rospy.Time.from_sec(time.time()))
      outbag.close()
