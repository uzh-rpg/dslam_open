# Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich,
#   Switzerland
#   You can contact the author at <titus at ifi dot uzh dot ch>
# Copyright (C) 2017-2018 Siddharth Choudhary, College of Computing,
#   Georgia Institute of Technology, Atlanta, GA, USA
# Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich,
#   Switzerland
#
# This file is part of dslam_open.
#
# dslam_open is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# dslam_open is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

import csv
from cv_bridge import CvBridge, CvBridgeError
import cv2
import gflags
import numpy as np
import os
import re
import rosbag
from sets import Set
from sklearn.neighbors import NearestNeighbors
import sys
import tf

#%%

gflags.DEFINE_string("bag_name", None, "Bag filename.")
gflags.DEFINE_string("rect_bag_name", None, "Bag filename with rect ims if separate.")
gflags.DEFINE_string("outdir", None, "Output directory.")
gflags.DEFINE_string("tf_file", None, "Most likely the _part1_floor2.gt.laser.poses.txt file")
FLAGS = gflags.FLAGS

#%%
class FilenameGen(object):
    def __init__(self, outdir):
        self.outdir = outdir
        if not os.path.isdir(os.path.join(self.outdir, "images", "left")):
            os.makedirs(os.path.join(self.outdir, "images", "left"))
        if not os.path.isdir(os.path.join(self.outdir, "images", "right")):
            os.makedirs(os.path.join(self.outdir, "images", "right"))

    def left_image_filename(self, i):
        return os.path.join(self.outdir, "images", "left",
            "{0:06d}.jpg".format(i))

    def right_image_filename(self, i):
        return os.path.join(self.outdir, "images", "right",
            "{0:06d}.jpg".format(i))

    def times_filename(self):
      return os.path.join(self.outdir, "times.txt")

    def calib_filename(self):
      return os.path.join(self.outdir, "calib.txt")


#%%
def transQuatToMat(t, q):
  R = tf.transformations.quaternion_matrix(np.ravel(q))[:3, :3]
  T = np.concatenate((R, t), axis=1)
  T = np.matrix(np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0))
  return T
#%%
def tfMsgToMat(msg):
  q = msg.rotation
  q = np.array([[q.x, q.y, q.z, q.w]]).T
  t = msg.translation
  t = np.array([[t.x, t.y, t.z]]).T
  return transQuatToMat(t,q)
#%%
class TfTracker(object):
  def __init__(self):
    self.relpose = {}

  def register(self, tfs_msg):
    for tform in tfs_msg.transforms:
      self.relpose[tform.child_frame_id] = \
          {'parent': tform.header.frame_id, \
          'T_P_C': tfMsgToMat(tform.transform)}

  def query(self, from_name, to_name):
    print from_name + ' ' + to_name
    try:
      tform = self.relpose[to_name]
      if tform['parent'] == from_name:
        return tform['T_P_C']
      T_from_P = self.query(from_name, tform['parent'])
      if T_from_P is None:
        return None;
      return T_from_P * tform['T_P_C']
    except:
      return None

#%%
def matchedWithin(database, query, dist):
  d2 = np.array(database, ndmin=2)
  if d2.shape[0] < d2.shape[1]:
    d2 = d2.T
  q2 = np.array(query, ndmin=2)
  if q2.shape[0] < q2.shape[1]:
    q2 = q2.T
  nn = NearestNeighbors(n_jobs=1, metric='euclidean').fit(d2)
  distances, indices = nn.kneighbors(q2, n_neighbors=1)
  filt = distances < dist
  return database[indices[filt]], q2[filt]

#%%

def main(argv):
  argv = FLAGS(argv)

  bag_name = FLAGS.bag_name
  rect_bag_name = FLAGS.rect_bag_name
  outdir = FLAGS.outdir
  tf_file = FLAGS.tf_file

  #%%
  assert bag_name is not None
  bag = rosbag.Bag(bag_name, 'r')
  if rect_bag_name is not None:
    rect_bag = rosbag.Bag(rect_bag_name, 'r')
    image_bag = rect_bag
  else:
    image_bag = bag
  bridge = CvBridge()

  #%%
  left_topic = '/wide_stereo/left/image_rect'
  right_topic = '/wide_stereo/right/image_rect'
  assert outdir is not None
  fname_gen = FilenameGen(outdir)

  #%% Unbag images:
  #%% 1) Select common times:
  print 'Getting left image times...'
  left_times = np.array([msg.header.stamp.to_nsec() for _,msg,_ in
      image_bag.read_messages(topics=left_topic)])
  print 'Getting right image times...'
  right_times = np.array([msg.header.stamp.to_nsec() for _,msg,_ in
      image_bag.read_messages(topics=right_topic)])
  #%%
  l_times_matched, r_times_matched = matchedWithin(
      left_times, right_times, 5e6)
  l_times_matched, unique_idc = np.unique(l_times_matched, return_index=True)
  r_times_matched = r_times_matched[unique_idc]
  times = np.array(l_times_matched, ndmin=2).T
  np.savetxt(fname_gen.times_filename(), times, fmt="%d")

  #%% 2) Extract images for common times.
  i = 0;
  print 'Extracting left images...'
  for _, msg, _ in image_bag.read_messages(topics=left_topic):
    if msg.header.stamp.to_nsec() in l_times_matched:
      image_type = msg.encoding
      cv_image = bridge.imgmsg_to_cv2(msg, image_type)
      assert cv2.imwrite(fname_gen.left_image_filename(i), cv_image)
      i = i + 1
  i = 0
  print 'Extracting right images...'
  for _, msg, t in image_bag.read_messages(topics=right_topic):
    if msg.header.stamp.to_nsec() in r_times_matched:
      image_type = msg.encoding
      cv_image = bridge.imgmsg_to_cv2(msg, image_type)
      assert cv2.imwrite(fname_gen.right_image_filename(i), cv_image);
      i = i + 1

  #%% Calibration: Get projection matrices
  left_info_topic = '/wide_stereo/left/camera_info'
  left_infos = bag.read_messages(topics=left_info_topic)
  left_info = left_infos.next()
  left_K = np.array(left_info.message.K, ndmin=2).reshape([3, 3])
  np.savetxt(os.path.join(outdir, 'left_K.txt'), left_K, fmt="%f")

  right_info_topic = '/wide_stereo/right/camera_info'
  right_infos = bag.read_messages(topics=right_info_topic)
  right_info = right_infos.next()
  right_K = np.array(right_info.message.K, ndmin=2).reshape([3, 3])
  np.savetxt(os.path.join(outdir, 'right_K.txt'), right_K, fmt="%f")

  #%% Calibration: Get baseline.
  msg_with_frames = None
  for _, msg, _ in bag.read_messages(topics="/tf"):
    have_l = False
    have_r = False
    for transform in msg.transforms:
      if transform.child_frame_id == '/wide_stereo_gazebo_l_stereo_camera_optical_frame':
        have_l = True
      if transform.child_frame_id == '/wide_stereo_gazebo_r_stereo_camera_optical_frame':
        have_r = True
    if have_l and have_r:
      msg_with_frames = msg
      break
  assert msg_with_frames is not None
  #%%
  for transform in msg_with_frames.transforms:
    if transform.child_frame_id == '/wide_stereo_gazebo_l_stereo_camera_optical_frame':
      pos = transform.transform.translation
      p_left = np.array([pos.x, pos.y, pos.z])
    if transform.child_frame_id == '/wide_stereo_gazebo_r_stereo_camera_optical_frame':
      pos = transform.transform.translation
      p_right = np.array([pos.x, pos.y, pos.z])
  baseline = np.linalg.norm(p_left-p_right)
  np.savetxt(os.path.join(outdir, 'baseline.txt'), np.array([baseline]), fmt="%f")

  #%% Ground truth: Positions only.
  poses = []
  pose_times = []
  with open(tf_file) as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    for row in reader:
      pose_times.append(int(row[0]) * 1000)
      pos = np.array([float(row[1]), float(row[2]), float(row[3])], \
        ndmin=2).T
      pose = transQuatToMat(pos, [0,0,0,1])
      poses.append(np.array(pose[:3, :]).flatten())
  poses = np.array(poses)

  #%% Select poses at correct times
  pose_times_arr = np.array(pose_times, ndmin=2).T
  nn = NearestNeighbors(n_jobs=1, metric='euclidean').fit(pose_times_arr)
  matching_poses = nn.kneighbors(times, n_neighbors=1, return_distance=False)
  matching_poses = matching_poses.ravel()
  matched_poses = poses[matching_poses]
  #%%
  np.savetxt(os.path.join(outdir, 'gt_poses.txt'), matched_poses, fmt="%f")
#%%
if __name__ == "__main__":
  main(sys.argv)
