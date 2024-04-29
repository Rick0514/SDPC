import sys, os, time, yaml

import numpy as np
import matplotlib.pyplot as plt

import rosbag, rospkg
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped

# need python3
# from pytransform3d import rotations as pr
# import pytransform3d.transformations as pytr
import tf.transformations as tf_tfs

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('sdpc')

print(pkg_dir)

conf = None
with open(os.path.join(pkg_dir, 'config/mld.yaml'), 'r') as f:
    conf = yaml.safe_load(f)

rospy.init_node('vis_bag')

bag = rosbag.Bag('{}{}'.format(pkg_dir, conf["out_bag"]))

hz = conf['lidar_info']['hz']
dur = 1.0 / hz
lidar_topics = conf["lidar_topics"]
odom_topic = conf["odom_topic"]

rostime = []
odom = []
# first read all odom msgs from bag, store in odom list
for topic, msg, t in bag.read_messages(topics=[odom_topic]):
    q = msg.pose.pose.orientation
    p = msg.pose.pose.position
    T = tf_tfs.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = np.array([p.x, p.y, p.z])
    odom.append(T)
    rostime.append(t)

print(len(odom))

exts = conf["exts"]
Texts = []
# exts is a 7*n array, each 7 means qx, qy, qz, qw, x, y, z
# make Texts to an list like odom storing Transformation
for i in range(0, len(exts), 7):
    q = exts[i:i+4]
    p = exts[i+4:i+7]
    T = tf_tfs.quaternion_matrix(q)
    T[:3, 3] = p
    Texts.append(T)

cur_time = -1
odom_idx = 0
# publish tf transformation of map to base_link and base_link to pc frame
static_tf = tf2_ros.StaticTransformBroadcaster()
dyn_tf = tf2_ros.TransformBroadcaster()
tfs = TransformStamped()
tfs.header.frame_id = "map"
tfs.child_frame_id = "base_link"
tfs.header.stamp = rostime[0]
q = tf_tfs.quaternion_from_matrix(odom[0])
p = odom[0][:3, 3]
tfs.transform.rotation.x = q[0]
tfs.transform.rotation.y = q[1]
tfs.transform.rotation.z = q[2]
tfs.transform.rotation.w = q[3]
tfs.transform.translation.x = p[0]
tfs.transform.translation.y = p[1]
tfs.transform.translation.z = p[2]
dyn_tf.sendTransform(tfs)

# publish static trans
ext_tf = []
for i in range(len(lidar_topics)):
    tfs = TransformStamped()
    tfs.header.stamp = rostime[0]
    tfs.header.frame_id = "base_link"
    tfs.child_frame_id = lidar_topics[i]
    q = tf_tfs.quaternion_from_matrix(Texts[i])
    p = Texts[i][:3, 3]
    tfs.transform.rotation.x = q[0]
    tfs.transform.rotation.y = q[1]
    tfs.transform.rotation.z = q[2]
    tfs.transform.rotation.w = q[3]
    tfs.transform.translation.x = p[0]
    tfs.transform.translation.y = p[1]
    tfs.transform.translation.z = p[2]
    ext_tf.append(tfs)

static_tf.sendTransform(ext_tf)
time.sleep(0.1)

for topic, msg, t in bag.read_messages(lidar_topics):
    if cur_time < 0:
        cur_time = t.to_sec()
    
    if t.to_sec() > dur / 2 + cur_time:
        cur_time = t.to_sec()
        odom_idx += 1

        tfs.header.frame_id = "map"
        tfs.child_frame_id = "base_link"
        tfs.header.stamp = rostime[odom_idx]
        q = tf_tfs.quaternion_from_matrix(odom[odom_idx])
        p = odom[odom_idx][:3, 3]
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        tfs.transform.translation.x = p[0]
        tfs.transform.translation.y = p[1]
        tfs.transform.translation.z = p[2]
        dyn_tf.sendTransform(tfs)

    # if topic.startswith('av'):
    rospy.Publisher(topic, PointCloud2, queue_size=5, latch=True).publish(msg)

    time.sleep(0.2)

    if rospy.is_shutdown():
        break

bag.close()
