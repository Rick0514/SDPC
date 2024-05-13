import sys, os, time, yaml

import numpy as np
import matplotlib.pyplot as plt

import rosbag, rospkg
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

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
gt_odom_topic = conf["gt_odom_topic"]

rostime = []
odom = []
gt_odom = []
odom_T = []
gt_odom_T = []
# first read all odom msgs from bag, store in odom list
for topic, msg, t in bag.read_messages(topics=[odom_topic, gt_odom_topic]):
    q = msg.pose.pose.orientation
    p = msg.pose.pose.position
    T = tf_tfs.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = np.array([p.x, p.y, p.z])
    if topic == odom_topic:
        odom.append(msg)
        odom_T.append(T)
        rostime.append(t)
    else:
        gt_odom.append(msg)
        gt_odom_T.append(T)

print(len(gt_odom))
print(odom[0].child_frame_id)
print(gt_odom[0].child_frame_id)

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

# pub the first tf
tf_list = []
tfs = TransformStamped()
tfs.header.frame_id = "map"
tfs.child_frame_id = odom[0].child_frame_id
tfs.header.stamp = rostime[0]
q = tf_tfs.quaternion_from_matrix(odom_T[0])
p = odom_T[0][:3, 3]
tfs.transform.rotation.x = q[0]
tfs.transform.rotation.y = q[1]
tfs.transform.rotation.z = q[2]
tfs.transform.rotation.w = q[3]
tfs.transform.translation.x = p[0]
tfs.transform.translation.y = p[1]
tfs.transform.translation.z = p[2]
tf_list.append(tfs)

tfs = TransformStamped()
tfs.header.frame_id = "map"
tfs.child_frame_id = gt_odom[0].child_frame_id
tfs.header.stamp = rostime[0]
q = tf_tfs.quaternion_from_matrix(gt_odom_T[0])
p = gt_odom_T[0][:3, 3]
tfs.transform.rotation.x = q[0]
tfs.transform.rotation.y = q[1]
tfs.transform.rotation.z = q[2]
tfs.transform.rotation.w = q[3]
tfs.transform.translation.x = p[0]
tfs.transform.translation.y = p[1]
tfs.transform.translation.z = p[2]
tf_list.append(tfs)
dyn_tf.sendTransform(tf_list)

# publish static trans
ext_tf = []
for i in range(len(lidar_topics)):
    tfs = TransformStamped()
    tfs.header.stamp = rostime[0]
    tfs.header.frame_id = gt_odom[0].child_frame_id
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

odom_path_pub = rospy.Publisher('odom_path', Path, queue_size=5, latch=True)
gt_path_pub = rospy.Publisher('gt_path', Path, queue_size=5, latch=True)
odom_path = Path()
odom_path.header.frame_id = "map"
odom_path.header.stamp = rostime[0]
gt_path = Path()
gt_path.header.frame_id = "map"
gt_path.header.stamp = rostime[0]

for topic, msg, t in bag.read_messages(lidar_topics):
    if cur_time < 0:
        cur_time = t.to_sec()
    
    if t.to_sec() > dur / 2 + cur_time:
        cur_time = t.to_sec()
        odom_idx += 1

        tf_list = []

        tfs = TransformStamped()
        tfs.header.frame_id = "map"
        tfs.child_frame_id = odom[0].child_frame_id
        tfs.header.stamp = rostime[odom_idx]
        q = tf_tfs.quaternion_from_matrix(odom_T[odom_idx])
        p = odom_T[odom_idx][:3, 3]
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        tfs.transform.translation.x = p[0]
        tfs.transform.translation.y = p[1]
        tfs.transform.translation.z = p[2]
        tf_list.append(tfs)

        pose = PoseStamped()
        pose.header = odom[odom_idx].header
        pose.pose = odom[odom_idx].pose.pose
        odom_path.poses.append(pose)
        odom_path_pub.publish(odom_path)

        tfs = TransformStamped()
        tfs.header.frame_id = "map"
        tfs.child_frame_id = gt_odom[0].child_frame_id
        tfs.header.stamp = rostime[odom_idx]
        q = tf_tfs.quaternion_from_matrix(gt_odom_T[odom_idx])
        p = gt_odom_T[odom_idx][:3, 3]
        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]
        tfs.transform.translation.x = p[0]
        tfs.transform.translation.y = p[1]
        tfs.transform.translation.z = p[2]
        tf_list.append(tfs)

        dyn_tf.sendTransform(tf_list)

        pose = PoseStamped()
        pose.header = gt_odom[odom_idx].header
        pose.pose = gt_odom[odom_idx].pose.pose
        gt_path.poses.append(pose)
        gt_path_pub.publish(gt_path)

        print(odom_idx)

    rospy.Publisher(topic, PointCloud2, queue_size=5, latch=True).publish(msg)

    time.sleep(0.4)

    if rospy.is_shutdown():
        break

bag.close()
