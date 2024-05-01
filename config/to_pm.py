import yaml
import numpy as np
from pytransform3d import rotations as pr
import pytransform3d.transformations as pytr

conf = None
with open('mld.yaml', 'r') as f:
    conf = yaml.safe_load(f)

exts = conf["exts"]

ln = int(len(exts) / 7)

Texts = []
for i in range(0, len(exts), 7):
    q = np.zeros(4)
    q[0] = exts[i+3]
    q[1:] = exts[i:i+3]
    rot = pr.matrix_from_quaternion(q)
    pos = exts[i+4:i+7]
    Texts.append(pytr.transform_from(rot, pos))

Twa = Texts[0]
for i in range(1, ln):
    Twb = Texts[i]
    Tab = pytr.invert_transform(Twa) @ Twb

    # turn Tab to 7-dim array: xyzwxyz
    q = pr.quaternion_from_matrix(Tab[:3, :3])
    p = Tab[:3, 3]

    # print 7 dim xyzwxyz using .4f a line, each separated
    print(','.join(['{:.4f}'.format(e) for e in np.concatenate([q[1:], q[:1], p])]) + ',')
    