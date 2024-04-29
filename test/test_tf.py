# need python3
from pytransform3d import rotations as pr
import pytransform3d.transformations as pytr

rot = pr.matrix_from_euler([0, 0, 0], 0, 1, 2, True)

Twb = pytr.transform_from(rot, [3.9855, 0, 0.5])

rot = pr.matrix_from_euler([0, 0.523599, 0], 0, 1, 2, True)
Tbl = pytr.transform_from(rot, [0, 0, 0.1])

print(Twb @ Tbl)