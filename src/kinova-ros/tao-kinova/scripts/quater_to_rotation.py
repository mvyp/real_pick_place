#! /usr/bin/env python3

from scipy.spatial.transform import Rotation as R
import numpy as np
print('test')
# use [:, np.newaxis] to transform from row vector to col vector
position = np.array([0.6453529828252734, -0.26022684372145516, 1.179122068068349])[:, np.newaxis]
share_vector = np.array([0,0,0,1], dtype=float)[np.newaxis, :]
print('share_vector:\n', share_vector)
print('position:\n',position)
r = R.from_quat([0.70710678, 0,       0,        0.70710678])
r.as_matrix()
print('rotation:\n',r.as_matrix())
rotation_matrix = r.as_matrix()
print(rotation_matrix)
 
#combine three matrix or vector together
m34 = np.concatenate((rotation_matrix, position), axis = 1)
print(m34)
m44 = np.concatenate((m34, share_vector), axis=0)
# m44 = np.hstack((m34, share_vector))
 
print(m44)
 
rot_vec = r.as_rotvec()
print('rot_vec:\n', rot_vec)
rot_euler = r.as_euler('zyx', degrees = False)
print('rot_euler:\n',rot_euler)
 
r = R.from_matrix(rotation_matrix)
print('as_quat():\n',r.as_quat())
print('as_rotvec():\n', r.as_rotvec())
print('as_euler():\n', r.as_euler('zyx', degrees=True))