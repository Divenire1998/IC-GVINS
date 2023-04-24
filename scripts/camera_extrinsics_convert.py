# -*- coding: utf-8 -*-


import numpy as np
import quaternion

# c系到v系
R_v_c = np.array(
    [-0.00680499, -0.0153215, 0.99985, -0.999977, 0.000334627, -0.00680066, -0.000230383, -0.999883,
     -0.0153234]).reshape(3, 3)
t_v_c = np.array([1.64239, 0.247401, 1.58411])

# 前左上b系到v系
R_v_bb = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]).reshape(3, 3)
t_v_bb = np.array([-0.07, 0, 1.7])

# v系到前左上b系
R_bb_v = R_v_bb.transpose()
t_bb_v = -np.matmul(R_bb_v, t_v_bb)

# c系到前左上b系
R_bb_c = np.matmul(R_bb_v, R_v_c)
t_bb_c = np.matmul(R_bb_v, t_v_c) + t_bb_v
q_bb_c = quaternion.from_rotation_matrix(R_bb_c)

print('R_bb_c', R_bb_c, '\n')
print('q_bb_c: [', q_bb_c.x, ',', q_bb_c.y, ',', q_bb_c.z, ',', q_bb_c.w, ']')
print('t_bb_c:', t_bb_c, '\n')

# 前左上bb系到前右下b系转换
R_b_bb = np.array([1, 0, 0, 0, -1, 0, 0, 0, -1]).reshape(3, 3)
t_b_bb = np.array([0, 0, 0])

# 转化为相机c系到前右下b系的外参
R_b_c = np.matmul(R_b_bb, R_bb_c)
t_b_c = np.matmul(R_b_bb, t_bb_c) + t_b_bb
q_b_c = quaternion.from_rotation_matrix(R_b_c)

print('R_b_c', R_b_c, '\n')
print('q_b_c: [', q_b_c.x, ',', q_b_c.y, ',', q_b_c.z, ',', q_b_c.w, ']')
print('t_b_c:', t_b_c)

# 转化为前右下b系到相机c系的外参
R_c_b = R_b_c.transpose()
t_c_b = -np.matmul(R_c_b, t_b_c)

print('R_c_b', R_c_b, '\n')
print('t_c_b:', t_c_b)
