import numpy as np
import quaternion
import math as m


def unix_second_to_gps_time(seconds):
    second_gps = seconds + 18 - 315964800
    week = m.floor(second_gps / 604800)
    sow = second_gps - week * 604800
    return week, sow


pose_file = 'urban38/urban38-pankyo' + '/global_pose.csv'
traj_file = '%s_traj.csv' % pose_file.split('.')[0]

pose_se3 = np.loadtxt(pose_file, delimiter=',')

traj = np.zeros((len(pose_se3), 8))

R_n_nn = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_bb_b = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

se3_0 = pose_se3[0, 1:13].reshape(3, 4)
pos_0 = se3_0[:, 3]

for k in range(len(pose_se3)):
    week, sow = unix_second_to_gps_time(pose_se3[k, 0] * 1.0e-9)
    traj[k, 0] = sow

    se3 = pose_se3[k, 1:13].reshape(3, 4)

    # Position
    pos = se3[:, 3] - pos_0
    traj[k, 1:4] = np.array([pos[1], pos[0], -pos[2]])

    # Rotation
    R_nn_bb = se3[:, 0:3]
    R_nn_b = np.matmul(R_nn_bb, R_bb_b)
    R_n_b = np.matmul(R_n_nn, R_nn_b)
    q_n_b = quaternion.from_rotation_matrix(R_n_b)
    traj[k, 4] = q_n_b.x
    traj[k, 5] = q_n_b.y
    traj[k, 6] = q_n_b.z
    traj[k, 7] = q_n_b.w

    if k % 10000 == 0:
        print(traj[k, 0])

np.savetxt(traj_file, traj, fmt='%0.6lf %0.6lf %0.6lf %0.6lf %0.6lf %0.6lf %0.6lf %0.6lf')
