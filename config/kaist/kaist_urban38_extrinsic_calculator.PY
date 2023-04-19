import numpy as np
from scipy.spatial.transform import Rotation as R

def se3_to_transformation_matrix(rotation, translation):
    """
    将旋转矩阵和平移向量转换为变换矩阵
    """
    transformation = np.eye(4)
    transformation[:3, :3] = rotation
    transformation[:3, 3] = translation
    return transformation

def transformation_matrix_to_se3(transformation):
    """
    从变换矩阵中提取旋转矩阵和平移向量
    """
    rotation = transformation[:3, :3]
    translation = transformation[:3, 3]
    return rotation, translation

# 定义两个 SE3 矩阵
R_vc = np.array([[-0.00680499, -0.0153215, 0.99985],
		[-0.999977, 0.000334627, -0.00680066],
		[-0.000230383, -0.999883, -0.0153234 ]
		])

t_vc = np.array([1.64239, 0.247401, 1.58411])


T_vc = se3_to_transformation_matrix(R_vc, t_vc)

R_vb= np.array([[1, 0, 0],
		[0, -1, 0],
		[0, 0, -1 ]
		])
t_vb = np.array([-0.07, 0, 1.7])
T_vb = se3_to_transformation_matrix(R_vb, t_vb)

# 计算 SE3 矩阵相乘
T_bc = np.dot(np.linalg.inv(T_vb), T_vc)

# 从 SE3 矩阵中提取四元数和平移向量
R3, t3 = transformation_matrix_to_se3(T_bc)
q3 = R.from_matrix(R3).as_quat()

#q3 = np.array([-0.50204296 , -0.50558566  ,0.49778311 , -0.49451793])
# 将四元数转换为旋转矩阵
R_mat = R.from_quat(q3).as_matrix()
# 将旋转矩阵转换为旋转轴和旋转角度
angle, axis,_ = R.from_matrix(R_mat).as_rotvec()

print("SE3 T_vc: \n", T_vc)
print("SE3 T_vb: \n", T_vb)
print("SE3 T_bc: \n", T_bc)
print("Quaternions of T_bc(x,y,z,w): \n",  "{:.16f},{:.16f},{:.16f},{:.16f}".format(q3[1],q3[2],q3[3],q3[0]))
print("Translation of T_bc: \n", "{:.8f},{:.8f},{:.8f}".format(t3[0],t3[1],t3[2]))

