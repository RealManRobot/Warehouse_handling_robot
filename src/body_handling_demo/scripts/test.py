import numpy as np
from scipy.spatial.transform import Rotation as R

# 已知坐标系A在坐标系O下的旋转四元数和平移向量
q = np.array([0, 0, 0, 1])  # 示例四元数
t = np.array([0, 0, 0])  # 示例平移向量

# 坐标系A自身旋转的欧拉角 (r, p, y)
# r, p, y = np.radians([0, 90, 0])  # 示例欧拉角，转换为弧度
r, p, y = [-2.624, 0, 0]

# 将欧拉角转换为四元数
q_new = R.from_euler('xyz', [r, p, y]).as_quat()

# 计算新的旋转四元数
q_prime = R.from_quat(q) * R.from_quat(q_new)
q_prime = q_prime.as_quat()

# 计算新的平移向量 (假设坐标系A自身旋转后没有平移)
t_prime = t

rqy = R.from_quat(q_prime).as_euler("xyz")

print("新的旋转四元数 q':", q_prime)
print("新的平移向量 t':", t_prime)
print("新的欧拉角r':", rqy)