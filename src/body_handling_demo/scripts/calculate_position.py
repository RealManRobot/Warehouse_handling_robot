#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import numpy as np


class Calculate_Position:
    
    def __init__(self):
        pass

    # 四元数相乘，表示先进行q2表示的旋转再进行q1表示的旋转。不符合交换定律
    def quaternion_multiply(self, q1, q2):
        '''
        返回四元数的实部（标量部分）表示旋转后的“方向”或“角度”,但由于四元数的旋转表示不依赖于标量部分,所以通常我们关注的是虚数部分w。
        返回四元数的虚数部分(x,y,z)表示旋转轴在旋转后的新的坐标系中的分量。
        '''
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        # 四元数相乘计算矩阵
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 + y1*w2 + z1*x2 - x1*z2,
            w1*z2 + z1*w2 + x1*y2 - y1*x2
        ])

    # 求出四元数共轭矩阵（表示四元数的逆运算），w实部轴不变，虚部反方向旋转
    def conjugate_quaternion(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    # 将向量v由四元数q表示的旋转变换
    def transform_vector_by_quaternion(self, q, v):
        # 求出四元数q的逆运算
        q_conjugate = self.conjugate_quaternion(q)
        # print(q_conjugate)

        # 四元数乘法可以用来表示和执行三维空间中的旋转。当我们需要将一个向量绕某个轴旋转时，
        # 我们可以将这个向量表示为一个纯四元数，然后使用四元数乘法来应用旋转。
        v_quaternion = np.array([0, v[0], v[1], v[2]])

        # 四元数乘法，相当于在三维空间中将向量 v 绕由四元数 q 定义的轴旋转指定的角度，然后将结果转换回原始坐标系。
        v_transformed = self.quaternion_multiply(self.quaternion_multiply(q, v_quaternion), q_conjugate)
        # print(v_transformed)
        return v_transformed[1:]

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    def quaternion_to_euler(self, w, x, y, z):
        # 俯仰角(Pitch)
        pitch = np.arcsin(2 * (w * y - z * x))
        
        # 翻滚角(Roll)
        roll = np.arctan2(2 * (w * x + z * y), 1 - 2 * (x * x + y * y))
        
        # 偏航角(Yaw)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        
        return np.array([roll, pitch, yaw])


if __name__=='__main__':
    calculator = Calculate_Position()
    # 目标Q的原始旋转四元数和平移向量 w, x, y, z
    q_original = np.array([-0.4050101114235415, -0.6075839806470373, 0.5201112218917696, -0.44304947011371243])

    print("初始旋转欧拉角为：", calculator.quaternion_to_euler(*q_original))

    p_original = np.array([-0.037342273040243304, -0.34348792978243164, -0.0864955364501239])

    # 以目标Q为参考坐标系的平移向量
    d_vector = np.array([0.0855, 0.17, 0.098])

    # 将d_vector从目标坐标系转换到基坐标系
    d_transformed = calculator.transform_vector_by_quaternion(q_original, d_vector)

    # 计算新的平移向量
    p_new = p_original + d_transformed

    # 新的位姿是相同的旋转四元数和新的平移向量
    q_new = q_original
    p_new = p_new

    print("新的旋转四元数:", q_new)
    print("新的平移向量:", p_new)