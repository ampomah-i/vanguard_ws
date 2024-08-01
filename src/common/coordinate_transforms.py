import numpy as np

def ned_to_enu(ned):
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    return R @ ned

def enu_to_ned(enu):
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    return R @ enu

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.array([np.sin(roll/2), 0, 0, np.cos(roll/2)])
    qy = np.array([0, np.sin(pitch/2), 0, np.cos(pitch/2)])
    qz = np.array([0, 0, np.sin(yaw/2), np.cos(yaw/2)])

    q = qx * qy * qz
    return q / np.linalg.norm(q)

def quaternion_to_euler(quaternion):
    w, x, y, z = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

def rotation_matrix_to_quaternion(M):
    m00, m01, m02 = M[0, :]
    m10, m11, m12 = M[1, :]
    m20, m21, m22 = M[2, :]

    qw = np.sqrt(1 + m00 + m11 + m22) / 2
    qx = (m21 - m12) / (4 * qw)
    qy = (m02 - m20) / (4 * qw)
    qz = (m10 - m01) / (4 * qw)

    return np.array([qw, qx, qy, qz])

def enu_to_ned_orientation(quaternion):
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    rot_matrix = quaternion_to_rotation_matrix(quaternion)
    rot_ned = R @ rot_matrix @ R.T
    return rotation_matrix_to_quaternion(rot_ned)

def quaternion_get_yaw(quaternion):
    _, _, yaw = quaternion_to_euler(quaternion)
    return yaw

