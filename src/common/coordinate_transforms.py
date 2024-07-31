import numpy as np
import tf_transformations

def ned_to_enu(ned):
    # Convert NED to ENU
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    enu = R @ ned
    return enu

def enu_to_ned(enu):
    # Convert ENU to NED
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    ned = R @ enu
    return ned

def quaternion_from_euler(roll, pitch, yaw):
    return tf_transformations.quaternion_from_euler(roll, pitch, yaw)

def quaternion_to_euler(quaternion):
    return tf_transformations.euler_from_quaternion(quaternion)

def enu_to_ned_orientation(quaternion):
    R = np.array([[0, 1, 0], 
                  [1, 0, 0], 
                  [0, 0, -1]])
    rot = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
    rot_ned = R @ rot @ R.T
    return tf_transformations.quaternion_from_matrix(np.vstack((np.hstack((rot_ned, [[0], [0], [0]])), [0, 0, 0, 1])))

def quaternion_get_yaw(quaternion):
    _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
    return yaw

