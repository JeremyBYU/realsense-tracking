import numpy as np
from scipy.spatial.transform import Rotation as R

def create_projection_matrix(fx, fy, ppx, ppy):
    proj_mat = np.array([[fx, 0, ppx, 0], [0, fy, ppy, 0], [0, 0, 1, 0]])
    return proj_mat


def create_transform(translate, rotation):
    transform = np.eye(4)
    transform[:3, 3] = np.array(translate)
    if isinstance(rotation,np.ndarray) or isinstance(rotation, list):
        rot = np.array(rotation)
        if rot.ndim == 2:
            # the provided the rotation matrix
            transform[:3, :3] = np.array(rotation)
        else:
            # they provided quaternions
            rot = R.from_quat(rot).as_matrix()
            transform[:3, :3] = rot
    elif isinstance(rotation, dict):
        # They provide a dictionary of euler angles
        rot = R.from_euler('xyz', angles=[rotation['roll'], rotation['pitch'], rotation['yaw']], degrees=True)
        rm = rot.as_matrix()
        transform[:3, :3] = rm
    else:
        raise ValueError("Rotation must be an ndarray or a dictionary object with euler angles")
    return transform




# Eigen::Matrix4d make_transform(rspub_pb::Vec4 &rotation, rspub_pb::Vec3 &trans)
# {
# 	Eigen::Matrix3d mat3 = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
# 	Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
# 	mat4.block(0, 0, 3, 3) = mat3;
# 	mat4(0, 3) = trans.x();
# 	mat4(1, 3) = trans.y();
# 	mat4(2, 3) = trans.z();
# 	return mat4;
# }