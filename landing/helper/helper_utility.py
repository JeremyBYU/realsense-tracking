import numpy as np
from scipy.spatial.transform import Rotation as R

from Common_pb2 import Vec3, Vec4
from TouchdownMessage_pb2 import TouchdownMessage, BODY, NED, SINGLE, INTEGRATED
from shapely.geometry import Polygon

from landing.helper.helper_logging import get_logger

def create_projection_matrix(fx, fy, ppx, ppy):
    proj_mat = np.array([[fx, 0, ppx, 0], [0, fy, ppy, 0], [0, 0, 1, 0]])
    return proj_mat

def apply_transform(points, pre_matrix, post_matrix=None):
    points = np.array(points)
    points_ = np.ones(shape=(4, points.shape[0]))
    points_[:3, :] = points.transpose()
    return (pre_matrix @ points_).transpose()


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

def create_proto_vec(vec):
    proto_vec = Vec3() if len(vec) == 3 else Vec4()
    proto_vec.x = vec[0]
    proto_vec.y = vec[1]
    proto_vec.z = vec[2]
    if len(vec) == 4:
        proto_vec.w = vec[3]
    return proto_vec


def get_mesh_data_from_message(mesh):
    n_triangles = mesh.n_triangles
    n_vertices = mesh.n_vertices
    triangles = np.copy(np.frombuffer(
        mesh.triangles, dtype=np.int32).reshape((n_triangles, 3)))
    vertices = np.copy(np.frombuffer(
        mesh.vertices, dtype=np.float64).reshape((n_vertices, 3)))
    if mesh.vertices_colors:
        vertices_colors = np.copy(np.frombuffer(
            mesh.vertices_colors, dtype=np.float64).reshape((n_vertices, 3)))
    else:
        vertices_colors = None
    halfedges = np.copy(np.frombuffer(mesh.halfedges, dtype=np.uint64))
    return vertices, triangles, halfedges, vertices_colors

def set_matrix(matrix_message, ndarray):
    matrix_message.data = np.ndarray.tobytes(ndarray)
    matrix_message.rows = ndarray.shape[0]
    matrix_message.cols = ndarray.shape[1]



def convert_double_matrix_bytes_to_numpy(dmb):
    return np.frombuffer(dmb.data, dtype=np.float64).reshape((dmb.rows, dmb.cols))

def convert_polygon_message_to_shapely(pm):
    shell = convert_double_matrix_bytes_to_numpy(pm.shell)
    holes = []
    for hole in pm.holes:
        hole_np = convert_double_matrix_bytes_to_numpy(hole)
        holes.append(hole_np)

    return Polygon(shell, holes)


def create_touchdown_message(touchdown_result, command_frame='body', integrated=False):
        # touchdown_results = dict(polygon=chosen_plane, alg_timings=alg_timings,
        #                          avg_peaks=avg_peaks, touchdown_point=touchdown_point)
    tm = TouchdownMessage()
    # short circuit
    if touchdown_result['polygon'] is None:
        return tm

    poly, meta = touchdown_result['polygon']
    tp = touchdown_result['touchdown_point']['point']
    dist = touchdown_result['touchdown_point']['dist']
    
    # Set Polygons
    set_matrix(tm.landing_site.shell, np.array(poly.exterior))
    for hole in poly.interiors:
        hole_message = tm.landing_site.holes.add()
        set_matrix(hole_message, np.array(hole))

    # Set Touchdown Point
    tm.touchdown_point.CopyFrom(create_proto_vec(tp))
    tm.touchdown_dist = dist

    # Set Type and Frame
    tm.type = INTEGRATED if integrated else SINGLE
    tm.frame = BODY if command_frame == 'body' else NED

    return tm


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