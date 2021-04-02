import open3d as o3d
import numpy as np
from .LineMesh import LineMesh
# from .LineMesh import LineMesh
# from airsimcollect.helper.helper_transforms import seg2rgb
# from airsimcollect.helper.helper_logging import logger

ORANGE = (255/255, 188/255, 0)
GREEN = (0, 255/255, 0)
BLUE = (0, 0, 255/255)
PURPLE = [199/255, 36/255, 177/255]


def remove_nans(a):
    return a[~np.isnan(a).any(axis=1)]

def create_o3d_pc(pc):
    if pc.ndim == 3:
        pc_ = pc.reshape((pc.shape[0] * pc.shape[1], 3))
    else:
        pc_ = pc
    pcd = o3d.geometry.PointCloud()
    pc_no_nans = remove_nans(pc_)
    pcd.points = o3d.utility.Vector3dVector(pc_no_nans)
    return pcd


def clear_polys(all_polys, vis):
    for line_mesh in all_polys:
        line_mesh.remove_line(vis, False)
    return []

def add_polys(all_polys, vis):
    for line_mesh in all_polys:
        line_mesh.add_line(vis, False)
    return []

def get_segments(all_polys):
    all_segments = []
    for line_mesh in all_polys:
        all_segments.extend(line_mesh.cylinder_segments)
    return all_segments

def get_extrinsics(vis):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    return camera_params.extrinsic


def set_view(vis, extrinsics):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    camera_params.extrinsic = extrinsics
    ctr.convert_from_pinhole_camera_parameters(camera_params)


def update_point_cloud(pcd, points):
    if points.ndim > 2:
        points = points.reshape((points.shape[0] * points.shape[1], 3))
    points_filt = points[~np.isnan(points).any(axis=1)]
    pcd.points = o3d.utility.Vector3dVector(points_filt)


def translate_meshes(meshes, shift_x=True):
    x_amt_ = 0
    y_amt_ = 0
    for i, mesh in enumerate(meshes):
        x_amt, y_amt, z_amt = mesh.get_axis_aligned_bounding_box().get_extent()
        if not shift_x:
            x_amt = 0.0
        else:
            y_amt = 0.0
        mesh.translate([x_amt_, y_amt_, 0])
        x_amt_ += x_amt
        y_amt_ += y_amt

# def handle_shapes(vis, planes, all_polys, line_radius=0.15, visible=True):
#     # print(all_polys, visible)
#     all_polys = clear_polys(all_polys, vis)
#     for plane, _ in planes:
#         lm = create_linemesh_from_shapely(plane)
#         all_polys.extend(lm)
#         if visible:
#             add_polys(lm, vis)
#     return all_polys

# def handle_linemeshes(vis, old_line_meshes, new_line_meshes):
#     all_polys = clear_polys(old_line_meshes, vis)
#     for line_mesh in new_line_meshes:
#         line_mesh.add_line(vis)
#     return new_line_meshes


# def update_linemesh(polygons, line_meshes, **kwargs):
#     line_meshes.remove_from_vis()
#     all_polys = []
#     for poly_ in polygons:
#         poly = poly_[0] if type(poly_) is tuple else poly_
#         lm = create_linemesh_from_shapely(poly, **kwargs)
#         all_polys.extend(lm)
#     line_meshes.line_meshes = all_polys
#     if line_meshes.visible:
#         line_meshes.add_to_vis()


def create_linemesh_from_linear_ring(linear_ring, height=0, line_radius=0.02, rotate_func=None, color=GREEN):
    points = np.array(linear_ring)
    if points.shape[1] == 2:
        height_np = np.ones((points.shape[0], 1)) * height
        points = np.concatenate((points, height_np), axis=1)
    if rotate_func:
        points = rotate_func(points)
    return LineMesh(points, colors=color, radius=line_radius)


def create_linemesh_from_shapely(polygon, height=0, line_radius=0.02, rotate_func=None, color=GREEN, **kwargs):
    all_line_meshes = [create_linemesh_from_linear_ring(
        polygon.exterior, height, line_radius, rotate_func, color=color)]

    for hole in polygon.interiors:
        all_line_meshes.append(create_linemesh_from_linear_ring(
            hole, height, line_radius, rotate_func, color=ORANGE))

    return all_line_meshes


# def update_frustum(vis, dist_to_plane=5.0, start_pos=np.array([0.0, 0.0, 0.0]), hfov=90, vfov=90,
#                    frustum:VisibleLineMeshes=None, radius=0.15, color=[1, 0, 0]):
    
#     frustum.remove_from_vis()

#     points = create_frustum(dist_to_plane, start_pos, hfov, vfov)
#     lines = [[0, 1], [0, 2], [0, 3], [0, 4],
#              [1, 2], [2, 3], [3, 4], [4, 1]]

#     frustum.line_meshes = [LineMesh(points, lines, colors=color, radius=radius)]
#     if frustum.visible:
#         frustum.add_to_vis()


# def create_frustum(dist_to_plane=5.0, start_pos=np.array([0.0, 0.0, 0.0]), hfov=90, vfov=90):
#     x = np.tan(np.radians(hfov/2.0)) * dist_to_plane
#     y = np.tan(np.radians(vfov/2.0)) * dist_to_plane

#     point0 = start_pos
#     point1 = start_pos - [x, y, -dist_to_plane]
#     point2 = start_pos - [-x, y, -dist_to_plane]
#     point3 = start_pos - [-x, -y, -dist_to_plane]
#     point4 = start_pos - [x, -y, -dist_to_plane]
#     points = np.stack([point0, point1, point2, point3, point4])

#     return points


def save_view_point(vis, filename=r"C:\Users\Jeremy\Documents\UMICH\Research\UnrealRooftopLanding\assets\o3d\o3d_view_default.json"):
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, param)


def load_view_point(vis, filename=r"C:\Users\Jeremy\Documents\UMICH\Research\UnrealRooftopLanding\assets\o3d\o3d_view_default.json"):
    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters(filename)
    ctr.convert_from_pinhole_camera_parameters(param)


# def toggle_visibility(geometry_set, geometry_key, vis):
#     """Toggles visibility of geometries by adding and removing to visualizer. Open3D has no opacity or visibility mechanism..."""
#     geometry = geometry_set[geometry_key]
#     geometry.toggle_visibility()


# def init_vis(width=700, height=700):

#     vis = o3d.visualization.VisualizerWithKeyCallback()
#     vis.create_window("3D Viewer", width, height)

#     # Create axis frame
#     axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
#     # Create mesh
#     mesh = o3d.geometry.TriangleMesh()
#     # create point cloud
#     pcd = VisibleGeometry(o3d.geometry.PointCloud(), vis, visible=True, name='pcd')
#     # create map
#     map_polys = VisibleLineMeshes([], vis, visible=True, name='map_polys')
#     # create polylidar polygons
#     pl_polys = VisibleLineMeshes([], vis, visible=False, name='pl_polys')
#     # Create a mesh
#     # Create a LineMesh for the Frustum
#     frustum = VisibleLineMeshes([], vis, visible=True, name='frustum')
#     # Create a Line Mesh for the interstion of the Frustum and predicted polygons
#     pl_isec = VisibleLineMeshes([], vis, visible=True, name='pl_isec')
#     gt_isec = VisibleLineMeshes([], vis, visible=False, name='gt_isec')

#     circle = VisibleLineMeshes([], vis, visible=True, name='circle_polys')

#     # get_image_data(client)
#     drone = o3d.io.read_triangle_mesh('assets/models/drone2.ply')
#     rot = o3d.geometry.get_rotation_matrix_from_xyz([np.pi/2, 0, 0])
#     drone = drone.rotate(rot, drone.get_center())
#     drone.compute_triangle_normals()
#     drone.compute_vertex_normals()
#     drone_ = VisibleGeometry(drone, vis, visible=True, name='drone')

#     # add geometries
#     vis.add_geometry(axis_frame)
#     # vis.add_geometry(mesh)

#     geometry_set = dict(axis_frame=axis_frame, mesh=mesh, pcd=pcd, map_polys=map_polys, drone=drone_,
#                         pl_polys=pl_polys, frustum=frustum, pl_isec=pl_isec, gt_isec=gt_isec, circle_polys=circle)

#     # geometry_set = dict(pcd=pcd, map_polys=map_polys, pl_polys=[
#     # ], axis_frame=axis_frame, mesh=mesh, frustum=frustum, isec_poly=isec_poly, vis_pcd=True,
#     #     vis_map=True, vis_pl=True, vis_mesh=False, vis_frustum=True, vis_isec=False)

#     return vis, geometry_set


# class DummyVis(object):
#     def __init__(self):
#         pass
    
#     def register_key_callback(self, *args, **kwargs):
#         pass

#     def add_geometry(self, *args, **kwargs):
#         pass

#     def remove_geometry(self, *args, **kwargs):
#         pass



    
