# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Advanced/color_map_optimization_for_reconstruction_system.py
import argparse
import os
import sys
import json
import copy
import time
import logging

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.colors as colors 
import numpy as np
import open3d as o3d

from server.open3d_util import create_lines
from server.grounddetector import filter_planes_and_holes2
from server.Utility.trajectory_io import read_trajectory
from server.Utility.file import get_rgbd_file_lists
from server.ReconstructionSystem.initialize_config import initialize_config
from polylidar import extractPolygons, extract_planes_and_polygons_from_mesh

logging.basicConfig(level=logging.DEBUG)
COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))


H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

np.set_printoptions(precision=3, suppress=True)


def draw_registration_result_original_color(source, target, transformation, other_geoms=[]):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target, *other_geoms])


def colored_point_cloud_registration(source, target, voxel_radius=[0.04, 0.02, 0.01], max_iter=[50, 30, 14],
                                     visualize=False):

    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    current_transformation = np.identity(4)
    if visualize:
        logging.debug("Before Colored Point Cloud Alignment")
        draw_registration_result_original_color(
            source, target, current_transformation, [axis_frame])
    logging.debug("1. Colored point cloud registration")
    t0 = time.perf_counter()
    for scale in range(len(voxel_radius)):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        logging.debug("%r", [iter, radius, scale])

        logging.debug("1-1. Downsample with a voxel size %.2f", radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        logging.debug("1-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        logging.debug("1-3. Applying colored point cloud registration")
        result_icp = o3d.registration.registration_colored_icp(
            source_down, target_down, radius * 1, current_transformation,
            o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter))
        # result_icp = o3d.registration.registration_icp(
        #     source_down, target_down, radius * 2, current_transformation,
        #     o3d.registration.TransformationEstimationPointToPlane(),
        #     o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
        #                                             relative_rmse=1e-6,
        #                                             max_iteration=iter))
        current_transformation = result_icp.transformation
        logging.debug("%r", current_transformation)
    t1 = time.perf_counter()
    logging.info("After Colored Point Cloud Alignment; elapsed time: %.1f, fitness: %.2f, inlier rmse: %.4f",
                 (t1-t0) * 1000, result_icp.fitness, result_icp.inlier_rmse)
    if visualize:
        draw_registration_result_original_color(
            source, target, current_transformation, [axis_frame])
    return result_icp


def convert_trajectory(traj, inv=True):
    new_traj = []
    for i in range(len(traj)):
        extrinsic_1 = np.linalg.inv(H_t265_d400) @ traj[i].pose @ H_t265_d400
        if inv:
            extrinsic_1 = np.linalg.inv(extrinsic_1)
        new_traj.append(extrinsic_1)
    return new_traj


def get_colored_point_cloud(idx, color_files, depth_files, intrinsic, extrinsic, depth_trunc=2.0):
    depth_1 = o3d.io.read_image(os.path.join(depth_files[idx]))
    color_1 = o3d.io.read_image(os.path.join(color_files[idx]))
    rgbd_image_1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_1, depth_1, convert_rgb_to_intensity=False, depth_trunc=depth_trunc)

    pcd_1 = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image_1, intrinsic=intrinsic, extrinsic=extrinsic)
    return pcd_1


def refine_trajectory(color_files, depth_files, intrinsic, traj,
                      min_fitness=0.50, max_rmse=0.06, n_frames=None):
    # 28
    if n_frames is None:
        n_frames = len(traj)
    # n_amount=5
    delta_transforms = []
    ipc_stats = []
    visualize = False
    for i in range(n_frames - 1):
        # visualize = False if i < 28 else True
        pcd_1 = get_colored_point_cloud(
            i, color_files, depth_files, intrinsic, traj[i])
        pcd_2 = get_colored_point_cloud(
            i+1, color_files, depth_files, intrinsic, traj[i+1])
        # visualize = True if i == 26 else False
        result_icp = colored_point_cloud_registration(
            pcd_2, pcd_1, voxel_radius=[0.01], max_iter=[50], visualize=visualize)
        if result_icp.fitness > min_fitness and result_icp.inlier_rmse < max_rmse:
            delta_transforms.append(result_icp.transformation)
        else:
            delta_transforms.append(np.identity(4))
        ipc_stats.append([result_icp.fitness, result_icp.inlier_rmse])
    ipc_stats = np.array(ipc_stats)
    mean_stats = np.mean(ipc_stats, axis=0)
    logging.info("Mean Stats [fitness, inlier_rmse]: %r", mean_stats)

    # new_traj_list = [np.identity(4)]
    new_traj_list = [traj[0]]
    delta_transform = np.identity(4)
    for i in range(1, n_frames):
        current_transform = np.linalg.inv(traj[i])
        delta_transform = delta_transforms[i-1]  @ delta_transform
        new_traj = np.linalg.inv(delta_transform  @ current_transform)
        print(delta_transforms[i-1])
        print()
        print(delta_transform)
        print()
        print()
        new_traj_list.append(new_traj)
        # new_traj_list.append(np.linalg.inv(current_transform))

    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    geoms = [axis_frame]
    for i in range(n_frames):
        extrinsics = new_traj_list[i]
        pcd_1 = get_colored_point_cloud(
            i, color_files, depth_files, intrinsic, extrinsics)
        geoms.append(pcd_1)

    o3d.visualization.draw_geometries(geoms)
    return new_traj_list


def integrate_volume_tsdf(depth_files, color_files, intrinsic, traj, cube_length=16.0, sdf_trunc=0.08, d_stride=4, n_frames=None):
    voxel_length = cube_length / 512.0
    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=0.08,
        depth_sampling_stride=d_stride,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8)

    if n_frames is None:
        n_frames = len(traj)

    t0 = time.perf_counter()
    integrate_time = []
    for i in range(n_frames):
        depth = o3d.io.read_image(os.path.join(depth_files[i]))
        color = o3d.io.read_image(os.path.join(color_files[i]))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False, depth_trunc=3.0)
        extrinsic = traj[i]
        t0_0 = time.perf_counter()
        volume.integrate(
            rgbd_image,
            intrinsic,
            extrinsic)
        t0_1 = time.perf_counter()
        integrate_time.append((t0_1 - t0_0) * 1000)

    integrate_time = np.sum(np.array(integrate_time))
    t1 = time.perf_counter()
    logging.info("Volume integration no file/io took: %.1f, total (file/io): %.1f",
                 integrate_time, (t1 - t0) * 1000)
    pcd = volume.extract_point_cloud()
    t2 = time.perf_counter()
    logging.info("Point Cloud Extraction took: %.1f, # Points: %d ",
                 (t2 - t1) * 1000, np.array(pcd.points).shape[0])
    mesh = volume.extract_triangle_mesh()
    t3 = time.perf_counter()
    logging.info("Mesh Extraction took: %.1f", (t3 - t2) * 1000)
    mesh.compute_vertex_normals()
    t4 = time.perf_counter()
    logging.info("Mesh Normal Extraction took: %.1f", (t4 - t3) * 1000)
    return mesh, pcd


def naive_integration(depth_files, color_files, intrinsic, traj, cube_length=16.0, d_stride=4, n_frames=None):
    voxel_length = cube_length / 512.0

    if n_frames is None:
        n_frames = len(traj)

    t0 = time.perf_counter()
    all_pc = []
    integrate_time = []
    for i in range(n_frames):
        depth = o3d.io.read_image(os.path.join(depth_files[i]))
        extrinsic = traj[i]
        # color = o3d.io.read_image(os.path.join(color_files[i]))
        # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        #     color, depth, convert_rgb_to_intensity=False, depth_trunc=3.0)
        # pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic, extrinsic)
        t0_0 = time.perf_counter()
        pc = o3d.geometry.PointCloud.create_from_depth_image(
            depth, intrinsic, extrinsic, depth_trunc=3.0, stride=d_stride)
        all_pc.append(pc.voxel_down_sample(voxel_length))
        t0_1 = time.perf_counter()
        integrate_time.append((t0_1 - t0_0) * 1000)

    t1 = time.perf_counter()
    integrate_time = np.sum(np.array(integrate_time))
    all_points = [np.array(pc.points) for pc in all_pc]
    new_points = np.concatenate(all_points, axis=0)
    t2 = time.perf_counter()
    pc.points = o3d.utility.Vector3dVector(new_points)
    t3 = time.perf_counter()
    pc = pc.voxel_down_sample(voxel_length)
    integrate_time = integrate_time + (t3-t1) * 1000
    # logging.info("Point Cloud Integration took: %.1f", (t1 - t0) * 1000)
    logging.info("Brute Force Point Cloud Integration (no file/io) took: %.1f, # Points: %d ",
                 integrate_time, np.array(pc.points).shape[0])
    # logging.info("Point Cloud integration took: %.1f, # Points: %d ", (t1 - t0) * 1000, max_points)
    return pc


def extract_polygons(pcd, polylidar_kwargs=dict(alpha=0.0, lmax=0.05, zThresh=0.01, normThresh=0.99, normThreshMin=0.95)):
    filter_ = dict(hole_area=dict(min=0.01, max=10),
                   hole_vertices=dict(min=3), plane_area=dict(min=0.1))
    postprocess = dict(filter=filter_, positive_buffer=0.0,
                       buffer=0.02, simplify=0.02)

    points = flip_axes(np.asarray(pcd.points))
    t0 = time.perf_counter()
    polygons = extractPolygons(points, **polylidar_kwargs)
    t1 = time.perf_counter()
    log_timing(t0, t1, "Polygon extraction using Polylidar took:")
    # print(points.shape)
    # print(polygons)

    planes, obstacles = filter_planes_and_holes2(polygons, points, postprocess)
    # print(planes)
    # print(obstacles)
    line_mesh_geoms = create_lines(planes, obstacles, line_radius=0.01, rotate_func=flip_axes_invert)
    geoms = [line_mesh.cylinder_segments for line_mesh in line_mesh_geoms]
    geoms = [segment for segment_list in geoms for segment in segment_list]

    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([pcd, axis_frame, *geoms])


def extract_polygons_new(mesh, vertices, triangles, halfedges, polylidar_kwargs=dict(alpha=0.0, lmax=0.05, zThresh=0.01, normThresh=0.99, normThreshMin=0.95, minTriangles=100)):
    filter_ = dict(hole_area=dict(min=0.01, max=10),
                   hole_vertices=dict(min=3), plane_area=dict(min=0.1))
    postprocess = dict(filter=filter_, positive_buffer=0.0,
                       buffer=0.01, simplify=0.01)
    vertices_flipped = flip_axes(vertices)

    t0 = time.perf_counter()
    planes, polygons = extract_planes_and_polygons_from_mesh(vertices_flipped, triangles, halfedges,**polylidar_kwargs)
    t1 = time.perf_counter()
    log_timing(t0, t1, "Polygon extraction using Polylidar (using precomputed smooth mesh) took:")
    # print(points.shape)
    # print(polygons)
    # print(triangulation)
    # print(planes)
    # print(polygons)

    # Paint the Planes
    triangles_3 = triangles.reshape(int(triangles.shape[0] / 3), 3)
    colors = np.asarray(mesh.vertex_colors)
    for i, plane in enumerate(planes):
        idx = i % len(COLOR_PALETTE)
        vertices_colored = triangles_3[plane, :].flatten()
        colors[vertices_colored] = np.array(COLOR_PALETTE[idx])

    # Get filtered planes and obstacles
    shapeley_planes, shapely_obstacles = filter_planes_and_holes2(polygons, vertices_flipped, postprocess)

    # Plot the lines
    line_mesh_geoms = create_lines(shapeley_planes, shapely_obstacles, line_radius=0.01, rotate_func=flip_axes_invert)
    geoms = [line_mesh.cylinder_segments for line_mesh in line_mesh_geoms]
    geoms = [segment for segment_list in geoms for segment in segment_list]

    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([mesh, axis_frame, *geoms])


def main(config, traj):
    path = config["path_dataset"]

    # Read RGBD images
    color_files, depth_files = get_rgbd_file_lists(path)
    if len(color_files) != len(depth_files):
        raise ValueError(
            "The number of color images {} must equal to the number of depth images {}."
            .format(len(color_files), len(depth_files)))

    if len(color_files) != len(traj):
        raise ValueError(
            "The number of color images {} must equal to the number of poses in trajectory {}."
            .format(len(color_files), len(traj)))

    # Read camera poses
    if config["path_intrinsic"]:
        intrinsic = o3d.io.read_pinhole_camera_intrinsic(
            config["path_intrinsic"])
    else:
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

    n_frames = len(traj)
    # n_frames = 20
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Refine trajectory with sequence wise colored point cloud registration
    # DOES NOT WORK. Experiments show that the SLAM from T265 is so good its not really need and
    # just causes issues.
    # new_traj = refine_trajectory(color_files, depth_files, intrinsic, traj, n_frames=n_frames)
    new_traj = traj
    logging.info("Integration of %d frames", n_frames)

    # Integrate only depth pixels into a downsamped point cloud
    pcd_only = naive_integration(
        depth_files, color_files, intrinsic, new_traj, n_frames=n_frames)
    logging.info("Visualize Naive Integration and DownSampled Point Cloud")
    o3d.visualization.draw_geometries([axis_frame, pcd_only])

    # Intregrate RGBD frames into a volume and mesh using TRSDF Volume
    # Extremely efficient, very smooth surfaces
    mesh, pcd = integrate_volume_tsdf(
        depth_files, color_files, intrinsic, new_traj, n_frames=n_frames)

    logging.info("Visualize TSDF Volume Integration Voxel Cloud")
    o3d.visualization.draw_geometries([axis_frame, pcd])
    logging.info("Visualize TSDF Volume Integration Mesh")
    o3d.visualization.draw_geometries([axis_frame, mesh])

    o3d.io.write_triangle_mesh(
        os.path.join(path, config["folder_scene"],
                     "mesh_with_trajectory.ply"), mesh)

    logging.info("Visualize Polygon Extraction with TSDF Point Cloud")
    extract_polygons(pcd)

    logging.info("Extract Half Mesh from TSDF Mesh")
    t0 = time.perf_counter()
    halfedges = np.asarray(o3d.geometry.HalfEdgeTriangleMesh.extract_halfedges(mesh))
    t1 = time.perf_counter()
    log_timing(t0, t1, "Efficient Half Mesh Extraction Took")
    vertices = np.asarray(mesh.vertices)
    t2 = time.perf_counter()
    log_timing(t1, t2, "Getting Vertices")
    triangles = np.asarray(mesh.triangles)
    t3 = time.perf_counter()
    log_timing(t2, t3, "Getting Triangles")
    triangles = triangles.reshape(triangles.shape[0] * 3)
    t4 = time.perf_counter()
    log_timing(t3, t4, "Reshaping Triangles")
    t5 = time.perf_counter()
    logging.info("Total Half Mesh Extraction took: %.1f, Formatting: %.1f", (t5-t0) * 1000, (t2-t1) * 1000)
    logging.info("Visualize Polygon Extraction using extracted Half Mesh from TSDF Volume Integration. NEW!!!")
    extract_polygons_new(mesh, vertices, triangles, halfedges)

def log_timing(t0, t1, message):
    logging.info("%s took %.1f ms", message, (t1-t0) * 1000)

def flip_axes_invert(points):
    points[:, -1] = -1 * points[:, -1]
    points = points[:, [0, 2, 1]]
    return np.ascontiguousarray(points)

def flip_axes(points):
    points = points[:, [0, 2, 1]]
    points[:, -1] = -1 * points[:, -1]
    return np.ascontiguousarray(points)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Color map optimizer for a reconstruction dataset')
    parser.add_argument('--config',
                        type=str,
                        required=True,
                        help='path to the config for the dataset '
                        'preprocessed by the Reconstruction System')
    parser.add_argument(
        '--traj',
        type=str,
        help='txt file that contains the trajectories')
    args = parser.parse_args()

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
    if args.config is not None:
        with open(args.config) as json_file:
            config = json.load(json_file)
            initialize_config(config)
    assert config is not None

    traj = None
    if args.traj is not None:
        traj = read_trajectory(args.traj)
    if traj is None:
        traj_path = os.path.join(
            config["path_dataset"], config["template_global_traj"])
        traj = read_trajectory(traj_path)
    traj = convert_trajectory(traj, inv=True)

    main(config, traj)
