# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Advanced/color_map_optimization_for_reconstruction_system.py

import argparse
import os, sys
import json

import open3d as o3d
import numpy as np

sys.path.append("../Utility")
# from ..ReconstructionSystem.initialize_config import *
# from ..Utility.trajectory_io import *
from initialize_config import initialize_config
from file import get_rgbd_file_lists
from trajectory_io import read_trajectory


H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

def main(config, traj):
    path = config["path_dataset"]

    # Read RGBD images
    color_files, depth_files = get_rgbd_file_lists(path)
    if len(color_files) != len(depth_files):
        raise ValueError(
            "The number of color images {} must equal to the number of depth images {}."
            .format(len(color_files), len(depth_files)))

    # import pdb; pdb.set_trace()
    if len(color_files) != len(traj):
        raise ValueError(
            "The number of color images {} must equal to the number of poses in trajectory {}."
            .format(len(color_files), len(depth_files)))

    # Read camera poses
    if config["path_intrinsic"]:
        intrinsic = o3d.io.read_pinhole_camera_intrinsic(
            config["path_intrinsic"])
    else:
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)


    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=4.0 / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8)

    # Load images
    # rgbd_images = []
    # pcd_list = []
    # print(traj)
    # pcd_big = None
    # for i in range(len(traj)):
    #     depth = o3d.io.read_image(os.path.join(depth_files[i]))
    #     color = o3d.io.read_image(os.path.join(color_files[i]))
    #     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    #         color, depth, convert_rgb_to_intensity=False)
    #     new_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    #     # print(traj[i].pose)
    #     new_pcd = new_pcd.transform(traj[i].pose)
    #     if pcd_big is None:
    #         pcd_big = new_pcd
    #     else:
    #         pcd_big = pcd_big + new_pcd
    #         pcd_big = pcd_big.voxel_down_sample(voxel_size=0.01)
    #     # pcd_list.append(new_pcd)
    # pcd_big = pcd_big.voxel_down_sample(voxel_size=0.01)
    # o3d.visualization.draw_geometries([pcd_big])

    for i in range(len(traj)):
        depth = o3d.io.read_image(os.path.join(depth_files[i]))
        color = o3d.io.read_image(os.path.join(color_files[i]))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False)
        extrinsic = np.dot(H_t265_d400,traj[i].pose)
        extrinsic = np.linalg.inv(extrinsic)
        volume.integrate(
            rgbd_image,
            intrinsic,
            extrinsic)


    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh(
    #     os.path.join(path, config["folder_scene"],
    #                  "color_map_after_optimization.ply"), mesh)


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

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    if args.config is not None:
        with open(args.config) as json_file:
            config = json.load(json_file)
            initialize_config(config)
    assert config is not None

    traj = None
    if args.traj is not None:
        traj = read_trajectory(args.traj)
    if traj is None:
        traj_path = os.path.join(config["path_dataset"], config["template_global_traj"])
        traj = read_trajectory(traj_path)

    main(config, traj)