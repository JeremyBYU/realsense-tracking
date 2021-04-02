import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

from polylidar.polylidarutil.plane_filtering import filter_planes
from polylidar import MatrixDouble, Polylidar3D, MatrixUInt8

from fastga import GaussianAccumulatorS2, MatX3d, IcoCharts
from fastga.peak_and_cluster import find_peaks_from_ico_charts

from landing.helper.helper_logging import logger
from landing.helper.helper_meshes import create_meshes, create_open_3d_mesh_from_tri_mesh
from landing.helper.o3d_util import create_linemesh_from_shapely, get_segments


def choose_dominant_plane_normal(avg_peaks, rooftop_normal=[0.0, 0.0, -1.0]):
    dots = np.array([avg_peaks[i, :].dot(rooftop_normal) for i in range(avg_peaks.shape[0])])
    index = np.argmax(dots)
    new_avg_peaks = np.array([avg_peaks[index, :]])
    return new_avg_peaks


def down_sample_normals(triangle_normals, down_sample_fraction=0.12, min_samples=100, flip_normals=False, **kwargs):
    num_normals = triangle_normals.shape[0]
    to_sample = int(down_sample_fraction * num_normals)
    to_sample = max(min([num_normals, min_samples]), to_sample)
    ds_step = int(num_normals / to_sample)
    triangle_normals_ds = np.ascontiguousarray(
        triangle_normals[:num_normals:ds_step, :])
    if flip_normals:
        triangle_normals_ds = triangle_normals_ds * -1.0
    return triangle_normals_ds


def get_image_peaks(ico_chart, ga, level=2, find_peaks_kwargs=dict(
        threshold_abs=2, min_distance=1, exclude_border=False, indices=False),
        cluster_kwargs=dict(t=0.10, criterion='distance'),
        average_filter=dict(min_total_weight=0.01),
        **kwargs):

    normalized_bucket_counts_by_vertex = ga.get_normalized_bucket_counts_by_vertex(
        True)

    t1 = time.perf_counter()
    # this takes microseconds
    ico_chart.fill_image(normalized_bucket_counts_by_vertex)
    # plt.imshow(np.asarray(ico_chart.image))
    # plt.show()
    average_vertex_normals = np.asarray(ga.get_average_normals_by_vertex(
        True)) if hasattr(ga, 'get_average_normals_by_vertex') else None
    peaks, clusters, avg_peaks, avg_weights = find_peaks_from_ico_charts(ico_chart, np.asarray(
        normalized_bucket_counts_by_vertex), average_vertex_normals, find_peaks_kwargs, cluster_kwargs, average_filter)
    t2 = time.perf_counter()

    elapsed_time = (t2 - t1) * 1000
    timings = dict(t_fastga_peak=elapsed_time)

    logger.debug("Peak Detection - Took (ms): %.2f", (t2 - t1) * 1000)

    return avg_peaks, timings


def extract_all_dominant_plane_normals(tri_mesh, level=5, with_o3d=False, ga_=None, ico_chart_=None, **kwargs):

    # Reuse objects if provided
    if ga_ is not None:
        ga = ga_
    else:
        ga = GaussianAccumulatorS2(level=level)

    if ico_chart_ is not None:
        ico_chart = ico_chart_
    else:
        ico_chart = IcoCharts(level=level)

    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    triangle_normals_ds = down_sample_normals(triangle_normals, **kwargs)

    # np.savetxt('bad_normals.txt', triangle_normals_ds)
    triangle_normals_ds_mat = MatX3d(triangle_normals_ds)
    t1 = time.perf_counter()
    ga.integrate(triangle_normals_ds_mat)
    t2 = time.perf_counter()

    logger.debug("Gaussian Accumulator - Normals Sampled: %d; Took (ms): %.2f",
                 triangle_normals_ds.shape[0], (t2 - t1) * 1000)

    avg_peaks, timings_dict = get_image_peaks(
        ico_chart, ga, level=level, with_o3d=with_o3d, **kwargs)

    elapsed_time_fastga = (t2 - t1) * 1000
    elapsed_time_peak = timings_dict['t_fastga_peak']
    elapsed_time_total = elapsed_time_fastga + elapsed_time_peak

    timings = dict(t_fastga_total=elapsed_time_total,
                   t_fastga_integrate=elapsed_time_fastga, t_fastga_peak=elapsed_time_peak)

    ga.clear_count()
    return avg_peaks, timings

def prefilter_polys(points, polygons, avg_peak, drone_pose=np.array([0, 0, 0])):
    return polygons

def filter_and_create_polygons(points, polygons, rm=None, line_radius=0.005,
                               postprocess=dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.05)),
                                                positive_buffer=0.00, negative_buffer=0.00, simplify=0.0), prefilter=False, avg_peak=np.array([0, 0, -1]),
                                                drone_pose=np.array([0.0, 0.0, 0.0])):
    " Apply polygon filtering algorithm, return Open3D Mesh Lines "
    if prefilter:
        polygons = prefilter_polys(points, polygons, avg_peak, drone_pose=drone_pose)
    t1 = time.perf_counter()
    # planes, obstacles = filter_planes(polygons, points, postprocess, rm=rm)
    planes = filter_planes(polygons, points, postprocess, rm=rm)
    t2 = time.perf_counter()
    return planes, (t2 - t1) * 1000


def extract_planes_and_polygons_from_mesh(tri_mesh, avg_peaks,
                                          polylidar_kwargs=dict(alpha=0.0, lmax=0.1, min_triangles=2000,
                                                                z_thresh=0.1, norm_thresh=0.95, norm_thresh_min=0.95, min_hole_vertices=50, task_threads=4),
                                          postprocess=dict(filter=dict(hole_area=dict(min=0.025, max=100.0), hole_vertices=dict(min=6), plane_area=dict(min=0.05)),
                                                           positive_buffer=0.00, negative_buffer=0.00, simplify=0.0),
                                          pl_=None, **kwargs):

    if pl_ is not None:
        pl = pl_
    else:
        pl = Polylidar3D(**polylidar_kwargs)

    avg_peaks_mat = MatrixDouble(avg_peaks)
    t0 = time.perf_counter()
    all_planes, all_polygons = pl.extract_planes_and_polygons_optimized(
        tri_mesh, avg_peaks_mat)
    t1 = time.perf_counter()

    polylidar_time = (t1 - t0) * 1000
    all_planes_shapely = []
    all_triangle_sets = []
    time_filter = []

    vertices = np.asarray(tri_mesh.vertices)
    for i in range(avg_peaks.shape[0]):
        avg_peak = avg_peaks[i, :]
        rm, _ = R.align_vectors([[0, 0, 1]], [avg_peak])
        polygons_for_normal = all_polygons[i]
        all_triangle_sets = all_planes[i]
        if len(polygons_for_normal) > 0:
            planes_shapely, filter_time = filter_and_create_polygons(
                vertices, polygons_for_normal, rm=rm, postprocess=postprocess, avg_peak=avg_peak, **kwargs)
            all_planes_shapely.extend(planes_shapely)
            time_filter.append(filter_time)

    timings = dict(t_polylidar_planepoly=polylidar_time,
                   t_polylidar_filter=np.array(time_filter).sum())
    # all_planes_shapely, all_obstacles_shapely, all_poly_lines, timings
    return all_planes_shapely, all_triangle_sets, timings


def extract_polygons_from_points(opc, pl, ga, ico, config,
                                 dynamic_decimation=False,
                                 gravity_vector=[0, 0, -1], **kwargs):
    """Will extract a polygon from the organized point cloud. Assumes point cloud is
    in the body frame of the vehicle.

    Args:
        opc (ndarray): Organized Point Cloud of shape (NXMX3)f64
        pl (polylidar.Polylidar): Polylidar3D Object
        ga (GassianAccumulator): Gaussian Accumulator (Normal Detection)
        ico (Icosahedron): Icosahedron (paired with accumulator)
        config (dict): Configuration object, has algorithm parameters
        dynamic_decimation (bool, optional): [description]. Defaults to False.
        gravity_vector (ndarray): A vector that is aligned with gravity

    Returns:
        polygons
    """

    logger.info("Extracting polygon from organized point cloud with shape: %s", opc.shape)
    # TODO if necessary
    if dynamic_decimation:
        pass
    else:
        pass
    # 1. Create mesh
    alg_timings = dict()
    tri_mesh, timings = create_meshes(opc, **config['mesh']['filter'])
    alg_timings.update(timings)

    o3d_mesh = create_open_3d_mesh_from_tri_mesh(tri_mesh)

    # 2. Get dominant plane normals
    avg_peaks, timings = extract_all_dominant_plane_normals(
        tri_mesh, ga_=ga, ico_chart_=ico, **config['fastga'])
    # only looking for most dominant plane of the rooftop
    avg_peaks = choose_dominant_plane_normal(avg_peaks, gravity_vector)
    alg_timings.update(timings)
    logger.info("FastGA found peaks: %s", avg_peaks)
    # 3. Extract Planes and Polygons, Filter, Simplify
    planes, triangle_sets, timings = extract_planes_and_polygons_from_mesh(tri_mesh, avg_peaks, pl_=pl,
                                                                           postprocess=config['polygon']['postprocess'], **kwargs)
    alg_timings.update(timings)
    # line_meshes = create_linemesh_from_shapely(planes[0][0])
    # o3d.visualization.draw_geometries([o3d_mesh, *get_segments(line_meshes)])

    return planes, alg_timings, tri_mesh, avg_peaks, triangle_sets


