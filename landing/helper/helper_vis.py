import numpy as np
import cv2

ORANGE = [6, 115, 249]

def project_points_img(points, proj_mat, width, height):
    """Projects points into image given a projection matrix
    Arguments:
        points {ndarray} -- 3D points
        proj_mat {ndarray, 3X4} -- Projection Matrix
        width {int} -- width of image
        height {height} -- height of image
    Returns:
        ndarray -- pixels
    """
    pixels = proj_mat.dot(points)
    pixels = np.divide(pixels[:2, :], pixels[2, :]).transpose().astype(np.int)

    # Remove pixels that are outside the image
    pixels[:, 0] = np.clip(pixels[:, 0], 0, width)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, height)
    # mask_x = (pixels[:, 0] < width) & (pixels[:, 0] > 0)
    # mask_y = (pixels[:, 1] < height) & (pixels[:, 1] > 0)

    # # Return the pixels and points that are inside the image
    # pixels = pixels[mask_x & mask_y]
    return pixels



def get_pix_coordinates(points_t, proj_mat, w, h):
    """Get Pixel coordinates of ndarray
    Arguments:
        pts {ndarray} -- 3D point clouds 3XN
        proj_mat {ndarray} -- 4X3 Projection Matrix
        w {int} -- width
        h {int} -- height
    Returns:
        ndarray -- Pixel coordinates
    """
    # points_t = np.ones(shape=(4, pts.shape[1]))
    # points_t[:3, :] = pts
    pixels = project_points_img(points_t, proj_mat, w, h)
    return pixels


def plot_opencv_linear_rings(linear_rings, color_image, proj_mat, rot_mat, color=(0, 255, 0), thickness=2):
    for i, linear_ring in enumerate(linear_rings):
        # Get 2D polygons and assign z component the height value of extracted plane
        if rot_mat is not None:
            pts = np.array(linear_ring.coords)[:,:3]  # NX3
            pts = np.column_stack((pts, np.ones((pts.shape[0]))))  # NX4
            # Transform flat plane coordinate system to original cordinate system of depth frame
            pts = pts.transpose()  # 3XN
            pts = rot_mat @ pts
        else:
            pts = np.transpose(np.array(linear_ring.coords)[:,:3])  # NX3

        # np.savetxt(f"polygon_{i}_cameraframe.txt", pts.transpose())
        # Project coordinates to image space
        pix_coords = get_pix_coordinates(pts, proj_mat, color_image.shape[1] - 1, color_image.shape[0] - 1)
        pix_coords = pix_coords.reshape((-1, 1, 2))
        cv2.polylines(color_image, [pix_coords], True, color, thickness=thickness)


def plot_polygons(plane_data, proj_mat, rot_mat, color_image, thickness=2, shell_color=(0, 255, 0), hole_color=ORANGE):
    """Plots the planes and obstacles (3D polygons) into the color image
    Arguments:
        planes {list(Polygons)} -- List of Shapely Polygon with height tuples
        proj_mat {ndarray} -- Projection Matrix
        rot_mat {ndarray} -- Rotation Matrix
        color_image {ndarray} -- Color Image
        config {dict} -- Configuration
    """
    # print(proj_mat)
    # print(rot_mat)
    plane, meta = plane_data
    plot_opencv_linear_rings(
        [plane.exterior], color_image, proj_mat, rot_mat, color=shell_color, thickness=thickness)

    plot_opencv_linear_rings(
        plane.interiors, color_image, proj_mat, rot_mat, color=hole_color,  thickness=thickness)