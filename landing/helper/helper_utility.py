import numpy as np

def create_projection_matrix(fx, fy, ppx, ppy):
    proj_mat = np.array([[fx, 0, ppx, 0], [0, fy, ppy, 0], [0, 0, 1, 0]])
    return proj_mat