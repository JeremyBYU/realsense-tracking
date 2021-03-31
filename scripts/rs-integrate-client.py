"Client that will connect to rs-integrate-server and extract mesh"
import sys
from pathlib import Path
import time
import logging

logging.basicConfig(level=logging.INFO)

import numpy as np
import ecal.core.core as ecal_core
import open3d as o3d
from ecal.core.service import Client

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "cmake-build").resolve()
sys.path.insert(1, str(BUILD_DIR))
BUILD_DIR = (THIS_DIR / ".." / "dk-x86_64-build").resolve()
sys.path.insert(1, str(BUILD_DIR))



from PoseMessage_pb2 import PoseMessage

counter = 0

def get_mesh_data(resp: ExtractResponse):
    mesh = resp.mesh
    n_triangles = mesh.n_triangles
    n_vertices = mesh.n_vertices
    triangles = np.frombuffer(
        mesh.triangles, dtype=np.int32).reshape((n_triangles, 3))
    vertices = np.frombuffer(
        mesh.vertices, dtype=np.float64).reshape((n_vertices, 3))
    vertices_colors = np.frombuffer(
        mesh.vertices_colors, dtype=np.float64).reshape((n_vertices, 3))
    halfedges = np.frombuffer(mesh.halfedges, dtype=np.uint64)
    return np.copy(vertices), np.copy(triangles), np.copy(halfedges), np.copy(vertices_colors)


def create_mesh_from_data(vertices, triangles, halfedges, vertices_colors=None):
    """Combines numpy arrays into a Open3D triangular Mesh
    
    Arguments:
        vertices {ndarray} -- vertices NX3
        triangles {ndarray} -- triangles NX3
        halfedges {ndarray} -- NX1
    
    Returns:
        o3d.geometry.TriangleMesh -- TriangleMesh
    """
    vertices_o3d = o3d.utility.Vector3dVector(vertices)
    triangles_o3d = o3d.utility.Vector3iVector(triangles)
    mesh = o3d.geometry.TriangleMesh(vertices_o3d, triangles_o3d)
    if vertices_colors is not None:
        vertices_colors[:,[0, 2]] = vertices_colors[:,[2, 0]]
        vertices_colors = np.ascontiguousarray(vertices_colors)
        mesh.vertex_colors = o3d.utility.Vector3dVector(vertices_colors)
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    return mesh


def client_resp_callback(service_info, response):
    logging.info("Service: %s; Method: %s", service_info['service_name'], service_info['method_name'])
    m_name = service_info['method_name']
    global counter
    if (m_name == 'IntegrateScene'):
        resp = IntegrateResponse()
        resp.ParseFromString(response)
    elif m_name == 'ExtractScene':
        resp = ExtractResponse()
        resp.ParseFromString(response)
        raw_data = get_mesh_data(resp)
        tri_mesh = create_mesh_from_data(*raw_data)
        logging.info("Triangle Size: %d", len(tri_mesh.triangles))
        if len(tri_mesh.triangles) > 0:
            o3d.visualization.draw_geometries([tri_mesh])
            resp_user = input("Save Mesh (Y/N):")
            if resp_user in ['y', 'Y', 'yes']:
                o3d.io.write_triangle_mesh('data/meshes/integrate_mesh.ply', tri_mesh)


def main():
      # initialize eCAL API
    ecal_core.initialize([], "RSIntegrationClient")

    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    client = Client("rspub_pb.IntegrateService")

    # and add it to the client
    client.add_response_callback(client_resp_callback)
    logging.info("Starting up RS Integration Client...")
    time.sleep(2.0)

    # Create new Scene to integrate
    request = IntegrateRequest()
    request.scene = "Default"
    request.type = ADD
    request_string = request.SerializeToString()
    _ = client.call_method("IntegrateScene", request_string)
    logging.info("Create a default scene for volume integration")
    time.sleep(0.5)
    # Start Integration on Scene
    request.type = START
    request_string = request.SerializeToString()
    _ = client.call_method("IntegrateScene", request_string)
    logging.info("Starting volume integration")
    time.sleep(0.5)
    # Extract Mesh on user input
    while True:
        _ = input("Press Enter to Show the current Mesh")
        request = ExtractRequest()
        request.scene = "Default"
        request.type = MESH
        request_string = request.SerializeToString()
        _ = client.call_method("ExtractScene", request_string)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
