import sys
from pathlib import Path
import time

import numpy as np
import ecal.core.core as ecal_core
import open3d as o3d
from ecal.core.service import Client

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "build").resolve()
sys.path.insert(1, str(BUILD_DIR))


from Integrate_pb2 import SceneRequestType, DataType, IntegrateRequest, IntegrateResponse, ExtractResponse, ADD, REMOVE, START, ExtractRequest, MESH


def get_mesh_data(resp:ExtractResponse):
    mesh = resp.mesh
    print(dir(mesh))
    n_triangles = mesh.n_triangles
    n_vertices = mesh.n_vertices
    n_halfedges = n_triangles * 3
    triangles = np.fromstring(mesh.triangles, dtype=np.int32).reshape((n_triangles, 3))
    vertices = np.fromstring(mesh.vertices, dtype=np.float64).reshape((n_vertices, 3))
    halfedges = np.fromstring(mesh.halfedges, dtype=np.uint64)
    # triangles.setflags(write=1)
    # vertices.setflags(write=1)
    return vertices, triangles, halfedges
    # print(triangles.shape)
    # print(triangles[:3,:])
    # print(vertices.shape)
    # print(vertices[:3,:])
    # print(halfedges.shape)
    # print(halfedges[:3])

def create_mesh_from_data(vertices, triangles, halfedges):
    vertices_o3d = o3d.utility.Vector3dVector(vertices)
    triangles_o3d = o3d.utility.Vector3iVector(triangles)
    mesh = o3d.geometry.TriangleMesh(vertices_o3d, triangles_o3d)
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    return mesh

def client_resp_callback(service_info, response):
    print(service_info)
    m_name = service_info['method_name']
    if (m_name == 'IntegrateScene'):
        resp = IntegrateResponse() 
        resp.ParseFromString(response)
        print(resp)

    elif m_name == 'ExtractScene':
        resp = ExtractResponse() 
        resp.ParseFromString(response)
        raw_data = get_mesh_data(resp)
        tri_mesh = create_mesh_from_data(*raw_data)
        o3d.visualization.draw_geometries([tri_mesh])
        print(tri_mesh)


def main():
      # initialize eCAL API
    ecal_core.initialize([], "Test_Integrate_Service")
    
    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    client = Client("rspub_pb.IntegrateService")

      # and add it to the client
    client.add_response_callback(client_resp_callback)
    time.sleep(2)

    request = IntegrateRequest()
    request.scene = "Default"
    request.type = ADD
    request_string = request.SerializeToString()


    _ = client.call_method("IntegrateScene", request_string)
    time.sleep(1.0)
    # Start Integration
    request.type = START
    request_string = request.SerializeToString()
    _ = client.call_method("IntegrateScene", request_string)
    time.sleep(1.0)
    # Wait to extract Mesh
    while True:
        _ = input("Press Enter to show Mesh")
        request = ExtractRequest()
        request.scene = "Default"
        request.type = MESH
        request_string = request.SerializeToString()
        _ = client.call_method("ExtractScene", request_string)
        time.sleep(1.0)

if __name__ == "__main__":
    main()
