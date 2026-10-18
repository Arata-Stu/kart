import sys
from pathlib import Path
import unittest
import numpy as np
import trimesh
sys.path.insert(0,str(Path(__file__).parent))
from core import paint, project, transform


class ColorTests(unittest.TestCase):
    def test_projection_and_extrinsic(self):
        pose=np.eye(4);pose[:3,3]=[1,2,3]
        k=np.array([[10,0,10],[0,10,10],[0,0,1.]])
        _,uv,ids=project(np.array([[1,2,5],[1,2,2],[100,2,5]]),pose,k,21,21)
        self.assertEqual(ids.tolist(),[0])
        np.testing.assert_allclose(uv[0],[10,10])

    def test_occluded_vertex_stays_gray(self):
        # Large front triangle occludes all rear vertices without shared vertices.
        v=np.array([[-1,-1,2],[1,-1,2],[0,1,2],[-.2,-.2,3],[.2,-.2,3],[0,.2,3]])
        mesh=trimesh.Trimesh(vertices=v,faces=[[0,1,2],[3,4,5]],process=False)
        rgb=np.zeros((101,101,3),dtype=np.uint8);rgb[:]=[250,10,30]
        colors=np.full((6,3),170,dtype=np.uint8);best=np.full(6,np.inf)
        before=v.copy()
        paint(v,mesh,rgb,np.eye(4),np.array([[50,0,50],[0,50,50],[0,0,1.]]),colors,best)
        np.testing.assert_array_equal(v,before)
        np.testing.assert_array_equal(colors[3:],np.full((3,3),170))
        self.assertTrue(np.isfinite(best[:3]).all())
        np.testing.assert_array_equal(colors[:3],np.tile([250,10,30],(3,1)))

    def test_real_bag_to_colored_ply_preserves_geometry(self):
        import tempfile, json
        from types import SimpleNamespace
        from rosbags.typesys import get_typestore, Stores
        from rosbags.rosbag2 import Writer
        from plyfile import PlyData, PlyElement
        from run import run
        store=get_typestore(Stores.LATEST); types=store.types
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);bag=root/'bag'
            ns=1750000000000000000
            header=types['std_msgs/msg/Header'](types['builtin_interfaces/msg/Time'](1750000000,0),'rgb_optical')
            k=np.array([50.,0,50,0,50,50,0,0,1])
            info=types['sensor_msgs/msg/CameraInfo'](header,101,101,'plumb_bob',
                np.zeros(5),k,np.eye(3).flatten(),np.zeros(12),0,0,
                types['sensor_msgs/msg/RegionOfInterest'](0,0,0,0,False))
            pixels=np.tile(np.array([25,150,230],dtype=np.uint8),(101,101,1))
            image=types['sensor_msgs/msg/Image'](header,101,101,'rgb8',0,303,pixels.flatten())
            with Writer(bag,version=9) as writer:
                for topic,msg in [('/rgb',image),('/info',info)]:
                    conn=writer.add_connection(topic,msg.__msgtype__,typestore=store)
                    writer.write(conn,ns,store.serialize_cdr(msg,msg.__msgtype__))
            vertex=np.array([(-1.,-1.,2.),(1.,-1.,2.),(0.,1.,2.)],
                            dtype=[('x','f4'),('y','f4'),('z','f4')])
            face=np.empty(1,dtype=[('vertex_indices','O')]);face[0]=(np.array([0,1,2],dtype='i4'),)
            mesh=root/'mesh.ply';out=root/'colored.ply';meta=root/'frames_meta.json'
            PlyData([PlyElement.describe(vertex,'vertex'),PlyElement.describe(face,'face')]).write(mesh)
            identity=dict(axis_angle=dict(x=0,y=0,z=0,angle_degrees=0),translation=dict(x=0,y=0,z=0))
            meta.write_text(json.dumps(dict(stereo_pair=[dict(left_camera_param_id='0')],
                camera_params_id_to_camera_params={'0':dict(sensor_meta_data=dict(sensor_to_vehicle_transform=identity))},
                keyframes_metadata=[dict(camera_params_id='0',timestamp_microseconds=str(ns//1000),camera_to_world=identity)])))
            run(SimpleNamespace(mesh=str(mesh),metadata=str(meta),bag=str(bag),output=str(out),
                base_frame='rgb_optical',image='/rgb',info='/info',max_frames=1))
            result=PlyData.read(out)
            for axis in 'xyz': np.testing.assert_array_equal(result['vertex'][axis],vertex[axis])
            np.testing.assert_array_equal(result['face']['vertex_indices'][0],[0,1,2])
            np.testing.assert_array_equal(result['vertex']['red'],[25]*3)
            self.assertEqual(json.loads(Path(str(out)+'.json').read_text())['coverage'],1.)

    def test_official_axis_angle(self):
        matrix=transform(dict(axis_angle=dict(x=0,y=0,z=1,angle_degrees=90),
                              translation=dict(x=1,y=2,z=3)))
        np.testing.assert_allclose(matrix @ [1,0,0,1],[1,3,3,1],atol=1e-8)


if __name__=='__main__': unittest.main()
