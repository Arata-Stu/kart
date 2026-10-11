import tempfile
from pathlib import Path
import unittest
import numpy as np
from evaluate_vslam import compare, rotation, read_bag, save


class EvaluationTests(unittest.TestCase):
    def samples(self, times, positions, quats, frame='world'):
        return np.array(times, dtype=np.int64), np.array(positions, float), np.array(quats, float), frame, 'base_link', []

    def test_initial_frame_alignment_preserves_scale_error(self):
        times = np.arange(11)*100_000_000
        positions = np.c_[np.arange(11)*.1, np.zeros((11,2))]
        truth = self.samples(times, positions, np.tile([0,0,0,1], (11,1)))
        q = [0,0,np.sin(.4),np.cos(.4)]
        est_positions = positions@rotation(q).T + [4,5,2]
        estimate = self.samples(times, est_positions, np.tile(q,(11,1)), 'odom')
        summary, rows = compare(truth, estimate, max_gap=.2)
        self.assertLess(summary['position_rmse_m'], 1e-12)
        self.assertLess(summary['orientation_rmse_deg'], 1e-5)
        estimate = self.samples(times, 2*positions@rotation(q).T+[4,5,2], np.tile(q,(11,1)), 'odom')
        summary, rows = compare(truth, estimate, max_gap=.2)
        self.assertAlmostEqual(summary['path_length_ratio'],2)
        self.assertAlmostEqual(summary['final_position_error_m'],1)
        with tempfile.TemporaryDirectory() as directory:
            save(directory, summary, rows)
            self.assertTrue((Path(directory)/'trajectory.svg').is_file())

    def test_interpolation_and_no_overlap_rejection(self):
        times = np.arange(11)*10_000_000
        positions = np.c_[np.arange(11)*.01,np.zeros((11,2))]
        truth = self.samples(times, positions, np.tile([0,0,0,1],(11,1)))
        estimate = self.samples(times[:-1]+5_000_000, positions[:-1]+[.005,0,0], np.tile([0,0,0,-1],(10,1)))
        summary, _ = compare(truth,estimate)
        self.assertLess(summary['position_rmse_m'],1e-12)
        estimate = self.samples(times+1_000_000_000, positions, np.tile([0,0,0,1],(11,1)))
        with self.assertRaisesRegex(ValueError,'shared header times'):
            compare(truth,estimate)

    def test_bag_reader_uses_header_not_bag_receipt_time(self):
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
        store = get_typestore(Stores.ROS2_HUMBLE)
        def msg(name,*args):return store.types[name](*args)
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory)/'truth'
            with Writer(bag,version=9) as writer:
                conn = writer.add_connection('/sim/odometry','nav_msgs/msg/Odometry',typestore=store)
                for i in range(3):
                    stamp = msg('builtin_interfaces/msg/Time',12,i*10_000_000)
                    header = msg('std_msgs/msg/Header',stamp,'sim_world')
                    pose = msg('geometry_msgs/msg/Pose',msg('geometry_msgs/msg/Point',float(i),0.,0.),msg('geometry_msgs/msg/Quaternion',0.,0.,0.,1.))
                    vec = msg('geometry_msgs/msg/Vector3',0.,0.,0.)
                    value = msg('nav_msgs/msg/Odometry',header,'base_link',msg('geometry_msgs/msg/PoseWithCovariance',pose,np.zeros(36)),msg('geometry_msgs/msg/TwistWithCovariance',msg('geometry_msgs/msg/Twist',vec,vec),np.zeros(36)))
                    writer.write(conn,100_000_000_000+i*10_000_000,store.serialize_cdr(value,'nav_msgs/msg/Odometry'))
            samples = read_bag(bag,'/sim/odometry')
            self.assertEqual(samples[0].tolist(),[12_000_000_000,12_010_000_000,12_020_000_000])
            self.assertEqual(samples[4],'base_link')


if __name__=='__main__':unittest.main()
