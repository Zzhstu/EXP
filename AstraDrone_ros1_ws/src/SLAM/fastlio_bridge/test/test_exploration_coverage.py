#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from exploration_native import SEARCH
from exploration_coverage import (ProgressWindow,SurfaceEvidence,visible_unknown_gain,
                                  raw_line_visible,suppress_disk)
from warehouse_map_audit import audit,write_xyz_pcd,read_xyz_pcd,side_samples,supported


class CoverageTest(unittest.TestCase):
    def test_sparse_free_voxel_centres_do_not_fill_unknown_holes(self):
        x,y=np.meshgrid(np.arange(.075,8.,.15),np.arange(.075,8.,.15))
        points=np.stack((x.ravel(),y.ravel()),axis=1)
        for resolution,expected in ((.2,True),(.1,False)):
            g=ExplorationGrid([0,0,8,8],resolution,.8,.4)
            g.observe_free(points);g.update_obstacles([])
            self.assertEqual(bool(g.body_known[g.cell((4.,4.))]),expected)

    def test_finer_grid_preserves_clearance_not_squeezing_through(self):
        for width,resolution,expected in ((1.9,.2,False),(1.9,.1,True),(1.8,.1,False),(1.6,.1,False)):
            g=ExplorationGrid([0,0,8,8],resolution,.8,.4);g.known[:]=True
            g.update_obstacles([(x,y) for x in (2.,2.+width) for y in np.arange(0,8,.05)])
            d,_=g.search(g.cell((2.+width/2,1.)))
            self.assertEqual(bool(np.isfinite(d[g.cell((2.+width/2,7.))])),expected)

    def test_convergence_needs_full_window_and_both_gains(self):
        p=ProgressWindow(10,1,5)
        p.update(0,100,10,0);p.update(9,100,10,3)
        self.assertFalse(p.summary(.2)['low_gain'])
        p.update(10,100,10,3)
        self.assertTrue(p.summary(.2)['low_gain'])
        p.update(11,100,20,4)
        self.assertFalse(p.summary(.2)['low_gain'])
        p.reset();self.assertFalse(p.summary(.2)['window_ready'])

    def test_stable_surface_count_cannot_hide_new_free_area(self):
        p=ProgressWindow(10,1,5)
        p.update(0,100,10,0);p.update(10,150,10,3)
        self.assertFalse(p.summary(.2)['low_gain'])

    def test_wall_blocks_unknown_gain(self):
        g=ExplorationGrid([0,0,10,10])
        g.known[:,:25]=True
        g.occupied[:,25]=True
        gain=visible_unknown_gain(g,[(25,18)],sensor_range=3)
        self.assertEqual(gain[0],0.)
        g.occupied[:]=False
        self.assertGreater(visible_unknown_gain(g,[(25,18)],sensor_range=3)[0],0.)

    def test_raw_surface_ray_stops_at_wall_not_at_its_endpoint(self):
        g=ExplorationGrid([0,0,10,10])
        g.occupied[:,25]=True
        self.assertTrue(raw_line_visible(g,(2.,5.),(5.1,5.)))
        self.assertFalse(raw_line_visible(g,(2.,5.),(7.,5.)))

    def test_repeated_scan_or_same_pose_not_independent_views(self):
        s=SurfaceEvidence()
        points=np.array([[2.1,2.1,1.1]])
        for stamp in (1,1,1,2,3):
            s.observe(points,(0.,2.1,1.1),stamp)
        self.assertEqual(s.records[(5,5,2)][0],3)
        self.assertFalse(s.complete((5,5,2)))
        s.observe(points,(2.1,0.,1.1),4)
        self.assertTrue(s.complete((5,5,2)))

    def test_removed_person_not_a_permanent_inspection_target(self):
        s=SurfaceEvidence()
        person=np.array([[1.,1.,1.]])
        s.observe(person,(0.,0.,1.),1)
        deficits,report=s.deficits(np.empty((0,3)))
        self.assertEqual(len(deficits),0)
        self.assertIsNone(report['observed_surface_quality_ratio'])

    def test_coarse_surface_endpoint_does_not_hide_first_return(self):
        g=ExplorationGrid([0,0,8,8]);g.known[:]=True
        g.update_obstacles([(7.7,y) for y in np.arange(.1,8,.2)])
        d,_=g.search(g.cell((2.,2.)))
        s=SurfaceEvidence()
        candidates,gain=s.inspection_candidates(g,d,np.array([[7.8,4.2,1.4]]))
        self.assertGreater(candidates.sum(),0)
        self.assertTrue(np.all(g.safe[candidates]))

    def test_surface_memory_cap_explicit_not_fake_complete(self):
        s=SurfaceEvidence(maximum=1)
        s.observe(np.array([[1.,1.,1.],[3.,3.,1.]]),(0,0,1),1)
        self.assertTrue(s.capacity_limited)
        self.assertEqual(len(s.records),1)

    def test_patch_visit_suppression_matches_whole_grid(self):
        a=np.ones((30,40),bool)
        yy,xx=np.ogrid[:30,:40]
        expected=a.copy();expected[(yy-1)**2+(xx-2)**2<5.5**2]=False
        suppress_disk(a,(1,2),5.5)
        np.testing.assert_array_equal(a,expected)

    @unittest.skipIf(SEARCH is None,'native library not built')
    def test_native_distances_equal_python_and_paths_safe(self):
        rng=np.random.RandomState(19)
        g=ExplorationGrid([0,0,12,10])
        g.safe=rng.rand(*g.shape)>.20
        start=(15,15);g.safe[start]=True
        a,_=g.search(start,'python');b,p=g.search(start,'native')
        np.testing.assert_allclose(a,b,atol=1e-10)
        goal=tuple(np.argwhere(np.isfinite(b))[-1])
        path=g.path(p,start,goal)
        self.assertTrue(path)
        for first,second in zip(path,path[1:]):
            self.assertTrue(g.segment_safe(g.xy(first),g.xy(second)))
        g.safe[:]=False
        distances,parents=g.search(start,'native')
        self.assertFalse(np.isfinite(distances).any())
        self.assertEqual(g.path(parents,start,goal),[])

    @unittest.skipIf(SEARCH is None,'native library not built')
    def test_weighted_search_matches_python_and_preserves_reachability(self):
        g=ExplorationGrid([0,0,12,10]);g.safe[:]=True
        g.safe[20:30,25:35]=False
        start,goal=(25,10),(25,50)
        plain,_=g.search(start,'python')
        a,p=g.search(start,'python',clearance_weight=1.5)
        b,q=g.search(start,'native',clearance_weight=1.5)
        np.testing.assert_allclose(a,b,atol=1e-10)
        np.testing.assert_array_equal(np.isfinite(plain),np.isfinite(a))
        edge=np.zeros_like(g.safe);edge[19:31,24:36]=True
        edge[20:30,25:35]=False
        _,plain_parent=g.search(start,'python')
        plain_path=g.path(plain_parent,start,goal)
        weighted_path=g.path(q,start,goal)
        self.assertLess(sum(bool(edge[c]) for c in weighted_path),sum(bool(edge[c]) for c in plain_path))
        for first,second in zip(weighted_path,weighted_path[1:]):
            self.assertTrue(g.segment_safe(g.xy(first),g.xy(second)))

    def test_audit_complete_side_reference_and_missing_one_side(self):
        box=dict(name='shelf/test',center=[0,0,1.],yaw=.3,size=[2.,2.,2.])
        reference=side_samples(box,.3,.25,1.9)
        alignment=dict(map_from_world_rotation=np.eye(3).tolist(),map_from_world_translation=[6,4,0])
        report,missing=audit(dict(boxes=[box]),reference+np.array([6,4,0]),alignment)
        self.assertTrue(report['shelf_side_acceptance'])
        self.assertFalse(report['full_3d_map_complete'])
        self.assertEqual(len(missing),0)
        report,missing=audit(dict(boxes=[box]),reference[reference[:,0]<0]+np.array([6,4,0]),alignment)
        self.assertFalse(report['shelf_side_acceptance'])
        self.assertGreater(len(missing),0)

    def test_nn_tolerance_is_euclidean_not_voxel_cube(self):
        np.testing.assert_array_equal(supported(np.array([[0.,0.,0.],[1.,1.,1.]]),
            np.array([[.2,.2,.2],[1.,1.,1.]]),.25),[False,True])

    def test_pcd_roundtrip_and_empty_missing_file(self):
        with tempfile.TemporaryDirectory() as folder:
            for points in (np.array([[1,2,3]],dtype=float),np.empty((0,3))):
                path=Path(folder)/'fixture.pcd';write_xyz_pcd(path,points)
                np.testing.assert_array_equal(read_xyz_pcd(path),points)


if __name__=='__main__':
    unittest.main()
