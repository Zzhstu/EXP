#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid,expand


class GridTest(unittest.TestCase):
    def grid(self):
        return ExplorationGrid([0,0,12,10],.2,.8,.4)

    def test_unknown_never_traversable(self):
        g=self.grid()
        g.known[5:30,5:30]=True
        g.update_obstacles([])
        d,p=g.search((15,15))
        self.assertTrue(np.isfinite(d[15,20]))
        self.assertFalse(np.isfinite(d[15,40]))
        self.assertFalse(g.safe[5,5])  # body must not straddle unknown

    def test_current_occupancy_wins_and_clearing_reopens(self):
        g=self.grid()
        g.known[:]=True
        wall=[(5,y) for y in np.arange(0,10,.1)]
        g.update_obstacles(wall)
        d,_=g.search(g.cell((2,5)))
        self.assertFalse(np.isfinite(d[g.cell((8,5))]))
        g.update_obstacles([])  # maintained map's real deletion, no ghost cache
        d,p=g.search(g.cell((2,5)))
        self.assertTrue(np.isfinite(d[g.cell((8,5))]))
        self.assertTrue(g.path(p,g.cell((2,5)),g.cell((8,5))))

    def test_frontiers_only_reachable_safe(self):
        g=self.grid()
        g.known[2:40,2:35]=True
        g.update_obstacles([])
        d,_=g.search((15,15))
        c,gain=g.frontiers(d)
        self.assertEqual(c.shape,gain.shape)
        self.assertGreater(c.sum(),0)
        self.assertTrue(np.all(g.safe[c]))
        self.assertTrue(np.all(np.isfinite(d[c])))

    def test_no_diagonal_corner_cut(self):
        g=self.grid()
        g.safe[:]=True
        g.safe[10,11]=False
        self.assertFalse(g.segment_safe(g.xy((10,10)),g.xy((11,11))))

    def test_no_wrap_at_border(self):
        a=np.zeros((10,10),bool)
        a[0,0]=True
        self.assertFalse(expand(a,2)[-1,-1])

    def test_invalid_params(self):
        with self.assertRaises(ValueError):
            ExplorationGrid([0,0,20,20],.2,.1,.4)
        with self.assertRaises(ValueError):
            ExplorationGrid([0,0,10000,10000])

    def test_growth_preserves_world_cells_and_new_area_is_unknown(self):
        g=self.grid()
        g.known[10:20,10:20]=True
        old_xy=g.xy((15,15))
        old_shape=g.shape
        pads=g.grow_to_include([(-1.,5.)],margin=2.,chunk=2.)
        self.assertIsNotNone(pads)
        self.assertEqual(pads[2],20)  # enough padding for point plus 2 m margin
        self.assertEqual(g.shape[0],old_shape[0])
        shifted=(15+pads[0],15+pads[2])
        np.testing.assert_allclose(g.xy(shifted),old_xy,atol=1e-12)
        self.assertTrue(g.known[shifted])
        self.assertFalse(g.known[0,0])
        g.update_obstacles([])
        d,_=g.search(shifted)
        self.assertFalse(np.isfinite(d[0,0]))

    def test_growth_only_occurs_near_window_edge_and_keeps_origin(self):
        g=self.grid()
        bounds=(g.x0,g.y0,g.x1,g.y1)
        self.assertEqual(g.grow_to_include([(5.,5.)],margin=2.,chunk=2.),(0,0,0,0))
        self.assertEqual((g.x0,g.y0,g.x1,g.y1),bounds)
        pads=g.grow_to_include([(-1.,-1.)],margin=2.,chunk=2.)
        self.assertEqual(pads[0],20)
        self.assertEqual(pads[2],20)
        self.assertAlmostEqual(g.x0,-4.)
        self.assertAlmostEqual(g.y0,-4.)

    def test_growth_capacity_failure_is_atomic(self):
        g=ExplorationGrid([0,0,4,4],.2,.8,.4,max_cells=500)
        before=(g.shape,g.x0,g.y0,g.known.copy())
        self.assertIsNone(g.grow_to_include([(20.,20.)],margin=1.,chunk=8.))
        self.assertEqual(g.shape,before[0])
        self.assertEqual((g.x0,g.y0),before[1:3])
        np.testing.assert_array_equal(g.known,before[3])

    def test_reconnect_moves_away_without_erasing_inflation(self):
        g=self.grid()
        g.known[:]=True
        points=np.array([(5.75,y) for y in np.arange(0,10,.05)])
        g.update_obstacles(points)
        position=(5.,5.)
        self.assertFalse(g.safe[g.cell(position)])
        recovery=g.reconnect(position,points)
        self.assertIsNotNone(recovery)
        self.assertLess(recovery[0],position[0])
        self.assertLessEqual(np.linalg.norm(np.array(recovery)-position),.100001)
        self.assertFalse(g.safe[g.cell(position)])  # never force-clear start

    def test_reconnect_does_not_cross_unknown_or_ignore_close_obstacle(self):
        g=self.grid()
        points=np.array([(5.75,y) for y in np.arange(0,10,.05)])
        g.update_obstacles(points)
        self.assertIsNone(g.reconnect((5,5),points))
        g.known[:]=True
        g.update_obstacles(points)
        escape=g.reconnect((5.2,5),points)  # .55m: escape from level-2 band
        self.assertIsNotNone(escape)
        self.assertLess(escape[0],5.2)
        self.assertIsNone(g.reconnect((5.31,5),points))  # <0.45m: rotor envelope floor

    def test_reconnect_rejects_opposite_shelf_and_nonfinite_points(self):
        g=self.grid();g.known[:]=True
        left=np.array([(4.48,y) for y in np.arange(0,10,.05)])
        right=np.array([(5.50,y) for y in np.arange(0,10,.05)])
        points=np.vstack((left,right,[[np.nan,5.]]))
        g.update_obstacles(points)
        self.assertIsNone(g.reconnect((5.,5.),points))  # too narrow for .80m plan

    def test_recovery_horizon_stays_bounded_and_monotone(self):
        g=self.grid();g.known[:]=True
        points=np.array([(5.75,y) for y in np.arange(0,10,.05)])
        g.update_obstacles(points);before=g.safe.copy();position=np.array([5.,5.])
        recovery=np.asarray(g.reconnect(position,points,command_horizon=.20))
        self.assertLessEqual(np.linalg.norm(recovery-position),.200001)
        distances=[np.linalg.norm(points-(position+t*(recovery-position)),axis=1).min() for t in np.linspace(0,1,20)]
        self.assertTrue(np.all(np.diff(distances)>=-.005))
        self.assertGreaterEqual(min(distances),.70)
        np.testing.assert_array_equal(g.safe,before)
        with self.assertRaises(ValueError): g.reconnect(position,points,command_horizon=2.)

    def test_waypoint_segment_stays_clear(self):
        g=self.grid()
        g.known[:]=True
        g.update_obstacles([(5,y) for y in np.arange(0,6,.1)])
        start,goal=g.cell((2,3)),g.cell((8,3))
        d,p=g.search(start)
        path=g.path(p,start,goal)
        self.assertTrue(path)
        waypoint=g.waypoint(path,g.xy(start),.6)
        self.assertTrue(g.segment_safe(g.xy(start),waypoint))


if __name__=='__main__':
    unittest.main()
