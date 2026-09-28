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
        self.assertIsNone(g.reconnect((5.2,5),points))  # .55m: reactive guard owns it

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
