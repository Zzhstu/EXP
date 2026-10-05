#!/usr/bin/env python3
"""Same-input search microbenchmark, NOT a Gazebo/flight/whole-system speed test."""
import argparse
import json
import sys
import time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from exploration_native import SEARCH


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=int,default=5)
    args=parser.parse_args()
    if SEARCH is None or not 1<=args.runs<=100:
        parser.error('Build the native search library first; runs must be in [1,100]')
    g=ExplorationGrid([0,0,50,30],.2,.8,.4)
    g.known[:]=True;g.update_obstacles([])
    start=g.cell((2.,2.))
    output=dict(scope='SEARCH ONLY; no sensors, flight, or physics; idle machine recommended',
                shape=list(g.shape),resolution_m=g.resolution,measurements=[])
    for weight in (0.,1.5):
        record=dict(clearance_weight=weight,samples_ms={})
        distances={}
        for backend in ('python','native'):
            samples=[]
            for _ in range(args.runs):
                began=time.perf_counter()
                distances[backend],_=g.search(start,backend,clearance_weight=weight)
                samples.append((time.perf_counter()-began)*1000.)
            record['samples_ms'][backend]=samples
        np.testing.assert_allclose(distances['python'],distances['native'],atol=1e-10)
        record['median_ms']={k:float(np.median(v)) for k,v in record['samples_ms'].items()}
        record['median_speedup']=record['median_ms']['python']/record['median_ms']['native']
        record['reached_cells']=int(np.isfinite(distances['native']).sum())
        output['measurements'].append(record)
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    main()
