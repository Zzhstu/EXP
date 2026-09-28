#!/usr/bin/env python3
"""Plot warehouse collision footprints and the RECORDED world-frame paths."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix',help='dynamic_demo_runs/ours_cangku_TIMESTAMP')
    args=parser.parse_args()
    meta=json.loads(Path(args.prefix+'_scene.json').read_text())
    tracks={'iris_mid360': [], '0': []}
    last=-1e9
    with rosbag.Bag(args.prefix+'.bag') as bag:
        for _,msg,stamp in bag.read_messages(topics=['/gazebo/model_states']):
            if stamp.to_sec()-last<.1:
                continue
            last=stamp.to_sec()
            for name in tracks:
                if name in msg.name:
                    p=msg.pose[msg.name.index(name)].position
                    tracks[name].append([p.x,p.y])
    fig,axes=plt.subplots(1,2,figsize=(13,7),gridspec_kw={'width_ratios':[1.8,1]})
    for ax in axes:
        for box in meta['boxes']:
            sx,sy=box['size'][:2]
            corners=np.array([[-sx,-sy],[sx,-sy],[sx,sy],[-sx,sy]])/2
            c,s=np.cos(box['yaw']),np.sin(box['yaw'])
            corners=corners.dot(np.array([[c,s],[-s,c]]))+box['center'][:2]
            ax.add_patch(Polygon(corners,facecolor='#bcc5ce',edgecolor='#5d6975',linewidth=.5))
        for name,color,label in [('iris_mid360','#137ab9','UAV truth'),('0','#cc543e','Pedestrian truth')]:
            if tracks[name]:
                points=np.array(tracks[name])
                ax.plot(points[:,0],points[:,1],color=color,label=label,linewidth=1.8)
                ax.scatter(*points[0],color=color,marker='o',s=25)
                ax.scatter(*points[-1],color=color,marker='x',s=40)
        ax.scatter(-6,6,marker='*',s=100,color='#298145',label='Nominal goal')
        ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_xlabel('Gazebo world X [m]');ax.set_ylabel('Gazebo world Y [m]')
    axes[0].set(xlim=(-11,42),ylim=(-17,12),title='Original cangku geometry: 52 shelves')
    axes[1].set(xlim=(-10,-2),ylim=(-5,8),title='Tested left aisle (not the whole warehouse)')
    axes[1].legend(fontsize=8,loc='lower left')
    fig.suptitle('Warehouse test: recorded trajectories, no ground-truth map given to the UAV')
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(args.prefix+'_warehouse_layout.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    main()
