#!/usr/bin/env python3
"""OFFLINE collision-box side-surface coverage audit. Never publishes goals.

Geometry is only an evaluation reference, not an input to the black-box
explorer. Counts exposed shelf side surfaces in a declared height interval;
not visual-mesh interiors, floor, ceiling, 3-D navigability or MOS accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def read_xyz_pcd(path):
    with Path(path).open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Missing PCD DATA header')
            fields = line.decode('ascii').strip().split()
            if fields and not fields[0].startswith('#'):
                header[fields[0]] = fields[1:]
            if fields and fields[0]=='DATA':
                break
        if (header.get('FIELDS')!=['x','y','z'] or header.get('SIZE')!=['4','4','4'] or
                header.get('TYPE')!=['F','F','F'] or header.get('COUNT')!=['1','1','1']):
            raise ValueError('Audit expects finite XYZ-only PCD from save_navigation_map')
        count = int(header['POINTS'][0])
        if header['DATA']==['binary']:
            data = stream.read()
            if len(data)!=count*12:
                raise ValueError('PCD payload length mismatch')
            points = np.frombuffer(data,dtype='<f4').reshape(count,3)
        elif header['DATA']==['ascii']:
            points = np.loadtxt(stream,dtype=np.float32).reshape(-1,3)
            if len(points)!=count:
                raise ValueError('PCD point count mismatch')
        else:
            raise ValueError('Compressed PCD is not supported by this audit')
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite map points')
        return points


def write_xyz_pcd(path, points):
    points = np.asarray(points,dtype='<f4').reshape(-1,3)
    header = ('# .PCD v0.7 - offline missing reference samples, NOT UAV map\n'
              'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\nDATA binary\n').format(len(points))
    with Path(path).open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())


def box_local(points, box):
    d = points-np.asarray(box['center'])
    c,s = math.cos(box['yaw']),math.sin(box['yaw'])
    return np.column_stack((c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1],d[:,2]))


def side_samples(box, step, lower, upper):
    half = np.asarray(box['size'])/2
    lo,hi = max(lower-box['center'][2],-half[2]),min(upper-box['center'][2],half[2])
    if hi<=lo:
        return np.empty((0,3))
    z = np.linspace(lo,hi,max(2,int(math.ceil((hi-lo)/step))+1))
    faces = []
    for axis in (0,1):
        other = 1-axis
        span = np.linspace(-half[other],half[other],max(2,int(math.ceil(2*half[other]/step))+1))
        v,zz = np.meshgrid(span,z)
        for sign in (-1,1):
            points = np.zeros((v.size,3))
            points[:,axis]=sign*half[axis]
            points[:,other]=v.ravel()
            points[:,2]=zz.ravel()
            faces.append(points)
    local = np.unique(np.vstack(faces),axis=0)
    c,s = math.cos(box['yaw']),math.sin(box['yaw'])
    result = np.column_stack((c*local[:,0]-s*local[:,1],s*local[:,0]+c*local[:,1],local[:,2]))
    return result+np.asarray(box['center'])


def supported(reference, points, tolerance):
    """Exact Euclidean NN-within-tolerance via bounded voxel buckets; no scipy."""
    buckets = {}
    for key,point in zip(np.floor(points/tolerance).astype(np.int64),points):
        buckets.setdefault(tuple(key),[]).append(point)
    buckets = {k:np.asarray(v) for k,v in buckets.items()}
    mask = np.zeros(len(reference),dtype=bool)
    tol2 = tolerance*tolerance
    for i,point in enumerate(reference):
        key = np.floor(point/tolerance).astype(np.int64)
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for dz in (-1,0,1):
                    candidates = buckets.get(tuple(key+(dx,dy,dz)))
                    if candidates is not None and np.min(np.sum((candidates-point)**2,axis=1))<=tol2:
                        mask[i]=True
                        break
                if mask[i]: break
            if mask[i]: break
    return mask


def audit(scene, points, alignment, step=.3, tolerance=.25, lower=.25, upper=1.9):
    if min(step,tolerance)<=0 or lower>=upper:
        raise ValueError('Invalid sampling/tolerance/height range')
    rotation = np.asarray(alignment['map_from_world_rotation'],dtype=float)
    translation = np.asarray(alignment['map_from_world_translation'],dtype=float)
    if (rotation.shape!=(3,3) or translation.shape!=(3,) or
            not np.isfinite(rotation).all() or not np.isfinite(translation).all() or
            not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5) or np.linalg.det(rotation)<.99):
        raise ValueError('Invalid rigid alignment')
    world = (points-translation)@rotation
    boxes = scene['boxes']
    racks,missing,refs = [],[],[]
    sizes = []
    for box in boxes:
        if 'shelf' not in box['name']:
            continue
        reference = side_samples(box,step,lower,upper)
        keep = np.ones(len(reference),dtype=bool)
        for other in boxes:
            if other is box:
                continue
            # Exclude surfaces buried inside a different solid collision box.
            keep &= ~(np.abs(box_local(reference,other)) < np.asarray(other['size'])/2-1e-5).all(axis=1)
        reference = reference[keep]
        refs.append(reference); sizes.append((box,len(reference)))
    combined = np.vstack(refs) if refs else np.empty((0,3))
    match = supported(combined,world,tolerance)
    offset = 0
    for box,count in sizes:
        kept = int(match[offset:offset+count].sum())
        racks.append(dict(name=box['name'],center_world=box['center'],reference_samples=count,
                          supported_samples=kept,coverage=kept/count if count else None))
        offset += count
    missing_world = combined[~match]
    missing_map = missing_world@rotation.T+translation
    ratios = [v['coverage'] for v in racks if v['coverage'] is not None]
    total = len(combined); matched = int(match.sum())
    report = dict(scope='exposed collision-box SHELF SIDE surfaces in declared world height band; '
                       'not full visual mesh, unreachable/occluded sides stay in denominator',
                  sampling_step_m=step,euclidean_tolerance_m=tolerance,height_world=[lower,upper],
                  alignment_note=alignment.get('note','User-supplied fixed rigid world/map alignment; no drift correction'),
                  reference_samples=total,supported_samples=matched,
                  shelf_side_coverage=matched/total if total else None,
                  minimum_rack_coverage=min(ratios) if ratios else None,
                  acceptance_total_threshold=.95,acceptance_each_rack_threshold=.90,
                  shelf_side_acceptance=bool(total and matched/total>=.95 and min(ratios)>=.90),
                  full_3d_map_complete=False,racks=racks)
    return report,missing_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--pcd',required=True)
    parser.add_argument('--alignment',required=True)
    parser.add_argument('--output',required=True,help='output JSON; also writes _missing.pcd')
    args = parser.parse_args()
    points = read_xyz_pcd(args.pcd)
    report,missing = audit(json.loads(Path(args.scene).read_text()),points,
                         json.loads(Path(args.alignment).read_text()))
    report.update(pcd=str(Path(args.pcd).resolve()),scene=str(Path(args.scene).resolve()),
                  map_points=len(points))
    Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf8')
    write_xyz_pcd(Path(args.output).with_suffix('').as_posix()+'_missing.pcd',missing)
    print(json.dumps({k:v for k,v in report.items() if k!='racks'},indent=2))


if __name__=='__main__':
    main()
