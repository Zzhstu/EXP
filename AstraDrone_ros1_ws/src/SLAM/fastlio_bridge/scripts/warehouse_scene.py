#!/usr/bin/env python3
"""Prepare a reproducible cangku copy; original world is never overwritten.

Only scene/control infrastructure uses these collision boxes. They are NOT
given to the UAV detector, mapper or planner as privileged ground truth.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def pose(element):
    return [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]


def compose(a, b):
    if any(abs(v) > 1e-5 for v in (a[3], a[4], b[3], b[4])):
        raise ValueError('Warehouse footprint extraction requires upright boxes')
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2], 0, 0, a[5]+b[5]]


def collision_boxes(world):
    boxes = []
    for model in world.findall('model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None:
                    continue
                size = [float(v) for v in size.split()]
                p = compose(compose(pose(model), pose(link)), pose(collision))
                # Exclude the roof/ground, retain surfaces at human/UAV height.
                if p[2]+size[2]/2 < .1 or p[2]-size[2]/2 > 1.9:
                    continue
                boxes.append(dict(name=model.get('name')+'/'+link.get('name'),
                                  center=p[:3], yaw=p[5], size=size))
    return boxes


def blocked(x, y, boxes, radius=.35):
    for box in boxes:
        dx, dy = x-box['center'][0], y-box['center'][1]
        c, s = math.cos(box['yaw']), math.sin(box['yaw'])
        local_x, local_y = c*dx+s*dy, -s*dx+c*dy
        ex = max(abs(local_x)-box['size'][0]/2, 0)
        ey = max(abs(local_y)-box['size'][1]/2, 0)
        if ex*ex+ey*ey <= radius*radius:
            return True
    return not (-9.5 < x < 40.3 and -15.7 < y < 10.7)


def blocked_segment(x0, y0, x1, y1, boxes, radius=.35):
    steps = max(1, int(math.ceil(math.hypot(x1-x0, y1-y0)/.05)))
    return any(blocked(x0+(x1-x0)*i/steps, y0+(y1-y0)*i/steps, boxes, radius)
               for i in range(steps+1))


def prepare(root, prefix):
    source = Path(root)/'simulation/astra_gazebo_worlds/cangku.world'
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    # The editor snapshot has different poses/heights and a 1477s clock.
    # Use explicit top-level model definitions consistently from t=0.
    for state in world.findall('state'):
        world.remove(state)
    for uri in world.findall('.//uri'):
        if uri.text.startswith('../'):
            target = (source.parent/uri.text).resolve()
            if not target.is_file():
                raise FileNotFoundError(str(target))
            uri.text = target.as_uri()
    boxes = collision_boxes(world)
    # Left main aisle avoids the rack footprints without shrinking UAV safety.
    for x, y in ((-6, -4), (-6, 6), (-8, 3)):
        if blocked(x, y, boxes, .95 if x == -6 else .35):
            raise ValueError('Scene changed: spawn/goal is obstructed')
    person = ET.SubElement(world, 'include')
    ET.SubElement(person, 'uri').text = 'model://person_standing'
    ET.SubElement(person, 'name').text = '0'
    ET.SubElement(person, 'pose').text = '-8 3 0 0 0 0'
    camera = world.find('gui/camera/pose')
    if camera is not None:
        camera.text = '-17 -15 16 0 0.7 0.8'
    output = Path(str(prefix)+'_cangku.world')
    tree.write(str(output), encoding='utf-8', xml_declaration=True)
    metadata = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    generated_world=str(output), removed_saved_state=True,
                    spawn_world=[-6, -4, .06], relative_goal=[0, 10, 0],
                    pedestrian_world=[-8, 3, 0], boxes=boxes,
                    note='Original geometry preserved; saved simulation state removed and mesh URIs resolved. '
                         'Human collision guard only; boxes are not provided to UAV navigation.')
    geometry = Path(str(prefix)+'_scene.json')
    geometry.write_text(json.dumps(metadata, indent=2), encoding='utf8')
    return output, geometry
