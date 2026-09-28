#!/usr/bin/env python3
"""Export EXACT maintained navigation topic as finite XYZ binary PCD + provenance.

Does not claim unseen/occluded ghosts are removed and does not crop actors using
Gazebo truth. A currently stationary person is legitimately occupied space.
"""
import hashlib
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Trigger, TriggerResponse


def write_snapshot(cloud, output, topic):
    rows = list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True))
    points = np.asarray(rows, dtype='<f4').reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('地图为空，拒绝覆盖上一次有效地图')
    path = Path(output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# .PCD v0.7 - navigation map snapshot\nVERSION 0.7\n'
              'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (len(points), len(points))
    payload = header.encode('ascii') + points.tobytes()
    metadata = dict(topic=topic, frame_id=cloud.header.frame_id,
                    stamp=cloud.header.stamp.to_sec(), points=len(points),
                    sha256=hashlib.sha256(payload).hexdigest(),
                    fields=['x', 'y', 'z'],
                    note='Exact finite XYZ from maintained RViz map; no extra ghost removal at export. '
                         'Unobserved/occluded old occupancy may remain; stationary people are obstacles.')
    # Write in the same filesystem then replace atomically; never truncate a
    # valid map on interruption. JSON checksum detects a split pair after crash.
    for target, data in ((path, payload), (path.with_suffix('.json'),
                         json.dumps(metadata, ensure_ascii=False, indent=2).encode('utf8'))):
        tmp = target.with_name(target.name + '.tmp')
        with tmp.open('wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(tmp), str(target))
    return metadata


class MapSaver:
    def __init__(self):
        self.lock = threading.Lock()
        self.save_lock = threading.Lock()
        self.cloud = None
        self.map_received = self.scan_received = 0.0
        self.topic = rospy.get_param('~topic', '/uav1/fastlio/cloud_map')
        self.output = rospy.get_param('~output')
        rospy.Subscriber(self.topic, PointCloud2, self.receive, queue_size=1)
        rospy.Subscriber('/uav1/fastlio/registered_scan', PointCloud2, self.scan, queue_size=1)
        self.service = rospy.Service('/demo/save_map', Trigger, self.save)

    def receive(self, msg):
        with self.lock:
            self.cloud, self.map_received = msg, time.monotonic()

    def scan(self, msg):
        self.scan_received = time.monotonic()

    def save(self, req):
        with self.save_lock:
            with self.lock:
                cloud, received = self.cloud, self.map_received
            now = time.monotonic()
            if cloud is None or now-received > 3 or now-self.scan_received > 3:
                return TriggerResponse(False, '地图或注册点云停止更新超过3秒；拒绝把旧数据称为最终地图')
            try:
                meta = write_snapshot(cloud, self.output, self.topic)
                return TriggerResponse(True, '%s (%d XYZ points, frame=%s)' % (
                    self.output, meta['points'], meta['frame_id']))
            except Exception as exc:
                rospy.logerr('Map export failed: %s', exc)
                return TriggerResponse(False, str(exc))


if __name__ == '__main__':
    rospy.init_node('navigation_map_saver')
    saver = MapSaver()
    rospy.spin()
