#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_scene import prepare, blocked, blocked_segment


class WarehouseSceneTest(unittest.TestCase):
    def test_generated_world_preserves_source_and_checks_obstacles(self):
        root = Path(__file__).resolve().parents[5]
        source = root/'simulation/astra_gazebo_worlds/cangku.world'
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as temp:
            world_file, scene_file = prepare(root, Path(temp)/'test')
            world = ET.parse(world_file).getroot().find('world')
            self.assertIsNone(world.find('state'))
            self.assertEqual([p.findtext('name') for p in world.findall('include')], ['0'])
            self.assertFalse(any(u.text.startswith('../') for u in world.findall('.//uri')))
            boxes = json.loads(scene_file.read_text())['boxes']
            self.assertFalse(blocked(-6, -4, boxes, .95))
            self.assertFalse(blocked_segment(-8, 3, -4.2, 3, boxes))
            self.assertTrue(blocked_segment(-8, 3, -12, 3, boxes))
            shelf = next(b for b in boxes if 'shelf' in b['name'])
            self.assertTrue(blocked(*shelf['center'][:2], boxes))
        self.assertEqual(digest, hashlib.sha256(source.read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()
