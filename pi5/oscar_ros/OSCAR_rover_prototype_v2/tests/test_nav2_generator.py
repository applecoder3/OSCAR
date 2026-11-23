import unittest
from pathlib import Path
import subprocess
import sys
import tempfile

try:
    import yaml
except ImportError:
    yaml=None

@unittest.skipIf(yaml is None, 'PyYAML unavailable outside ROS container')
class Params(unittest.TestCase):
    def test_config_edit(self):
        sample={'global_costmap':{'global_costmap':{'ros__parameters':{'obstacle_layer':{'observation_sources':'old'}}}},
                'local_costmap':{'local_costmap':{'ros__parameters':{'obstacle_layer':{'observation_sources':'old'}}}},
                'controller_server':{'ros__parameters':{'controller_plugins':['FollowPath'], 'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner'}}}}
        with tempfile.TemporaryDirectory() as d:
            a=Path(d)/'a.yaml'; b=Path(d)/'b.yaml'
            a.write_text(yaml.safe_dump(sample))
            script=Path(__file__).resolve().parents[1]/'scripts/prepare_nav2.py'
            subprocess.run([sys.executable,str(script),str(a),str(b)],check=True)
            res=yaml.safe_load(b.read_text())
            self.assertEqual(res['controller_server']['ros__parameters']['FollowPath']['max_vel_y'],0.0)
            self.assertEqual(res['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'],'/scan')

if __name__ == '__main__': unittest.main()
