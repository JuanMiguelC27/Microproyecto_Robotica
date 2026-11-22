import hashlib
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import gecko_joints as gj
from generate_gecko_models import generate

SOURCE = Path(__file__).resolve().parents[4] / 'Micro2_URDFV2/urdf/Micro2_URDFV2.urdf'


def models(tmp_path):
    return generate(SOURCE, tmp_path)


def test_source_is_not_modified(tmp_path):
    before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    models(tmp_path)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == before


def test_rviz_meshes_are_existing_file_uris(tmp_path):
    root = ET.parse(models(tmp_path)['rviz']).getroot()
    names = [m.get('filename') for m in root.iter('mesh')]
    visuals = root.findall('./link/visual')
    assert len(visuals) == 9
    assert all(n.startswith('file://') and Path(n[7:]).is_file() for n in names)


def test_gazebo_has_no_collisions_and_no_gravity(tmp_path):
    root = ET.parse(models(tmp_path)['gazebo']).getroot()
    assert list(root.iter('collision')) == []
    refs = {g.get('reference') for g in root.findall('gazebo')
            if g.find('gravity') is not None}
    assert refs == {link.get('name') for link in root.findall('link')}


def test_gazebo_plugins(tmp_path):
    root = ET.parse(models(tmp_path)['gazebo']).getroot()
    names = [p.get('name') for p in root.iter('plugin')]
    assert names.count('ignition::gazebo::systems::JointController') == 6
    assert names.count('ignition::gazebo::systems::JointPositionController') == 2
    assert names.count('ignition::gazebo::systems::JointStatePublisher') == 1


def test_gazebo_model_converts_to_sdf(tmp_path):
    out = subprocess.run(['ign', 'sdf', '-p', str(models(tmp_path)['gazebo'])],
                         capture_output=True, text=True, check=True).stdout
    assert out.count('<joint name=') == 8
    assert out.count('<collision') == 0


def test_rsp_params_contain_description(tmp_path):
    data = yaml.safe_load(models(tmp_path)['rsp_params'].read_text())
    description = data['robot_state_publisher']['ros__parameters']['robot_description']
    assert '<robot' in description


def test_bridge_args():
    args = gj.bridge_args()
    assert len(args) == 8
    assert all(a.endswith('@std_msgs/msg/Float64]ignition.msgs.Double') for a in args)
