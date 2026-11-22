from pathlib import Path

import yaml

CONFIG = yaml.safe_load(
    (Path(__file__).resolve().parents[2] / 'rviz/gecko.rviz').read_text()
)


def test_fixed_frame():
    assert CONFIG['Visualization Manager']['Global Options']['Fixed Frame'] == 'world'


def test_panel_is_loaded():
    assert 'gecko_rviz_panel/GeckoPanel' in [p['Class'] for p in CONFIG['Panels']]


def test_displays():
    displays = {d['Class']: d for d in CONFIG['Visualization Manager']['Displays']}
    model = displays['rviz_default_plugins/RobotModel']
    assert model['Description Topic']['Value'] == '/robot_description'
    assert displays['rviz_default_plugins/Marker']['Topic']['Value'] == '/gecko/wall/marker'
    assert 'rviz_default_plugins/Grid' in displays
