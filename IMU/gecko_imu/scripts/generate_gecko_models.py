#!/usr/bin/env python3
"""Generate the RViz and Gazebo models of the Gecko from the exported URDF.

The exported URDF is never modified. Its package:// mesh paths belong to a
ROS 1 package that RViz 2 and Gazebo cannot resolve, so absolute paths are
written to generated copies instead.
"""

import argparse
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

from gecko_joints import (
    SERVO_JOINTS,
    VELOCITY_JOINTS,
    bridge_args,
    position_topic,
    velocity_topic,
)

PLUGIN_PREFIX = 'ignition::gazebo::systems::'


def _resolve_meshes(root: ET.Element, mesh_dir: Path, scheme: str) -> None:
    for mesh in root.iter('mesh'):
        name = Path(mesh.get('filename', '')).name
        mesh.set('filename', f'{scheme}{(mesh_dir / name).resolve()}')


def _add_plugin(parent: ET.Element, library: str, system: str,
                params: Dict[str, str]) -> None:
    plugin = ET.SubElement(parent, 'plugin', {
        'filename': f'ignition-gazebo-{library}-system',
        'name': f'{PLUGIN_PREFIX}{system}',
    })
    for key, value in params.items():
        ET.SubElement(plugin, key).text = value


def _to_gazebo(root: ET.Element) -> None:
    # Contact is resolved by the twin node, so physics must not act on the body.
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        extension = ET.SubElement(root, 'gazebo', {'reference': link.get('name')})
        ET.SubElement(extension, 'gravity').text = 'false'

    plugins = ET.SubElement(root, 'gazebo')
    for joint in VELOCITY_JOINTS:
        _add_plugin(plugins, 'joint-controller', 'JointController', {
            'joint_name': joint,
            'topic': velocity_topic(joint),
        })
    for joint in SERVO_JOINTS:
        _add_plugin(plugins, 'joint-position-controller', 'JointPositionController', {
            'joint_name': joint,
            'topic': position_topic(joint),
            'use_velocity_commands': 'true',
            'p_gain': '10',
            'i_gain': '0',
            'd_gain': '0',
            'cmd_max': '5',
            'cmd_min': '-5',
        })
    _add_plugin(plugins, 'joint-state-publisher', 'JointStatePublisher', {})


def generate(source_urdf: Path, out_dir: Path) -> Dict[str, Path]:
    """Write both models and the robot_state_publisher parameters."""
    source_urdf = Path(source_urdf)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = source_urdf.resolve().parent.parent / 'meshes'
    source = ET.parse(source_urdf).getroot()

    rviz = copy.deepcopy(source)
    _resolve_meshes(rviz, mesh_dir, 'file://')
    rviz_path = out_dir / 'gecko_rviz.urdf'
    ET.ElementTree(rviz).write(rviz_path, encoding='utf-8', xml_declaration=True)

    gazebo = copy.deepcopy(source)
    _resolve_meshes(gazebo, mesh_dir, '')
    _to_gazebo(gazebo)
    gazebo_path = out_dir / 'gecko_gazebo.urdf'
    ET.ElementTree(gazebo).write(gazebo_path, encoding='utf-8', xml_declaration=True)

    # A JSON string is valid YAML and keeps the XML safely quoted.
    params_path = out_dir / 'robot_state_publisher.yaml'
    description = json.dumps(rviz_path.read_text(encoding='utf-8'))
    params_path.write_text(
        'robot_state_publisher:\n'
        '  ros__parameters:\n'
        f'    robot_description: {description}\n',
        encoding='utf-8',
    )
    return {'rviz': rviz_path, 'gazebo': gazebo_path, 'rsp_params': params_path}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--print-bridge-args', action='store_true')
    args = parser.parse_args()

    if args.print_bridge_args:
        print('\n'.join(bridge_args()))
        return
    if args.source is None or args.out is None:
        parser.error('--source and --out are required')
    for path in generate(args.source, args.out).values():
        print(path)


if __name__ == '__main__':
    main()
