"""Forward kinematics from the same URDF used by MoveIt; no fitted mapping."""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


class UrdfChain:
    def __init__(self, description, base, tip, joint_names):
        root = ET.fromstring(description)
        parents = {joint.find('child').attrib['link']: joint for joint in root.findall('joint')}
        chain, link, seen = [], tip, set()
        while link != base:
            if link in seen or link not in parents:
                raise ValueError('URDF has no unique chain from base to tool')
            seen.add(link)
            joint = parents[link]
            kind, name = joint.attrib['type'], joint.attrib['name']
            if kind not in ('fixed', 'revolute', 'continuous') or (kind != 'fixed' and name not in joint_names):
                raise ValueError('unsupported joint in tool chain: '+name)
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.,0.,0.]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0.,0.,0.]
            transform = np.eye(4)
            transform[:3,3], transform[:3,:3] = xyz, Rotation.from_euler('xyz',rpy).as_matrix()
            axis_node = joint.find('axis')
            axis = np.array([float(v) for v in axis_node.get('xyz','1 0 0').split()]) if axis_node is not None else np.array([1.,0.,0.])
            if kind == 'fixed':
                axis = np.array([1.,0.,0.])
            if not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
                raise ValueError('invalid URDF joint axis')
            chain.append((transform, None if kind == 'fixed' else joint_names.index(name), axis/np.linalg.norm(axis)))
            link = joint.find('parent').attrib['link']
        self.chain, self.count = list(reversed(chain)), len(joint_names)

    def __call__(self, q):
        q = np.asarray(q,float)
        if q.shape != (self.count,) or not np.isfinite(q).all():
            raise ValueError('invalid joints for URDF forward kinematics')
        result = np.eye(4)
        for origin,index,axis in self.chain:
            result = result @ origin
            if index is not None:
                rotation = np.eye(4)
                rotation[:3,:3] = Rotation.from_rotvec(axis*q[index]).as_matrix()
                result = result @ rotation
        return result
