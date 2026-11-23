"""Temporary MoveIt obstacles from a stationary, measured depth snapshot."""
import json
import csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from ibvs.depth import observed_voxels, scene_near_depth, validate_frozen_target


class ObservedScene:
    def __init__(self, experiment):
        from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
        self.e = experiment
        self.identifier = 'ibvs_observed_' + experiment.output.name
        self.applied = False
        self.apply = experiment.r.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = experiment.r.create_client(GetStateValidity, '/check_state_validity')
        if not self.apply.wait_for_service(timeout_sec=5) or not self.validity.wait_for_service(timeout_sec=5):
            raise RuntimeError('observed-scene collision services unavailable')

    def update(self, remove=False, centers=None):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id, obj.id = 'base_link', self.identifier
        obj.pose.orientation.w = 1.
        obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            for center in centers:
                box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.02, .02, .02])
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, center)
                pose.orientation.w = 1.
                obj.primitives.append(box)
                obj.primitive_poses.append(pose)
        request.scene.world.collision_objects = [obj]
        if not self.e.wait(self.apply.call_async(request)).success:
            raise RuntimeError('MoveIt rejected observed-scene update')
        self.applied = not remove

    def install(self, target_depth, resume_run=None):
        observation = self.e.last_observation
        T = self.e.fk(self.e.r.q) @ self.e.T_EC
        # Robot fingers are in the near field. Freeze only surfaces beyond them,
        # then retain those same obstacles throughout the entire approach.
        tcp_in_camera = np.linalg.inv(self.e.T_EC) @ self.e.T_tcp
        near = scene_near_depth(self.e.c, self.e.T_EC, self.e.T_tcp)
        separation = .02 if 'scene_min_depth_m' in self.e.c else .04
        info = dict(voxel_m=.01, collision_box_m=.02, min_snapshot_depth_m=near,
                    camera_to_tcp_m=tcp_in_camera[:3, 3].tolist(),
                    limitation='Observed surfaces only; existing hand-eye estimate, no unseen geometry.')
        if resume_run is None:
            if target_depth <= near+separation:
                raise RuntimeError('target too close for a new scene; use --resume-run with the earlier verified snapshot')
            filtered = self.e.scene_depth(observation['depth_m'],observation['color'])
            centers = observed_voxels(filtered, observation['K'], observation['distortion'],
                                      T, near, self.e.c['depth_max_m'])
            snapshot = dict(centers_base_m=centers, camera_in_base=T,
                            snapshot_joints_deg=np.degrees(self.e.r.q), snapshot_corners_px=observation['corners_px'],
                            snapshot_depths_m=np.full(4, target_depth), K=observation['K'], distortion=observation['distortion'])
        else:
            source = Path(resume_run).expanduser().resolve()
            summary = json.loads((source/'summary.json').read_text())
            if summary.get('preflight', {}).get('status') != 'PREDICTED_REACHABLE' or summary.get('steps', 0) < 1:
                raise RuntimeError('resume source did not pass its reach check and execute a servo step')
            source_config = json.loads((source/'config.json').read_text())
            source_reference = source_config['reference']
            for key in ('camera_frame_id', 'image_size_px', 'camera_topic_prefix', 'feature_type'):
                if source_reference.get(key) != self.e.reference.get(key):
                    raise RuntimeError('resume source uses a different camera or feature type')
            info['goal_changed'] = source_reference['corners_image_px'] != self.e.reference['corners_image_px']
            snapshot = dict(np.load(source/'clearance_scene.npz'))
            if 'snapshot_joints_deg' not in snapshot:
                initial = np.load(source/'initial_observation.npz')
                with (source/'trial.csv').open() as record:
                    first = next(csv.DictReader(record))
                snapshot.update(snapshot_joints_deg=np.asarray(json.loads(first['joints_deg'])),
                                snapshot_corners_px=initial['corners_px'], snapshot_depths_m=np.asarray(json.loads(first['depths_m'])),
                                K=initial['K'], distortion=initial['distortion'])
            if (not np.allclose(snapshot['K'], observation['K'], atol=1e-6, rtol=0)
                    or not np.allclose(snapshot['distortion'], observation['distortion'], atol=1e-6, rtol=0)):
                raise RuntimeError('camera calibration differs from frozen scene')
            expected = self.e.fk(np.radians(snapshot['snapshot_joints_deg'])) @ self.e.T_EC
            offset = np.linalg.inv(expected) @ snapshot['camera_in_base']
            if np.linalg.norm(offset[:3, 3]) > .002 or Rotation.from_matrix(offset[:3, :3]).magnitude() > np.radians(.2):
                raise RuntimeError('robot model or camera mounting differs from frozen scene')
            validation = validate_frozen_target(snapshot, T, observation['corners_px'], target_depth)
            info.update(resumed_from=str(source), target_validation=validation)
        centers = snapshot['centers_base_m']
        info['points'] = len(centers)
        np.savez_compressed(self.e.output/'clearance_scene.npz', **snapshot)
        (self.e.output/'clearance_scene.json').write_text(json.dumps(info, indent=2)+'\n')
        self.update(centers=centers)
        self.check(self.e.r.q)

    def check(self, q):
        if self.e.skip_checks:
            return
        from moveit_msgs.srv import GetStateValidity
        request = GetStateValidity.Request()
        request.robot_state = self.e.state(q)
        # Empty group checks the complete robot, including the open gripper.
        request.group_name = ''
        response = self.e.wait(self.validity.call_async(request))
        if not response.valid:
            contacts = [(item.contact_body_1, item.contact_body_2) for item in response.contacts]
            raise RuntimeError('observed-scene collision or invalid state: '+str(contacts))

    def close(self):
        if self.applied:
            self.update(remove=True)
