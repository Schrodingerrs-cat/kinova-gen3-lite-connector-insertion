#!/usr/bin/env python3
"""Undamped center/size IBVS approach, followed by four-corner alignment."""
import argparse
from datetime import datetime, timezone
import json
import os
import shutil
import signal
import time
from types import SimpleNamespace

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from alignment_test import Experiment, ROOT, PROJECT, load_inputs
from ibvs.alignment import camera_joint_jacobian, within_limits
from ibvs.control import Convergence
from ibvs.rectangles import TrackingLost
from ibvs.shape_control import shape_ibvs_step, shape_metrics, task_cost, project_motion, STEP_SCALES
from observed_scene import ObservedScene


def choose_step(e, q, xy, Z, task, previous_command=None):
    """An undamped task inverse; smaller steps remain available at any error."""
    K, D = np.asarray(e.camera['K'], float), np.asarray(e.camera['distortion'], float)
    focal = np.diag(K)[:2]
    desired = e.tracker.star[0]
    target_px = e.tracker.target_px[0]
    Jcam = camera_joint_jacobian(e.fk, q, e.T_EC)
    limits = np.radians([1., 5., 1., 1., 1., 1.] if task == 'center_size' else [.25, 1., .25, .25, .25, .25])
    dq = shape_ibvs_step(xy, desired, Z, Jcam, e.c['gain'], task,
                         [1., 4., 1., 1., 1., 1.], limits,
                         e.c['max_step_m'], np.radians(e.c['max_step_deg']))
    T0 = e.fk(q) @ e.T_EC
    original = task_cost(xy, desired, focal, task)
    current_px, _ = cv2.projectPoints(np.column_stack([xy, np.ones(4)]), np.zeros(3), np.zeros(3), K, D)
    center = shape_metrics(current_px.reshape(4, 2), target_px)['center_error_px']
    candidates = []
    for scale in STEP_SCALES:
        increment = dq*scale
        # Predict the new IBVS increment; retain the integrated actuator reference.
        target = (q if previous_command is None else previous_command)+increment
        T = e.fk(q+increment) @ e.T_EC
        try:
            pixels = project_motion(xy, Z, np.linalg.inv(T) @ T0, K, D)
        except ValueError:
            continue
        if (not np.isfinite(pixels).all() or np.any(pixels < 3)
                or np.any(pixels >= np.asarray(e.reference['image_size_px'])-3)):
            continue
        metrics = shape_metrics(pixels, target_px)
        if task == 'center_size' and metrics['center_error_px'] > max(5., center+1.):
            continue
        cost = task_cost(e.tracker.normalize(pixels), desired, focal, task)
        if cost < original-1e-8:
            candidates.append((cost, target, increment, metrics, scale))
    if not candidates:
        raise RuntimeError('undamped IBVS has no improving bounded step, including fine resolution')
    return min(candidates, key=lambda item: item[0])


def model_residual(metrics, step):
    print('Sampled path clear to model residual %.2f px; measured IBVS must still meet 5 px.' % metrics['max_corner_px'],flush=True)
    return dict(status='MODEL_RESIDUAL',steps=step,**metrics,
                note='Approximate-depth rollout stopped within one pixel of tolerance; not physical convergence. Live steps remain collision checked.')


def reach_check(e, scene, xy, Z):
    """Stationary geometric rollout; collision checks never move the arm."""
    q = e.r.q.copy()
    T0 = e.fk(q) @ e.T_EC
    world_points = np.column_stack([xy, np.ones(4)])*Z[:, None]
    target = e.tracker.target_px[0]
    K, D = np.asarray(e.camera['K'], float), np.asarray(e.camera['distortion'], float)
    task = 'center_size'
    started = time.monotonic()
    records = []
    try:
        for step in range(500):
            T = e.fk(q) @ e.T_EC
            relative = np.linalg.inv(T) @ T0
            points = world_points @ relative[:3, :3].T + relative[:3, 3]
            if np.any(points[:, 2] <= 0):
                raise RuntimeError('preflight features behind camera')
            pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, D)
            pixels = pixels.reshape(4, 2)
            metrics = shape_metrics(pixels, target)
            records.append(dict(step=step, joints_deg=np.degrees(q).tolist(), **metrics))
            if metrics['max_corner_px'] <= e.c['tolerance_px']:
                result = dict(status='PREDICTED_REACHABLE', steps=step, **metrics,
                              note='Model/depth prediction only; physical convergence still required.')
                print('Stationary reach/clearance check passed: predicted max corner %.2f px.' % metrics['max_corner_px'], flush=True)
                return result
            if (len(records) >= 30 and metrics['max_corner_px'] <= e.c['tolerance_px']+1.
                    and np.ptp([row['rms_px'] for row in records[-20:]]) < .02):
                return model_residual(metrics,step)
            if time.monotonic()-started > 120:
                raise RuntimeError('stationary reach check time limit')
            if metrics['center_error_px'] < 5 and .9 < metrics['size_ratio'] < 1.1:
                task = 'corners'
            try:
                _, proposed, _, _, _ = choose_step(e, q, e.tracker.normalize(pixels), points[:, 2], task)
            except RuntimeError as error:
                if str(error).startswith('undamped IBVS has no improving') and metrics['max_corner_px'] <= e.c['tolerance_px']+1.:
                    return model_residual(metrics,step)
                raise
            # Check swept configurations as well as endpoints, including the gripper.
            for fraction in (.25, .5, .75, 1.):
                sample = q+(proposed-q)*fraction
                within_limits(e.fk(sample), e.T_EC, e.T_tcp, e.origin, e.floor,
                              e.c['max_camera_travel_m'], np.radians(e.c['max_camera_rotation_deg']))
                scene.check(sample)
            q = proposed
            if step % 25 == 0:
                print('Stationary check %d: center %.1f px, size %.1f%%, max %.1f px' %
                      (step, metrics['center_error_px'], metrics['size_ratio']*100, metrics['max_corner_px']), flush=True)
        if metrics['max_corner_px'] <= e.c['tolerance_px']+1.:
            return model_residual(metrics,step)
        raise RuntimeError('stationary reach check did not reach the requested image')
    finally:
        (e.output/'reach_check.json').write_text(json.dumps(records, indent=2)+'\n')


def acquire(e, deadline, frames=1):
    count, epoch = 0, getattr(e, 'tracking_epoch', 0)
    while time.monotonic() < deadline:
        try:
            observation = e.capture()
        except TrackingLost:
            count = 0
            continue
        current_epoch = getattr(e, 'tracking_epoch', 0)
        if current_epoch != epoch:
            count = 0
            epoch = current_epoch
        count += 1
        if count >= frames:
            return observation
    raise TrackingLost('square was not recovered before the stationary hold timeout')


def servo(e, args, report):
    e.servo_report = report
    report.update(recoveries=0, steps=0, motion_attempts=0)
    xy, Z, rms, maximum = acquire(e, time.monotonic()+getattr(args, 'wait_target_s', 30.))
    report['initial_rms_px'] = rms
    report['initial_metrics'] = shape_metrics(e.detection[1][0], e.tracker.target_px[0])
    scene = ObservedScene(e)
    try:
        scene.install(float(np.mean(Z)), getattr(args, 'resume_run', None))
        report['preflight'] = reach_check(e, scene, xy, Z)
        if not args.live:
            report['status'] = 'PREVIEW'
            return
        e.record_correction = True
        xy, Z, rms, maximum = acquire(e, time.monotonic()+e.c.get('tracking_recovery_timeout_s', 12.))
        commanded = e.r.q.copy()
        convergence = Convergence(e.c['tolerance_px'], e.c['converge_frames'])
        task, started, motion_steps, attempts = 'center_size', time.monotonic(), 0, 0
        best_rms, growth, settled = rms, 0, False
        epoch, verified_stamps = getattr(e, 'tracking_epoch', 0), []
        e.event('IBVS_START', rms_px=rms, max_corner_px=maximum)
        while True:
            if time.monotonic()-started > e.c['timeout_s']:
                raise RuntimeError('shape-servo time limit')
            current_epoch = getattr(e, 'tracking_epoch', 0)
            if current_epoch != epoch:
                convergence.n = 0
                verified_stamps = []
                epoch = current_epoch
            metrics = shape_metrics(e.detection[1][0], e.tracker.target_px[0])
            best_rms = min(best_rms,rms)
            report.update(final_metrics=metrics, steps=motion_steps, motion_attempts=attempts,
                          best_rms_px=min(best_rms,rms))
            try:
                if task == 'corners' and maximum <= 8. and not settled:
                    e.visual_stage = 'HOLD: verify alignment'
                    e.hold(e.c.get('near_goal_hold_s', .7))
                    xy,Z,rms,maximum = acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    settled = True
                    convergence.n = 0
                    verified_stamps = []
                    continue
                if maximum <= e.c['tolerance_px']:
                    verified_stamps.append(getattr(e,'last_image_stamp',None))
                else:
                    verified_stamps = []
                if convergence.update(maximum):
                    report.update(status='CONVERGED' if motion_steps else 'ALREADY_ALIGNED',
                                  verification_camera_stamps=verified_stamps[-int(e.c['converge_frames']):],
                                  final_metrics=metrics)
                    e.visual_stage = 'CONVERGED: 5 fresh observations'
                    e.event('CONVERGED', **metrics, fresh_observations=convergence.n)
                    cv2.imwrite(str(e.output/'converged.png'),e.bgr(e.r._img['color']))
                    cv2.imwrite(str(e.output/'converged_features.png'),e.latest_annotated)
                    return
                if maximum <= e.c['tolerance_px']:
                    xy,Z,rms,maximum = acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    continue
                if attempts >= args.steps:
                    raise RuntimeError('shape-servo step limit')
                if task == 'center_size' and metrics['center_error_px'] < 5 and .9 < metrics['size_ratio'] < 1.1:
                    task = 'corners'
                    e.event('IBVS_FINE_START')
                e.visual_stage = 'IBVS: center and size' if task == 'center_size' else 'IBVS: four-corner alignment'
                q = e.r.q.copy()
                _,target,dq,predicted_metrics,scale = choose_step(e,q,xy,Z,task,commanded)
                path = e.joint_plan(np.degrees(target),local=True)
                if np.degrees(np.abs(path-path[0])).max(axis=0)[[0,2,3,4,5]].max() > 3:
                    raise RuntimeError('other joint path exceeds 3-degree guard')
                for sample in path:
                    scene.check(sample)
                T0,T1 = e.fk(q)@e.T_EC,e.fk(target)@e.T_EC
                distance = np.linalg.norm(T1[:3,3]-T0[:3,3])
                angle = Rotation.from_matrix(T0[:3,:3].T@T1[:3,:3]).magnitude()
                duration = max(e.c['min_move_s'],1.875*distance/e.c['linear_speed_m_s'],
                               1.875*np.degrees(angle)/e.c['angular_speed_deg_s'])
                attempts += 1
                e.move(path,duration)
                commanded = target
                motion_steps += 1
                e.servo_steps = motion_steps
                settled = False
                xy,Z,rms,maximum = e.capture()
                actual = shape_metrics(e.detection[1][0],e.tracker.target_px[0])
                with (e.output/'priority_commands.jsonl').open('a') as record:
                    record.write(json.dumps(dict(step=motion_steps,task=task,scale=scale,
                        joint_increment_deg=np.degrees(dq).tolist(),predicted=predicted_metrics,actual=actual))+'\n')
                print('Step %d %s: RMS %.2f px, max %.2f px, size %.1f%%' %
                      (motion_steps,task,rms,maximum,actual['size_ratio']*100),flush=True)
                best_rms = min(best_rms,rms)
                growth = growth+1 if rms > best_rms+e.c['error_growth_px'] else 0
                if growth >= e.c['error_growth_frames']:
                    raise RuntimeError('measured corner error exceeded the growth margin')
            except TrackingLost as error:
                e.cancel()
                report['recoveries'] += 1
                if report['recoveries'] > e.c.get('max_tracking_recoveries',10):
                    raise TrackingLost('tracking recovery limit reached') from error
                e.visual_stage = 'HOLD: reacquire the same square'
                e.event('TRACKING_HOLD', recovery=report['recoveries'], reason=str(error))
                print('Holding for square reacquisition in the same run.',flush=True)
                e.hold(e.c['arrival_settle_s'])
                deadline = min(started+e.c['timeout_s'],time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                xy,Z,rms,maximum = acquire(e,deadline,int(e.c.get('tracking_recovery_frames',3)))
                e.fresh(camera=True)
                commanded = e.r.q.copy()
                convergence.n = 0
                verified_stamps = []
                epoch = getattr(e,'tracking_epoch',0)
                settled = False
                scene.check(e.r.q)
                e.event('TRACKING_RESUMED', recovery=report['recoveries'])
                print('Square recovered; continuing the same IBVS run.',flush=True)
    finally:
        scene.close()


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--show-image', action='store_true')
    parser.add_argument('--bringup', action='store_true')
    parser.add_argument('--home-first', action='store_true', help='home then recorded post-grasp before IBVS')
    parser.add_argument('--steps', type=int, default=800)
    parser.add_argument('--wait-target-s', type=float, default=3., help='stationary wait for a visible rectangle')
    parser.add_argument('--resume-run', type=Path, help='reuse a verified earlier fixed-scene snapshot after visual validation')
    parser.add_argument('--near-scene', action='store_true', help='fresh scene from 11 cm for this verified D405/gripper setup')
    parser.add_argument('--record-sequence',action='store_true',help='also record positioning; default records only IBVS')
    args = parser.parse_args()
    if args.steps < 1 or not np.isfinite(args.wait_target_s) or args.wait_target_s <= 0:
        parser.error('steps and wait-target-s must be positive')
    if args.resume_run and args.home_first:
        parser.error('--resume-run continues the current pose and cannot be combined with --home-first')
    if args.resume_run and args.near_scene:
        parser.error('choose either a fresh near scene or a saved scene')
    c, ref, g = load_inputs(ROOT/'config/alignment.yaml', ROOT/'config/center_rectangle.yaml')
    c['max_servo_joint_step_deg'] = 5.5
    if args.near_scene:
        c['scene_min_depth_m'] = .11
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    out = PROJECT/'Media/IBVS'/('shape_ibvs_'+stamp)
    out.mkdir()
    os.environ.setdefault('ROS_LOG_DIR', str(out/'ros_logs'))
    if args.show_image:
        for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR', 'GIO_EXTRA_MODULES',
                    'GDK_PIXBUF_MODULEDIR', 'GDK_PIXBUF_MODULE_FILE', 'GTK_MODULES'):
            os.environ.pop(key, None)
        os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    report = dict(status='INCOMPLETE', process_id=os.getpid(), method='Undamped center/size IBVS then four-corner IBVS; weighted joint mobility',
                  live=args.live, settings=c, reference=ref,
                  recording_scope='full sequence and IBVS' if args.record_sequence else 'IBVS only',
                  resume_run=str(args.resume_run.expanduser().resolve()) if args.resume_run else None)
    (out/'config.json').write_text(json.dumps(report, indent=2)+'\n')
    for source in (__file__, ROOT/'ibvs/shape_control.py', ROOT/'ros2_iface/observed_scene.py'):
        shutil.copyfile(source, out/os.path.basename(source))
    for relative in ('ros2_iface/alignment_test.py', 'ibvs/alignment.py', 'ibvs/control.py', 'ibvs/depth.py',
                     'ibvs/frames.py', 'ibvs/rectangles.py', 'ibvs/overlay.py', 'ibvs/recording.py', 'ibvs/kinematics.py', 'config/center_rectangle.yaml'):
        snapshot = out/'source'/relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/relative, snapshot)
    def interrupt(number, _frame):
        raise KeyboardInterrupt(signal.Signals(number).name)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    e = None
    try:
        e = Experiment(SimpleNamespace(live=args.live, bringup=args.bringup, shutdown=False,
                                      show_image=args.show_image,from_post_grasp=False,position_only=False,
                                      calibration_only=False,hand_eye=None,record_sequence=args.record_sequence), c, ref, g, out)
        if args.home_first:
            if not args.live:
                raise RuntimeError('--home-first requires --live; preview checks the current pose without moving')
            report.update(e.run())
        else:
            e.startup()
            e.phase = 'ALIGN'
            e.visual_stage = 'STATIONARY reach/clearance check'
            e.origin = e.fk(np.radians(g['post_grasp']['joint_waypoints_deg'][-1])) @ e.T_EC
            servo(e, args, report)
    except KeyboardInterrupt as error:
        report.update(status='INTERRUPTED',interrupt_reason=str(error))
    except Exception as error:
        report.update(status='FAILED', error=str(error))
        print('STOPPED:', error, flush=True)
    finally:
        if e is not None:
            report.update({k:v for k,v in getattr(e,'servo_report',{}).items() if k!='status'})
            report['poses_completed'] = e.poses_completed
            if e.r.q is not None:
                report['final_joints_deg'] = np.degrees(e.r.q).tolist()
            e.close()
            report['correction_video'] = str(out/'ibvs_correction.mp4') if (out/'ibvs_correction.mp4').exists() else None
            report['full_sequence_video'] = str(out/'features.mp4') if (out/'features.mp4').exists() else None
        rclpy.try_shutdown()
        (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print('Run evidence:', out, flush=True)
    return 0 if report['status'] in ('CONVERGED', 'ALREADY_ALIGNED', 'PREVIEW') else 1


if __name__ == '__main__':
    raise SystemExit(main())
