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

from alignment_test import Experiment, ROOT, PROJECT, load_inputs, add_advance_arguments, scale_speeds
from ibvs.advance import backproject, clearance, fit_wall, plan_travel
from ibvs.depth import scene_near_depth
from ibvs.alignment import camera_joint_jacobian, within_limits, smooth_servo_path
from ibvs.control import Convergence
from ibvs.guards import check_axial
from ibvs.markers import TrackingLost
from ibvs.shape_control import shape_ibvs_step, shape_metrics, task_cost, project_motion, STEP_SCALES
from observed_scene import ObservedScene


def smooth_profile(settings,control_hz=2.):
    """Software tuning for a bounded free-space trial; measured limits stay separate."""
    if not np.isfinite(control_hz) or not .2<=control_hz<=3.:
        raise ValueError('control-hz must be between 0.2 and 3 for this step-and-observe runner')
    settings.update(control_hz=float(control_hz),align_orientation=True,smooth_joint_segments=True,
        joint_speed_deg_s=8.,linear_speed_m_s=.008,angular_speed_deg_s=3.,
        min_move_s=.35,arrival_settle_s=.10,position_settle_s=.5,near_goal_hold_s=.4,final_verify_hold_s=.5)


def fast_profile(settings):
    """UNVALIDATED software tuning: bigger steps, no rate cap, short settling. First run supervised, e-stop in reach."""
    settings.update(align_orientation=True,smooth_joint_segments=True,collision_segment_deg=1.,gain=.6,
        joint_speed_deg_s=20.,linear_speed_m_s=.03,angular_speed_deg_s=10.,min_move_s=.15,
        arrival_settle_s=.05,position_settle_s=.3,near_goal_hold_s=.25,final_verify_hold_s=.3,
        max_step_m=.006,max_step_deg=1.5,approach_limits_deg=[2.5,5.,2.5,2.5,2.5,2.5],
        fine_limits_deg=[.6,1.5,.6,.6,.6,.6])


def approach_task(e):
    return 'center_size_angle' if e.c.get('align_orientation',False) else 'center_size'


def choose_step(e, q, xy, Z, task, previous_command=None):
    """An undamped task inverse; smaller steps remain available at any error."""
    K, D = np.asarray(e.camera['K'], float), np.asarray(e.camera['distortion'], float)
    focal = np.diag(K)[:2]
    desired = e.tracker.star[0]
    target_px = e.tracker.target_px[0]
    Jcam = camera_joint_jacobian(e.fk, q, e.T_EC)
    limits = np.radians(e.c.get('approach_limits_deg',[1.,5.,1.,1.,1.,1.]) if task != 'corners'
                        else e.c.get('fine_limits_deg',[.25,1.,.25,.25,.25,.25]))
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
        if task != 'corners' and metrics['center_error_px'] > max(5., center+1.):
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
    task = approach_task(e)
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
                scene.check(q)
                result = dict(status='PREDICTED_REACHABLE', steps=step, **metrics,
                              note='Model/depth prediction only; physical convergence still required.')
                print('Stationary reach/clearance check passed: predicted max corner %.2f px.' % metrics['max_corner_px'], flush=True)
                return result
            if (len(records) >= 30 and metrics['max_corner_px'] <= e.c['tolerance_px']+1.
                    and np.ptp([row['rms_px'] for row in records[-20:]]) < .02):
                scene.check(q);return model_residual(metrics,step)
            if time.monotonic()-started > 120:
                raise RuntimeError('stationary reach check time limit')
            if metrics['center_error_px'] < 5 and .9 < metrics['size_ratio'] < 1.1:
                task = 'corners'
            try:
                _, proposed, _, _, _ = choose_step(e, q, e.tracker.normalize(pixels), points[:, 2], task)
            except RuntimeError as error:
                if str(error).startswith('undamped IBVS has no improving') and metrics['max_corner_px'] <= e.c['tolerance_px']+1.:
                    scene.check(q);return model_residual(metrics,step)
                raise
            # Check swept configurations as well as endpoints, including the gripper.
            for fraction in (.25, .5, .75, 1.):
                sample = q+(proposed-q)*fraction
                within_limits(e.fk(sample), e.T_EC, e.T_tcp, e.origin, e.floor,
                              e.c['max_camera_travel_m'], np.radians(e.c['max_camera_rotation_deg']))
            if step % e.c.get('preflight_check_every',1) == 0:
                scene.check(proposed)
            q = proposed
            if step % 25 == 0:
                print('Stationary check %d: center %.1f px, size %.1f%%, max %.1f px' %
                      (step, metrics['center_error_px'], metrics['size_ratio']*100, metrics['max_corner_px']), flush=True)
        if metrics['max_corner_px'] <= e.c['tolerance_px']+1.:
            scene.check(q);return model_residual(metrics,step)
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
    raise TrackingLost('marker was not recovered before the stationary hold timeout')


def advance(e, scene, args, report, execute):
    """After alignment: move straight toward the wall along the measured wall normal in short guarded steps.
    The wall plane and the clearance of everything the gripper holds come from the depth image. No force sensing:
    each step must be followed by the arm (progress check), and without --allow-contact the travel stops short of the wall."""
    obs = e.last_observation
    depth, K, D = obs['depth_m'], obs['K'], obs['distortion']
    held = np.isfinite(depth) & (depth > 0) & (depth < scene_near_depth(e.c, e.T_EC, e.T_tcp))   # gripper and held plug, live
    corners = obs['corners_px']
    x0, y0 = int(corners[:, 0].mean()), int(corners[:, 1].max()) + 20
    n, d, support = fit_wall(depth, (x0 - 160, y0, x0 + 160, min(depth.shape[0], y0 + 240)), held, K, D,
                             e.c['depth_min_m'], e.c['depth_max_m'])
    tilt = float(np.degrees(np.arccos(np.clip(-n[2], -1, 1))))
    if tilt > 10.:
        raise RuntimeError('wall normal is %.1f deg from the camera axis: not a plausible wall view' % tilt)
    tip = (np.linalg.inv(e.T_EC) @ e.T_tcp[:, 3])[:3]
    free = clearance(np.vstack([backproject(depth, held, K, D), tip]), n, d)
    planned, trimmed = plan_travel(free, args.advance_m, args.advance_margin_m, args.allow_contact)
    report['advance'] = info = dict(requested_m=args.advance_m, planned_m=planned, clearance_m=free, wall_tilt_deg=tilt,
                                    wall_support_points=support, margin_m=args.advance_margin_m,
                                    allow_contact=bool(args.allow_contact), trimmed=bool(trimmed), executed_m=0.)
    print('ADVANCE: gripper and held object are %.0f mm from the wall (plane %.1f deg off the camera axis). '
          'Requested %.0f mm -> %.0f mm%s' % (free * 1e3, tilt, args.advance_m * 1e3, planned * 1e3,
                                              ' TRIMMED to stay %.0f mm short of the wall; --allow-contact does the full stroke' % (args.advance_margin_m * 1e3) if trimmed else ''), flush=True)
    if not execute or planned < .002:
        return
    e.phase, e.visual_stage = 'ADVANCE', 'ADVANCE: forward along the wall normal'
    e.event('ADVANCE_START', **info)
    direction = (e.fk(e.r.q)[:3, :3] @ e.T_EC[:3, :3]) @ -n          # base-frame unit vector, fixed for the whole move
    total, ax, settle = 0., dict(max_travel_m=planned + .01, min_progress_frac=.5), e.c['arrival_settle_s']
    e.c['arrival_settle_s'] = min(settle, .1)                          # the trajectory itself is smooth: no long wait per chunk
    try:
        while total < planned - 1e-6:
            step = min(args.advance_step_m, planned - total)
            q = e.r.q.copy()
            T0 = e.fk(q)
            count = max(1, int(np.ceil(step / .01 - 1e-9)))              # IK waypoint every ~1 cm, one flowing trajectory per chunk
            waypoints = [q]
            for k in range(1, count + 1):
                T = T0.copy()
                T[:3, 3] += step * k / count * direction
                q_ik = e.r.ik(T, np.degrees(waypoints[-1]))
                if q_ik is None:
                    raise RuntimeError('advance: no IK %.0f mm into the next chunk (after %.0f mm)' % (step * k / count * 1e3, total * 1e3))
                waypoints.append(waypoints[-1] + np.radians((q_ik - np.degrees(waypoints[-1]) + 180) % 360 - 180))
            path = e.check_joint_path(np.array(waypoints), local=True)
            if not args.allow_contact:
                for sample in path:
                    scene.check(sample)
            e.move(path, max(e.c['min_move_s'], step / args.advance_speed_mps))
            measured = float((e.fk(e.r.q)[:3, 3] - T0[:3, 3]) @ direction)
            why = check_axial(step, measured, total, ax)
            total += max(measured, 0.)
            info['executed_m'] = total
            with (e.output / 'advance.jsonl').open('a') as record:
                record.write(json.dumps(dict(commanded_m=step, measured_m=measured, total_m=total, joints_deg=np.degrees(e.r.q).tolist(), stop=why)) + '\n')
            print('Advance %.0f / %.0f mm' % (total * 1e3, planned * 1e3), flush=True)
            if why:
                raise RuntimeError('advance stopped after %.0f mm: %s (the arm did not follow the command)' % (total * 1e3, why))
    finally:
        e.c['arrival_settle_s'] = settle
    e.event('ADVANCE_DONE', executed_m=total)


def run_advance(e, scene, args, report, execute):
    if getattr(args, 'advance_m', 0.) <= 0:
        return
    try:
        advance(e, scene, args, report, execute)
    except Exception as error:
        report.setdefault('advance', {})['error'] = str(error)
        report['status'] = 'ADVANCE_FAILED' if execute else report.get('status', 'PREVIEW')
        print('ADVANCE STOPPED:', error, flush=True)


def servo(e, args, report):
    e.servo_report = report
    report.update(recoveries=0, steps=0, motion_attempts=0)
    xy, Z, rms, maximum = acquire(e, time.monotonic()+getattr(args, 'wait_target_s', 30.))
    report['initial_rms_px'] = rms
    report['marker_id'], report['goal_corners_px'] = int(e.tracker.locked), e.tracker.target_px[0].tolist()
    e.event('MARKER_CHOSEN', marker_id=report['marker_id'], candidates=e.tracker.candidates)
    print('Marker %d chosen live (visible: %s). Goal: centred, upright, %.0f px wide.' % (
        report['marker_id'], e.tracker.candidates, np.linalg.norm(e.tracker.target_px[0][1] - e.tracker.target_px[0][0])), flush=True)
    report['initial_metrics'] = shape_metrics(e.detection[1][0], e.tracker.target_px[0])
    scene = ObservedScene(e)
    try:
        scene.install(float(np.mean(Z)), getattr(args, 'resume_run', None))
        report['preflight'] = reach_check(e, scene, xy, Z)
        if not args.live:
            report['status'] = 'PREVIEW'
            run_advance(e, scene, args, report, execute=False)
            return
        e.record_correction = True
        xy, Z, rms, maximum = acquire(e, time.monotonic()+e.c.get('tracking_recovery_timeout_s', 12.))
        commanded = e.r.q.copy()
        convergence = Convergence(e.c['tolerance_px'], e.c['converge_frames'])
        task, started, motion_steps, attempts = approach_task(e), time.monotonic(), 0, 0
        first_cycle=previous_cycle=None
        final_hold_done=False
        initial_tool=e.fk(e.r.q).copy()
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
                if e.c.get('final_verify_hold_s') and maximum<=e.c['tolerance_px'] and not final_hold_done:
                    e.visual_stage='HOLD: final stationary verification'
                    e.hold(e.c['final_verify_hold_s'])
                    xy,Z,rms,maximum=acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    final_hold_done=True;convergence.n=0;verified_stamps=[]
                    e.event('FINAL_ALIGNMENT_HOLD',seconds=e.c['final_verify_hold_s'])
                    continue
                if maximum>e.c['tolerance_px']:final_hold_done=False
                if task == 'corners' and maximum <= 8. and not settled:
                    e.visual_stage = 'HOLD: verify alignment'
                    e.hold(e.c.get('near_goal_hold_s', .7))
                    xy,Z,rms,maximum = acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    settled = True
                    e.event('NEAR_GOAL_HOLD',once_per_tracking_epoch=bool(e.c.get('smooth_joint_segments')))
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
                    report['ibvs_elapsed_s']=time.monotonic()-started
                    report['tool_rotation_deg']=float(np.degrees(Rotation.from_matrix(
                        initial_tool[:3,:3].T@e.fk(e.r.q)[:3,:3]).magnitude()))
                    e.visual_stage = 'CONVERGED: 5 fresh observations'
                    e.event('CONVERGED', **metrics, fresh_observations=convergence.n)
                    cv2.imwrite(str(e.output/'converged.png'),e.bgr(e.r._img['color']))
                    cv2.imwrite(str(e.output/'converged_features.png'),e.latest_annotated)
                    run_advance(e, scene, args, report, execute=True)
                    return
                if maximum <= e.c['tolerance_px']:
                    xy,Z,rms,maximum = acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    continue
                if attempts >= args.steps:
                    raise RuntimeError('shape-servo step limit')
                if task != 'corners' and metrics['center_error_px'] < 5 and .9 < metrics['size_ratio'] < 1.1:
                    task = 'corners'
                    e.event('IBVS_FINE_START')
                period=1./e.c['control_hz'] if e.c.get('control_hz') else 0.
                delay=0. if previous_cycle is None else previous_cycle+period-time.monotonic()
                if delay>0:
                    e.hold(delay)
                    xy,Z,rms,maximum=acquire(e,time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                    continue
                cycle_started=time.monotonic()
                interval=None if previous_cycle is None else cycle_started-previous_cycle
                if first_cycle is None:first_cycle=cycle_started
                previous_cycle=cycle_started
                e.visual_stage = 'IBVS: center, size and orientation' if task=='center_size_angle' else ('IBVS: center and size' if task=='center_size' else 'IBVS: four-corner alignment')
                q = e.r.q.copy()
                _,target,dq,predicted_metrics,scale = choose_step(e,q,xy,Z,task,commanded)
                chosen=time.monotonic()
                path = e.direct_path(target)
                if e.c.get('smooth_joint_segments',False):path=smooth_servo_path(path,e.c.get('collision_segment_deg',.25))
                if np.degrees(np.abs(path-path[0])).max(axis=0)[[0,2,3,4,5]].max() > 3:
                    raise RuntimeError('other joint path exceeds 3-degree guard')
                for sample in path:
                    within_limits(e.fk(sample),e.T_EC,e.T_tcp,e.origin,e.floor,
                        e.c['max_camera_travel_m'],np.radians(e.c['max_camera_rotation_deg']))
                    scene.check(sample)
                planned=time.monotonic()
                T0,T1 = e.fk(q)@e.T_EC,e.fk(target)@e.T_EC
                distance = np.linalg.norm(T1[:3,3]-T0[:3,3])
                angle = Rotation.from_matrix(T0[:3,:3].T@T1[:3,:3]).magnitude()
                duration = max(e.c['min_move_s'],1.875*distance/e.c['linear_speed_m_s'],
                               1.875*np.degrees(angle)/e.c['angular_speed_deg_s'])
                attempts += 1
                e.move(path,duration)
                moved=time.monotonic()
                commanded = target
                motion_steps += 1
                e.servo_steps = motion_steps
                if not e.c.get('smooth_joint_segments'):settled = False
                xy,Z,rms,maximum = e.capture()
                actual = shape_metrics(e.detection[1][0],e.tracker.target_px[0])
                elapsed=time.monotonic()-cycle_started
                report['achieved_control_hz']=(motion_steps-1)/(cycle_started-first_cycle) if motion_steps>1 else None
                report['ibvs_elapsed_s']=time.monotonic()-started
                with (e.output/'loop_timing.jsonl').open('a') as record:
                    record.write(json.dumps(dict(step=motion_steps,task=task,requested_hz=e.c.get('control_hz'),
                        period_s=interval,choose_s=chosen-cycle_started,check_s=planned-chosen,move_s=moved-planned,
                        observation_s=elapsed-(moved-cycle_started),cycle_s=elapsed))+'\n')
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
                e.visual_stage = 'HOLD: reacquire the marker'
                e.event('TRACKING_HOLD', recovery=report['recoveries'], reason=str(error))
                print('Holding for marker reacquisition in the same run.',flush=True)
                e.hold(e.c['arrival_settle_s'])
                deadline = min(started+e.c['timeout_s'],time.monotonic()+e.c.get('tracking_recovery_timeout_s',12.))
                xy,Z,rms,maximum = acquire(e,deadline,int(e.c.get('tracking_recovery_frames',3)))
                e.fresh(camera=True)
                commanded = e.r.q.copy()
                convergence.n = 0
                verified_stamps = []
                epoch = getattr(e,'tracking_epoch',0)
                settled = False
                final_hold_done=False
                scene.check(e.r.q)
                e.event('TRACKING_RESUMED', recovery=report['recoveries'])
                print('Marker recovered; continuing the same IBVS run.',flush=True)
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
    parser.add_argument('--wait-target-s', type=float, default=3., help='stationary wait for the visible marker')
    parser.add_argument('--resume-run', type=Path, help='reuse a verified earlier fixed-scene snapshot after visual validation')
    parser.add_argument('--near-scene', action='store_true', help='fresh scene from 11 cm for this verified D405/gripper setup')
    parser.add_argument('--record-sequence',action='store_true',help='also record positioning; default records only IBVS')
    parser.add_argument('--reference', type=Path, default=ROOT/'config/marker_goal.yaml')
    parser.add_argument('--socket', help='optional: force the marker of this socket (usb_a, audio, ac) instead of choosing live')
    parser.add_argument('--marker-id', type=int, help='optional: force this ArUco id instead of choosing live')
    add_advance_arguments(parser)
    parser.add_argument('--close-gripper',action='store_true',help='close the gripper before the servo')
    parser.add_argument('--fast',action='store_true',help='UNVALIDATED faster profile: bigger steps, short settling, coarser collision sampling')
    parser.add_argument('--smooth',action='store_true',help='orientation-aware approach, continuous straight servo segments and shorter settling')
    parser.add_argument('--control-hz',type=float,help='maximum step-loop frequency; actual rate is limited by planning, motion and fresh feedback')
    args = parser.parse_args()
    launched=time.monotonic()
    if args.steps < 1 or not np.isfinite(args.wait_target_s) or args.wait_target_s <= 0:
        parser.error('steps and wait-target-s must be positive')
    if args.resume_run and args.home_first:
        parser.error('--resume-run continues the current pose and cannot be combined with --home-first')
    if args.resume_run and args.near_scene:
        parser.error('choose either a fresh near scene or a saved scene')
    c, ref, g = load_inputs(ROOT/'config/alignment.yaml', args.reference, args.socket, args.marker_id)
    if args.control_hz is not None and (not np.isfinite(args.control_hz) or not .2<=args.control_hz<=3.):
        parser.error('control-hz must be between 0.2 and 3')
    if not (0. <= args.advance_m <= .30 and 0. < args.advance_step_m <= .05 and 0. < args.advance_speed_mps <= .02 and args.advance_margin_m >= 0.):
        parser.error('advance: distance 0..0.30 m, step 0..0.05 m, speed 0..0.02 m/s, margin >= 0')
    if not 1. <= args.speed_scale <= 3.:
        parser.error('--speed-scale must be between 1 and 3')
    if args.fast and (args.smooth or args.control_hz is not None):parser.error('--fast replaces --smooth and --control-hz')
    if args.fast:fast_profile(c)
    elif args.smooth:smooth_profile(c,2. if args.control_hz is None else args.control_hz)
    elif args.control_hz is not None:c['control_hz']=args.control_hz
    scale_speeds(c, args.speed_scale)
    args.advance_speed_mps = min(.03, args.advance_speed_mps * args.speed_scale)
    c['max_servo_joint_step_deg'] = 5.5
    if args.near_scene:
        c['scene_min_depth_m'] = .11
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    out = PROJECT/'Media/IBVS'/('shape_ibvs_'+stamp)
    out.mkdir(parents=True)
    os.environ.setdefault('ROS_LOG_DIR', str(out/'ros_logs'))
    if args.show_image:
        for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR', 'GIO_EXTRA_MODULES',
                    'GDK_PIXBUF_MODULEDIR', 'GDK_PIXBUF_MODULE_FILE', 'GTK_MODULES'):
            os.environ.pop(key, None)
        os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    report = dict(status='INCOMPLETE', process_id=os.getpid(), method=('Center/size/angle IBVS then four-corner IBVS; bounded quintic motion' if (args.smooth or args.fast) else 'Undamped center/size IBVS then four-corner IBVS; weighted joint mobility'),
                  live=args.live, settings=c, reference=ref,
                  recording_scope='full sequence and IBVS' if args.record_sequence else 'IBVS only',
                  resume_run=str(args.resume_run.expanduser().resolve()) if args.resume_run else None)
    (out/'config.json').write_text(json.dumps(report, indent=2)+'\n')
    for source in (__file__, ROOT/'ibvs/shape_control.py', ROOT/'ros2_iface/observed_scene.py'):
        shutil.copyfile(source, out/os.path.basename(source))
    for relative in ('ros2_iface/alignment_test.py', 'ibvs/alignment.py', 'ibvs/control.py', 'ibvs/depth.py',
                     'ibvs/frames.py', 'ibvs/markers.py', 'ibvs/overlay.py', 'ibvs/recording.py', 'ibvs/kinematics.py', 'config/marker_goal.yaml'):
        snapshot = out/'source'/relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/relative, snapshot)
    shutil.copyfile(args.reference, out/'source'/('selected_reference'+args.reference.suffix))
    def interrupt(number, _frame):
        raise KeyboardInterrupt(signal.Signals(number).name)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    e = None
    try:
        e = Experiment(SimpleNamespace(live=args.live, bringup=args.bringup, shutdown=False,
                                      show_image=args.show_image,from_post_grasp=False,position_only=False,
                                      calibration_only=False,hand_eye=None,record_sequence=args.record_sequence,
                                      close_gripper=args.close_gripper,advance_m=args.advance_m,advance_step_m=args.advance_step_m,
                                      advance_speed_mps=args.advance_speed_mps,advance_margin_m=args.advance_margin_m,
                                      allow_contact=args.allow_contact), c, ref, g, out)
        if args.home_first:
            if not args.live:
                raise RuntimeError('--home-first requires --live; preview checks the current pose without moving')
            report.update(e.run())
        else:
            e.startup()
            if args.close_gripper:
                e.close_gripper()
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
        report['wall_elapsed_s']=time.monotonic()-launched
        (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print('Run evidence:', out, flush=True)
    return 0 if report['status'] in ('CONVERGED', 'ALREADY_ALIGNED', 'PREVIEW') else 1


if __name__ == '__main__':
    raise SystemExit(main())
