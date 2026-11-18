#!/usr/bin/env python3
"""Offline CAD registration. No ROS imports or robot commands.
The sticker selects an image region only; no tag/map/archived grasp pose is read.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import cv2
import trimesh
import yaml
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from housing_grasp.color_marker import marker_mask, head_mask


def rotation(yaw, pitch=0, roll=0):
    if pitch != 0 or roll != 0:
        return Rotation.from_euler('ZYX',[yaw,pitch,roll]).as_matrix()
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def register(points, model, six_dof=False):
    """Observed-to-model robust ICP, upright constraint, 24 yaw initializations.
    Translation in mm; yaw in radians. Partial views do not penalize hidden CAD.
    """
    tree = cKDTree(model)
    center = np.median(points, axis=0)
    solutions = []
    for yaw in np.deg2rad(np.arange(0, 360, 15)):
        x0 = np.r_[center - np.median(model, axis=0), yaw]
        if six_dof:
            x0 = np.r_[x0, 0., 0.]
            for _ in range(60):
                local = (points-x0[:3]) @ rotation(x0[3], *x0[4:])
                d, idx = tree.query(local)
                keep = d <= np.percentile(d,90)
                src, dst = model[idx[keep]], points[keep]
                cs, cd = src.mean(0), dst.mean(0)
                U, _, Vt = np.linalg.svd((src-cs).T @ (dst-cd))
                R = Vt.T @ np.diag([1,1,np.linalg.det(Vt.T@U.T)]) @ U.T
                x0 = np.r_[cd-R@cs, Rotation.from_matrix(R).as_euler('ZYX')]

        def residual(x):
            local = (points - x[:3]) @ rotation(x[3], *x[4:])
            _, idx = tree.query(local)
            return (local - model[idx]).ravel()
        fit = least_squares(residual, x0, loss='soft_l1', f_scale=1.5,
                            max_nfev=100, diff_step=1e-4)
        dist = tree.query((points - fit.x[:3]) @ rotation(fit.x[3], *fit.x[4:]))[0]
        solutions.append((float(np.mean(2*1.5**2*(np.sqrt(1+(dist/1.5)**2)-1))), fit.x, dist))
    return sorted(solutions, key=lambda q: q[0])



def register_on_plane(points, model, normal, offset):
    """Fit only translation in the measured panel plane and yaw about its normal."""
    n=np.asarray(normal,float);n=n/np.linalg.norm(n)
    e=np.array([1.,0,0]);e-=n*(n@e);e/=np.linalg.norm(e)
    basis=np.column_stack([e,np.cross(n,e),n]);origin=-offset*1000*n
    tree=cKDTree(model);center=(np.median(points,axis=0)-origin)@basis
    results=[]
    for yaw in np.deg2rad(np.arange(0,360,15)):
        def unpack(q):
            R=basis@rotation(q[2]);t=origin+basis[:,:2]@q[:2]
            return R,t
        def residual(q):
            R,t=unpack(q);local=(points-t)@R;_,idx=tree.query(local)
            return (local-model[idx]).ravel()
        f=least_squares(residual,np.r_[center[:2],yaw],loss='soft_l1',f_scale=1.5,max_nfev=150,diff_step=1e-4)
        R,t=unpack(f.x);dist=tree.query((points-t)@R)[0]
        x=np.r_[t,Rotation.from_matrix(R).as_euler('ZYX')]
        results.append((float(np.mean(2*1.5**2*(np.sqrt(1+(dist/1.5)**2)-1))),x,dist))
    return sorted(results,key=lambda q:q[0])


def main():
    root = Path(__file__).resolve().parents[3]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('frame', type=Path, help='saved NPZ with depth (m), color BGR, K, T_BC')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--roi', type=int, nargs=4, metavar=('X0','Y0','X1','Y1'), help='manual image region selecting target, not a 3D pose')
    ap.add_argument('--board-plane',type=Path,help='NPZ containing measured board normal and offset in metres; fixes seating plane and connector axis')
    ap.add_argument('--six-dof', action='store_true', help='also estimate pitch and roll; diagnostic, can overfit distorted depth')
    ap.add_argument('--geometry', choices=['housing','above-panel'], default='housing', help='above-panel also models strain relief and CAD cable stub')
    ap.add_argument('--clip-roi', action='store_true', help='also restrict depth mask to manual ROI; diagnostic segmentation experiment')
    ap.add_argument('--dark-v', type=int, default=90)
    ap.add_argument('--radius-mm', type=float, default=15)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    frame = np.load(args.frame)
    depth, color, K, T = [frame[k] for k in ('depth', 'color', 'K', 'T_BC')]
    cfg = yaml.safe_load((root/'Code/Grasp/config/grasp.yaml').read_text())['color_marker']
    cfg.update(dark_v=args.dark_v, footprint_radius_m=args.radius_mm/1000)
    region_color = color.copy()
    if args.roi:
        x0,y0,x1,y1=args.roi
        region_color[:] = 255
        region_color[y0:y1,x0:x1] = color[y0:y1,x0:x1]
    marker = marker_mask(region_color, cfg)
    v, u = np.nonzero(marker & np.isfinite(depth) & (depth > 0))
    if len(v) < 10 and args.roi:
        x0,y0,x1,y1=args.roi
        region = np.zeros(depth.shape, bool)
        region[y0:y1,x0:x1] = True
        v,u = np.nonzero(region & np.isfinite(depth) & (depth>0))
    if len(v) < 10:
        raise RuntimeError('Too few sticker depth pixels to select target region')
    def backproject(v, u):
        z = depth[v, u]
        return np.c_[(u-K[0,2])*z/K[0,0], (v-K[1,2])*z/K[1,1], z] @ T[:3,:3].T + T[:3,3]
    center = np.median(backproject(v,u), axis=0)
    if args.roi:
        # Restrict colour selection before connected components; background blue must not win.
        hsv = cv2.cvtColor(region_color, cv2.COLOR_BGR2HSV)
        mask = (hsv[...,2] < args.dark_v) | marker
        mask = cv2.morphologyEx(mask.astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)).astype(bool)
    else:
        mask, _ = head_mask(color, K, T, center, cfg)
    if args.clip_roi:
        if not args.roi:
            raise ValueError('--clip-roi requires --roi')
        region = np.zeros(depth.shape, bool)
        x0,y0,x1,y1=args.roi
        region[y0:y1,x0:x1] = True
        mask &= region
    v, u = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    points = backproject(v,u)*1000
    if len(points) < 100:
        raise RuntimeError('Fewer than 100 target depth points')
    mesh = trimesh.load(root/'CAD_files/USB_A_Plug.glb', force='mesh')
    surface, faces = trimesh.sample.sample_surface(mesh, 40000, seed=0)
    # Housing only; STEP-derived GLB is metres, metal end at +z.
    housing = (surface[:,2] <= .010001)
    if args.geometry == 'housing':
        housing &= surface[:,2] >= -.010001
    model = surface[housing]*1000
    model = np.c_[model[:,0], -model[:,1], 10-model[:,2]]
    rng = np.random.default_rng(0)
    order = rng.permutation(len(points)); split = int(.8*len(points))
    train, test = points[order[:split]], points[order[split:]]
    if args.board_plane:
        plane=np.load(args.board_plane)
        solutions=register_on_plane(train,model,plane['normal'],float(plane['offset']))
    else:
        solutions = register(train, model, args.six_dof)
    score, x, _ = solutions[0]
    aligned = model @ rotation(x[3], *x[4:]).T + x[:3]
    tree = cKDTree(model)
    distances = tree.query((points-x[:3]) @ rotation(x[3], *x[4:]))[0]
    held = tree.query((test-x[:3]) @ rotation(x[3], *x[4:]))[0]
    candidates = [dict(score_mm2=q[0], translation_mm=q[1][:3].tolist(),
                       yaw_deg=float(np.degrees(q[1][3])%360)) for q in solutions]
    near=[q for q in candidates if q['score_mm2'] <= candidates[0]['score_mm2']*1.2+1e-6]
    yaw_spread=max(abs((q['yaw_deg']-candidates[0]['yaw_deg']+90)%180-90) for q in near)
    report = dict(frame=str(args.frame.resolve()), model='CAD_files/USB_A_Plug.glb',
        board_plane=str(args.board_plane) if args.board_plane else None, geometry=args.geometry, six_dof=args.six_dof, method=('panel-constrained CAD fit; seated and perpendicular assumption' if args.board_plane else 'robust observed-to-CAD ICP; 24 yaw starts; no ArUco'),
        rotation_matrix=rotation(x[3], *x[4:]).tolist(),
        tilt_deg=float(np.degrees(np.arccos(np.clip(rotation(x[3], *x[4:])[2,2],-1,1)))),
        region='dark housing plus sticker within image radius; may include cable or depth artefacts',
        roi_pixels=args.roi, clip_roi=args.clip_roi, dark_v=args.dark_v, radius_mm=args.radius_mm, points=len(points), train_points=len(train), held_out_points=len(test),
        translation_mm=x[:3].tolist(), yaw_deg=float(np.degrees(x[3])%360),
        all_points_median_p95_mm=np.percentile(distances,[50,95]).tolist(),
        held_out_median_p95_mm=np.percentile(held,[50,95]).tolist(),
        held_out_within_2mm_percent=float(np.mean(held<2)*100),
        near_optimal_yaw_spread_deg=float(yaw_spread),
        ambiguous=bool(yaw_spread>10),
        fit_gate=dict(p95_limit_mm=5.0, min_within_2mm_percent=80.0, max_near_optimal_yaw_spread_deg=10, near_optimal_cost_ratio=1.2, purpose='provisional residual and ambiguity rejection screen, not calibrated accuracy certification'),
        fit_gate_pass=bool(np.percentile(held,95)<=5 and np.mean(held<2)>=.8 and yaw_spread<=10),
        status='OFFLINE_DIAGNOSTIC_ONLY_NOT_APPROVED_FOR_EXECUTION',
        assumptions=[('CAD origin seated on measured board plane; axis perpendicular to board' if args.board_plane else ('pitch and roll estimated; partial geometry may be ambiguous' if args.six_dof else 'connector upright in robot base; no roll/pitch estimation')),
                     'fixed CAD scale; color-depth alignment and T_BC assumed correct',
                     'random pixel holdout is not independent multi-view validation',
                     'one-sided residual can be ambiguous on partial planar surfaces'],
        solutions=candidates)
    (args.out/'fit.json').write_text(json.dumps(report,indent=2))
    Tb = np.eye(4); Tb[:3,:3]=rotation(x[3], *x[4:]); Tb[:3,3]=x[:3]/1000
    np.savez_compressed(args.out/'fit.npz', observed_m=points/1000, aligned_cad_m=aligned/1000,T_base_plug=Tb, residual_mm=distances)
    preview = color.copy(); preview[mask] = (preview[mask]*.4 + np.array([0,180,0])*.6).astype('uint8')
    cam = (aligned/1000-T[:3,3]) @ T[:3,:3]
    uv = np.rint(cam[:,:2]/cam[:,2,None] * [K[0,0],K[1,1]] + [K[0,2],K[1,2]]).astype(int)
    for p in uv[::40]:
        if 0<=p[0]<color.shape[1] and 0<=p[1]<color.shape[0]:
            cv2.circle(preview,tuple(p),1,(255,0,255),-1)
    cv2.imwrite(str(args.out/'overlay_full.png'), preview)
    ys,xs=np.nonzero(mask); pad=55
    crop=preview[max(0,ys.min()-pad):ys.max()+pad,max(0,xs.min()-pad):xs.max()+pad]
    cv2.imwrite(str(args.out/'overlay_close.png'),cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
    fig, axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for ax, dims, labels in zip(axes,[(0,1),(0,2)],[('Base X (mm)','Base Y (mm)'),('Base X (mm)','Base Z (mm)')]):
        a,b=dims
        ax.scatter(aligned[::4,a],aligned[::4,b],s=2,c='magenta',alpha=.25,label='Fitted CAD surface')
        ax.scatter(points[:,a],points[:,b],s=5,c=distances,cmap='viridis',vmin=0,vmax=8,label='Observed depth')
        ax.set(xlabel=labels[0],ylabel=labels[1]);ax.set_aspect('equal');ax.legend(loc='upper center',bbox_to_anchor=(.5,1.15),fontsize=8)
    fig.suptitle(('Six-DOF' if args.six_dof else 'Upright') + ' CAD fit: magenta model, depth coloured by residual (0–8 mm)')
    fig.savefig(args.out/'cloud_overlay.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
    ax.hist(held,bins=30); ax.set(xlabel='Held-out point distance to CAD surface sample (mm)',ylabel='Pixel count',title='Fit residuals: held-out pixels from the same frame')
    fig.savefig(args.out/'residuals.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='solutions'},indent=2))

if __name__=='__main__':
    main()
