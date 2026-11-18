#!/usr/bin/env python3
"""Offline board-plane comparison and one joint CAD fit to three saved views.
Uses exactly the target points retained by each previous fit. No ROS imports.
"""
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fit_cad_cloud import register,rotation,register_on_plane

ROOT=Path(__file__).resolve().parents[3]
IDS=['1794783676530903674','1794783719890507798','1794783802114166114']


def board_plane(frame,xy):
    depth,color,K,C=[frame[k] for k in ('depth','color','K','T_BC')]
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    valid=np.isfinite(depth)&(depth>0)&(hsv[...,1]<100)&(hsv[...,2]>110)
    v,u=np.nonzero(valid);z=depth[v,u]
    points=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@C[:3,:3].T+C[:3,3]
    keep=(np.linalg.norm(points[:,:2]-xy,axis=1)<.06)&(points[:,2]>-.025)&(points[:,2]<.015)
    P=points[keep]
    if len(P)<300:raise ValueError('Not enough board points; inspect image instead of assuming a plane')
    rng=np.random.default_rng(0);best=None;count=0
    for _ in range(600):
        q=P[rng.choice(len(P),3,replace=False)];n=np.cross(q[1]-q[0],q[2]-q[0]);length=np.linalg.norm(n)
        if length<1e-10:continue
        n/=length
        if n[2]<0:n=-n
        if n[2]<np.cos(np.deg2rad(15)):continue
        inlier=abs((P-q[0])@n)<.002
        if inlier.sum()>count:best,count=inlier,int(inlier.sum())
    if best is None:raise ValueError('No approximately horizontal board plane')
    center=P[best].mean(0);n=np.linalg.svd(P[best]-center,full_matrices=False)[2][2]
    if n[2]<0:n=-n
    offset=-n@center;res=abs(P[best]@n+offset)*1000
    height=-(n[:2]@xy+offset)/n[2]
    selection=np.zeros(depth.shape,bool);selection[v[keep],u[keep]]=True
    return dict(points=len(P),inlier_fraction=count/len(P),normal=n.tolist(),offset_m=float(offset),height_at_common_xy_mm=float(height*1000),inlier_median_p95_mm=np.percentile(res,[50,95]).tolist()),selection


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--prepare-only',action='store_true');ap.add_argument('--board-constrained',action='store_true',help='diagnostic: assume connector seated on and perpendicular to consensus board plane');ap.add_argument('--seating-gap-mm',type=float,default=0.,help='assumed exposed metal height along board normal, diagnostic only');args=ap.parse_args()
    if not np.isfinite(args.seating_gap_mm) or args.seating_gap_mm<0:ap.error('seating gap must be finite and nonnegative')
    if args.seating_gap_mm and not args.board_constrained:ap.error('--seating-gap-mm requires --board-constrained')
    suffix=('_board_constrained'+('_gap_%gmm'%args.seating_gap_mm if args.seating_gap_mm else '')) if args.board_constrained else ''
    out=ROOT/'diagnostics/cad_fit'/('recent_three_1794783802114166114'+suffix);out.mkdir(parents=True,exist_ok=True)
    frames=[];clouds=[];xy=np.array([.272,-.018]);planes=[]
    for name in IDS:
        folder=ROOT/'Media/Grasp/work/cad_live'/name
        with np.load(folder/'frame.npz') as f:frame={k:f[k] for k in f.files}
        with np.load(folder/'fit.npz') as f:clouds.append(f['observed_m']*1000)
        plane,mask=board_plane(frame,xy);plane['capture']=name;planes.append(plane);frames.append(frame)
        preview=frame['color'].copy();preview[mask]=(preview[mask]*.5+[0,120,0]).clip(0,255).astype('uint8');cv2.imwrite(str(out/(name+'_board_selection.png')),preview)
        print('View',name,'board height %.2f mm, board inlier p95 %.2f mm'%(plane['height_at_common_xy_mm'],plane['inlier_median_p95_mm'][1]),flush=True)
    board=dict(reference_xy_m=xy.tolist(),planes=planes,height_spread_mm=float(np.ptp([p['height_at_common_xy_mm'] for p in planes])),selection='White points within60mm of commonXY and base Z[-25,15]mm; horizontal-plane RANSAC. Inspect masks; not ground truth.')
    (out/'board_comparison.json').write_text(json.dumps(board,indent=2))
    if args.prepare_only:
        print('Inputs and board-plane preparation complete. No CAD fitting or robot commands.');return
    mesh=trimesh.load(ROOT/'CAD_files/USB_A_Plug.glb',force='mesh');P,_=trimesh.sample.sample_surface(mesh,40000,seed=0);P=P[P[:,2]<=.010001]*1000;model=np.c_[P[:,0],-P[:,1],10-P[:,2]]
    train=[];tests=[];rng=np.random.default_rng(0)
    for points in clouds:
        idx=rng.permutation(len(points));split=int(.8*len(points));tr=points[idx[:split]]
        train.append(tr[rng.choice(len(tr),min(400,len(tr)),replace=False)]);tests.append(points[idx[split:]])
    print('Fitting ONE CAD pose to combined points from all three views...',flush=True)
    if args.board_constrained:
        normal=np.mean([p['normal'] for p in planes],axis=0);normal/=np.linalg.norm(normal)
        height=np.median([p['height_at_common_xy_mm'] for p in planes])/1000
        offset=-normal@np.r_[xy,height]-args.seating_gap_mm/1000
        solutions=register_on_plane(np.vstack(train),model,normal,offset)
    else:
        solutions=register(np.vstack(train),model,six_dof=True)
    score,x,_=solutions[0];R=rotation(x[3],*x[4:]);aligned=model@R.T+x[:3];tree=cKDTree(model);metrics=[]
    for name,test in zip(IDS,tests):
        dist=tree.query((test-x[:3])@R)[0];metrics.append(dict(capture=name,median_p95_mm=np.percentile(dist,[50,95]).tolist(),within_2mm_percent=float(100*np.mean(dist<2))))
    near=[s for s in solutions if s[0]<=score*1.2+1e-6];spread=max(abs((np.degrees(s[1][3]-x[3])+90)%180-90) for s in near)
    report=dict(assumed_seating_gap_mm=args.seating_gap_mm,board_constrained=args.board_constrained,board_constraint_assumption=('connector axis perpendicular to consensus board; housing origin raised by assumed seating gap; board variation remains unresolved' if args.board_constrained else None),captures=IDS,method=('Joint CAD fit constrained to measured board normal and assumed housing height' if args.board_constrained else 'One joint six-DOF CAD fit to exact saved target points; no ICP corrections, no tag pose, no new hand-eye'),per_view=metrics,translation_mm=x[:3].tolist(),rotation_matrix=R.tolist(),near_optimal_yaw_spread_deg=float(spread),board_height_spread_mm=board['height_spread_mm'],surface_screen_pass=bool(all(m['median_p95_mm'][1]<=5 and m['within_2mm_percent']>=80 for m in metrics) and spread<=10),status='OFFLINE_DIAGNOSTIC_ONLY_NO_EXECUTION_PLAN',limitations=['Random pixel holdout is not independent pose ground truth','Partial-surface fit can remain biased even with low residuals','Board segmentation must be inspected','No robot scene collision checks performed'])
    (out/'joint_fit.json').write_text(json.dumps(report,indent=2));T=np.eye(4);T[:3,:3]=R;T[:3,3]=x[:3]/1000;np.savez_compressed(out/'joint_fit.npz',T_base_plug=T,observed_m=np.vstack(clouds)/1000,aligned_cad_m=aligned/1000)
    fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
    for i,points in enumerate(clouds):
        for ax,(j,k) in zip(axes,[(0,1),(0,2)]):ax.scatter(points[:,j],points[:,k],s=5,label=f'View {i+1}',alpha=.45)
    for ax,(j,k) in zip(axes,[(0,1),(0,2)]):
        ax.scatter(aligned[::6,j],aligned[::6,k],c='black',s=2,alpha=.2,label='Joint CAD');ax.set(xlabel='Base X (mm)',ylabel='Base '+('Y' if k==1 else 'Z')+' (mm)');ax.set_aspect('equal')
    axes[0].legend(loc='upper center',bbox_to_anchor=(.5,1.16),ncol=4,fontsize=8);fig.suptitle('Three saved views: one joint CAD fit');fig.savefig(out/'joint_overlay.png',dpi=150);plt.close(fig)
    for name,frame in zip(IDS,frames):
        C,K=frame['T_BC'],frame['K'];cam=(aligned/1000-C[:3,3])@C[:3,:3];uv=np.rint(cam[:,:2]/cam[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int);img=frame['color'].copy()
        for pixel in uv[::40]:
            if 0<=pixel[0]<img.shape[1] and 0<=pixel[1]<img.shape[0]:cv2.circle(img,tuple(pixel),1,(255,0,255),-1)
        cv2.imwrite(str(out/(name+'_joint_overlay.png')),img)
    print(json.dumps(report,indent=2),flush=True);print('Saved results:',out,flush=True)

if __name__=='__main__':main()
