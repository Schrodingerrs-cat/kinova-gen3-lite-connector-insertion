"""Offline board/target depth diagnostic using explicit image polygons (no tag pose)."""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from housing_grasp.color_marker import ransac_plane
ap=argparse.ArgumentParser();ap.add_argument('frame',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
f=np.load(a.frame);d,c,K,T=[f[k] for k in ('depth','color','K','T_BC')]
polygon=np.array([[344,225],[373,184],[423,270],[408,319]],np.int32)
b=np.zeros(d.shape,np.uint8);cv2.fillConvexPoly(b,polygon,1)
hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV);b=(b>0)&(hsv[...,1]<100)&(hsv[...,2]>110)
# Exclude connector and screw neighbourhoods using dark image pixels.
dark=(hsv[...,2]<100).astype('uint8');b &= cv2.dilate(dark,np.ones((7,7),np.uint8))==0
valid=np.isfinite(d)&(d>0);b &=valid
v,u=np.nonzero(b)
def points(v,u):
 z=d[v,u];return np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@T[:3,:3].T+T[:3,3]
P=points(v,u);n,offset,count=ransac_plane(P,thresh=.002,iters=800)
res=abs(P@n+offset)*1000
roi=np.zeros(d.shape,bool);roi[280:315,405:450]=True
m=roi&(hsv[...,2]<110)&valid
vv,uu=np.nonzero(m);G=points(vv,uu);height=(G@n+offset)*1000
report=dict(board_polygon_px=polygon.tolist(),board_points=len(P),normal_base=n.tolist(),plane_offset_m=float(offset),tilt_deg=float(np.degrees(np.arccos(n[2]))),inlier_fraction=count/len(P),board_residual_median_p95_mm=np.percentile(res,[50,95]).tolist(),connector_points=len(G),connector_height_p5_p50_p95_mm=np.percentile(height,[5,50,95]).tolist(),board_camera_depth_p5_p50_p95_mm=np.percentile(d[b]*1000,[5,50,95]).tolist(),connector_camera_depth_p5_p50_p95_mm=np.percentile(d[m]*1000,[5,50,95]).tolist(),note='Manual board polygon for this frame only; target may include boot/cable. Relative height is invariant to rigid hand-eye errors.')
(a.out/'board.json').write_text(json.dumps(report,indent=2));np.savez(a.out/'board.npz',normal=n,offset=offset,board_points=P,target_points=G)
preview=c.copy();preview[b]=(0,255,0);preview[m]=(0,0,255);cv2.imwrite(str(a.out/'board_selection.png'),preview)
print(json.dumps(report,indent=2))
