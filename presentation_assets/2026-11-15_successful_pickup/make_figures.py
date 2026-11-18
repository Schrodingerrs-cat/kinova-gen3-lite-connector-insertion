"""Presentation figures from the recorded successful run; no ROS/hardware."""
from pathlib import Path
import json,re,zipfile
import numpy as np,cv2
from scipy.spatial import cKDTree
from scipy.ndimage import maximum_filter,minimum_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
RUN=ROOT/'Media/Grasp/work/fresh_nearby_runs/1794789916872077379';MOTION=ROOT/'Media/Grasp/work/video_runs/1794789954005073051'
report=json.loads((RUN/'fit/fit.json').read_text());fit=np.load(RUN/'fit/fit.npz');P=fit['observed_m']*1000;M=fit['aligned_cad_m']*1000
COLORS=['#0072B2','#E69F00','#009E73'];NAMES=['View 1','View 2 · sideways','View 3'];plt.rcParams.update({'font.family':'DejaVu Sans','font.size':16,'axes.titlesize':20,'axes.labelsize':16,'legend.fontsize':12,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white'})
clouds=[];offset=0
for row in report['per_view']:
 n=row['points'];clouds.append(P[offset:offset+n]);offset+=n
assert offset==len(P)

def save(fig,name):
 if any(getattr(ax,'name','')=='3d' for ax in fig.axes):
  fig.set_layout_engine(None)
  for ax in fig.axes:
   ax.set_position([.06,.18,.88,.70])
   if ax.get_legend() is not None:
    handles,labels=ax.get_legend_handles_labels();ax.get_legend().remove();fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.02,.86),fontsize=12)
  if fig._suptitle:fig._suptitle.set_position((.5,.97))
 else:
  fig.set_layout_engine('constrained',rect=(0,.12,1,.86))
 fig.savefig(OUT/(name+'.png'),dpi=220,facecolor='white');fig.savefig(OUT/(name+'.svg'),facecolor='white');plt.close(fig)

def axes3d(ax,points):
 ax.xaxis.set_major_locator(MaxNLocator(4));ax.yaxis.set_major_locator(MaxNLocator(4));ax.zaxis.set_major_locator(MaxNLocator(5));span=np.ptp(points,axis=0);ax.set_box_aspect(np.maximum(span,1));ax.set(xlabel='Base X (mm)',ylabel='Base Y (mm)',zlabel='Base Z (mm)');ax.view_init(elev=24,azim=-58);ax.tick_params(labelsize=11);ax.xaxis.labelpad=12;ax.yaxis.labelpad=12;ax.zaxis.labelpad=10

# Exact target points used by the fitter, retaining view identity.
fig=plt.figure(figsize=(13,8),layout='constrained');ax=fig.add_subplot(111,projection='3d')
for name,col,cloud in zip(NAMES,COLORS,clouds):ax.scatter(*cloud.T,s=12,c=col,alpha=.8,label=f'{name}: {len(cloud):,} points',depthshade=False)
axes3d(ax,P);ax.legend(loc='upper left',bbox_to_anchor=(0,1.03));fig.suptitle('Combined connector point cloud',fontsize=26);fig.text(.5,.015,f'{len(P):,} segmented depth points · three fresh views · robot-base coordinates',ha='center',fontsize=14)
save(fig,'01_combined_connector_cloud')

fig=plt.figure(figsize=(13,8),layout='constrained');ax=fig.add_subplot(111,projection='3d')
ax.scatter(*M[::5].T,s=4,c='#858585',alpha=.22,label='Fitted CAD surface (includes hidden sides)',depthshade=False)
for name,col,cloud in zip(NAMES,COLORS,clouds):ax.scatter(*cloud.T,s=13,c=col,alpha=.9,label=name,depthshade=False)
axes3d(ax,np.vstack([P,M]));ax.legend(loc='upper left',bbox_to_anchor=(0,1.03));fig.suptitle('One CAD pose fitted to all three views',fontsize=26);fig.text(.5,.015,'Grey: CAD surface samples  |  Colour: measured connector points',ha='center',fontsize=14)
save(fig,'02_combined_cloud_with_cad')

# Whole-scene visualization, cleaned for display only; target fit uses P above.
scene=[];rgb=[];rng=np.random.default_rng(0)
for file in sorted((RUN/'scan').glob('view_*.npz')):
 f=np.load(file);d,c,K,T=[f[k] for k in ('depth','color','K','T_BC')];valid=np.isfinite(d)&(d>0);hi=maximum_filter(np.where(valid,d,0),3);lo=minimum_filter(np.where(valid,d,np.inf),3);valid&=(hi-lo)<.006;v,u=np.nonzero(valid);z=d[v,u];q=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]@T[:3,:3].T+T[:3,3];keep=(q[:,0]>.12)&(q[:,0]<.5)&(abs(q[:,1])<.22)&(q[:,2]>-.09)&(q[:,2]<.10);q=q[keep];col=c[v[keep],u[keep],::-1]/255.;idx=rng.choice(len(q),min(14000,len(q)),replace=False);scene.append(q[idx]*1000);rgb.append(col[idx])
Q=np.vstack(scene);C=np.vstack(rgb);fig=plt.figure(figsize=(13,8),layout='constrained');ax=fig.add_subplot(111,projection='3d');ax.scatter(*Q.T,c=C,s=1,alpha=.8,depthshade=False);axes3d(ax,Q);ax.view_init(elev=55,azim=-75);fig.suptitle('Combined RGB-D scene: board and workspace',fontsize=26);fig.text(.5,.015,'Three views transformed with recorded camera poses · cropped and depth-edge filtered for display',ha='center',fontsize=13)
save(fig,'03_combined_scene_cloud')

# Camera overlays, with enough local board context to interpret placement.
fig,axs=plt.subplots(1,3,figsize=(16,7),layout='constrained')
for ax,row,name in zip(axs,report['per_view'],NAMES):
 view=row['view'];img=cv2.imread(str(RUN/'fit'/(view+'_overlay.png')));x0,y0,x1,y1=report['rois'][view];pad=55;crop=img[max(0,y0-pad):min(img.shape[0],y1+pad),max(0,x0-pad):min(img.shape[1],x1+pad)];ax.imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));ax.set_title(name,pad=14);ax.axis('off')
fig.suptitle('CAD alignment in the three fresh camera views',fontsize=26);fig.text(.5,.025,'Magenta dots: projected CAD surface samples  ·  Full model projection includes hidden surfaces',ha='center',fontsize=14)
save(fig,'04_camera_cad_overlays')

# Actual recorded held-out pixel metrics and the disclosed experimental profile.
med=np.array([r['median_p95_mm'][0] for r in report['per_view']]);p95=np.array([r['median_p95_mm'][1] for r in report['per_view']]);fractions=np.array([r['within_2mm_percent'] for r in report['per_view']]);x=np.arange(3)
fig,axs=plt.subplots(1,2,figsize=(15,7),layout='constrained');a,b=axs
bars1=a.bar(x-.18,med,.36,color='#0072B2',label='Median');bars2=a.bar(x+.18,p95,.36,color='#E69F00',label='95th percentile');a.bar_label(bars1,fmt='%.2f',padding=5,fontsize=13);a.bar_label(bars2,fmt='%.2f',padding=5,fontsize=13);a.axhline(5,color='#555555',ls='--',lw=1.5,label='5 mm screen');a.set(xticks=x,xticklabels=['View 1','Sideways','View 3'],ylabel='Distance to CAD surface (mm)',ylim=(0,6.5),title='Held-out surface residuals');a.legend(loc='upper left',fontsize=12);a.grid(axis='y',alpha=.15);a.set_axisbelow(True)
bars=b.bar(x,fractions,color=COLORS,width=.55);b.bar_label(bars,labels=[f'{v:.1f}%' for v in fractions],padding=6,fontsize=14);b.axhline(70,color='#555555',ls='--',label='70% experimental screen');b.set(xticks=x,xticklabels=['View 1','Sideways','View 3'],ylabel='Held-out pixels within 2 mm (%)',ylim=(0,105),title='Agreement with the fitted CAD');b.legend(loc='lower left',fontsize=12);b.grid(axis='y',alpha=.15);b.set_axisbelow(True)
fig.suptitle('Fit quality in the successful demonstration',fontsize=25);fig.text(.5,.015,'Random held-out depth pixels; these are surface residuals, not absolute grasp accuracy. Profile: supervised_pickup.',ha='center',fontsize=13)
save(fig,'05_fit_quality_metrics')

# Stage endpoints from this run's actual controller log, not a simulated path.
log=(MOTION/'console.log').read_text();points=[np.fromstring(s,sep=' ') for s in re.findall(r'Measured tool XYZ:\s*\[([^\]]+)\]',log)];assert len(points)==5,len(points)
pre=json.load(open(MOTION/'joint_cad_pregrasp_plan.consumed.json'));heights=[np.array(pre['start_tool_matrix'])[2,3]*1000]+[p[2]*1000 for p in points];labels=['Home','Pregrasp','20 mm\nstandoff','Grasp','5 mm\ntest lift','Final lift']
fig,ax=plt.subplots(figsize=(14,7),layout='constrained');ax.plot(np.arange(6),heights,'o-',c='#0072B2',lw=3,ms=10);ax.set(xticks=np.arange(6),xticklabels=labels,ylabel='Measured tool-frame height (mm)',ylim=(0,310),title='Measured tool height through the pickup');ax.grid(axis='y',alpha=.2)
for i,h in enumerate(heights):ax.annotate(f'{h:.1f}',(i,h),xytext=(0,12),textcoords='offset points',ha='center',fontsize=15)
fig.text(.5,.02,'Recorded stage endpoints only—not a continuous trajectory. Commanded extraction: 5 mm + 50 mm.',ha='center',fontsize=14)
save(fig,'06_measured_pickup_stages')

# Auditable data/captions, suitable for report or PPT footnotes.
summary=dict(source_perception_run=str(RUN),source_motion_run=str(MOTION),views=3,target_points=len(P),metrics=report['per_view'],yaw_spread_deg=report['near_optimal_yaw_spread_deg'],fit_quality=report['fit_quality'],measured_tool_heights_mm=heights,physical_outcome='User reported successful extraction and holding; not independently verified by an in-hand sensor',success_rate='Not measured; one demonstration shown')
(OUT/'figure_data.json').write_text(json.dumps(summary,indent=2))
(OUT/'CAPTIONS.md').write_text('''# Figures for the successful fresh-perception USB-A pickup

All figures use perception run1794789916872077379 and motion run1794789954005073051 (2026-11-15). PNG files are high-resolution; SVG files support scaling/editing.

1. **Combined connector cloud:** Exact segmented points used for fitting, coloured by camera view.1426 points from three freshly captured RGB-D views, transformed into robot-base coordinates without table ICP.
2. **Combined cloud with CAD:** One rigid CAD pose fitted jointly to those views. CAD includes housing, strain relief and cable stub above the panel. Grey surfaces include sides unseen by the camera.
3. **Combined full scene:** RGB-coloured board/workspace points from the same captures, cropped and depth-edge filtered for visualization. This differs from the smaller segmented target cloud used by the fitter.
4. **Camera overlays:** Projected fitted CAD samples on all three real camera images, including the sideways viewpoint. The ArUco tag was not used to determine connector pose; the coloured housing sticker located the search region.
5. **Fit metrics:** Held-out pixel median / p95 errors:1.15/2.57mm,1.25/2.90mm,0.61/3.94mm. Within2mm:84.2%,76.2%,86.7%. These measure depth-to-CAD agreement, not absolute robot/grasp accuracy. Yaw spread10.54degrees. Experimental supervised_pickup profile used70% minimum inliers,5mm maximum p95 and15degree maximum yaw spread; strict profile would reject this run.
6. **Pickup stages:** Measured tool-frame heights from controller logs. Lines connect recorded stage endpoints and do not represent continuously sampled motion. The successful physical pickup was confirmed by the user; encoder feedback alone did not verify holding.

Suggested spoken description: “We captured three nearby RGB-D views, including a sideways view, combined the segmented connector points in the robot frame, and fitted the CAD model to that cloud. We transformed CAD grasp candidates into robot coordinates, then executed a Cartesian approach, gripper closure and55mm extraction. The run completed successfully under supervision.”

Do not claim100% success rate or submillimetre absolute grasp accuracy from this single demonstration. No uncertainty bounds on absolute pose or measured contact force were established.
''')
(OUT/'CAPTIONS.md').write_text((OUT/'CAPTIONS.md').read_text().replace('1426',f'{len(P):,}'))
archive=OUT.parent/'USB_A_pickup_PPT_figures.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
 for p in OUT.iterdir():
  if p.suffix in ('.png','.svg','.json','.md'):z.write(p,p.name)
print('Created',len(list(OUT.glob('*.png'))),'PNG figures plusSVG,data,captions.');print(archive)
