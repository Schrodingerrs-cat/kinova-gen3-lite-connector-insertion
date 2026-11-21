"""Measured camera-fixed gripper surfaces, explicitly labelled during calibration."""
from pathlib import Path
import cv2
import numpy as np


class SelfDepthTemplate:
    def __init__(self, data):
        self.mask = np.asarray(data['mask'], bool)
        self.depth = np.asarray(data['depth_m'], float)
        self.K = np.asarray(data['K'], float)
        self.distortion = np.asarray(data['distortion'], float)
        self.frame_id = str(np.asarray(data['frame_id']).item())
        self.gripper_position = float(data['gripper_position'])
        self.samples = np.asarray(data.get('sample_depths_m',self.depth[self.mask][None,:]),float)
        self.lab = None if 'lab' not in data else np.asarray(data['lab'],float)
        if (self.mask.ndim != 2 or self.depth.shape != self.mask.shape or self.mask.sum() < 100
                or not np.isfinite(self.depth[self.mask]).all() or np.any(self.depth[self.mask] <= 0)
                or self.K.shape != (3,3) or not np.isfinite(self.K).all()
                or not np.isfinite(self.distortion).all() or not np.isfinite(self.gripper_position)):
            raise ValueError('invalid measured gripper-depth template')
        if (self.samples.ndim!=2 or self.samples.shape[1]!=self.mask.sum()
                or not np.isfinite(self.samples).all()
                or (self.lab is not None and (self.lab.shape!=(self.mask.sum(),3) or not np.isfinite(self.lab).all()))):
            raise ValueError('invalid gripper appearance/depth samples')

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            return cls(dict(data))

    @classmethod
    def calibrate(cls, frames, approved_mask, K, distortion, frame_id, gripper_position, color=None):
        frames = np.asarray(frames, float)
        mask = np.asarray(approved_mask, bool)
        if frames.ndim != 3 or len(frames) < 5 or frames.shape[1:] != mask.shape:
            raise ValueError('at least five independent depth frames and a labelled mask are required')
        samples = frames[:,mask]
        valid = np.isfinite(samples) & (samples > 0)
        # Keep only pixels observed consistently across the stationary calibration.
        complete = valid.sum(axis=0)>=3 if color is not None else valid.all(axis=0)
        medians = np.full(samples.shape[1], np.nan)
        if complete.any():
            medians[complete] = np.nanmedian(np.where(valid[:,complete],samples[:,complete],np.nan),axis=0)
        stable = complete.copy()
        if color is None:
            stable[complete] &= np.ptp(samples[:,complete],axis=0) <= .004
        indices = np.flatnonzero(mask)
        selected = np.zeros(mask.shape,bool)
        selected.flat[indices[stable]] = True
        depth = np.zeros(mask.shape,float)
        depth.flat[indices[stable]] = medians[stable]
        data=dict(mask=selected,depth_m=depth,K=K,distortion=distortion,
                  frame_id=frame_id,gripper_position=gripper_position)
        if color is not None:
            if color.shape != (*mask.shape,3):raise ValueError('self calibration requires the matching color image')
            data['sample_depths_m']=np.where(valid[:,stable],samples[:,stable],0.)
            data['lab']=cv2.cvtColor(color,cv2.COLOR_BGR2LAB)[selected].astype(float)
        return cls(data)

    def save(self, path):
        data=dict(mask=self.mask,depth_m=self.depth,K=self.K,distortion=self.distortion,
                  frame_id=self.frame_id,gripper_position=self.gripper_position,sample_depths_m=self.samples)
        if self.lab is not None:data['lab']=self.lab
        np.savez_compressed(path,**data)

    def exclude(self, depth, K, distortion, frame_id, gripper_position, color=None):
        """Return a mask of verified self returns; retain changed/closer geometry."""
        if (depth.shape != self.mask.shape or frame_id != self.frame_id
                or np.shape(K) != self.K.shape or not np.allclose(K,self.K,rtol=0,atol=1e-6)
                or np.shape(distortion) != self.distortion.shape
                or not np.allclose(distortion,self.distortion,rtol=0,atol=1e-6)
                or not np.isfinite(gripper_position) or abs(gripper_position-self.gripper_position) > .01):
            raise ValueError('camera or gripper opening differs from self-depth calibration')
        values=depth[self.mask]
        valid=np.isfinite(values)&(values>0)
        agrees=np.any((self.samples>0)&(np.abs(self.samples-values[None,:])<=.003+1e-9),axis=0)&valid
        if self.lab is not None:
            if color is None or color.shape!=(*depth.shape,3):raise ValueError('matching color image required for gripper self-filter')
            current=cv2.cvtColor(color,cv2.COLOR_BGR2LAB)[self.mask].astype(float)
            # RGB-visible opaque fingers occlude farther aligned-depth returns.
            # Preserve closer surfaces and pixels whose appearance has changed.
            agrees=valid&(values>=self.depth[self.mask]-.003-1e-9)
            agrees &= (np.linalg.norm(current[:,1:]-self.lab[:,1:],axis=1)<=12)&(np.abs(current[:,0]-self.lab[:,0])<=35)
        matched=np.zeros(depth.shape,bool);matched[self.mask]=agrees
        denominator=valid.sum() if self.lab is not None else self.mask.sum()
        support=float(matched.sum()/max(1,denominator))
        if support < .8 or matched.sum()<100:
            raise ValueError('measured gripper depth no longer matches its calibration')
        return matched, dict(method='labelled visible gripper and self-occluded depth' if self.lab is not None else 'labelled measured gripper surfaces',
            template_pixels=int(self.mask.sum()),valid_template_pixels=int(valid.sum()),
            excluded_pixels=int(matched.sum()),support_fraction=support,tolerance_m=.003)


class SelfDepthBank:
    """Choose among independently measured appearances of the same camera/gripper."""
    def __init__(self,paths):
        self.models=[(Path(path).name,SelfDepthTemplate.load(path)) for path in paths]
        if not self.models:raise ValueError('at least one self-depth reference is required')

    def exclude(self,*args,**kwargs):
        accepted=[];errors=[]
        for name,model in self.models:
            try:
                mask,details=model.exclude(*args,**kwargs)
                accepted.append((details['support_fraction'],mask,dict(details,source_template=name)))
            except ValueError as error:errors.append(str(error))
        if not accepted:raise ValueError('no measured gripper reference matches: '+'; '.join(errors))
        _,mask,details=max(accepted,key=lambda item:item[0])
        return mask,details
