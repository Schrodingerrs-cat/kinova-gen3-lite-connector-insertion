"""Measured image templates anchored at user-labelled hole centers."""
from pathlib import Path
import cv2
import numpy as np


def appearance(patch):
    lab = cv2.cvtColor(np.asarray(patch,np.uint8),cv2.COLOR_BGR2LAB).astype(float)
    y,x = np.mgrid[:len(lab),:len(lab)]
    radius = np.hypot(x-(len(lab)-1)/2,y-(len(lab)-1)/2)
    center = np.median(lab[radius<=1.5],axis=0)
    background = np.median(lab[(radius>=6)&(radius<=8)],axis=0)
    return float(np.linalg.norm(center-background))


def edge_image(image):
    """Unsigned color edges retain hole outlines when a reflective rim changes brightness."""
    lab=cv2.cvtColor(np.asarray(image,np.uint8),cv2.COLOR_BGR2LAB).astype(np.float32)
    lab=cv2.GaussianBlur(lab,(3,3),.8)
    gx=cv2.Sobel(lab,cv2.CV_32F,1,0,ksize=3)
    gy=cv2.Sobel(lab,cv2.CV_32F,0,1,ksize=3)
    return np.sqrt(np.sum(gx*gx+gy*gy,axis=2))


def circle_center(image,expected,search=6):
    u,v=np.rint(expected).astype(int);pad=14
    x0,x1=max(0,u-pad),min(image.shape[1],u+pad+1);y0,y1=max(0,v-pad),min(image.shape[0],v+pad+1)
    if x1-x0<15 or y1-y0<15:return None
    lab=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2LAB).astype(np.float32)
    background=np.median(lab[:,:,1:].reshape(-1,2),axis=0)
    contrast=np.linalg.norm(lab[:,:,1:]-background,axis=2).astype(np.float32)
    contrast=cv2.GaussianBlur(contrast,(3,3),.5)
    angles=np.arange(24)*2*np.pi/24;directions=np.column_stack([np.cos(angles),np.sin(angles)])
    def evaluate(centers,radii):
        cc=np.repeat(centers,len(radii),axis=0);rr=np.tile(radii,len(centers))
        inner=cc[:,None,:]+np.maximum(.5,rr-.75)[:,None,None]*directions[None,:,:]
        outer=cc[:,None,:]+(rr+1.25)[:,None,None]*directions[None,:,:]
        inside=cv2.remap(contrast,inner[:,:,0].astype(np.float32),inner[:,:,1].astype(np.float32),cv2.INTER_LINEAR)
        outside=cv2.remap(contrast,outer[:,:,0].astype(np.float32),outer[:,:,1].astype(np.float32),cv2.INTER_LINEAR)
        delta=inside-outside;coverage=np.mean(delta>6.,axis=1);strength=np.median(delta,axis=1)
        score=np.where((coverage>=.65)&(strength>=8),strength*coverage,0.)
        best=int(np.argmax(score));return score[best],cc[best],rr[best],coverage[best]
    dx,dy=np.meshgrid(np.arange(-search,search+1),np.arange(-search,search+1))
    centers=np.column_stack([dx.ravel(),dy.ravel()])+np.array(expected)-[x0,y0]
    score,center,radius,coverage=evaluate(centers,np.arange(1.5,6.01,.5))
    if score<=0:return None
    dx,dy=np.meshgrid(np.arange(-.75,.76,.25),np.arange(-.75,.76,.25))
    fine=np.column_stack([dx.ravel(),dy.ravel()])+center
    score,center,radius,coverage=evaluate(fine,np.arange(max(1.5,radius-.5),min(6.,radius+.5)+.01,.25))
    return center+[x0,y0],radius,float(score),float(coverage)


def template_data(image, points):
    points = np.asarray(points,float).reshape(12,2)
    patches,radii,contrasts = [],[],[]
    yy,xx = np.mgrid[-8:9,-8:9];distance=np.hypot(xx,yy)
    for point in points:
        if np.any(point<9) or np.any(point>=np.array(image.shape[1::-1])-9):
            raise ValueError('a labelled hole is too near the image boundary')
        patch = cv2.getRectSubPix(image,(17,17),tuple(point.astype(float)))
        contrast = appearance(patch)
        if contrast<8:
            raise ValueError('a labelled hole has insufficient image contrast')
        lab = cv2.cvtColor(patch,cv2.COLOR_BGR2LAB).astype(float)
        center = np.median(lab[distance<=1.5],axis=0)
        background = np.median(lab[(distance>=6)&(distance<=8)],axis=0)
        foreground=(np.linalg.norm(lab-center,axis=2)<.75*contrast)&(distance<=6)
        foreground[8,8]=True
        _,labels,_,_=cv2.connectedComponentsWithStats(foreground.astype(np.uint8))
        component=labels==labels[8,8]
        radius=float(np.clip(np.sqrt(component.sum()/np.pi),1.5,5.))
        patches.append(patch);radii.append(radius);contrasts.append(contrast)
    return dict(templates=np.asarray(patches),seed_points=points,radii_px=np.asarray(radii),
                contrasts=np.asarray(contrasts),image_size_px=np.array(image.shape[1::-1]))


class HoleTemplates:
    def __init__(self,path):
        with np.load(path,allow_pickle=False) as data:
            self.patches=np.asarray(data['templates'],np.float32)
            self.seed_points=np.asarray(data['seed_points'],float)
            self.radii=np.asarray(data['radii_px'],float)
            self.contrasts=np.asarray(data['contrasts'],float)
            self.image_size=np.asarray(data['image_size_px'],int)
        if (self.patches.shape!=(12,17,17,3) or self.seed_points.shape!=(12,2)
                or self.radii.shape!=(12,) or self.contrasts.shape!=(12,)
                or not all(np.isfinite(a).all() for a in (self.patches,self.seed_points,self.radii,self.contrasts))
                or np.any(self.radii<=0) or np.any(self.contrasts<8)):
            raise ValueError('invalid hole-template calibration')
        self.edges=np.asarray([edge_image(patch) for patch in self.patches])

    def detect(self,image,expected):
        if image.ndim!=3 or image.shape[2]!=3 or np.any(np.array(image.shape[1::-1])!=self.image_size):
            raise ValueError('calibrated hole tracking requires the original-size color image')
        expected=np.asarray(expected,float).reshape(12,2)
        H,inliers=cv2.findHomography(self.seed_points,expected,cv2.RANSAC,3.)
        if H is None or inliers.sum()<4 or not np.isfinite(H).all():return []
        try:inverse=np.linalg.inv(H)
        except np.linalg.LinAlgError:return []
        rectified=cv2.warpPerspective(image,inverse,tuple(self.image_size))
        edges=edge_image(rectified)
        candidates=[];strong=set();supported=[]
        for index,point in enumerate(self.seed_points):
            if np.any(expected[index]<9) or np.any(expected[index]>=self.image_size-9):continue
            circle=circle_center(image,expected[index])
            if circle is not None:
                point,radius,_signal,_coverage=circle
                candidates.append((np.pi*radius**2,point,radius));strong.add(index)
                continue
            u,v=np.rint(point).astype(int)
            x0,x1=max(0,u-20),min(image.shape[1],u+21)
            y0,y1=max(0,v-20),min(image.shape[0],v+21)
            if x1-x0<17 or y1-y0<17:continue
            crop=edges[y0:y1,x0:x1]
            if min(crop.shape[:2])<17:continue
            scores=cv2.matchTemplate(crop,self.edges[index],cv2.TM_CCOEFF_NORMED)
            _,score,_,peak=cv2.minMaxLoc(scores)
            if not np.isfinite(score) or score<.75:continue
            px,py=peak
            alternative=scores.copy()
            alternative[max(0,py-3):py+4,max(0,px-3):px+4]=-1
            margin=score-float(alternative.max())
            if margin<.08:continue
            offset=[]
            for axis,p in [(1,px),(0,py)]:
                if p==0 or p==scores.shape[axis]-1:
                    offset.append(0.);continue
                a,b,c=(scores[py,px-1:px+2] if axis==1 else scores[py-1:py+2,px])
                denominator=float(a-2*b+c)
                offset.append(float(np.clip(.5*(a-c)/denominator,-.5,.5)) if denominator<-1e-6 else 0.)
            center=np.array([x0+px+8+offset[0],y0+py+8+offset[1]])
            patch=cv2.getRectSubPix(rectified,(17,17),tuple(center))
            if appearance(patch)<8.:continue
            center=cv2.perspectiveTransform(center.reshape(1,1,2),H).reshape(2)
            candidate=(np.pi*self.radii[index]**2,center,self.radii[index])
            if score>=.8:
                candidates.append(candidate);strong.add(index)
            elif margin>=.15 and np.linalg.norm(center-expected[index])<=3.:
                supported.append((index,candidate))
        # A marginal edge match needs three strong, non-collinear neighbours in
        # its own rectangle and a tight temporal gate. It is still a measured peak.
        for index,candidate in supported:
            peers=[i for i in range(4*(index//4),4*(index//4)+4) if i in strong]
            if len(peers)==3 and abs(cv2.contourArea(expected[peers].astype(np.float32)))>50:
                candidates.append(candidate)
        return candidates
