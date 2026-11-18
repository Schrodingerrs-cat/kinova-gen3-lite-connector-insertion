"""Explicit fit-screen profiles. Passing is not physical grasp verification."""
import numpy as np


def assess_fit(per_view,yaw_spread_deg,profile='strict'):
    profiles={'strict':(80.,10.),'supervised_pickup':(70.,15.)}
    if profile not in profiles:raise ValueError('Unknown fit profile')
    minimum,maximum=profiles[profile]
    valid=bool(per_view) and np.isfinite(yaw_spread_deg) and 0<=yaw_spread_deg<=maximum
    for row in per_view:
        p95=float(row['median_p95_mm'][1]);fraction=float(row['within_2mm_percent'])
        valid=valid and np.isfinite([p95,fraction]).all() and 0<=p95<=5 and minimum<=fraction<=100
    return dict(profile=profile,experimental=profile!='strict',thresholds=dict(max_p95_mm=5.,min_within_2mm_percent=minimum,max_yaw_spread_deg=maximum),accepted=bool(valid),note='Experimental pickup screen only; not pose accuracy or holding verification' if profile!='strict' else 'Provisional fit screen, not physical grasp verification')
