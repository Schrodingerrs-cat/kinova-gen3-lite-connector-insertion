#!/usr/bin/env python3
"""Resume detection and CAD fitting offline from a named saved scan. Never moves."""
import argparse,re,subprocess,sys,time
from pathlib import Path
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('scan_id');a=ap.parse_args()
if not re.fullmatch(r'\d{8}_\d{6}',a.scan_id):ap.error('Use scan ID YYYYMMDD_HHMMSS')
ROOT=Path(__file__).resolve().parents[3];scan=ROOT/'Media/Grasp/work/scans'/a.scan_id
if not scan.is_dir():ap.error('Saved scan not found')
out=ROOT/'diagnostics/cad_fit'/('saved_scan_'+a.scan_id+'_'+str(time.time_ns()));out.mkdir(parents=True)
print('OFFLINE ONLY: no ROS connection or robot commands. Saved scan:',scan,flush=True)
for script,flags in [('find_scan_target_rois.py',[scan,'--out',out/'detection']),('fit_cad_multiview.py',[scan,'--rois',out/'detection/rois.json','--clean-depth','--out',out/'fit'])]:
    result=subprocess.run([sys.executable,str(Path(__file__).parent/script),*map(str,flags)])
    if result.returncode:raise SystemExit('Stage failed. No pickup requested. Results: '+str(out))
print('Results:',out,flush=True);print('No pickup requested. Inspect fit before any execution.',flush=True)
