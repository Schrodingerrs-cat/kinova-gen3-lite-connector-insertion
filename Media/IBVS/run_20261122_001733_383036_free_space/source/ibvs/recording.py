"""Camera-timestamped video with a playable H.264 delivery option."""
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np


class TimedVideo:
    def __init__(self, path, fps=15., h264=False):
        self.path, self.fps, self.h264 = Path(path), float(fps), h264
        self.capture_path = self.path.with_suffix('.capture.mp4') if h264 else self.path
        self.writer = self.previous = None
        self.first_stamp = self.last_stamp = None
        self.frames = self.received = self.duplicated = 0

    def write(self, frame, stamp):
        if not np.isfinite(stamp):
            raise ValueError('video requires a finite camera timestamp')
        if self.first_stamp is None:
            self.first_stamp = stamp
            height, width = frame.shape[:2]
            self.writer = cv2.VideoWriter(str(self.capture_path), cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
            if not self.writer.isOpened():
                raise RuntimeError('cannot record '+str(self.capture_path))
        if self.last_stamp is not None and (stamp < self.last_stamp or stamp-self.last_stamp > 120.):
            raise RuntimeError('camera timestamp discontinuity during recording')
        self.last_stamp = stamp
        self.received += 1
        index = int(np.floor((stamp-self.first_stamp)*self.fps+1e-6))
        while self.frames < index:
            self.writer.write(self.previous)
            self.frames += 1
            self.duplicated += 1
        if self.frames == index:
            self.writer.write(frame)
            self.frames += 1
            self.previous = frame.copy()

    def release(self):
        if self.writer is None:
            return
        self.writer.release()
        self.writer = None
        if self.h264:
            print('Encoding '+self.path.name, flush=True)
            result = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(self.capture_path),
                                     '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p',
                                     '-movflags', '+faststart', str(self.path)], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError('video encoding failed: '+result.stderr[-500:])
        info = dict(file=self.path.name, fps=self.fps, frames=self.frames, duration_s=self.frames/self.fps,
                    received_frames=self.received, held_frames=self.duplicated,
                    first_camera_stamp=self.first_stamp, last_camera_stamp=self.last_stamp,
                    source='one continuous camera stream; no separate test runs combined')
        self.path.with_suffix('.json').write_text(json.dumps(info, indent=2)+'\n')
