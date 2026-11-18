"""Fresh colour frames with timestamp-matched robot TF. No motion commands."""
import time,json
from pathlib import Path
import cv2,numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data
from robot_ros import BASE,TOOL,matrix
from capture_quality import corners_refined,burst_measurement


def setup(robot):
    return robot.create_subscription(CameraInfo,'/camera/camera/color/camera_info',lambda m:setattr(robot,'color_info',m),qos_profile_sensor_data)


def capture(robot,aruco,folder,count=8):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    poses=[];corners=[];metadata=[];deadline=time.monotonic()+15
    cached=robot._img.get("color")
    last=(cached.header.stamp.sec*10**9+cached.header.stamp.nanosec) if cached is not None else -1
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    try:
        for i in range(count):
            if not robot.spin_until(lambda: 'color' in robot._img and stamp(robot._img['color'])>last and hasattr(robot,'color_info'),min(3,max(0,deadline-time.monotonic()))):
                raise ValueError('No fresh colour frame or color camera_info')
            msg=robot._img['color'];info=robot.color_info;last=stamp(msg)
            if not 0<=(robot.get_clock().now().nanoseconds-last)/1e9<1:raise ValueError('Colour image timestamp stale or incompatible with ROS clock')
            if msg.header.frame_id!=info.header.frame_id:raise ValueError('Colour image and camera_info frames differ')
            if info.distortion_model not in ('plumb_bob','rational_polynomial',''):raise ValueError('Unsupported distortion model '+info.distortion_model)
            if (msg.height,msg.width)!=(info.height,info.width):raise ValueError('Colour image and intrinsics dimensions differ')
            if msg.encoding not in ('rgb8','bgr8'):raise ValueError('Unsupported encoding '+msg.encoding)
            color=np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=msg.data,strides=(msg.step,3,1)).copy()
            if msg.encoding=='rgb8':color=cv2.cvtColor(color,cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(folder/f'frame_{i:02d}.png'),color)
            K=np.array(info.k).reshape(3,3);dist=np.array(info.d)
            if i==0:K0,dist0=K.copy(),dist.copy()
            elif not np.array_equal(K,K0) or not np.array_equal(dist,dist0):raise ValueError('Intrinsics changed during burst')
            uv=corners_refined(color,aruco['dictionary'],aruco['marker_id'])
            if uv[:,0].min()<8 or uv[:,1].min()<8 or uv[:,0].max()>msg.width-9 or uv[:,1].max()>msg.height-9:raise ValueError('Tag too close to image border; need >=8px margin')
            when=rclpy.time.Time.from_msg(msg.header.stamp)
            if not robot.spin_until(lambda:robot.tfbuf.can_transform(BASE,TOOL,when),2):raise ValueError('No robot TF at image timestamp; no latest-TF fallback')
            tr=robot.tfbuf.lookup_transform(BASE,TOOL,when).transform
            T=matrix([tr.translation.x,tr.translation.y,tr.translation.z],[tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
            poses.append(T);corners.append(uv)
            metadata.append(dict(stamp_ns=last,image_frame=msg.header.frame_id,info_frame=info.header.frame_id,K=K.tolist(),distortion=dist.tolist(),distortion_model=info.distortion_model,T_base_tool=T.tolist(),corners_px=uv.tolist()))
        BT,CM,quality=burst_measurement(corners,poses,K,dist,aruco['marker_size_m'])
        record=dict(accepted=True,frames=metadata,quality=quality,aruco=aruco,reference_frame_index=count//2,T_base_tool=BT.tolist(),T_cam_marker=CM.tolist(),note='Median image corners with middle image timestamp TF; stationary-burst gates required. Planar PnP ambiguity is recorded, not eliminated.')
        (folder/'capture.json').write_text(json.dumps(record,indent=2));return BT,CM,quality
    except Exception as exc:
        (folder/'capture.json').write_text(json.dumps(dict(accepted=False,error=str(exc),frames=metadata),indent=2));raise
