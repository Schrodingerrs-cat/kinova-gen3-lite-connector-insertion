# static MuJoCo scene of all exported CAD parts; renders stills + orbit video
import sys,glob,os,numpy as np,mujoco,imageio.v2 as iio
parts,out=sys.argv[1],sys.argv[2]
col={"gripper_model":"0.1 0.1 0.1 1","USB_A_Plug":"0.1 0.4 0.9 1","Conector_AC":"0.9 0.2 0.2 1","Connector_TRRS":"0.1 0.7 0.2 1",
     "Wall_for_cables":"0.95 0.6 0.1 1"}
arm={"base_model","shoulder","arm","forearm","lower_wrist","upper_wrist"}
a="<mujoco><visual><global offwidth='1280' offheight='960'/></visual><asset>"
b="<worldbody><light pos='0 -1 1' dir='0 1 -1'/><light pos='0 1 1' dir='0 -1 -1'/><body euler='90 0 0'>"  # CAD is Y-up
for f in sorted(glob.glob(parts+"/*.stl")):
    n=os.path.basename(f)[:-4]
    a+=f"<mesh name='{n}' file='{os.path.abspath(f)}'/>"
    c=col.get(n,"0.55 0.55 0.6 1" if n in arm else "0.8 0.8 0.8 1")
    b+=f"<geom type='mesh' mesh='{n}' rgba='{c}' contype='0' conaffinity='0'/>"
m=mujoco.MjModel.from_xml_string(a+"</asset>"+b+"</body></worldbody></mujoco>")
d=mujoco.MjData(m); mujoco.mj_forward(m,d)
r=mujoco.Renderer(m,960,720); cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
def shot(lookat,dist,az,el):
    cam.lookat[:]=[lookat[0],-lookat[2],lookat[1]]; cam.distance=dist; cam.azimuth=az; cam.elevation=el
    r.update_scene(d,cam); return r.render()
views={"overview":([0,0.12,-0.3],1.5,60,-20),"insertion_closeup":([0,0.12,-0.5],0.4,60,-15),
       "cradle_closeup":([0,0.0,-0.25],0.5,60,-30),"side":([0,0.15,-0.3],1.5,0,0)}
for k,v in views.items(): iio.imwrite(f"{out}/screenshots/{k}.png",shot(*v))
iio.mimsave(f"{out}/videos/orbit.mp4",[shot([0,0.12,-0.3],1.4,az,-20) for az in range(0,360,3)],fps=20)
