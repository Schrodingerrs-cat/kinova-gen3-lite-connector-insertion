# STEP assembly -> one world-frame STL per product (metres, CAD frame)
import re,sys,io,contextlib,collections,numpy as np,trimesh,cascadio
step,out=sys.argv[1],sys.argv[2]
glb=out+"/arm.glb"
cascadio.step_to_glb(step,glb,tol_linear=0.5,tol_angular=0.5,merge_primitives=False)
sys.argv=["x",step,"0"]
with contextlib.redirect_stdout(io.StringIO()):
    exec(open("Sim/RL/tools/step_tree.py").read())
nauo={nm:c for d,v in kids.items() for nm,c in v}
sc=trimesh.load(glb); g=sc.graph
grp=collections.defaultdict(list)
for n in g.nodes_geometry:
    T,gn=g[n]; m=sc.geometry[gn].copy(); m.apply_transform(T)
    k=nauo.get(n.split("_")[0])
    if k: grp[pname[k]].append(m)
for name,ms in grp.items():
    safe=re.sub(r"[^A-Za-z0-9]+","_",name).strip("_")
    trimesh.util.concatenate(ms).export(f"{out}/{safe}.stl")
    print(safe,len(ms))
