# print STEP assembly tree: product names, children counts (flat regex parse)
import re,sys,collections
txt=open(sys.argv[1],errors="ignore").read()
ent={}
for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;",txt,re.S):
    ent[int(m[1])]=(m[2],m[3])
def refs(s): return [int(x) for x in re.findall(r"#(\d+)",s)]
prod={i:re.match(r"\s*'([^']*)'",a).group(1) for i,(t,a) in ent.items() if t=="PRODUCT"}
form={i:refs(a)[-1] for i,(t,a) in ent.items() if t.startswith("PRODUCT_DEFINITION_FORMATION")}
pdef={i:form[refs(a)[0]] for i,(t,a) in ent.items() if t=="PRODUCT_DEFINITION" and refs(a) and refs(a)[0] in form}
pname={d:prod[f] for d,f in pdef.items() if f in prod}
kids=collections.defaultdict(list)
for i,(t,a) in ent.items():
    if t=="NEXT_ASSEMBLY_USAGE_OCCURRENCE":
        nm=re.match(r"\s*'([^']*)'",a).group(1); r=refs(a)
        kids[r[0]].append((nm,r[1]))
roots=set(pname)-{c for v in kids.values() for _,c in v}
def show(d,ind=0,maxd=int(sys.argv[2])):
    c=collections.Counter(pname.get(x) for _,x in kids[d])
    print("  "*ind+pname[d],f"({len(kids[d])} kids)")
    if ind<maxd:
        seen=set()
        for nm,x in kids[d]:
            if pname[x] in seen: continue
            seen.add(pname[x]); show(x,ind+1)
for r in roots: show(r)
