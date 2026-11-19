import json,os,sys,glob
root=sys.argv[1]
for d in sorted(os.listdir(root)):
    p=os.path.join(root,d)
    print("==",d)
    f=os.path.join(p,"mission_cmd.json")
    if os.path.exists(f):
        j=json.load(open(f)); a=j.get("args",{})
        print("  keys:",[k for k in j.keys()])
        for k in ("robots","visit","visit_order","points","binding","split","on_problem","site","mission","timeout_s"):
            if k in a: print("   ",k,"=",repr(a[k])[:300])
        print("   argv:", " ".join(j["argv"][1:])[:600])
        for k in j:
            if k not in("argv","args"): print("   ",k,":",repr(j[k])[:300])
    for name in ("visit_order.json","split.json"):
        f=os.path.join(p,name)
        if os.path.exists(f): print("  ",name,open(f).read()[:700].replace("\n"," "))
