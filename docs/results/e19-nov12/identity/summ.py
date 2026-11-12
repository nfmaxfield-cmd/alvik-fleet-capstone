import json, sys
for f in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f)]
    for cond in ("held", "held, never still"):
        rs = [r for r in rows if r["cond"] == cond]
        both_named = sum(r["both"] for r in rs)
        both_right = sum(1 for r in rs if r["both"] and not r["wrong"])
        any_wrong = sum(1 for r in rs if r["wrong"])
        wrong_names = sum(len(r["wrong"]) for r in rs)
        named = sum(len(r["named"]) for r in rs)
        right = named - wrong_names
        print("%-22s %-18s trials %d | both named %2d | both named RIGHT %2d | trials with a wrong name %2d | names: %d given, %d right, %d wrong" % (f, cond, len(rs), both_named, both_right, any_wrong, named, right, wrong_names))
