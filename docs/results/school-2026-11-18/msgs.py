import csv,sys,time
# prints a messages.csv compactly: msgs.py <file> [skip-substr ...]
rows=list(csv.reader(open(sys.argv[1])))[1:]
skip=sys.argv[2:]
prev=None;n=0;sk=0
for r in rows:
    if len(r) < 4: continue
    if any(s in r[3] for s in skip): sk+=1; continue
    key=(r[1],r[3][:60])
    if key==prev: n+=1; continue
    if n: print("      x%d more"%n)
    n=0; prev=key
    print(time.strftime("%H:%M:%S",time.gmtime(float(r[0])-5*3600)), r[1], r[3][:210])
print("skipped",sk,"of",len(rows))
