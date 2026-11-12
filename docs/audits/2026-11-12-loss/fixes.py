import csv,json,bisect,collections,sys
SC='/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/'
O=json.load(open(SC+'lossaudit/onsets.json'))
cache={}
def fo(run):
    if run in cache: return cache[run]
    import glob
    p=[g for g in glob.glob(SC+'**/'+run+'/follow.csv',recursive=True)][0]
    d=collections.defaultdict(list)
    for r in csv.DictReader(open(p)):
        try: d[r['robot']].append((float(r['t_recv']),int(r['fixes_used']),int(r['fixes_rejected']),float(r['pos_sd_cm'])))
        except ValueError: pass
    cache[run]=d; return d
for o in O:
    F=fo(o['run'])[o['robot']]; ft=[f[0] for f in F]
    a=F[max(0,bisect.bisect(ft,o['t_on'])-1)]; b=F[min(len(F)-1,bisect.bisect(ft,o['t_on']+o['build_s']+o['dur_s']))]
    o['fix_used']=b[1]-a[1]; o['fix_rej']=b[2]-a[2]; o['sd_on']=a[3]; o['span']=round(b[0]-a[0],1)
json.dump(O,open(SC+'lossaudit/onsets.json','w'),indent=1)
for o in O[:12]+O[93:]: print(o['run'][4:13],o['robot'][:5],'span',o['span'],'used',o['fix_used'],'rej',o['fix_rej'],'sd',o['sd_on'],'peak',o['peak'],'ownmv',o['own_moved'],'cammv',o['cam_moved'],o['kinds'][:5])
