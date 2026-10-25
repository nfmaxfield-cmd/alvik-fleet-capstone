"""A live view of the E02 runner in a browser (http://localhost:8090 on the laptop): the overhead picture with
what the runner thinks, so the first minutes of a session are easy to watch (2026-10-22; the first screen of the
Live lab, docs/build-plans/live-lab-outline.md).

Drawn on the picture: the board outline; every robot the runner knows of (name, where it is parked or was found);
the robot being followed (green ring), and the hold circle while it should stand still; everything that differs
from the floor memory (yellow: robot-shaped, grey: not, e.g. a hand); the robot-map rays when mapping. Beside
it: the phase, the run, and the runner's last lines (camera / wheels checks, help calls). Read-only: the page can't
drive anything.

    started by e02.py (--view-port, default 8090; 0 = off); open http://localhost:8090 on the laptop
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>E02 live view</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="stylesheet" media="print" onload="this.media='all'" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600;700&display=swap">
<style>
:root{--bg:#0E1116;--panel:#151A22;--line:#222833;--fg:#E6E9EE;--mut:#8E98A7;--mut2:#9AA4B2;--amber:#E0A33A;--amber-t:#FFD58A;
--amber-bg:#3A2A0B;--follow:#7FB8FF;--board:#6FD3C1;--ok:#6CCB8F;--bad:#F07B72;--grid:#262D39;--slate:#3A4352;
--s1:#3987e5;--s2:#d95926;--s3:#199e70;--s4:#c98500;--runs:#4F9D74;--find:#8B79D9;
--sans:"IBM Plex Sans",system-ui,-apple-system,"Segoe UI",Roboto,"DejaVu Sans","Liberation Sans",Arial,sans-serif;
--mono:"IBM Plex Mono",ui-monospace,"DejaVu Sans Mono","Liberation Mono",Menlo,Consolas,monospace}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 var(--sans)}
a{color:var(--follow)}
.mono{font-family:var(--mono)}
.top{display:flex;flex-wrap:wrap;align-items:center;gap:10px 18px;padding:10px 16px;border-bottom:1px solid var(--line);background:#0B0E13;position:sticky;top:0;z-index:5}
.brand{display:flex;align-items:baseline;gap:10px;min-width:0}
.brand b{font-size:18px;letter-spacing:.08em}
.brand .mono{color:var(--mut);font-size:12px;overflow-wrap:anywhere}
.chip{display:flex;flex-direction:column;justify-content:center;padding:6px 14px;border-radius:8px;border:1px solid var(--line);min-width:150px}
.chip b{font-size:17px;letter-spacing:.06em}.chip span{font-size:12px;color:var(--mut2)}
.chip.run{border-color:#2D5A43;background:#10241B}.chip.run b{color:var(--ok)}
.chip.hold{border-color:var(--amber);background:var(--amber-bg)}.chip.hold b,.chip.hold span{color:var(--amber-t)}
.chip.stop{border-color:#5A2D2D;background:#241212}.chip.stop b{color:var(--bad)}
.stats{display:flex;flex-wrap:wrap;gap:4px 16px;font:12px var(--mono);color:var(--mut2)}
.stats span b{color:var(--fg);font-weight:500}.stats .warn b{color:var(--bad)}
.top .lab{margin-left:auto;font-size:13px}
main{display:flex;gap:16px;padding:16px;align-items:flex-start}
.left{flex:1 1 0;min-width:0;display:flex;flex-direction:column;gap:16px}
.right{flex:0 0 380px;min-width:0;display:flex;flex-direction:column;gap:12px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-width:0}
h2{font-size:14px;font-weight:600;margin:0 0 8px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
h2 .badge{font:11px var(--mono);padding:1px 7px;border-radius:9px;border:1px solid var(--line);color:var(--mut2);font-weight:400}
h2 .badge.slow{color:#FFB3AC;border-color:#7A3A35;background:#2A1513}
h2 .badge.okb{color:var(--ok);border-color:#2D5A43}
.note{margin:8px 0 0;color:var(--fg);font-size:13px}
.src{margin:3px 0 0;color:var(--mut);font-size:11px}
#pic{display:block;width:100%;height:auto;border-radius:6px;background:#000;aspect-ratio:4/3}
.th{display:flex;flex-wrap:wrap;gap:4px 14px;font:12px var(--mono);color:var(--mut2);margin-top:8px}
.th .amber{color:var(--amber-t)}.th .red{color:var(--bad)}.th .ok{color:var(--ok)}
.two{display:grid;grid-template-columns:1fr 1fr;gap:16px}
svg{display:block;width:100%;height:auto;overflow:visible}
svg text{fill:var(--mut);font:11px var(--mono)}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12px;color:var(--mut2);margin-top:6px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
.empty{color:var(--mut);font-size:13px;padding:24px 0;text-align:center}
/* plan */
.prow{display:flex;align-items:center;gap:8px;margin:5px 0}
.prow .pl{width:30px;font:12px var(--mono);color:var(--mut2)}
.sq{display:flex;flex-wrap:wrap;gap:3px}
.sq i{width:13px;height:13px;border-radius:3px;border:1px solid #3A4352;display:block}
.sq i.done{border-color:transparent}.sq i.run{border:2px solid var(--follow)}
.sq i.failed{background:repeating-linear-gradient(45deg,#5A2D2D 0 2px,transparent 2px 5px);border-color:#7A3A35}
.sq i.left_out{opacity:.35;border-style:dashed}
/* hold */
.hold{border:2px solid var(--amber);background:#1C1608}
.hold .hk{font:12px var(--mono);color:var(--amber-t);letter-spacing:.04em}
.hold .hq{font-size:21px;line-height:1.3;font-weight:600;color:#FFF3DA;margin:8px 0 4px;overflow-wrap:anywhere}
.hold .hs{color:var(--amber-t);font-size:13px;margin:0 0 10px}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.btns button{font:inherit;text-align:left;display:flex;flex-direction:column;justify-content:flex-start;padding:12px;border-radius:9px;border:1px solid #6B5021;background:var(--amber-bg);color:#FFF3DA;cursor:pointer;min-height:76px}
.btns button b{display:block;font-size:16px;margin-bottom:3px}.btns button>span{display:block;font-size:12px;color:#E9CF9C;line-height:1.3}
.btns button:hover:not(:disabled){background:#4A3610;border-color:var(--amber)}
.btns button.stopb{border-color:#7A3A35;background:#2A1513}.btns button.stopb>span{color:#E8B4AE}
.btns button:disabled{opacity:.45;cursor:default}
.btns button.picked{opacity:1;outline:2px solid var(--amber-t)}
.hsent{font-size:13px;color:var(--amber-t);margin-top:8px;min-height:1em}
details{margin-top:8px;font-size:12px;color:var(--mut2)}details pre{white-space:pre-wrap;font:12px var(--mono);margin:6px 0 0}
.ask{border:2px solid var(--amber)}
.ask pre{white-space:pre-wrap;font:14px var(--sans);margin:0 0 8px}
.ask input{width:100%;font:15px var(--sans);padding:8px;border-radius:6px;border:1px solid var(--line);background:#0E1116;color:var(--fg)}
.ask button{margin-top:8px;font:15px var(--sans);padding:8px 18px;border-radius:6px;border:1px solid var(--amber);background:var(--amber-bg);color:#FFF3DA;cursor:pointer}
/* robots */
.robot .rh{display:flex;align-items:baseline;gap:8px;flex-wrap:wrap}
.robot .rn{font-size:16px;font-weight:600}.robot .rs{color:var(--mut2);font-size:13px}
.robot.follow{border-color:#2F4A6B}.robot.follow .rn{color:var(--follow)}
.tag{font:11px var(--mono);padding:1px 6px;border-radius:8px;border:1px solid #2F4A6B;color:var(--follow)}
.tag.out{border-color:#5A2D2D;color:var(--bad)}
.bat{display:flex;align-items:center;gap:8px;margin:8px 0 2px}
.bar{flex:0 0 110px;height:10px;border-radius:5px;background:var(--grid);overflow:hidden}
.bar div{height:100%;border-radius:5px;background:var(--mut2)}.bar div.low{background:var(--bad)}
.bat .mono{font-size:12px;min-width:38px}.bat svg{flex:1;height:22px;min-width:40px}
.chips{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0 2px}
.chips span{font:12px var(--mono);padding:2px 7px;border-radius:9px;border:1px solid var(--line)}
.chips .ok{color:var(--ok);border-color:#2D5A43}.chips .bad{color:#FFB3AC;border-color:#7A3A35;background:#2A1513}
.chips .little{color:var(--mut)}
.edge{margin-top:8px}.edge .et{font-size:12px;color:var(--mut2);margin-top:2px}
#log{white-space:pre-wrap;font:11.5px/1.45 var(--mono);color:var(--mut2);max-height:260px;overflow:auto;margin:0}
/* trust table */
.tw{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:13px}
th{font-weight:500;color:var(--mut2);font-size:12px;text-align:left;padding:4px 8px;border-bottom:1px solid var(--line);white-space:nowrap}
td{padding:5px 8px;border-bottom:1px solid #1B212B;white-space:nowrap}
td.n{font-family:var(--mono);text-align:right}th.n{text-align:right}
td .u{color:var(--mut);font-size:11px}
td.ok{color:var(--ok)}td.bad{color:#FFB3AC}td.little{color:var(--mut)}
tr.bad td{background:#1E1412}
td svg{width:90px;height:22px}
tr.grp td{border-bottom:none;padding-top:9px}
.tip{position:fixed;pointer-events:none;background:#0B0E13;border:1px solid var(--line);border-radius:6px;padding:5px 8px;font:12px var(--mono);color:var(--fg);z-index:20;display:none;white-space:nowrap}
.calw{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.calw h3{font-size:12px;font-weight:500;color:var(--mut2);margin:0 0 4px}
@media (max-width:1100px){.right{flex-basis:330px}}
@media (max-width:860px){.two,.calw{grid-template-columns:1fr}}
@media (max-width:600px){
 .top{padding:8px 12px;gap:8px}.top .stats{font-size:11px;gap:2px 10px}.chip{flex:1 1 100%}.top .lab{margin-left:0}
 main{flex-direction:column;align-items:stretch;padding:10px 12px;gap:12px}
 .left,.right{display:contents}
 #picp{order:1}#hold{order:2}#ask{order:3}#robots{order:4}#robsrc{order:4}#perfp{order:5}#trustp{order:6}#tlp{order:7}#planres{order:8}#calp{order:9}#logp{order:10}
 #robots{display:flex;flex-direction:column;gap:12px}
 .btns{grid-template-columns:1fr}.btns button{min-height:64px}
 .hold .hq{font-size:19px}
}
</style></head><body>
<header class="top">
 <div class="brand"><b>MIRLT</b><span class="mono" id="sess">-</span><span class="mono" id="mode"></span></div>
 <div class="chip run" id="chip"><b id="chipb">RUNNING</b><span id="chips">-</span></div>
 <div class="stats" id="stats"></div>
 <a class="lab" href="/lab">live lab &rarr;</a>
</header>
<main>
<section class="left">
 <div class="panel" id="picp">
  <img id="pic" src="/frame.jpg" alt="overhead camera with the runner's view drawn on it">
  <div class="th" id="th"></div>
  <p class="note">The overhead camera's latest picture with what the runner thinks: teal is the board outline (dashed: the same outline at a robot's top height), the amber strip is the virtual border along the open edges, the blue ring is the robot being followed (dashed circle: where it should stand still), yellow dots look like a robot and grey dots are other changes (a hand, a shadow).</p>
  <p class="note" id="bnote" style="display:none"></p>
  <p class="src">Data: the overhead camera through the runner (e02.py), drawn by this page's server about once a second. Tracker line: the tracker's own counters over the last 30 s and the last perf.csv row.</p>
 </div>
 <div class="panel" id="tlp">
  <h2>Session timeline <span class="badge mono" id="tlspan"></span></h2>
  <svg id="tl" viewBox="0 0 1000 46" preserveAspectRatio="none" style="height:46px"></svg>
  <div class="legend"><span><i style="background:var(--slate)"></i>setup</span><span><i style="background:var(--runs)"></i>runs</span><span><i style="background:var(--find)"></i>finding</span><span><i style="background:repeating-linear-gradient(45deg,var(--amber) 0 2px,var(--amber-bg) 2px 5px)"></i>holds</span><span id="tlaxis" class="mono" style="margin-left:auto"></span></div>
  <p class="note">What the session has been doing since it started: setting up, driving runs, finding the robots against the boards, and the holds where every robot waited for you.</p>
  <p class="src">Measured: the runner's phase each second. Data: perf.csv's phase column, the schedule's run start and end times and the holds (e02_state.json).</p>
 </div>
 <div class="two" id="planres">
  <div class="panel"><h2 id="plant">Plan</h2><div id="plan"></div>
   <div class="legend"><span><i style="background:var(--mut2)"></i>scored</span><span><i style="border:2px solid var(--follow)"></i>running now</span><span><i style="border:1px solid #3A4352"></i>to come</span><span><i style="background:repeating-linear-gradient(45deg,#5A2D2D 0 2px,transparent 2px 5px)"></i>failed</span></div>
   <p class="note">One square per planned run, by condition; a square fills once its run is scored.</p>
   <p class="src">U uncalibrated · C calibrated · K camera-steered · KC calibrated + camera fixes. Data: the session's schedule (e02_state.json).</p></div>
  <div class="panel"><h2>Back at start, by condition</h2><div id="res"></div>
   <p class="note">How far from its starting spot each robot ended a run; each dot is one run and the bar is the average, so lower is better.</p>
   <p class="src">Measured: end error, cm (camera position at the end against the start, live, before the footage re-measurement). Data: the runner's run results (schedule in e02_state.json); runs a safety stop ended are left out.</p></div>
 </div>
 <div class="panel" id="perfp">
  <h2>Processing speed <span class="badge" id="perfb">-</span></h2>
  <svg id="perf" viewBox="0 0 760 210"></svg>
  <p class="note" id="perfn">How many camera pictures the runner works through each second, and how old a picture is by the time its position is ready; shaded seconds were slow.</p>
  <div class="legend"><span><i style="background:#E6E9EE"></i>average</span><span><i style="background:#9AA4B2;opacity:.55"></i>worst picture age</span><span><i style="background:#F07B72;opacity:.35"></i>slow seconds</span><span><i style="background:#8B79D9"></i>planned load on</span><span><i style="border-top:2px dashed #F07B72;height:0;border-radius:0"></i>slow line</span></div>
  <p class="src">Measured: pictures handled a second (of about 25 the camera sends) and a picture's age when its position was ready, s. Slow: <span id="prule">-</span> (the runner's own speed watch, e02_load.py). Data: the runner's perf rows (perf.csv), last 5 minutes.</p>
 </div>
 <div class="panel" id="trustp">
  <h2>How much each sensor is trusted</h2>
  <div class="tw"><table id="trust"><thead><tr><th>Sensor</th><th class="n">Learned noise</th><th class="n">Healthy noise</th><th class="n">k</th><th>State</th><th>Trend</th></tr></thead><tbody></tbody></table></div>
  <p class="note" id="trustn">Each sensor's noise as the system has learned it, against its noise when healthy; k is the ratio, so 1 is as healthy and <span class="dk">2</span> or more means it looks bad and is trusted less.</p>
  <p class="src">Measured: learned noise (1 sd): camera cm per axis at the finds (or its still-robot scatter), wheels cm and gyro deg per self-calibration stretch (each robot's own share), front sensor cm against the map. Evidence count in brackets; trend = the learned noise's last values (dashed: healthy, dotted: looks bad). Data: the runner's sensor trust (e02_trust.py, logs/sensor_trust*.json).</p>
 </div>
 <div class="panel" id="calp">
  <h2>Self-calibration, settling run by run</h2>
  <div class="calw"><div><h3>Gyro scale</h3><div id="calg"></div></div><div><h3>Wheel scale</h3><div id="calw"></div></div></div>
  <div class="legend" id="calleg"></div>
  <p class="note">Each line is one robot's correction as the system learned it from the camera; it starts at a guess and should flatten as runs are added.</p>
  <p class="src">Measured: gyro scale (true turn / the gyro's reading) and wheel scale (true distance / the wheels' reading), y; the run each was refitted after, x. Data: the runner's accepted self-calibration fits (fits in e02_state.json).</p>
 </div>
</section>
<aside class="right">
 <div class="panel hold" id="hold" hidden>
  <div class="hk">HOLD · <span id="hkind"></span> · <span id="hsince"></span></div>
  <div class="hq" id="hq"></div>
  <p class="hs">Every robot is stopped. The session waits for your answer, with no time limit.</p>
  <div class="btns">
   <button data-a="c"><b>Carry on</b><span>every robot carries on from where it stopped</span></button>
   <button data-a="a"><b>Carry on, don't ask</b><span>and from now on a lost robot is found again without asking (the border still asks)</span></button>
   <button data-a="o"><b>Leave <span class="hrb">it</span> out</b><span><span class="hrb">it</span> stays where it is for the rest of the session; the others carry on</span></button>
   <button data-a="s" class="stopb"><b>Stop the session</b><span>every robot stays stopped; carry it on later with <span style="white-space:nowrap">--resume</span></span></button>
  </div>
  <div class="hsent" id="hsent"></div>
  <details><summary>The runner's full question</summary><pre id="hprompt"></pre></details>
 </div>
 <div class="panel ask" id="ask" hidden>
  <h2>The runner asks</h2><pre id="askq"></pre>
  <input id="askin" placeholder="type an answer (or leave empty)">
  <button id="askbtn">Send</button>
  <p class="src" id="askdef"></p>
 </div>
 <div id="robots" style="display:flex;flex-direction:column;gap:12px"></div>
 <p class="src" id="robsrc" style="margin:-4px 2px 0"><span style="color:var(--fg);font-size:13px">Each card shows what that robot is doing, its battery, how far its sensors are trusted and how close it is to an open edge.</span><br>Battery: the robot's own reading from its voltage (it sags while driving), line: this session's readings (battery.csv and what the robots report). Sensor chips: k from the sensor trust table below. Edge bar: distance to the nearest open edge, cm (amber: inside the virtual border; dashed: where the robot stops itself), from the camera tracker or its last parked spot (e02_state.json) and the runner's border settings.</p>
 <div class="panel" id="logp"><h2>Log <span class="badge" id="phase">-</span></h2><pre id="log"></pre>
  <p class="src">The runner's last lines (e02_log.txt).</p></div>
</aside>
</main>
<div class="tip" id="tip"></div>
<script>
"use strict";
const $=id=>document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
const CC={U:'var(--s1)',C:'var(--s2)',K:'var(--s3)',KC:'var(--s4)'};
const RC=['var(--s1)','var(--s2)','var(--s3)','var(--s4)'];
const last={};
function changed(k,v){const j=JSON.stringify(v===undefined?null:v);if(last[k]===j)return false;last[k]=j;return true}
function el(t,a,txt){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);if(txt!=null)e.textContent=txt;return e}
function esc(s){return String(s==null?'':s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function dur(s){if(s==null)return '-';s=Math.max(0,Math.round(s));const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;
 return (h?h+':'+String(m).padStart(2,'0'):m)+':'+String(x).padStart(2,'0')}
function num(v,d){return v==null?'-':Number(v).toFixed(d)}
const tip=$('tip');
function showTip(e,html){tip.innerHTML=html;tip.style.display='block';const w=tip.offsetWidth;
 tip.style.left=Math.min(window.innerWidth-w-8,e.clientX+12)+'px';tip.style.top=(e.clientY+14)+'px'}
function hideTip(){tip.style.display='none'}
function tipOn(node,html){node.addEventListener('mousemove',e=>showTip(e,html));node.addEventListener('mouseleave',hideTip)}
let robotOrder=[];
function rcol(n){let i=robotOrder.indexOf(n);if(i<0){robotOrder.push(n);i=robotOrder.length-1}return RC[i%RC.length]}

/* picture: the next one is asked for a second after the last arrived */
function pic(){const im=new Image();im.onload=()=>{$('pic').src=im.src;setTimeout(pic,1000)};im.onerror=()=>setTimeout(pic,2000);im.src='/frame.jpg?t='+Date.now()}

function header(h,s){if(!h)return;
 $('sess').textContent=h.session;$('mode').textContent=h.mode?'· '+h.mode:'';
 const c=$('chip');c.className='chip '+({hold:'hold',stopped:'stop'}[h.state]||'run');
 $('chipb').textContent=h.state.toUpperCase();
 $('chips').textContent=h.state=='hold'?'every robot stopped · waiting for you':h.state=='stopped'?'the session is stopping':((h.phase||'-')+(h.order?' · run '+h.order:''));
 const st=[['elapsed',dur(h.elapsed_s)],['held',dur(h.held_s)],
  ['pictures/s',(h.fps==null?'-':Math.round(h.fps))+' of '+h.camera_fps,h.slow],['picture age',h.age_s==null?'-':num(h.age_s,2)+' s',h.slow]];
 if(h.rec_fps!=null)st.push(['recorder',num(h.rec_fps,1)+' fps']);
 $('stats').innerHTML=st.map(q=>'<span'+(q[2]?' class="warn"':'')+'>'+q[0]+' <b>'+esc(q[1])+'</b></span>').join('');}

function health(h){const e=$('th');if(!h){e.innerHTML='';return}const L=h.levels||{};const pc=v=>v==null?'-':Math.round(100*v)+'%';
 const it=(lv,t)=>'<span class="'+(lv||'')+'">'+t+'</span>';
 e.innerHTML='<span>Tracker'+(h.robot?' ('+esc(h.robot)+')':'')+':</span>'+(h.pictures?it(L.seen,'seen in '+pc(h.seen_share)+' of '+h.pictures+' pictures'):it('','no robot followed in the last 30 s'))+
  it(L.score,'lock match '+(h.score==null?'-':h.score))+it(L.disagree,'detector disagreed '+h.disagree+'/'+h.checks)+it(L.seen_age,'last seen '+(h.seen_age_s==null?'-':h.seen_age_s+' s ago'))}

/* questions: a hold gets the four big buttons; any other question a text box */
let sent={};
function post(id,text){return fetch('/answer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:id,text:text})})}
const WORDS={c:'Carry on',a:"Carry on, don't ask",o:'Leave it out',s:'Stop the session'};
document.querySelectorAll('#hold button').forEach(b=>b.onclick=()=>{const id=$('hold').dataset.qid;if(!id)return;const a=b.dataset.a;
 if(a=='s'&&!confirm('Stop the session? Every robot stays stopped; you can carry it on later with --resume.'))return;
 sent[id]=a;post(id,a).then(()=>askRender(lastQ,lastHold,true)).catch(()=>{$('hsent').textContent='Not sent: the runner did not answer. Try again.';delete sent[id]})});
$('askbtn').onclick=()=>{const a=$('ask');post(a.dataset.qid,$('askin').value).then(()=>{a.hidden=true;sent[a.dataset.qid]='text'})};
$('askin').addEventListener('keydown',e=>{if(e.key=='Enter')$('askbtn').click()});
let lastQ=null,lastHold=null;
function askRender(q,hold,force){lastQ=q;lastHold=hold;
 const H=$('hold'),A=$('ask');
 if(hold){H.hidden=false;A.hidden=true;
  $('hkind').textContent=hold.kind=='border'?'virtual border':hold.kind=='lost'?'lost robot':(hold.kind||'');
  $('hsince').textContent=hold.since_s==null?'':'waiting '+dur(hold.since_s);
  $('hq').textContent=(hold.robot?hold.robot+': ':'')+(hold.why||'');
  document.querySelectorAll('#hold .hrb').forEach(x=>x.textContent=hold.robot||'it');
  const qid=q?String(q.id):'';H.dataset.qid=qid;$('hprompt').textContent=q?q.prompt:'';
  const picked=sent[qid];
  document.querySelectorAll('#hold button').forEach(b=>{b.disabled=!qid||!!picked;b.classList.toggle('picked',picked==b.dataset.a)});
  $('hsent').textContent=!qid?'The question is not on this page yet (a dry run with --auto answers its holds by itself).':picked?'Sent: '+WORDS[picked]+'. Waiting for the runner to take it.':'';
  return}
 H.hidden=true;
 if(q&&!sent[String(q.id)]){if(A.dataset.qid!=String(q.id)){A.dataset.qid=q.id;$('askin').value=''}
  A.hidden=false;$('askq').textContent=q.prompt;
  $('askdef').textContent='Empty = the default ('+(q.default||'Enter')+')'+(q.timeout_s?'; no answer in '+Math.round(q.timeout_s/60)+' min = the default':'')}
 else A.hidden=true}

/* robot cards */
function spark(vals,lo,hi,col,refs){const w=100,h=22,s=el('svg',{viewBox:'0 0 '+w+' '+h,preserveAspectRatio:'none'});
 if(!vals.length)return s;const y=v=>h-2-(h-4)*(Math.min(hi,Math.max(lo,v))-lo)/((hi-lo)||1);
 (refs||[]).forEach(r=>s.appendChild(el('line',{x1:0,x2:w,y1:y(r[0]),y2:y(r[0]),stroke:r[1],'stroke-dasharray':r[2],'stroke-width':1,'vector-effect':'non-scaling-stroke'})));
 const n=vals.length,xs=i=>n<2?w/2:2+(w-4)*i/(n-1);
 if(n>1)s.appendChild(el('polyline',{points:vals.map((v,i)=>xs(i)+','+y(v)).join(' '),fill:'none',stroke:col,'stroke-width':1.5,'vector-effect':'non-scaling-stroke'}));
 s.appendChild(el('circle',{cx:xs(n-1),cy:y(vals[n-1]),r:2,fill:col}));return s}
const SICON={ok:'✓',bad:'!',little:'?'},SWORD={ok:'healthy',bad:'looks bad',little:'too little evidence'};
function robots(list){const box=$('robots');box.innerHTML='';
 (list||[]).forEach(r=>{rcol(r.name);const d=document.createElement('div');d.className='panel robot'+(r.cls=='follow'?' follow':'');
  let h='<div class="rh"><span class="rn">'+esc(r.name)+'</span><span class="rs">'+esc(r.state)+'</span>'+(r.cls=='follow'?'<span class="tag">followed</span>':'')+(r.cls=='out'?'<span class="tag out">'+esc(r.state)+'</span>':'')+'</div>';
  const b=r.battery;h+='<div class="bat"><div class="bar"><div class="'+(b!=null&&b<20?'low':'')+'" style="width:'+(b==null?0:Math.max(0,Math.min(100,b)))+'%"></div></div><span class="mono">'+(b==null?'-':Math.round(b)+'%')+'</span><span class="bs"></span></div>';
  h+='<div class="src">'+(b==null?'battery not heard yet':'battery reported at '+new Date(r.battery_t*1000).toTimeString().slice(0,5))+'</div>';
  h+='<div class="chips">'+(r.sensors||[]).map(q=>'<span class="'+q.state+'" title="'+SWORD[q.state]+', '+q.n+' evidence items">'+SICON[q.state]+' '+q.name+' ×'+num(q.k,1)+(q.state=='ok'?'':' '+SWORD[q.state])+'</span>').join('')+'</div>';
  h+='<div class="edge"></div>';d.innerHTML=h;
  const tr=(r.battery_trend||[]).map(q=>q[1]);d.querySelector('.bs').appendChild(spark(tr,0,100,'var(--mut2)'));
  const eg=d.querySelector('.edge');const e=r.edge;
  if(e){const mx=Math.max(40,(e.border_cm||0)+20,Math.min(80,e.dist_cm+5)),W=300,x=v=>4+(W-8)*Math.min(mx,Math.max(0,v))/mx;
   const s=el('svg',{viewBox:'0 0 '+W+' 26'});s.appendChild(el('rect',{x:4,y:9,width:W-8,height:8,rx:4,fill:'#262D39'}));
   if(e.border_cm)s.appendChild(el('rect',{x:4,y:9,width:x(e.border_cm)-4,height:8,rx:4,fill:'#E0A33A',opacity:.45}));
   if(e.fw_border_cm){s.appendChild(el('line',{x1:x(e.fw_border_cm),x2:x(e.fw_border_cm),y1:5,y2:21,stroke:'#FFD58A','stroke-dasharray':'2 2'}))}
   const inb=e.border_cm&&e.dist_cm<e.border_cm;s.appendChild(el('circle',{cx:x(e.dist_cm),cy:13,r:6,fill:inb?'#F07B72':(r.cls=='follow'?'#7FB8FF':'#E6E9EE'),stroke:'#151A22','stroke-width':2}));
   s.appendChild(el('text',{x:4,y:26,style:'font-size:9px'},'edge'));s.appendChild(el('text',{x:W-4,y:26,'text-anchor':'end',style:'font-size:9px'},mx+' cm'));
   eg.appendChild(s);const t=document.createElement('div');t.className='et';
   t.textContent=num(e.dist_cm,0)+' cm from the nearest open edge'+(e.border_cm?' · virtual border '+num(e.border_cm,0)+' cm':'')+(e.fw_border_cm?', its own stop '+num(e.fw_border_cm,0)+' cm':'')+' ('+e.where+' spot)';eg.appendChild(t);
   }
  else eg.innerHTML='<div class="src">Distance to the open edge: not known yet (no board or no position).</div>';
  box.appendChild(d)})}

/* timeline */
function timeline(t){const s=$('tl');s.innerHTML='';if(!t)return;const span=Math.max(1,t.span_s),X=v=>1000*Math.max(0,v)/span;
 const defs=el('defs',{});const p=el('pattern',{id:'hst',width:6,height:6,patternUnits:'userSpaceOnUse',patternTransform:'rotate(45)'});
 p.appendChild(el('rect',{width:6,height:6,fill:'#3A2A0B'}));p.appendChild(el('rect',{width:2.5,height:6,fill:'#E0A33A'}));defs.appendChild(p);s.appendChild(defs);
 s.appendChild(el('rect',{x:0,y:8,width:1000,height:30,fill:'#1B212B'}));
 const col={setup:'#3A4352',runs:'#4F9D74',finding:'#8B79D9'};
 (t.segs||[]).forEach(q=>{const r=el('rect',{x:X(q[0]),y:8,width:Math.max(1,X(q[1])-X(q[0])),height:30,fill:col[q[2]]||'#3A4352'});tipOn(r,q[2]+': '+dur(q[0])+' – '+dur(q[1]));s.appendChild(r)});
 (t.runs||[]).forEach(q=>{const r=el('rect',{x:X(q[0]),y:8,width:Math.max(1,X(q[1])-X(q[0])),height:30,fill:'#4F9D74'});tipOn(r,'run: '+dur(q[0])+' – '+dur(q[1]));s.appendChild(r)});
 (t.holds||[]).forEach(q=>{const r=el('rect',{x:X(q[0]),y:2,width:Math.max(3,X(q[1])-X(q[0])),height:42,fill:'url(#hst)',stroke:'#E0A33A','stroke-width':1,'vector-effect':'non-scaling-stroke'});
  tipOn(r,'hold ('+esc(q[3])+', '+esc(q[2])+'): '+dur(q[0])+' – '+dur(q[1]));s.appendChild(r)});
 $('tlspan').textContent=dur(span);$('tlaxis').textContent='0:00 → now ('+dur(span)+')'}

/* plan */
function plan(p){if(!p)return;$('plant').textContent='Plan · '+p.done+' of '+p.total+' runs';const box=$('plan');
 if(!p.runs.length){box.innerHTML='<div class="empty">No schedule yet: it is made when setup ends.</div>';return}
 box.innerHTML='';['U','C','K','KC'].forEach(c=>{const rs=p.runs.filter(r=>r.cond==c);if(!rs.length)return;
  const row=document.createElement('div');row.className='prow';row.innerHTML='<span class="pl">'+c+'</span><span class="sq"></span><span class="mono" style="font-size:11px;color:var(--mut)">'+rs.filter(r=>r.status=='done').length+'/'+rs.length+'</span>';
  const sq=row.querySelector('.sq');rs.forEach(r=>{const i=document.createElement('i');const st=r.status=='running'||String(r.order)==p.current&&r.status!='done'?'run':r.status;
   i.className=st;if(st=='done')i.style.background=CC[c];
   tipOn(i,'run '+r.order+' · '+esc(r.robot)+' · route '+r.route+' · '+st.replace('_',' ')+(r.err!=null?' · '+num(r.err,1)+' cm':''));sq.appendChild(i)});box.appendChild(row)})}

/* results: dots by condition */
function results(rows){const box=$('res');box.innerHTML='';rows=rows||[];
 if(!rows.length){box.innerHTML='<div class="empty">No scored runs yet: a dot appears here as each run ends.</div>';return}
 let hi=4;rows.forEach(r=>hi=Math.max(hi,r.end_err_cm));hi=Math.ceil(hi/4)*4;const W=320,H=180,L=34,T=8,B=H-24,y=v=>B-(B-T)*v/hi;
 const s=el('svg',{viewBox:'0 0 '+W+' '+H});
 for(let i=0;i<=4;i++){const v=hi*i/4;s.appendChild(el('line',{x1:L,x2:W,y1:y(v),y2:y(v),stroke:'#262D39'}));s.appendChild(el('text',{x:L-6,y:y(v)+4,'text-anchor':'end'},v%1?v.toFixed(1):v))}
 s.appendChild(el('text',{x:0,y:T-1,style:'font-size:9px'},'cm'));
 const cs=['U','C','K','KC'],cw=(W-L)/4;
 cs.forEach((c,i)=>{const cx=L+cw*(i+.5),v=rows.filter(r=>r.cond==c);
  v.forEach((r,j)=>{const d=el('circle',{cx:cx-15+((j*7)%31),cy:y(r.end_err_cm),r:4,fill:CC[c],stroke:'#151A22','stroke-width':2});tipOn(d,c+' · run '+r.order+' · '+esc(r.robot)+' · route '+r.route+': '+num(r.end_err_cm,1)+' cm');s.appendChild(d)});
  if(v.length){const m=v.reduce((a,r)=>a+r.end_err_cm,0)/v.length;s.appendChild(el('line',{x1:cx-22,x2:cx+22,y1:y(m),y2:y(m),stroke:'#E6E9EE','stroke-width':2,'stroke-linecap':'round'}))}
  s.appendChild(el('text',{x:cx,y:H-6,'text-anchor':'middle'},c+' ('+v.length+')'))});
 box.appendChild(s)}

/* processing speed: pictures a second (top) and picture age (bottom), the last 5 minutes */
function perf(p){const s=$('perf');s.innerHTML='';if(!p)return;
 $('prule').textContent=p.rule||'-';
 const b=$('perfb');b.className='badge '+(p.slow?'slow':p.slow==null?'':'okb');
 b.textContent=p.slow?'SLOW: '+p.why:p.slow==null?'no camera yet':'keeping up'+(p.slow_s?' · '+p.slow_s+' slow s in 5 min':'')+(p.load?' · planned load on':'');
 const rows=p.rows||[],W=760,L=40,R=W-8,X=t=>L+(R-L)*(t+p.window_s)/p.window_s;
 const pan=(top,h,lo,hi,ttl,fmt)=>{const y=v=>top+h-h*(Math.min(hi,Math.max(lo,v))-lo)/(hi-lo);
  for(let i=0;i<=2;i++){const v=lo+(hi-lo)*i/2;s.appendChild(el('line',{x1:L,x2:R,y1:y(v),y2:y(v),stroke:'#262D39'}));s.appendChild(el('text',{x:L-6,y:y(v)+4,'text-anchor':'end'},fmt(v)))}
  s.appendChild(el('text',{x:L,y:top-5},ttl));return y};
 rows.forEach(q=>{if(q[4])s.appendChild(el('rect',{x:X(q[0])-1.1,y:14,width:2.4,height:174,fill:'#F07B72',opacity:.16}));
  if(q[5])s.appendChild(el('rect',{x:X(q[0])-1.1,y:192,width:2.4,height:4,fill:'#8B79D9'}))});
 const y1=pan(16,70,0,p.camera_fps,'pictures handled a second',v=>Math.round(v));
 if(p.slow_fps!=null)s.appendChild(el('line',{x1:L,x2:R,y1:y1(p.slow_fps),y2:y1(p.slow_fps),stroke:'#F07B72','stroke-dasharray':'3 3',opacity:.7}));
 let amax=Math.max((p.slow_age_s||0.5)*1.5,...rows.map(q=>q[3]||0));amax=amax>1?Math.ceil(amax*2)/2:Math.ceil(amax*10)/10;
 const y2=pan(118,70,0,amax,'picture age when its position was ready (s)',v=>v.toFixed(amax>1?1:2));
 if(p.slow_age_s!=null)s.appendChild(el('line',{x1:L,x2:R,y1:y2(p.slow_age_s),y2:y2(p.slow_age_s),stroke:'#F07B72','stroke-dasharray':'3 3',opacity:.7}));
 const line=(i,y,c,w,o)=>{const pts=rows.filter(q=>q[i]!=null).map(q=>X(q[0]).toFixed(1)+','+y(q[i]).toFixed(1));if(pts.length>1)s.appendChild(el('polyline',{points:pts.join(' '),fill:'none',stroke:c,'stroke-width':w,opacity:o,'stroke-linejoin':'round'}))};
 line(1,y1,'#E6E9EE',2,1);line(3,y2,'#9AA4B2',1,.55);line(2,y2,'#E6E9EE',2,1);
 s.appendChild(el('text',{x:L,y:208},'5 min ago'));s.appendChild(el('text',{x:R,y:208,'text-anchor':'end'},'now'));
 if(!rows.length)s.appendChild(el('text',{x:(L+R)/2,y:105,'text-anchor':'middle',style:'font-size:13px'},'No camera pictures handled yet'));
 const cross=el('line',{x1:0,x2:0,y1:12,y2:196,stroke:'#7FB8FF',opacity:0});s.appendChild(cross);
 const hit=el('rect',{x:L,y:10,width:R-L,height:188,fill:'transparent'});s.appendChild(hit);
 hit.addEventListener('mousemove',e=>{if(!rows.length)return;const bb=s.getBoundingClientRect(),vx=(e.clientX-bb.left)*W/bb.width;
  let best=rows[0];rows.forEach(q=>{if(Math.abs(X(q[0])-vx)<Math.abs(X(best[0])-vx))best=q});cross.setAttribute('x1',X(best[0]));cross.setAttribute('x2',X(best[0]));cross.setAttribute('opacity',.8);
  showTip(e,Math.round(-best[0])+' s ago: '+num(best[1],1)+' pictures/s · age '+num(best[2],2)+' s (worst '+num(best[3],2)+' s)'+(best[4]?' · slow':'')+(best[5]?' · load on':''))});
 hit.addEventListener('mouseleave',()=>{cross.setAttribute('opacity',0);hideTip()})}

/* sensor trust table */
function trust(t){const tb=document.querySelector('#trust tbody');tb.innerHTML='';
 if(!t){tb.innerHTML='<tr><td colspan="6" class="little">No sensor trust in this runner.</td></tr>';return}
 document.querySelectorAll('.dk').forEach(x=>x.textContent=t.degraded_k);
 let grp=null;
 (t.rows||[]).forEach(r=>{const g=r.robot=='*'?'Shared':r.robot;if(g!=grp){grp=g;const gr=document.createElement('tr');gr.className='grp';gr.innerHTML='<td colspan="6" style="color:var(--mut);font-size:11px;letter-spacing:.06em">'+esc(g.toUpperCase())+'</td>';tb.appendChild(gr)}
  const tr=document.createElement('tr');tr.className=r.state=='bad'?'bad':'';
  const st=(r.state=='little'?'? too little evidence ('+r.n+' of '+r.min_n+')':(SICON[r.state]+' '+SWORD[r.state]+' ('+r.n+')'))+(r.lag?'<br><span class="u">'+esc(r.lag)+'</span>':'');
  tr.innerHTML='<td>'+esc(r.label)+'</td><td class="n">'+num(r.sd,2)+' <span class="u">'+r.unit+'</span></td><td class="n">'+num(r.healthy,1)+' <span class="u">'+r.unit+'</span></td><td class="n">×'+num(r.k,2)+'</td><td class="'+r.state+'">'+st+'</td><td></td>';
  const hi=Math.max(r.healthy*t.degraded_k*1.2,...r.trend);
  tr.lastChild.appendChild(spark(r.trend,0,hi,r.state=='bad'?'#F07B72':'#E6E9EE',[[r.healthy,'#6CCB8F','3 2'],[r.healthy*t.degraded_k,'#F07B72','1 2']]));
  tb.appendChild(tr)});
 if(t.shared_note){const tr=document.createElement('tr');tr.innerHTML='<td colspan="6" class="little" style="white-space:normal">Note: '+esc(t.shared_note)+'.</td>';tb.appendChild(tr)}}

/* self-calibration */
function calib(rows){rows=rows||[];const leg=$('calleg');
 const robotsIn=[...new Set(rows.map(r=>r.robot))];robotsIn.forEach(rcol);
 leg.innerHTML=robotsIn.map(n=>'<span><i style="background:'+rcol(n)+'"></i>'+esc(n)+'</span>').join('');
 [['gyro_scale','calg',3],['wheel_scale','calw',3]].forEach(([k,id,dp])=>{const box=$(id);box.innerHTML='';const v=rows.filter(r=>r[k]!=null);
  if(!v.length){box.innerHTML='<div class="empty">No accepted fit yet.</div>';return}
  let lo=Math.min(...v.map(r=>r[k])),hi=Math.max(...v.map(r=>r[k]));const pad=Math.max(0.005,(hi-lo)*0.15);lo-=pad;hi+=pad;
  const n=Math.max(2,...v.map(r=>r.order||0)),W=420,H=170,L=44,T=8,B=H-20,X=o=>L+(W-L-8)*(o||0)/n,Y=x=>B-(B-T)*(x-lo)/(hi-lo);
  const s=el('svg',{viewBox:'0 0 '+W+' '+H});
  for(let i=0;i<=3;i++){const x=lo+(hi-lo)*i/3;s.appendChild(el('line',{x1:L,x2:W-8,y1:Y(x),y2:Y(x),stroke:'#262D39'}));s.appendChild(el('text',{x:L-6,y:Y(x)+4,'text-anchor':'end'},x.toFixed(dp)))}
  s.appendChild(el('text',{x:L,y:H-4},'run 0'));s.appendChild(el('text',{x:W-8,y:H-4,'text-anchor':'end'},'run '+n));
  robotsIn.forEach(rb=>{const q=v.filter(r=>r.robot==rb).sort((a,b)=>(a.order||0)-(b.order||0));if(!q.length)return;
   if(q.length>1)s.appendChild(el('polyline',{points:q.map(r=>X(r.order)+','+Y(r[k])).join(' '),fill:'none',stroke:rcol(rb),'stroke-width':2}));
   q.forEach(r=>{const c=el('circle',{cx:X(r.order),cy:Y(r[k]),r:4,fill:rcol(rb),stroke:'#151A22','stroke-width':2});tipOn(c,esc(rb)+' after run '+r.order+': '+k.replace('_',' ')+' '+Number(r[k]).toFixed(4));s.appendChild(c)})});
  box.appendChild(s)})}

function render(s){const L=s.live||{};
 header(L.header,s);
 if(changed('ask',[s.question,L.hold&&{r:L.hold.robot,k:L.hold.kind,w:L.hold.why},sent]))askRender(s.question,L.hold);
 else if(L.hold)$('hsince').textContent=L.hold.since_s==null?'':'waiting '+dur(L.hold.since_s);
 if(changed('health',s.health))health(s.health);
 if(changed('note',s.note)){$('bnote').textContent=s.note||'';$('bnote').style.display=s.note?'':'none'}
 if(changed('log',s.log)){const g=$('log');g.textContent=(s.log||[]).join('\n');g.scrollTop=g.scrollHeight}
 $('phase').textContent=(s.phase||'-')+(s.order?' · run '+s.order:'');
 if(changed('robots',L.robots))robots(L.robots);
 if(changed('timeline',L.timeline))timeline(L.timeline);
 if(changed('plan',L.plan))plan(L.plan);
 if(changed('results',L.results))results(L.results);
 if(changed('perf',L.perf))perf(L.perf);
 if(changed('trust',L.trust))trust(L.trust);
 if(changed('calib',L.calib))calib(L.calib)}
function tick(){fetch('/status.json',{cache:'no-store'}).then(r=>r.json()).then(render).catch(()=>{$('chips').textContent='no answer from the runner'}).finally(()=>setTimeout(tick,1000))}
tick();pic();
</script></body></html>"""


CAMERA_FPS = 25           # the overhead camera sends about this many pictures a second
PHASE_CAT = dict(setup="setup", parking="setup", exploring="setup", mapping="setup", route="runs",
                 find="finding", finding="finding")   # the runner's phases -> the timeline's bands
BATTERY_KEEP = 400        # battery readings kept per robot
BATTERY_SEND = 120        # ... and sent to the page


def _r(v, nd):
    return None if v is None else round(float(v), nd)


class LiveView:
    """Serves the page. runner: an e02.Runner (read only: feed.last_img, tracker, st, cam, board, phase, order)."""

    def __init__(self, runner, port=8090, bind="127.0.0.1"):
        self.r = runner
        self.port = port
        self.bind = bind               # "0.0.0.0": other computers and phones on the WiFi can open it (the Pi run)
        self.question = None           # dict(id, prompt, default, timeout_s): the runner waits for an answer
        self.answers = {}              # id -> text
        self.lines = []
        self.lock = threading.Lock()
        self.srv = None

    def log(self, line):
        with self.lock:
            self.lines.append(line)
            del self.lines[:-14]

    def start(self):
        view = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                path = self.path.split("?")[0]
                try:
                    if path == "/":
                        body, ctype = view.page("main").encode(), "text/html; charset=utf-8"
                    elif path == "/frame.jpg":
                        body, ctype = view.frame_jpg(), "image/jpeg"
                    elif path == "/lab":
                        body, ctype = view.page("lab").encode(), "text/html; charset=utf-8"
                    elif path == "/lab.json":
                        body, ctype = json.dumps(view.lab_state()).encode(), "application/json"
                    elif path == "/status.json":
                        body, ctype = json.dumps(view.status()).encode(), "application/json"
                    else:
                        extra = view.extra_get(self.path)
                        if extra is None:
                            self.send_error(404)
                            return
                        body, ctype = extra
                except Exception as e:          # never break the session over a picture
                    body, ctype = ("view error: %s" % e).encode(), "text/plain"
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass                        # the browser went away mid-answer (a closed tab): nothing to do

            def do_POST(self):
                if self.path.split("?")[0] != "/answer":
                    self.send_error(404)
                    return
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                    d = json.loads(self.rfile.read(n).decode() or "{}")
                    with view.lock:
                        if view.question and str(d.get("id")) == str(view.question["id"]):
                            view.answers[view.question["id"]] = str(d.get("text") or "")
                    body = b"ok"
                except Exception as e:
                    body = ("error: %s" % e).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self.srv = ThreadingHTTPServer((self.bind, self.port), H)
        except OSError as e:
            return "live view not started (port %d: %s)" % (self.port, e)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        if self.bind == "127.0.0.1":
            return "live view: open http://localhost:%d in a browser" % self.port
        import socket
        host = socket.gethostname()
        return ("live view: open http://%s.local:%d in a browser on any computer or phone on this WiFi (questions "
                "are asked there too)" % (host, self.port))

    def ask(self, prompt, default="", timeout_s=None):
        """Put a question on the page and wait for its answer (or timeout_s: then None)."""
        with self.lock:
            qid = int(time.time() * 1000)
            self.question = dict(id=qid, prompt=prompt.strip(), default=default, timeout_s=timeout_s)
        t_end = None if timeout_s is None else time.time() + timeout_s
        try:
            while t_end is None or time.time() < t_end:
                with self.lock:
                    if qid in self.answers:
                        return self.answers.pop(qid).strip()
                time.sleep(0.3)
            return None
        finally:
            with self.lock:
                if self.question and self.question["id"] == qid:
                    self.question = None

    def stop(self):
        if self.srv:
            self.srv.shutdown()

    # hooks for e02_replay_view.py (the live view serves the pages as they are and nothing else)
    def page(self, name):
        return PAGE if name == "main" else LAB_PAGE

    def extra_get(self, path):
        """Another GET path: (body bytes, content type), or None for a 404."""
        return None

    def lab_state(self):
        import e02_live as LIVE
        r = self.r
        lab = getattr(r, "lab", None)
        st = r.st
        return dict(filter=lab.state() if lab is not None else dict(robot=None),
                    calibration=LIVE.calibration_series(st), results=LIVE.results_series(st),
                    phase=getattr(r, "phase", ""), t=time.time())

    HEALTH_WINDOW_S = 30.0    # chosen: long enough to smooth one bad second, short enough to show trouble coming

    def health(self):
        """Tracker health for the engineer (engineer-loop.md item 4): over the last HEALTH_WINDOW_S, where the
        position came from (both / lock-on alone / detector alone / held / none, shares of pictures), re-locks, the
        detector's checks that disagreed; now: the lock's match score, how long since the robot was last seen; from
        perf.csv's last row: pictures handled a second, a picture's age when its position was ready, pictures
        dropped. Each gets a level: ok / amber / red (limits below, chosen; a DOE on the confidence threshold would
        set them). Data: the tracker's own counters (track.csv) and perf.csv."""
        r = self.r
        tr = r.tracker
        if tr is None or not hasattr(tr, "confidence"):
            return None
        now = time.time()
        c = tr.confidence()
        st = c.get("stats") or {}
        hist = getattr(self, "_hist", None)
        if hist is None:
            from collections import deque
            hist = self._hist = deque()
        hist.append((now, dict(st)))
        while len(hist) > 2 and hist[1][0] < now - self.HEALTH_WINDOW_S:
            hist.popleft()
        old = hist[0][1]
        d = {k: st.get(k, 0) - old.get(k, 0) for k in st}
        pics = sum(d.get(k, 0) for k in ("both", "lock", "detector", "held", "none"))
        share = {k: (d.get(k, 0) / pics if pics else None) for k in ("both", "lock", "detector", "held", "none")}
        perf = (getattr(r, "perf_rows", None) or [None])[-1]
        seen_age = None if not getattr(tr, "last_seen", None) else round(now - tr.last_seen, 1)

        def lvl(v, amber, red, low_bad=False):
            if v is None:
                return "none"
            if low_bad:
                return "red" if v < red else "amber" if v < amber else "ok"
            return "red" if v > red else "amber" if v > amber else "ok"

        seen_share = None if not pics else 1.0 - (share["none"] or 0) - (share["held"] or 0)
        out = dict(window_s=self.HEALTH_WINDOW_S, robot=tr.active or "", source=c.get("source"),
                   score=c.get("score"), share=share, pictures=pics, relocks=d.get("relocks", 0),
                   disagree=d.get("disagree", 0), checks=d.get("checks", 0), seen_age_s=seen_age,
                   fps=None if perf is None else perf.get("fps_handled"),
                   age_s=None if perf is None else perf.get("age_max_s"),
                   dropped=None if perf is None else perf.get("dropped"))
        out["levels"] = dict(seen=lvl(seen_share, 0.9, 0.7, low_bad=True),
                             score=lvl(c.get("score"), 0.7, 0.5, low_bad=True),
                             age=lvl(out["age_s"], 0.5, 1.0), fps=lvl(out["fps"], 10, 5, low_bad=True),
                             disagree=lvl(out["disagree"], 0, 2), seen_age=lvl(seen_age if tr.active else None, 1.0, 3.0))
        out["seen_share"] = seen_share
        return out

    def status(self):
        r = self.r
        tr = r.tracker
        with self.lock:
            log = list(self.lines)
        note = BRICKS_NOTE if getattr(r, "bricks_view", None) else ""
        with self.lock:
            q = dict(self.question) if self.question else None
        try:
            live = self.live_state()
        except Exception as e:          # the console must never break the session (or the old fields)
            live = dict(error=str(e))
        return dict(phase=getattr(r, "phase", ""), order=getattr(r, "order", ""),
                    active=(tr.active if tr is not None and tr.active else ""), log=log, t=time.time(), note=note,
                    health=self.health(), question=q, live=live)

    # --- the operations console (2026-10-25, Nick's approved mockup): everything below only reads the runner ----
    def live_state(self):
        """The console's extra state: header, trust table, processing speed, timeline, plan, results, robots
        (state, battery and its trend, sensors, distance to the open edge), self-calibration and the open hold.
        Each part is built on its own: one that fails comes back as None and the rest still show."""
        lk = self.__dict__.setdefault("_live_lock", threading.Lock())   # two viewers: one builder at a time
        with lk:
            return self._live_state()

    def _live_state(self):
        out = {}
        for k, f in (("header", self._live_header), ("trust", self._live_trust), ("perf", self._live_perf),
                     ("timeline", self._live_timeline), ("plan", self._live_plan), ("results", self._live_results),
                     ("robots", self._live_robots), ("calib", self._live_calib), ("hold", self._live_hold)):
            try:
                out[k] = f()
            except Exception as e:
                out[k] = None
                out.setdefault("errors", {})[k] = "%s: %s" % (type(e).__name__, e)
        return out

    def _names(self):
        try:
            return list(self.r.st.get("robots") or [])
        except Exception:
            return []

    def _holds(self):
        """[dict(t0, t1 (None while open), robot, kind, why, answer)] from the state's holds."""
        out = []
        for h in list(self.r.st.get("holds") or []):
            if len(h) < 7:
                continue
            t0, dur = h[0], h[6]
            out.append(dict(t0=t0, t1=None if dur is None else t0 + dur, robot=h[1], kind=h[2], why=h[3],
                            answer=h[5]))
        return out

    def _hold_open(self):
        hv = getattr(self.r, "hold", None)
        return bool(hv is not None and hasattr(hv, "is_set") and hv.is_set())

    def _live_header(self):
        r, now = self.r, time.time()
        a = getattr(r, "a", None)
        mode = []
        if a is not None:
            mode.append("home" if getattr(a, "home", False) else "away")
            if getattr(a, "dry_run", False):
                mode.append("dry run")
            if getattr(a, "on_pi", False):
                mode.append("on the Pi")
            if getattr(a, "degrade", ""):
                mode.append("degraded: " + str(a.degrade))
        held = 0.0
        for h in self._holds():
            held += ((h["t1"] if h["t1"] is not None else (now if self._hold_open() else h["t0"])) - h["t0"])
        if self._hold_open():
            state = "hold"
        elif getattr(r, "stop_requested", False) or getattr(r, "phase", "") in ("session over", "stopped"):
            state = "stopped"
        else:
            state = "running"
        perf = self._perf_now()
        t0 = getattr(r, "t_launch", None)
        rec = ((r.st.get("setup") or {}).get("pi") or {}).get("fps")
        return dict(session=os.path.basename(str(getattr(r, "dir", "") or "")) or "-", mode=" · ".join(mode),
                    state=state, phase=getattr(r, "phase", "") or "", order=str(getattr(r, "order", "") or ""),
                    elapsed_s=None if t0 is None else round(now - t0), held_s=round(held),
                    fps=perf.get("fps"), age_s=perf.get("age_s"), slow=perf.get("slow"), slow_why=perf.get("why"),
                    rec_fps=rec, camera_fps=CAMERA_FPS)

    # processing speed: "slow" is the runner's own (e02_load.SpeedWatch, 2026-10-25: below 0.75 x its baseline
    # pictures a second or above 1.5 x its baseline picture age, 5 s in a row; the baseline is its first 2 minutes
    # without load). These fixed marks are only the fallback for a runner without the watcher (a replay, an old
    # session): chosen from what the runner treats as trouble (LATE_PICTURE_S, the tracker health's amber fps).
    SLOW_FPS = 10.0
    SLOW_AGE_S = 1.0       # on the second's worst picture age (fallback only)
    STALL_S = 3.0          # no perf row for this long = the camera loop has stalled
    PERF_WINDOW_S = 300.0  # the chart's last 5 minutes

    def _watch(self):
        sw = getattr(self.r, "speed", None)
        return sw if sw is not None and hasattr(sw, "is_slow_row") else None

    def _slow_row(self, row):
        """Was this second slow? The runner's own mark when it has one (its 5-in-a-row rule), else this view's."""
        if "slow" in row:
            return bool(row["slow"])
        sw = self._watch()
        if sw is not None:
            try:
                return bool(sw.is_slow_row(row))
            except Exception:
                return False
        return bool((row.get("fps_handled") is not None and row["fps_handled"] < self.SLOW_FPS) or
                    (row.get("age_max_s") is not None and row["age_max_s"] > self.SLOW_AGE_S))

    def _limits(self):
        """(fps limit, age limit, age column, words, baseline dict or None)."""
        sw = self._watch()
        if sw is not None and sw.base_fps is not None:
            return (sw.fps_frac * sw.base_fps, sw.age_frac * sw.base_age, "mean",
                    "under %.0f%% of this session's baseline (%.1f a second) or a picture age over %.1f x its "
                    "baseline (%.2f s), %d s in a row" % (100 * sw.fps_frac, sw.base_fps, sw.age_frac, sw.base_age,
                                                          getattr(sw, "hold_s", 5)),
                    dict(fps=round(sw.base_fps, 1), age_s=round(sw.base_age, 3)))
        if sw is not None:
            return (None, None, "mean", "learning this session's baseline (its first minutes without load)", None)
        return (self.SLOW_FPS, self.SLOW_AGE_S, "max", "under %g a second or a picture older than %g s" % (
            self.SLOW_FPS, self.SLOW_AGE_S), None)

    def _perf_now(self):
        rows = getattr(self.r, "perf_rows", None) or []
        if not rows:
            return dict(fps=None, age_s=None, slow=None, why="")
        last = rows[-1]
        sw = self._watch()
        slow = bool(sw.slow) if sw is not None else self._slow_row(last)
        why = ""
        if slow:
            why = "%.0f pictures a second, picture age %.2f s" % (last.get("fps_handled") or 0,
                                                                 last.get("age_mean_s") or 0)
        gap = time.time() - (last.get("t") or 0)
        if gap > self.STALL_S:
            slow, why = True, "no picture handled for %.0f s" % gap
        return dict(fps=last.get("fps_handled"), age_s=last.get("age_mean_s"), slow=slow, why=why)

    def _live_perf(self):
        now = time.time()
        rows = getattr(self.r, "perf_rows", None) or []
        pts = []
        for row in reversed(rows):
            t = row.get("t") or 0
            if t < now - self.PERF_WINDOW_S:
                break
            pts.append([round(t - now, 1), _r(row.get("fps_handled"), 1), _r(row.get("age_mean_s"), 2),
                        _r(row.get("age_max_s"), 2), 1 if self._slow_row(row) else 0, 1 if row.get("load") else 0])
        pts.reverse()
        p = self._perf_now()
        fl, al, acol, words, base = self._limits()
        ld = getattr(self.r, "load", None)
        return dict(window_s=self.PERF_WINDOW_S, rows=pts, slow_fps=None if fl is None else round(fl, 1),
                    slow_age_s=None if al is None else round(al, 3), age_col=acol, rule=words, baseline=base,
                    slow=p["slow"], why=p["why"], camera_fps=CAMERA_FPS, slow_s=sum(1 for q in pts if q[4]),
                    load=None if ld is None else bool(getattr(ld, "on", False)),
                    load_s=sum(1 for q in pts if q[5]))

    TREND_N = 30           # learned noise values kept per sensor for the trend
    TRUST_EVERY_S = 2.0    # the trust table is rebuilt at most this often

    def _live_trust(self):
        tr = getattr(self.r, "trust", None)
        if tr is None or not hasattr(tr, "camera_sd"):
            return None
        now = time.time()
        cache = getattr(self, "_trust_cache", None)
        if cache is not None and now - cache[0] < self.TRUST_EVERY_S:
            return cache[1]
        import e02_trust as TR
        hist = getattr(self, "_trust_hist", None)
        if hist is None:
            hist = self._trust_hist = {}
        rows = []

        def add(sensor, robot, label, unit, sd, n):
            healthy = TR.HEALTHY[sensor]
            k = sd / healthy if healthy else None
            if n < TR.MIN_N[sensor]:
                state = "little"
            elif tr.bad(sensor, robot):
                state = "bad"
            else:
                state = "ok"
            # the runner's flag changes only at its next trust check (e02.trust_note, with hysteresis): say when the
            # numbers have already crossed back (or over) so a "looks bad" next to k 0.4 is not a puzzle
            lag = ""
            if k is not None and state == "bad" and k < TR.RECOVERED_K:
                lag = "back under %g: cleared at the runner's next check" % TR.RECOVERED_K
            elif k is not None and state == "ok" and k >= TR.DEGRADED_K:
                lag = "over %g: the runner's next check decides" % TR.DEGRADED_K
            key = "%s/%s" % (sensor, robot)
            h = hist.setdefault(key, [])
            if not h or abs(h[-1][1] - sd) > 0.005 or (n != h[-1][2]):
                h.append((round(now, 1), round(sd, 3), n))
                del h[:-self.TREND_N]
            rows.append(dict(key=key, sensor=sensor, robot=robot, label=label, unit=unit, sd=round(sd, 2),
                             healthy=healthy, k=None if k is None else round(k, 2), n=n, min_n=TR.MIN_N[sensor],
                             state=state, lag=lag, trend=[q[1] for q in h]))
        sd, n = tr.camera_sd()
        add("camera", "*", "Camera", "cm", sd, n)
        for rb in self._names():
            sd, n, _how = tr.own_sd("wheels", rb)
            add("wheels", rb, "Wheels", "cm/stretch", sd, n)
            sd, n, _how = tr.own_sd("gyro", rb)
            add("gyro", rb, "Gyro", "deg/stretch", sd, n)
            sd, n = tr.front_sd(rb)
            add("front", rb, "Front sensor", "cm", sd, n)
        out = dict(rows=rows, degraded_k=TR.DEGRADED_K, recovered_k=TR.RECOVERED_K,
                   shared_note=("every robot's stretches are noisier than the camera at the finds explains (the "
                                "camera away from the find corner, or every robot's wheels alike)")
                   if tr.shared_unexplained("wheels") else "")
        self._trust_cache = (now, out)
        return out

    def _live_timeline(self):
        """Phases over time: setup / runs / finding (from perf.csv's phase each second and the schedule's run
        times), and the holds (from the state's holds)."""
        r, now = self.r, time.time()
        rows = getattr(r, "perf_rows", None) or []
        tl = getattr(self, "_tl", None)
        if tl is None or tl["n"] > len(rows):
            tl = self._tl = dict(n=0, segs=[])
        segs = tl["segs"]
        for row in rows[tl["n"]:]:
            cat = PHASE_CAT.get(row.get("phase") or "", "setup")
            t = row.get("t") or now
            if segs and segs[-1][2] == cat and t - segs[-1][1] < 10:
                segs[-1][1] = t
            else:
                if segs:
                    segs[-1][1] = max(segs[-1][1], t)
                segs.append([t, t, cat])
        tl["n"] = len(rows)
        t0 = getattr(r, "t_launch", None) or (segs[0][0] if segs else now)
        runs = [[q["t_start"], q.get("t_end") or now] for q in (r.st.get("schedule") or []) if q.get("t_start")]
        holds = [[h["t0"], h["t1"] if h["t1"] is not None else now, h["robot"], h["kind"]] for h in self._holds()]
        out = [[round(a - t0, 1), round(b - t0, 1), c] for a, b, c in segs if b >= t0]
        if out and out[-1][2] == PHASE_CAT.get(getattr(r, "phase", "") or "", "setup"):
            out[-1][1] = round(now - t0, 1)
        return dict(span_s=round(now - t0, 1), segs=out[-400:],
                    runs=[[round(a - t0, 1), round(b - t0, 1)] for a, b in runs if b >= t0],
                    holds=[[round(a - t0, 1), round(b - t0, 1), w, k] for a, b, w, k in holds if b >= t0])

    def _live_plan(self):
        sch = list(self.r.st.get("schedule") or [])
        runs = [dict(order=q.get("order"), cond=q.get("cond"), robot=q.get("robot"), route=q.get("route"),
                     status=q.get("status"), err=q.get("end_err_cm")) for q in sch]
        done = sum(1 for q in runs if q["status"] == "done")
        return dict(runs=runs, done=done, total=sum(1 for q in runs if q["status"] != "left_out"),
                    current=str(getattr(self.r, "order", "") or ""))

    def _live_results(self):
        import e02_live as LIVE
        return LIVE.results_series(self.r.st)

    def _live_calib(self):
        import e02_live as LIVE
        return LIVE.calibration_series(self.r.st)

    def _live_hold(self):
        if not self._hold_open():
            return None
        hs = [h for h in self._holds() if h["answer"] is None]
        h = hs[-1] if hs else (self._holds() or [None])[-1]
        if h is None:
            return dict(robot="", kind="", why="", since_s=None)
        return dict(robot=h["robot"], kind=h["kind"], why=h["why"], since_s=round(time.time() - h["t0"]))

    def _battery_hist(self):
        """Battery % over time per robot: battery.csv's rows (read once, if the runner writes it), then what the
        robots report as it comes in (the listener's latest)."""
        r = self.r
        bh = getattr(self, "_bat", None)
        if bh is None:
            bh = self._bat = {}
            path = os.path.join(str(getattr(r, "dir", "") or ""), "battery.csv")
            try:
                with open(path) as f:
                    for line in f.readlines()[1:]:
                        p = line.strip().split(",")
                        if len(p) >= 6:
                            bh.setdefault(p[1], []).append((float(p[0]), float(p[5])))
            except (OSError, ValueError):
                pass
        lis = getattr(r, "listener", None)
        for n, v in list(dict(getattr(lis, "battery", None) or {}).items()):
            try:
                tb, b = v
            except (TypeError, ValueError):
                continue
            h = bh.setdefault(n, [])
            if not h or tb > h[-1][0]:
                h.append((float(tb), float(b)))
                del h[:-BATTERY_KEEP]
        return bh

    def _robot_where(self, n):
        """(x, y, 'live' | 'last known') for a robot, or None."""
        r = self.r
        tr = r.tracker
        if tr is not None and getattr(tr, "active", None) == n:
            w = tr.where()
            if w is not None and time.time() - w[0] < 2.0:
                return w[1], w[2], "live"
        q = (r.st.get("positions") or {}).get(n)
        if q and q[0] is not None and q[1] is not None:
            return q[0], q[1], "last known"
        return None

    def _edge(self, n):
        r = self.r
        w = self._robot_where(n)
        if w is None:
            return None
        bf = getattr(r, "border_frame", None)
        f = None
        if callable(bf):
            f = bf(w[0], w[1])
        elif r.board is not None:
            f = (r.board, w[0], w[1])
        if f is None or not hasattr(f[0], "edge_room"):
            return None
        d = f[0].edge_room(f[1], f[2])[0]
        b = r.border_cm() if callable(getattr(r, "border_cm", None)) else None
        fw = r.fw_border_cm() if callable(getattr(r, "fw_border_cm", None)) else None
        return dict(dist_cm=round(d, 1), border_cm=None if b is None else round(b, 1),
                    fw_border_cm=None if fw is None else round(fw, 1), where=w[2])

    def _robot_state(self, n):
        r = self.r
        tr = r.tracker
        if n in (getattr(r, "held_out", None) or ()):
            return "left out", "out"
        if n in (getattr(r, "unavailable", None) or ()):
            return "not available", "out"
        mv = (getattr(r, "moves", None) or {}).get(n) or {}
        leg = n in (getattr(r, "leg_live", None) or {})
        kind = (getattr(r, "leg_kind", None) or {}).get(n)
        follow = tr is not None and getattr(tr, "active", None) == n
        run = mv.get("run")
        words = []
        if leg:
            words.append("turning" if kind == "turn" else "driving")
        if run not in (None, "-"):
            words.append("run %s" % run)
        if not words:
            words.append("still")
        return " · ".join(words), "follow" if follow else ("move" if leg else "still")

    def _live_robots(self):
        tr = getattr(self.r, "trust", None)
        bh = self._battery_hist()
        trows = {q["key"]: q for q in ((self._live_trust() or {}).get("rows") or [])} if tr is not None else {}
        out = []
        now = time.time()
        for n in self._names():
            words, cls = self._robot_state(n)
            b = bh.get(n) or []
            sensors = []
            for s, lab in (("camera", "Camera"), ("wheels", "Wheels"), ("gyro", "Gyro"), ("front", "Front")):
                q = trows.get("%s/%s" % (s, "*" if s == "camera" else n))
                if q:
                    sensors.append(dict(name=lab, k=q["k"], state=q["state"], n=q["n"]))
            try:
                edge = self._edge(n)
            except Exception:
                edge = None
            out.append(dict(name=n, state=words, cls=cls, battery=b[-1][1] if b else None,
                            battery_t=round(b[-1][0]) if b else None,
                            battery_trend=[[round(t), v] for t, v in b[-BATTERY_SEND:]],
                            sensors=sensors, edge=edge))
        return out

    def frame_jpg(self):
        ok, buf = cv2.imencode(".jpg", self.frame_bgr(), [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def frame_bgr(self):
        """The picture with the runner's view drawn on it (BGR)."""
        r = self.r
        img = r.feed.last_img if r.feed is not None else None
        if img is None:
            img = np.zeros((480, 640), np.uint8)
        v = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
        cam, tr, bd = r.cam, r.tracker, r.board
        if cam is not None:
            def px(x, y, z=0.0):
                u, w = cam.project((x, y, z))
                return (int(round(u)), int(round(w))) if np.isfinite(u) and np.isfinite(w) else None
            if bd is not None:
                pts = [px(bd.x0, bd.y0), px(bd.x1, bd.y0), px(bd.x1, bd.y1), px(bd.x0, bd.y1)]
                self.draw_border_band(v, px, bd)
                if all(pts):
                    cv2.polylines(v, [np.array(pts, np.int32)], True, BOARD_BGR, 1)
                # the same outline at the height of a robot's top (where the rings are drawn): the camera looks down
                # at a slant, so a robot touching a board leans past the floor's line in the picture (Nick, 2026-10-23)
                pts = [px(bd.x0, bd.y0, 3.7), px(bd.x1, bd.y0, 3.7), px(bd.x1, bd.y1, 3.7), px(bd.x0, bd.y1, 3.7)]
                if all(pts):
                    for a_, b_ in zip(pts, pts[1:] + pts[:1]):
                        n_ = max(1, int(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) // 8))
                        for k in range(0, n_, 2):
                            p1_ = (int(a_[0] + (b_[0] - a_[0]) * k / n_), int(a_[1] + (b_[1] - a_[1]) * k / n_))
                            p2_ = (int(a_[0] + (b_[0] - a_[0]) * (k + 1) / n_), int(a_[1] + (b_[1] - a_[1]) * (k + 1) / n_))
                            cv2.line(v, p1_, p2_, BOARD_TOP_BGR, 1)
            for c in list(getattr(tr, "last_cands", []) or []):
                cv2.circle(v, (int(c["u"]), int(c["v"])), 4, (0, 220, 220) if c["shape"] else (150, 150, 150), -1)
            live = tr.where() if tr is not None else None
            for n, q in list((r.st.get("positions") or {}).items()):
                if tr is not None and n == tr.active and live is not None and time.time() - live[0] < 2.0:
                    q = (live[1], live[2])        # the robot being followed: its label goes with it (2026-10-23:
                                                  # it stayed at its park spot while the robot drove out to map)
                if q:
                    p = px(q[0], q[1], 3.7)
                    if p:
                        cv2.putText(v, n, (p[0] + 12, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
            if tr is not None:
                pos, hint = tr.where(), tr.hint
                if pos is not None:
                    p = px(pos[1], pos[2], 3.7)
                    if p:
                        cv2.circle(v, p, int(tr.rpx * 0.75), FOLLOW_BGR, 2)
                if hint is not None:
                    p = px(hint[0], hint[1], 3.7)
                    if p:
                        rr = int(hint[2] * cam.px_per_cm(hint[0], hint[1], 3.7))
                        for a in range(0, 360, 30):
                            cv2.ellipse(v, p, (rr, rr), 0, a, a + 15, FOLLOW_BGR, 1)
            lab = getattr(r, "lab", None)
            if lab is not None:
                fs = lab.state()
                if fs.get("robot") and tr is not None and tr.active == fs["robot"]:
                    p = px(fs["x"], fs["y"], 3.7)
                    if p:
                        k = cam.px_per_cm(fs["x"], fs["y"], 3.7)
                        ax = (max(1, int(fs["sd_major_cm"] * k)), max(1, int(fs["sd_minor_cm"] * k)))
                        cv2.ellipse(v, p, ax, -fs["angle_deg"], 0, 360, (255, 0, 255), 1)
            bv = getattr(r, "bricks_view", None)
            if bv:
                draw_bricks(v, cam, bv)
            draw_explore(v, r)
            m = (r.st.get("setup") or {}).get("map") or {}
            for ray in m.get("rays", []):
                s0, pt = ray.get("cam_start"), ray.get("point")
                if s0 and pt:
                    a, b = px(s0[0], s0[1]), px(pt[0], pt[1])
                    if a and b:
                        cv2.line(v, a, b, (0, 0, 255) if ray.get("kind") == "edge" else (255, 160, 0), 1)
        return v

    def draw_border_band(self, v, px, bd):
        """The virtual border on the picture (in place): the strip within border_cm of each open edge, shaded
        amber on the floor, with its inner line. Not while exploring (the board isn't fitted yet). The border is the
        runner's own border_cm() (home / away, and wider while the camera looks bad)."""
        r = self.r
        if getattr(r, "exploring", False) or not hasattr(bd, "open_edges"):
            return
        b = r.border_cm() if callable(getattr(r, "border_cm", None)) else None
        if not b:
            return
        cx, cy = 0.5 * (bd.x0 + bd.x1), 0.5 * (bd.y0 + bd.y1)
        polys, lines = [], []
        for ax, val in bd.open_edges():
            if ax == "x":
                inner = val - b if val > cx else val + b
                q = [px(val, bd.y0), px(val, bd.y1), px(inner, bd.y1), px(inner, bd.y0)]
            else:
                inner = val - b if val > cy else val + b
                q = [px(bd.x0, val), px(bd.x1, val), px(bd.x1, inner), px(bd.x0, inner)]
            if all(q):
                polys.append(np.array(q, np.int32))
                lines.append((q[2], q[3]))
        if not polys:
            return
        over = v.copy()
        cv2.fillPoly(over, polys, BAND_BGR)
        cv2.addWeighted(over, 0.28, v, 0.72, 0, dst=v)
        for a_, b_ in lines:
            cv2.line(v, a_, b_, BAND_BGR, 1, cv2.LINE_AA)


BOARD_BGR = (193, 211, 111)        # teal #6FD3C1: the board outline on the floor
BOARD_TOP_BGR = (130, 145, 70)     # darker teal: the same outline at a robot's top height (dashed)
FOLLOW_BGR = (255, 184, 127)       # blue #7FB8FF: the robot being followed
BAND_BGR = (58, 163, 224)          # amber #E0A33A: the virtual border's strip along the open edges

EXPLORE_COLOURS = {"Left": (255, 200, 0), "Center": (0, 200, 255), "Right": (255, 0, 200)}


def draw_explore(v, r):
    """Explore (Nick, 2026-10-23: "virtual border being created live"): in picture pixels, as the camera saw them
    at each stop (setup.explore_live, written by the runner): every robot's stops joined in order (its colour; a grey
    cross where the camera's spot did not fit what the wheels did), each ray's end (orange: the front sensor met a
    board; red: an edge sensor met an open edge), and the border so far (dashed white: the outline round the ray
    ends). Drawn only while exploring; afterwards the fitted board outline takes over."""
    if not getattr(r, "exploring", False):
        return
    live = (r.st.get("setup") or {}).get("explore_live") or {}
    stops, contacts = live.get("stops") or [], live.get("contacts") or []
    last = {}
    for s in stops:
        n, u, w = s[0], int(s[1]), int(s[2])
        skip = len(s) > 3 and s[3]
        col = EXPLORE_COLOURS.get(n, (200, 200, 200))
        if skip:
            cv2.drawMarker(v, (u, w), (150, 150, 150), cv2.MARKER_TILTED_CROSS, 8, 1)
            continue
        if n in last:
            cv2.line(v, last[n], (u, w), col, 1)
        cv2.circle(v, (u, w), 2, col, -1)
        last[n] = (u, w)
    pts = []
    for c in contacts:
        n, u, w, kind = c[0], int(c[1]), int(c[2]), c[3]
        cv2.circle(v, (u, w), 5, (0, 0, 255) if kind == "edge" else (0, 160, 255), 2)
        pts.append((u, w))
    if len(pts) >= 3:
        hull = cv2.convexHull(np.array(pts, np.int32)).reshape(-1, 2)
        for a_, b_ in zip(hull, np.roll(hull, -1, 0)):
            n_ = max(1, int(math.hypot(*(b_ - a_)) // 8))
            for k in range(0, n_, 2):
                p1_ = tuple(int(x) for x in a_ + (b_ - a_) * k / n_)
                p2_ = tuple(int(x) for x in a_ + (b_ - a_) * (k + 1) / n_)
                cv2.line(v, p1_, p2_, (255, 255, 255), 1)
    cv2.putText(v, "Explore: dots = robot stops seen by the camera; circles = ray ends (orange board, red edge); "
                   "dashed = border so far", (6, v.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1)


BRICKS_NOTE = ("Brick demo: red outlines = the objects the camera found; dashed red = the room kept round them; blue "
               "line = the planned path (white dot: start, white ring: goal); magenta = crossing 1 (factory settings); "
               "orange = crossing 2 (its calibration, wheels and gyro only); green = crossing 3 (with camera fixes).")


def draw_bricks(v, cam, bv):
    """The brick demo on a BGR picture (in place): bricks' outlines, the planned path, start and goal, and each
    crossing's camera track. bv: dict(bricks [dict(outline)], room_cm, path [(x, y)], start, goal,
    tracks {1: [(x, y)], 2: [...]})."""
    def px(x, y):
        u, w = cam.project((x, y, 0.0))
        return (int(round(u)), int(round(w))) if np.isfinite(u) and np.isfinite(w) else None
    for b in bv.get("bricks") or []:
        pts = [px(*q) for q in b.get("outline") or []]
        pts = [q for q in pts if q]
        if len(pts) >= 3:
            cv2.polylines(v, [np.array(pts, np.int32)], True, (40, 40, 230), 2, cv2.LINE_AA)
        room = bv.get("room_cm")
        if room and b.get("outline"):
            # the room kept round the brick: its outline pushed out by room_cm (drawn dashed)
            o = np.array(b["outline"], float)
            c = o.mean(axis=0)
            ring = []
            for k in range(0, 360, 12):
                d = np.array([math.cos(math.radians(k)), math.sin(math.radians(k))])
                ext = float(np.max((o - c) @ d))
                q = px(*(c + d * (ext + room)))
                if q:
                    ring.append(q)
            for a, b2 in list(zip(ring, ring[1:] + ring[:1]))[::2]:
                cv2.line(v, a, b2, (60, 60, 200), 1, cv2.LINE_AA)
    cols = {1: (255, 60, 255), 2: (0, 140, 255), 3: (80, 220, 80), "1": (255, 60, 255), "2": (0, 140, 255),
            "3": (80, 220, 80)}           # the ladder: factory, calibrated, camera fixes
    for k, tr in (bv.get("tracks") or {}).items():
        pts = [px(*q[-2:]) for q in tr]
        pts = [q for q in pts if q]
        if len(pts) >= 2:
            cv2.polylines(v, [np.array(pts, np.int32)], False, cols.get(k, (200, 200, 200)), 2, cv2.LINE_AA)
    path = [px(*q) for q in bv.get("path") or []]
    path = [q for q in path if q]
    if len(path) >= 2:
        cv2.polylines(v, [np.array(path, np.int32)], False, (255, 160, 40), 2, cv2.LINE_AA)
        for q in path[1:-1]:
            cv2.circle(v, q, 3, (255, 160, 40), -1)
    if bv.get("start"):
        q = px(*bv["start"])
        if q:
            cv2.circle(v, q, 4, (255, 255, 255), -1)
    if bv.get("goal"):
        q = px(*bv["goal"])
        if q:
            cv2.circle(v, q, 7, (255, 255, 255), 2)
    return v


LAB_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Live lab</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0e1116;--card:#171b22;--fg:#e8eaed;--dim:#9aa0a6;--wheels:#f2a33a;--gyro:#5aa9ff;--camera:#3ccf8e;
--grid:#2a2f38;--U:#e06c75;--C:#e5c07b;--K:#61afef;--KC:#98c379}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
header{padding:14px 16px 0}h1{font-size:20px;margin:0}header p{color:var(--dim);margin:4px 0 0}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;padding:16px}
.card{background:var(--card);border-radius:10px;padding:14px}
.card h2{font-size:15px;margin:0 0 4px}.card .note{color:var(--dim);font-size:13px;margin:6px 0 0}
.card .src{color:var(--dim);font-size:11px;margin:6px 0 0;opacity:.85}
img{width:100%;border-radius:6px}
.bar{display:flex;align-items:center;gap:8px;margin:8px 0}.bar span{width:64px;color:var(--dim)}
.track{flex:1;height:14px;background:var(--grid);border-radius:7px;overflow:hidden}
.fill{height:100%;border-radius:7px;transition:width .4s}
.big{font-size:18px;margin:8px 0}
svg text{fill:var(--dim);font-size:11px}
</style></head><body>
<header><h1>Live lab</h1><p>What the system is learning while the robots run. <a href="/" style="color:#8cf">camera view</a></p></header>
<main>
<div class="card"><h2>The camera, with the filter's view</h2><img id="pic" src="/frame.jpg" alt="overhead camera">
<p class="note">Magenta: where the filter thinks the moving robot is, and how unsure it is (the ellipse; smaller is surer).</p>
<p class="src">Measured: the filter's position and 1-sigma uncertainty (cm). Data: the robot's wheels and gyro (RAW rows) and the overhead camera, in the live Kalman filter (tools/fusion/ekf.py).</p></div>
<div class="card"><h2>How much the filter trusts each sensor now</h2>
<div id="robot" class="note"></div>
<p style="margin:10px 0 2px">Where it is</p>
<div class="track" style="height:22px;display:flex"><div id="p_motion" class="fill" style="background:var(--wheels);border-radius:0;width:50%"></div><div id="p_camera" class="fill" style="background:var(--camera);border-radius:0;width:50%"></div></div>
<div class="note"><span style="color:var(--wheels)">&#9632;</span> wheels and gyro <b id="v_motion">-</b> &nbsp; <span style="color:var(--camera)">&#9632;</span> camera <b id="v_camera">-</b></div>
<p style="margin:12px 0 2px">Which way it turns</p>
<div class="track" style="height:22px;display:flex"><div id="t_gyro" class="fill" style="background:var(--gyro);border-radius:0;width:50%"></div><div id="t_wheels" class="fill" style="background:var(--wheels);border-radius:0;width:50%"></div></div>
<div class="note"><span style="color:var(--gyro)">&#9632;</span> gyro <b id="v_gyro">-</b> &nbsp; <span style="color:var(--wheels)">&#9632;</span> wheels <b id="v_wheels">-</b></div>
<div id="words" class="big"></div>
<p class="note">Where it is: how much of any error the camera corrects within a second, against what the wheels and
gyro carry forward. Which way it turns: the gyro doesn't slip, the wheels do, more the faster it turns.
How unsure it is now: <b id="sigma">-</b>.</p>
<p class="src">Measured: the share of each reading's surprise the filter took on (0-100%), averaged per sensor over the last few seconds. Data: RAW rows from the moving robot and the camera's positions.</p></div>
<div class="card"><h2>Self-calibration, settling run by run</h2><svg id="cal" viewBox="0 0 340 210" width="100%"></svg>
<p class="note">Each line is one robot's gyro scale as the system learned it from the camera: it starts at a guess and settles.</p>
<p class="src">Measured: gyro scale (true turn / the gyro's reading), y; the run it was refitted after, x. Data: the runner's accepted self-calibration fits (fits.csv).</p></div>
<div class="card"><h2>Results so far: how far from its start each robot ended</h2><svg id="res" viewBox="0 0 340 210" width="100%"></svg>
<p class="note">One dot per run, by condition; the bar is the average. Lower is better.</p>
<p class="src">Measured: end error, cm (camera position at the end vs the start, live, before the footage re-measurement). Data: the runner's run results (e02_runs.csv). Runs a safety stop ended are not shown.</p></div>
</main>
<script>
const COL={U:'var(--U)',C:'var(--C)',K:'var(--K)',KC:'var(--KC)'};
const RCOL=['#f2a33a','#5aa9ff','#3ccf8e','#c678dd','#e06c75'];
let busy=false;
function pic(){if(busy)return;busy=true;const im=new Image();im.onload=()=>{document.getElementById('pic').src=im.src;busy=false};im.onerror=()=>{busy=false};im.src='/frame.jpg?t='+Date.now();}
function svgEl(t,a){const e=document.createElementNS('http://www.w3.org/2000/svg',t);for(const k in a)e.setAttribute(k,a[k]);return e}
function axes(svg,ylab,lo,hi){svg.innerHTML='';for(let i=0;i<=4;i++){const y=20+i*40;svg.appendChild(svgEl('line',{x1:40,x2:330,y1:y,y2:y,stroke:'#2a2f38'}));
 const t=svgEl('text',{x:4,y:y+4});const val=hi-(hi-lo)*i/4;t.textContent=val.toFixed(ylab=='gyro'?3:(Number.isInteger(val)?0:1));svg.appendChild(t)}}
function cal(rows){const svg=document.getElementById('cal');const v=rows.filter(r=>r.gyro_scale!=null);
 let lo=1.0,hi=1.12;v.forEach(r=>{lo=Math.min(lo,r.gyro_scale-0.01);hi=Math.max(hi,r.gyro_scale+0.01)});axes(svg,'gyro',lo,hi);
 const robots=[...new Set(v.map(r=>r.robot))];const n=Math.max(2,...v.map(r=>r.order||0));
 robots.forEach((rb,i)=>{const pts=v.filter(r=>r.robot==rb).map(r=>[40+290*(r.order||0)/n,20+160*(hi-r.gyro_scale)/(hi-lo)]);
  if(pts.length)svg.appendChild(svgEl('polyline',{points:pts.map(p=>p.join(',')).join(' '),fill:'none',stroke:RCOL[i%5],'stroke-width':2}));
  pts.forEach(p=>svg.appendChild(svgEl('circle',{cx:p[0],cy:p[1],r:3,fill:RCOL[i%5]})));
  const t=svgEl('text',{x:44+i*70,y:205});t.textContent=rb;t.setAttribute('style','fill:'+RCOL[i%5]);svg.appendChild(t)});}
function res(rows){const svg=document.getElementById('res');let hi=8;rows.forEach(r=>hi=Math.max(hi,Math.ceil(r.end_err_cm/8)*8));axes(svg,'cm',0,hi);
 ['U','C','K','KC'].forEach((c,i)=>{const x=80+i*70;const v=rows.filter(r=>r.cond==c);
  v.forEach((r,j)=>svg.appendChild(svgEl('circle',{cx:x-12+(j%5)*6,cy:20+160*(1-r.end_err_cm/hi),r:3,fill:COL[c],opacity:.8})));
  if(v.length){const m=v.reduce((a,r)=>a+r.end_err_cm,0)/v.length;svg.appendChild(svgEl('line',{x1:x-18,x2:x+18,y1:20+160*(1-m/hi),y2:20+160*(1-m/hi),stroke:COL[c],'stroke-width':3}))}
  const t=svgEl('text',{x:x-8,y:200});t.textContent=c+' ('+v.length+')';svg.appendChild(t)});}
function tick(){pic();fetch('/lab.json').then(r=>r.json()).then(s=>{const f=s.filter||{};
 document.getElementById('robot').textContent=f.robot?('Following '+f.robot+(f.run?' (run '+f.run+')':'')):'No robot moving right now ('+(s.phase||'-')+').';
 const w=f.weights;if(w){const set=(id,v)=>{document.getElementById(id).style.width=(100*v)+'%'};
  set('p_motion',w.position.motion);set('p_camera',w.position.camera);set('t_gyro',w.turning.gyro);set('t_wheels',w.turning.wheels);
  const txt=(id,v)=>{document.getElementById(id).textContent=Math.round(100*v)+'%'};
  txt('v_motion',w.position.motion);txt('v_camera',w.position.camera);txt('v_gyro',w.turning.gyro);txt('v_wheels',w.turning.wheels)}
 document.getElementById('words').textContent=f.words||'';document.getElementById('sigma').textContent=f.sigma_xy_cm!=null?f.sigma_xy_cm+' cm':'-';
 cal(s.calibration||[]);res(s.results||[]);}).catch(()=>{});}
setInterval(tick,700);tick();
</script></body></html>"""
