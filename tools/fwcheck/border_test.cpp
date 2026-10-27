#include <cmath>
#include <cstring>
#include <cstdio>
#include <map>
#include <string>
std::map<std::string,float> PV;
float P(const char* n){ return PV[n]; }
namespace pose { float x=0,y=0,dist=0; bool known=true; }
#define pragma
#include "border_only.h"
int fails=0;
void chk(const char* what, bool ok){ printf("%s %s\n", ok?"ok  ":"FAIL", what); if(!ok) fails++; }
int main(){
  PV["bd_on"]=1; PV["bd_x0"]=0; PV["bd_x1"]=152; PV["bd_y0"]=0; PV["bd_y1"]=76; PV["border_cm"]=6;
  PV["bd_k_x0"]=0; PV["bd_k_x1"]=1; PV["bd_k_y0"]=2; PV["bd_lo_y0"]=60; PV["bd_hi_y0"]=152; PV["bd_k_y1"]=1;
  chk("barrier side x0 has no border (nearest open: y1, 38 cm)", fabsf(border::dist4(3,38)-38)<1e-4);
  chk("open side x1: 4 cm inside", fabsf(border::dist4(148,38)-4)<1e-4);
  chk("partial y0, alongside the barrier part: not it (nearest open: y1, 73 cm)", fabsf(border::dist4(30,3)-73)<1e-4);
  chk("partial y0, alongside the open part: 3", fabsf(border::dist4(100,3)-3)<1e-4);
  chk("partial y0, 4 cm short of the open stretch (half width): counts", fabsf(border::dist4(56,3)-3)<1e-4);
  chk("open y1 nearest", fabsf(border::dist4(80,74)-2)<1e-4);
  chk("past an open side: negative", border::dist4(153,38)<0);
  PV["bd_on"]=0; chk("bd_on 0: off", border::dist4(148,38)==border::UNSET); PV["bd_on"]=1;
  // follow tick: towards the open x1 side inside border_cm stops; along it or away does not
  border::reset(); pose::x=140; pose::y=38; pose::dist=0; border::followTick(pose::dist,true);
  pose::x=147; pose::dist=7; chk("towards the open side inside the border: stop", border::followTick(pose::dist,true));
  border::reset(); pose::x=148; pose::y=20; pose::dist=0; border::followTick(0,true);
  pose::y=40; pose::dist=20; chk("along the open side: no stop", !border::followTick(20,true));
  border::reset(); pose::x=149; pose::dist=0; border::followTick(0,true);
  pose::x=145; pose::dist=-4; chk("backing away from it: no stop", !border::followTick(-4,true));
  return fails;
}
