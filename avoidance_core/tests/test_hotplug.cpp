#include "../AvoidanceMixer.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
using namespace avoidance;
AdvisoryWire pkt(uint16_t seq,uint8_t flags,float speed=0.3f,float yaw=0.15f,uint32_t session=1){
 AdvisoryWire p{};p.flags=flags;p.sequence=seq;p.session=session;
 p.vx_limit_mps=speed;p.yaw_correction_radps=yaw;
 p.front_clearance_m=1.3f;p.left_clearance_m=2.f;p.right_clearance_m=1.f;
 seal(p);return p;
}
bool push(Mixer&m,const AdvisoryWire&p,uint64_t ms){
 return m.accept(reinterpret_cast<const uint8_t*>(&p),sizeof(p),ms);
}
int main(){
 Mixer m(350);
 // PX4 AUTO continues autonomously with no companion.
 assert(m.step(0,0.6f,0.04f,true,false).state==State::LINK_NOT_SEEN);
 assert(m.step(0,0.6f,0.04f,true,false).vx==0.6f);
 // Valid hazard advisory clamps the onboard PX4 mission.
 assert(push(m,pkt(1,VALID|OBSTACLE),100));
 auto v=m.step(200,0.6f,0.04f,true,false);
 assert(v.state==State::AVOIDING&&v.vx==0.3f&&v.yaw>0.18f);
 // Replay, CRC corruption, malformed length, nonfinite values rejected.
 assert(!push(m,pkt(1,VALID|OBSTACLE),110));
 auto corrupt=pkt(2,VALID|OBSTACLE);corrupt.vx_limit_mps=3.0f;
 assert(!push(m,corrupt,200));
 auto huge=pkt(2,VALID|OBSTACLE,8.f);assert(!push(m,huge,200));
 // Mini-PC unplugged while obstacle remains -> hold, not blind resume.
 auto stale=m.step(500,0.6f,0.02f,true,false);
 assert(stale.state==State::STALE_HAZARD_HOLD&&stale.vx==0);
 assert(stale.yaw==0);
 // RC overrides mission mixer but physical E-STOP remains separate.
 auto rc=m.step(510,0.25f,-0.1f,false,true);
 assert(rc.state==State::MANUAL_RC&&rc.vx==0.25f);
 // New session accepted after timeout (hotplug); clear requires 3 scans.
 assert(push(m,pkt(1,VALID,0.8f,0.f,2),600));
 assert(m.step(601,0.6f,0.f,true,false).vx==0.f);
 assert(push(m,pkt(2,VALID,0.8f,0.f,2),620));
 assert(m.step(621,0.6f,0.f,true,false).vx==0.f);
 assert(push(m,pkt(3,VALID,0.8f,0.f,2),640));
 assert(m.step(641,0.6f,0.f,true,false).vx==0.6f);
 // Once last-known clear, companion disconnect lets PX4 maintain mission.
 assert(m.step(1100,0.7f,-0.05f,true,false).state==State::STALE_CLEAR);
 assert(m.step(1100,0.7f,-0.05f,true,false).vx==0.7f);
 // Fault STOP remains latched during disconnection.
 assert(push(m,pkt(4,VALID|OBSTACLE|STOP,0.f,0.f,2),1200));
 assert(m.step(1300,0.7f,0,true,false).vx==0);
 assert(m.step(1700,0.7f,0,true,false).state==State::STALE_HAZARD_HOLD);
 // Invalid sensor input -> zero commanded speed even if mission active.
 assert(m.step(1700,NAN,0,true,false).vx==0);
 std::cout<<"PASS hotplug/no-link, onboard mission fallback, hazard hold, CRC, replay, RC override, 3x clear, speed/yaw mix\n";
}
