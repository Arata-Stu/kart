#include "kart_vehicle/bridge_protocol.hpp"
#include "kart_vehicle/host_session.hpp"
#include "kart_vehicle/serial_port.hpp"
#include <cassert>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <thread>
using namespace kart_vehicle;
std::string frame(const std::string & payload) {
  std::ostringstream out; out<<payload<<','<<std::hex<<std::uppercase<<std::setw(4)<<std::setfill('0')<<crc16_ccitt(payload)<<"\r\n"; return out.str();
}
int main() {
  assert(crc16_ccitt("123456789")==0x29B1);
  CommandFrame cmd; cmd.sequence=42; cmd.steering_milli=-500; cmd.throttle_milli=750; cmd.flags=3;
  assert(encode_command(cmd)=="JPB1,C,42,-500,750,0,0,3,2524\n");
  bool rejected=false;
  try {auto bad=cmd; bad.reverse_milli=1; encode_command(bad);} catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected); rejected=false;
  try {auto bad=cmd; bad.brake_milli=1; encode_command(bad);} catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected);
  auto status=parse_status(frame("JPB1,S,12,1500,1500,1800,1250,1125,6000,1,2,0"));
  assert(status && status->active_path==ActivePath::automatic && status->vbec_mv==6000);
  assert(!parse_status("JPB1,S,12,1500,1500,1800,1250,1125,6000,1,2,0,FFFF"));
  assert(!parse_status(frame("JPB1,S,12,1500,1500,1800,1250,1125,6000,9,2,0")));
  assert(!parse_status(frame("JPB1,S,12,1500,1500,1800,9999,1125,6000,1,2,0")));
  auto rc=propo_pwm_to_command(1000,1000,PropoCalibration{}); assert(rc && rc->steering==1 && rc->throttle==1);
  rc=propo_pwm_to_command(2000,2000,PropoCalibration{}); assert(rc && rc->steering==-1 && rc->reverse==1);
  assert(!propo_pwm_to_command(0,1500,PropoCalibration{}));
  StatusSequence seq; assert(seq.accept(0xfffffffeU)); assert(seq.accept(0xffffffffU)); assert(seq.accept(0));
  assert(!seq.accept(0)); assert(!seq.accept(0xffffffffU)); assert(seq.accept(1));
  HostSession session;
  assert(session.output({0,1,0,0},true,0,-1,0).flags==1);
  session.start(1);
  assert(session.output({},false,1.1,-1,0).flags==1); // Callback ordering grace, no HOST flag.
  assert(session.output({},true,1.12,-1,0).flags==3);
  assert(session.phase()==HostSession::Phase::neutral_sent);
  session.status(*status); assert(session.phase()==HostSession::Phase::armed);
  auto motion=session.output({0.5,0.3,0,0},true,1.13,-1,0);
  assert(motion.steering_milli==-500 && motion.throttle_milli==300);
  assert(session.output({},false,1.2,-1,0).flags==1); // Stale source latches disarm.
  assert(session.output({0,1,0,0},true,1.21,-1,0).flags==1);
  session.start(2); session.output({},true,2.01,-1,0);
  assert(session.output({0,0.3,0,0},true,2.02,-1,0).flags==1); // Motion before ack refused.
  session.start(3); assert(session.output({},true,3.51,-1,0).flags==1); // Handshake timeout.
  session.start(4); session.output({},true,4.01,-1,0); session.status(*status);
  status->fault_bits=1; session.status(*status); assert(session.phase()==HostSession::Phase::disarmed);
  status->fault_bits=0; session.status(*status);
  assert(session.phase()==HostSession::Phase::disarmed); // Healthy status cannot rearm.
  session.start(5); session.output({},true,5.01,-1,0); session.status(*status);
  status->active_path=ActivePath::disabled; session.status(*status);
  assert(session.phase()==HostSession::Phase::disarmed); // Board reboot/disarm.
  session.start(6); session.output({},false,6.16,-1,0);
  assert(session.phase()==HostSession::Phase::disarmed); // No input during arm.
  status->active_path=ActivePath::automatic;
  kart_system::ControlTrim trim{0.1,0.03};
  session.start(7);
  auto trimmed=session.output({},true,7.01,-1,0,trim,true);
  assert(trimmed.flags==3 && trimmed.steering_milli==0 && trimmed.throttle_milli==0 && trimmed.reverse_milli==0);
  session.status(*status);
  trimmed=session.output({},true,7.02,-1,0,trim,true);
  assert(trimmed.steering_milli==-100 && trimmed.throttle_milli==0 && trimmed.reverse_milli==0);
  trimmed=session.output({0,0.2,0,0},true,7.03,-1,0,trim,true);
  assert(trimmed.throttle_milli==230);
  trimmed=session.output({0,0,0,0.2},true,7.04,-1,0,trim,true);
  assert(trimmed.reverse_milli==0 && trimmed.brake_milli==170);
  assert(session.esc().state()==EscState::State::brake);
  trimmed=session.output({0,0.2,0,0},true,7.05,-1,0,trim,false);
  assert(trimmed.throttle_milli==200 && trimmed.steering_milli==-100);
  session.stop(); trimmed=session.output({},true,7.06,-1,0,trim,true);
  assert(trimmed.flags==1 && trimmed.steering_milli==0 && trimmed.throttle_milli==0);
  assert(session.esc().state()==EscState::State::unknown);
  // POSIX pseudo terminal: framing, fragmented status, nonblocking reads, no physical USB.
  int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK); assert(master>=0);
  assert(grantpt(master)==0 && unlockpt(master)==0);
  {
    SerialPort port(ptsname(master));
    const auto wire=frame("JPB1,S,12,1500,1500,1800,1250,1125,6000,1,2,0");
    assert(::write(master,wire.data(),10)==10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(port.read_lines().empty());
    assert(::write(master,wire.data()+10,wire.size()-10)==static_cast<ssize_t>(wire.size()-10));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const auto lines=port.read_lines(); assert(lines.size()==1 && parse_status(lines[0]));
    port.write_line(encode_command(cmd));
    char buffer[256]; const auto n=::read(master,buffer,sizeof(buffer));
    assert(n>0 && std::string(buffer,static_cast<size_t>(n)).find(encode_command(cmd))!=std::string::npos);
  }
  ::close(master);
  std::cout << "bridge protocol, host session, pseudo-terminal: passed\n";
}
