#include "gateway/motor/BlMotor.hpp"
#include "gateway/core/GatewayCore.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace gateway;
struct Lock final:MotorCritical { void enter() noexcept override {} void leave() noexcept override {} };
struct FakeBus final:MotorBus {
  std::uint64_t& clock;
  std::array<std::int16_t,2> position{{1000,-2000}};
  std::array<bool,3> enabled{{true,true,true}};
  std::uint16_t wheel_pos=1020;
  std::int16_t wheel_rpm=0;
  std::vector<std::vector<std::uint8_t>> frames;
  std::function<void()> hook;
  int fail_id=0, fail_addr=-1, fail_inst=0;
  bool corrupt=false;
  int fail_remaining=-1;
  bool drop_speed_write=false, ignore_off=false, ignore_enable=false;
  bool ignore_config=false, wheel_write_replies=true;
  std::array<std::uint8_t,3> response_level{{2,2,2}}, mode{{0,0,0}};
  BusError error=BusError::None, injected_error=BusError::ReplyTimeout;
  std::uint8_t status=0;
  std::uint16_t gain_cap=5000;
  std::uint32_t latency=50;
  explicit FakeBus(std::uint64_t& time):clock(time) {}
  std::uint64_t now() const noexcept override { return clock; }
  BusError lastError() const noexcept override { return error; }
  void setWheelWriteReplies(bool enabled) noexcept override { wheel_write_replies=enabled; }
  bool request(BusFormat fmt,std::uint8_t id,std::uint8_t inst,const std::uint8_t* p,
               std::size_t n,std::size_t read,BusReply& out) noexcept override {
    error=BusError::None;
    if (!permitted()) {error=permit_.deadline_us && clock>=permit_.deadline_us?BusError::Deadline:BusError::Cancelled;return false;}
    std::array<std::uint8_t,48> tx{};
    const auto size=motorPacket(id,inst,p,n,tx); assert(size);
    frames.emplace_back(tx.begin(),tx.begin()+size); clock+=latency;
    if (hook) { auto f=std::move(hook);hook=nullptr;f(); }
    const unsigned j=id==2?0:id==3?1:2;
    // The physical write can succeed while its reply is lost.
    if (inst==3 && n>=2 && p[0]==0x18 && (p[1]? !ignore_enable: !ignore_off)) enabled[j]=p[1];
    if (inst==3 && n==11 && p[0]==0x1a) {
      position[j]=static_cast<std::int16_t>(p[5]|(std::uint16_t(p[6])<<8)); enabled[j]=true;
    }
    if (inst==3 && n==3 && p[0]==0x20 && !drop_speed_write) wheel_rpm=static_cast<std::int16_t>(p[1]|(std::uint16_t(p[2])<<8));
    if (inst==3 && n==2 && !ignore_config) {
      if (p[0]==0x10) response_level[j]=p[1];
      if (p[0]==0x2d) mode[j]=p[1];
    }
    if (fail_remaining!=0 && fail_id==id && (fail_addr<0 || (n && p[0]==fail_addr)) && (!fail_inst || fail_inst==inst)) {
      if (fail_remaining>0) --fail_remaining;
      error=injected_error; return false;
    }
    if (fmt==BusFormat::Wheel && inst==3 && !wheel_write_replies) {
      error=BusError::WriteEchoOnly; return false;
    }
    const unsigned echo=fmt==BusFormat::Wheel?5:7, count=echo+4+read;
    std::array<std::uint8_t,48> rx{};
    rx[0]=rx[1]=255;rx[2]=id;rx[3]=count-4;rx[4]=status;
    if (j<2) { rx[5]=position[j]&255;rx[6]=(std::uint16_t(position[j])>>8)&255; }
    rx[echo]=n+2;rx[echo+1]=inst;rx[echo+2]=n?p[0]:0;
    auto* data=rx.data()+echo+3;
    if (read && p[0]==0x10) data[0]=response_level[j];
    if (read && p[0]==0x2d) data[0]=mode[j];
    if (read && p[0]==0x18) data[0]=enabled[j];
    if (read && p[0]==0x32) { data[0]=0xff;data[1]=0xff;data[2]=gain_cap&255;data[3]=gain_cap>>8; }
    if (read && p[0]==0x20) { data[0]=wheel_rpm&255;data[1]=(std::uint16_t(wheel_rpm)>>8)&255; }
    if (read && p[0]==0x24) {
      data[0]=wheel_pos&255;data[1]=wheel_pos>>8;
      data[2]=wheel_rpm&255;data[3]=(std::uint16_t(wheel_rpm)>>8)&255;
    }
    unsigned sum=0;for(unsigned i=2;i<count-1;++i)sum+=rx[i];
    rx[count-1]=~sum;if(corrupt)rx[count-1]^=1;
    const bool ok=motorReply(fmt,id,inst,n?p[0]:0,n+2,rx.data(),count,read,out);
    if (!ok) error=BusError::ReplyInvalid;
    return ok;
  }
  const std::vector<std::uint8_t>& last(unsigned id,unsigned address) const {
    for(auto i=frames.rbegin();i!=frames.rend();++i)
      if((*i)[2]==id&&(*i)[4]==3&&(*i)[5]==address)return *i;
    assert(false);return frames[0];
  }
};
struct Rig {
  std::uint64_t now=100000;
  Lock lock;
  BlMotorBackend motor;
  FakeBus left,right;
  BlMotorWorker lw,rw;
  explicit Rig(BlMotorConfig config={}):motor(lock,config),left(now),right(now),lw(motor,0,left),rw(motor,1,right) { idle(); }
  void workers() { lw.step();rw.step(); }
  void idle() { for(unsigned i=0;i<8;++i){workers();now+=1000;} }
  void home() {
    assert(motor.disabled()&&!motor.ready());
    assert(motor.captureHome({2.35706715F,2.35706715F,0,0},now)); assert(motor.ready());
  }
  HcrTarget target(unsigned profile=HCR_HOME_PD) {
    HcrTarget t{};t.epoch=7;t.profile=profile;t.flags=3;
    t.target[0]=t.target[1]=t.target[2]=t.target[3]=2.35706715F;
    return t;
  }
  MotorApplied run(HcrTarget t,unsigned& seq,unsigned iterations=80) {
    MotorApplied out{};std::uint64_t deadline=now+20000,next=now+5000;
    bool applied=false;
    for(unsigned i=0;i<iterations;++i) {
      if(now>=next){++seq;deadline=now+20000;next=now+5000;}
      const auto result=motor.apply(t,seq,deadline,now,out);
      assert(result!=ApplyResult::Fault);
      applied|=result==ApplyResult::Applied;
      workers();now+=500;
    }
    assert(applied);return out;
  }
};
int16_t signed16(const std::vector<std::uint8_t>& f,unsigned i) {
  return static_cast<std::int16_t>(f[i]|(unsigned(f[i+1])<<8));
}
void close(float a,float b,float eps=.001F) { assert(std::abs(a-b)<eps); }
HcrFrame decode(const WirePacket& packet) {
  HcrParser p{};HcrFrame f{};unsigned n=0;
  for(unsigned i=0;i<packet.size;++i)n+=hcr_feed(&p,packet.bytes[i],&f);
  assert(n==1);return f;
}
void gatewayIntegration(bool release_joints=false) {
  Rig r;r.home();GatewayCore core(r.motor);
  HcrFrame f{};f.type=HCR_HELLO;f.size=12;f.seq=1;f.stamp_us=1;
  hcr_put32(f.payload,7);hcr_put16(f.payload+4,200);hcr_put16(f.payload+6,100);hcr_put32(f.payload+8,HCR_SCHEMA);
  assert(core.receive(f,r.now).size);
  f.type=HCR_STATUS;f.size=16;f.seq=2;f.stamp_us=2;hcr_put32(f.payload+4,1);hcr_put32(f.payload+8,0);
  core.receive(f,r.now);
  const auto sensor=[&](){ImuObservation imu{};imu.valid=true;imu.stamp_us=r.now;core.setImu(imu);core.tick(r.now);};
  sensor();f={};f.type=HCR_STATUS;f.size=16;f.seq=3;f.stamp_us=r.now;
  hcr_put32(f.payload,7);hcr_put32(f.payload+4,2);core.receive(f,r.now);
  unsigned seq=3;std::uint64_t next=0;bool saw_pending=false,saw_applied=false;
  const auto start=r.now;std::uint64_t first_ack=0;
  for(unsigned i=0;i<(release_joints?1200U:300U);++i) {
    const auto iteration_start=r.now;
    sensor();
    if(r.now>=next) {
      auto sample=decode(core.telemetry(r.now));
      core.telemetrySent(7,sample.seq,sample.stamp_us);
      if(hcr_u32(sample.payload+4)&HCR_TARGET_VALID) {
        assert(hcr_u32(sample.payload+8)<seq);saw_applied=true;if(!first_ack)first_ack=r.now;
      } else saw_pending=true;
      f={};f.type=HCR_TARGET;f.size=52;f.seq=seq++;f.stamp_us=r.now;
      hcr_put32(f.payload,7);hcr_put32(f.payload+4,sample.seq);
      hcr_put16(f.payload+8,release_joints?HCR_RECOVERY:HCR_HOME_PD);
      hcr_put16(f.payload+10,release_joints && i>=600 && i<800 ? 0 : 3);
      for(unsigned j=0;j<4;++j)hcr_putfloat(f.payload+12+j*4,2.35706715F);
      core.receive(f,r.now);next=r.now+5000;
    }
    if (release_joints) {
      const auto mid_io=[&](){sensor();assert(core.diagnostics(r.now).fault==LocalFault::None);};
      r.left.hook=mid_io;r.right.hook=mid_io;
    }
    r.workers();
    if (release_joints && i>=100) {
      // Service still runs during each wait. No extra worker step between
      // 200 Hz batches: released feedback must not rely on background slots.
      assert(r.now-iteration_start<5000);
      while(r.now<iteration_start+5000) {
        r.now=std::min(r.now+500,iteration_start+5000);sensor();
        assert(core.diagnostics(r.now).fault==LocalFault::None);
      }
    } else r.now+=500;
    assert(core.diagnostics(r.now).fault==LocalFault::None);
  }
  assert(saw_pending&&saw_applied);
  assert(first_ack-start<=15000); // zero reference is read back; nonzero still waits 25 ms
  core.estop();assert(!core.diagnostics(r.now).enabled);
  r.workers();assert(r.motor.disabled());
  for(auto on:r.left.enabled)assert(!on);
  for(auto on:r.right.enabled)assert(!on);
  std::printf("PASS: GatewayCore -> async two-bus backend -> applied ACK; nominal first ACK %llu us\n",
              static_cast<unsigned long long>(first_ack-start));
  if(release_joints) std::puts("PASS: GatewayCore >5 s stream includes 1 s joint release and re-enable without sensor fault");
}
void peer() {
  Rig r; r.left.latency=r.right.latency=0;
  r.now=50000; r.motor.trip(MotorError::Bus); r.motor.resetFault(); r.idle();r.home();
  GatewayCore core(r.motor); HcrParser parser{};
  const auto sensor=[&](){ImuObservation imu{};imu.valid=true;imu.stamp_us=r.now;core.setImu(imu);core.tick(r.now);};
  // The reference peer starts at 100000 us; initialization is before that clock.
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream in(line);char op;std::uint64_t now;in>>op>>now;
    while(r.now<now) {
      r.now=std::min(now,r.now+500);r.workers();sensor();
    }
    // LF RESET preserves home, but cannot clear the local E-stop latch.
    sensor();WirePacket reply{};
    if(op=='F') {
      std::string hex;in>>hex;
      for(std::size_t i=0;i+1<hex.size();i+=2) {
        HcrFrame f{};const auto b=std::stoul(hex.substr(i,2),nullptr,16);
        if(hcr_feed(&parser,static_cast<std::uint8_t>(b),&f))reply=core.receive(f,now);
      }
    } else if(op=='T') {
      reply=core.telemetry(now);
      if(reply.size){auto f=decode(reply);core.telemetrySent(hcr_u32(f.payload),f.seq,f.stamp_us);}
    } else if(op=='C') {
      unsigned code;in>>code;HcrFrame f{};f.type=HCR_STATUS;f.size=16;f.seq=2;f.stamp_us=now;
      hcr_put32(f.payload,7);hcr_put32(f.payload+4,code==1?2:1);reply=core.receive(f,now);
    } else if(op=='E')core.estop();
    if(!reply.size)std::cout<<'-';
    else for(unsigned i=0;i<reply.size;++i){constexpr char h[]="0123456789abcdef";std::cout<<h[reply.bytes[i]>>4]<<h[reply.bytes[i]&15];}
    auto d=core.diagnostics(now);
    std::cout<<' '<<d.armed<<' '<<d.enabled<<' '<<static_cast<unsigned>(d.fault)<<' '<<d.applied_seq<<'\n'<<std::flush;
  }
}
int main(int argc,char** argv) {
  if(argc>1 && !std::strcmp(argv[1],"--peer")){peer();return 0;}

  { // Kneeling releases joints while TARGETs continue at 200 Hz. A worker
    // may have a batch on every step, so background polling is not guaranteed.
    for (unsigned flags : {0U,2U}) {
      Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
      MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);
      t.profile=HCR_RECOVERY;t.flags=flags;
      r.left.latency=r.right.latency=200;
      r.left.frames.clear();r.right.frames.clear();
      for (unsigned tick=0;tick<200;++tick) {
        const auto start=r.now;
        if (!r.motor.feedback(r.now).encoders_valid) {
          const auto state=r.motor.diagnostics();
          std::fprintf(stderr,"released TARGET flags=%u tick=%u hip_age=%llu knee_age=%llu wheel_age=%llu us\n",
            flags,tick,static_cast<unsigned long long>(r.now-state.leg[0].stamp[0]),
            static_cast<unsigned long long>(r.now-state.leg[0].stamp[1]),
            static_cast<unsigned long long>(r.now-state.leg[0].stamp[2]));
        }
        assert(r.motor.feedback(r.now).encoders_valid);
        assert(r.motor.apply(t,++seq,r.now+20000,r.now,a)!=ApplyResult::Fault);
        const auto previous=a.seq;
        for (auto* bus:{&r.left,&r.right}) for(auto& position:bus->position) ++position;
        r.lw.step();
        assert(r.motor.apply(t,seq,r.now+20000,r.now,a)!=ApplyResult::Fault);
        assert(a.seq==previous); // never ACK only the left bus
        r.rw.step();
        assert(r.motor.apply(t,seq,r.now+20000,r.now,a)==ApplyResult::Applied);
        assert(a.seq==seq);
        const auto feedback=r.motor.feedback(r.now);
        for (unsigned side=0;side<2;++side) {
          const auto& bus=side?r.right:r.left;
          const auto state=r.motor.diagnostics().leg[side];
          for(unsigned j=0;j<3;++j) {
            assert(state.stamp[j]>=start);
            assert(feedback.encoder_stamp_us[2*j+side]==state.stamp[j]);
          }
          for(unsigned j=0;j<2;++j) close(state.raw_q[j],bus.position[j]*6.283185307F/16384);
        }
        for (auto* bus:{&r.left,&r.right}) {
          assert(!bus->enabled[0] && !bus->enabled[1]);
          assert(bus->enabled[2]==bool(flags&2U));
        }
        assert(r.now-start<5000);r.now=start+5000;
      }
      for (auto* bus:{&r.left,&r.right}) for (const auto& frame:bus->frames)
        assert(!(frame[2]!=1 && frame[4]==3 && frame[5]==0x1a));
      // Catching can subsequently re-enable the same joints without losing HOME.
      t.flags=3;r.run(t,seq);
      assert(r.motor.feedback(r.now).homed);
      for (auto* bus:{&r.left,&r.right}) for (bool on:bus->enabled) assert(on);
    }
    std::puts("PASS: continuous 200 Hz released-joint batches retain all six real samples and two-bus ACK");
  }

  { // The released-joint PING is real I/O: failure/cancellation/expiry cannot
    // fabricate feedback or complete a batch, even with torque already OFF.
    for (unsigned side:{0U,1U}) for (unsigned id:{2U,3U})
      for (unsigned failure=0;failure<5;++failure) {
        Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
        t.flags=0;r.run(t,seq);MotorApplied a{};
        r.motor.apply(t,seq,r.now+20000,r.now,a);const auto previous=a.seq;
        auto& bus=side?r.right:r.left;auto& worker=side?r.rw:r.lw;
        const auto stamp=r.motor.diagnostics().leg[side].stamp[id-2];
        if(failure==0 || failure==1) {
          bus.fail_id=id;bus.fail_inst=1;
          bus.injected_error=failure==0?BusError::ReplyTimeout:BusError::ReplyInvalid;
        } else if(failure==2) bus.status=0x20;
        else if(failure==3) bus.hook=[&](){r.motor.disable();};
        else bus.latency=21000;
        r.motor.apply(t,++seq,r.now+20000,r.now,a);worker.step();
        const auto d=r.motor.diagnostics();
        assert(d.leg[side].stamp[id-2]==stamp);
        assert(!d.in_flight && a.seq==previous);
        if(failure!=3) assert(d.error!=MotorError::None);
        for (auto on:bus.enabled) assert(!on);
      }
    std::puts("PASS: released feedback rejects missing/corrupt/status replies, cancellation and expired I/O");
  }

  { // Supplier's corrected golden MIT packet, independent of the mock.
    std::array<std::uint8_t,48> out{};
    const std::uint8_t p[]={0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0};
    const std::uint8_t expected[]={255,255,1,13,3,0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0,0x69};
    assert(motorPacket(1,3,p,sizeof p,out)==sizeof expected&&!std::memcmp(out.data(),expected,sizeof expected));
    const std::uint8_t reply[]={255,255,1,7,0,0,0x10,13,3,0x1a,0xbd};BusReply result{};
    assert(motorReply(BusFormat::Joint,1,3,0x1a,13,reply,sizeof reply,0,result));
    assert(result.position==4096);
    assert(!motorReply(BusFormat::Wheel,1,3,0x1a,13,reply,sizeof reply,0,result));
    assert(!motorPacket(0xfe,3,p,sizeof p,out));
  }
  { // HCR HOME uses the actual backend's software offset, with zero driver writes.
    Rig r;GatewayCore core(r.motor);
    HcrFrame hello{};hello.type=HCR_HELLO;hello.size=12;hello.seq=1;hello.stamp_us=1;
    hcr_put32(hello.payload,7);hcr_put16(hello.payload+4,200);hcr_put16(hello.payload+6,100);hcr_put32(hello.payload+8,HCR_SCHEMA);
    core.receive(hello,r.now);core.tick(r.now);
    auto telemetry=decode(core.telemetry(r.now));core.telemetrySent(7,telemetry.seq,telemetry.stamp_us);
    HcrFrame home{};home.type=HCR_ENCODER_HOME;home.size=8;home.seq=2;home.stamp_us=2;
    hcr_put32(home.payload,7);hcr_put32(home.payload+4,telemetry.seq);
    const auto left=r.left.frames.size(),right=r.right.frames.size();
    auto result=decode(core.receive(home,r.now));
    assert(hcr_u32(result.payload+8)==HCR_HOME_APPLIED && r.motor.ready());
    assert(left==r.left.frames.size() && right==r.right.frames.size());
    assert(r.motor.disabled());auto f=r.motor.feedback(r.now);
    close(f.q[0],2.35706715F);close(f.q[1],2.35706715F);close(f.q[2],0);close(f.q[3],0);
    // Moving any joint must reject capture, even if motor enable is OFF.
    r.left.position[0]+=100;r.now+=4000;r.workers();core.tick(r.now);
    telemetry=decode(core.telemetry(r.now));core.telemetrySent(7,telemetry.seq,telemetry.stamp_us);
    home.seq=3;home.stamp_us=3;hcr_put32(home.payload+4,telemetry.seq);
    result=decode(core.receive(home,r.now));assert(hcr_u32(result.payload+8)==HCR_HOME_REFUSED);
    assert(r.motor.disabled());
    // Once manually repositioned and stationary, recapture the new reference
    // without rebooting, writing a driver register or enabling any motor.
    r.now+=4000;r.workers();core.tick(r.now);
    f=r.motor.feedback(r.now);assert(std::abs(f.q[0]-2.35706715F)>.01F);
    telemetry=decode(core.telemetry(r.now));core.telemetrySent(7,telemetry.seq,telemetry.stamp_us);
    home.seq=4;home.stamp_us=4;hcr_put32(home.payload+4,telemetry.seq);
    const auto left_after=r.left.frames.size(),right_after=r.right.frames.size();
    result=decode(core.receive(home,r.now));assert(hcr_u32(result.payload+8)==HCR_HOME_APPLIED);
    assert(left_after==r.left.frames.size() && right_after==r.right.frames.size());
    f=r.motor.feedback(r.now);close(f.q[0],2.35706715F);
    assert(r.motor.disabled() && !core.diagnostics(r.now).armed);
  }
  { // Wheel write goes first only once a completed MIT batch holds the legs.
    const auto first_mit=[](const FakeBus& bus,std::size_t from) {
      for(std::size_t i=from;i<bus.frames.size();++i){const auto& f=bus.frames[i];if(f[4]==3&&f[5]==0x1a)return i;}
      return bus.frames.size();};
    const auto first_wheel=[](const FakeBus& bus,std::size_t from) {
      for(std::size_t i=from;i<bus.frames.size();++i){const auto& f=bus.frames[i];
        if(f[2]==1&&f[4]==3&&f[5]==0x20&&signed16(f,6)!=0)return i;}
      return bus.frames.size();};
    { // legs engaged by a prior zero-wheel batch: wheel first, on both buses
      Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
      t.profile=HCR_POLICY_PD;t.target[4]=10;t.target[5]=-23;
      const auto lfrom=r.left.frames.size(),rfrom=r.right.frames.size();
      ++seq;r.run(t,seq);
      const auto lw=first_wheel(r.left,lfrom),lm=first_mit(r.left,lfrom);
      const auto rw=first_wheel(r.right,rfrom),rm=first_mit(r.right,rfrom);
      assert(lw<r.left.frames.size() && lm<r.left.frames.size() && lw<lm);
      assert(rw<r.right.frames.size() && rm<r.right.frames.size() && rw<rm);
      assert(signed16(r.left.last(1,0x20),6)==95 && signed16(r.right.last(1,0x20),6)==220);
    }
    { // first batch of a session keeps legs before wheels
      Rig r;r.home();auto t=r.target(HCR_POLICY_PD);t.target[4]=10;t.target[5]=-23;
      const auto lfrom=r.left.frames.size(),rfrom=r.right.frames.size();
      unsigned seq=1;r.run(t,seq);
      const auto lw=first_wheel(r.left,lfrom),lm=first_mit(r.left,lfrom);
      const auto rw=first_wheel(r.right,rfrom),rm=first_mit(r.right,rfrom);
      assert(lm<r.left.frames.size() && rm<r.right.frames.size());
      assert(lw>lm && rw>rm);
    }
    std::puts("PASS: wheel command precedes joint blocks only after the legs are held");
  }
  { Rig r;assert(r.motor.disabled()&&!r.motor.ready());r.home();
    auto f=r.motor.feedback(r.now);close(f.q[0],2.35706715F);close(f.q[2],0);assert(!f.torque_valid);
    auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.profile=HCR_POLICY_PD;t.target[0]=-.5;t.target[1]=-.4;t.target[2]=.6;t.target[3]=.7;
    t.target[4]=10;t.target[5]=-23;++seq;const auto out=r.run(t,seq);
    // Lumi physical LEFT UART2: hip +, knee +, wheel +. RIGHT UART1: -, -, -.
    const auto& lh=r.left.last(2,0x1a);const auto& lc=r.left.last(3,0x1a);
    const auto& rh=r.right.last(2,0x1a);const auto& rc=r.right.last(3,0x1a);
    assert(signed16(lh,10)<1000 && signed16(lc,10)>-2000);
    assert(signed16(rh,10)>1000 && signed16(rc,10)<-2000);
    assert(signed16(r.left.last(1,0x20),6)==95);
    assert(signed16(r.right.last(1,0x20),6)==220);
    close(out.target[0],-.5);close(out.target[2],.6);close(out.target[4],9.948377F);
    close(out.target[5],-23.038347F);
    assert(signed16(lh,14)==0 && signed16(lc,14)==0); // tau bytes, not input torque
    f=r.motor.feedback(r.now);close(f.q[2],1.1);close(f.q[3],1.1);
    t.profile=HCR_RECOVERY;t.feedforward[0]=.1;t.feedforward[1]=-.2;
    t.feedforward[2]=.3;t.feedforward[3]=-.4;++seq;r.run(t,seq);
    assert(signed16(r.left.last(2,0x1a),12)==1);
    assert(signed16(r.left.last(3,0x1a),12)==3); // direct knee FF=.3, sign +
    assert(signed16(r.right.last(2,0x1a),12)==2);
    assert(signed16(r.right.last(3,0x1a),12)==4); // direct knee FF=-.4, sign -
    r.motor.disable();assert(!r.motor.disabled());r.workers();assert(r.motor.disabled());
    assert(r.motor.resetFault() && r.motor.ready()); // ordinary LF RESET preserves home
  }
  { // Read timestamps, not wheel write ACKs, establish freshness; unwrap 1020->5.
    Rig r;r.home();auto old=r.motor.feedback(r.now);r.left.wheel_pos=5;r.now+=4000;r.workers();
    auto f=r.motor.feedback(r.now);close(f.q[4]-old.q[4],9*2*3.141592653589793F/1024);
    r.now+=21000;assert(!r.motor.feedback(r.now).encoders_valid);
    assert(!r.motor.captureHome({0,0,0,0},r.now));
  }
  { // An idle scheduling gap is distinguished from a failed bus transaction.
    Rig r; r.now+=21000; r.workers();
    const auto d=r.motor.diagnostics();
    assert(d.error==MotorError::Feedback);
    assert(d.leg[0].feedback_error==FeedbackError::WheelGap);
    assert(d.leg[0].feedback_id==1 && d.leg[0].feedback_gap_us>20000);
    assert(!d.leg[0].feedback_active && d.leg[0].bus_error==BusError::None);
    assert(d.leg[0].feedback_previous==d.leg[0].feedback_received);
    assert(r.motor.resetFault()); r.idle();
    assert(r.motor.diagnostics().leg[0].feedback_error==FeedbackError::None);
  }
  { // A pending right bus cannot be acknowledged as an applied six-channel batch.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto old=a.seq;r.lw.step();
    r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==old);
    r.rw.step();assert(r.motor.apply(t,seq,r.now+20000,r.now,a)==ApplyResult::Applied&&a.seq==seq);
  }
  { // Latest queued TARGET wins only before either bus claims it.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a); // consume final completion
    const auto applied=a.seq;
    ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto old=r.motor.request(0).batch;
    t.target[0]-=.01F;t.target[2]-=.01F;++seq;
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto fresh=r.motor.request(0).batch;
    assert(fresh.token!=old.token && fresh.seq==seq && a.seq==applied);
    assert(r.motor.claimBatch(0,old,r.now)==BatchClaim::Superseded);
    assert(r.motor.executionStats(r.now).superseded==1);
    r.lw.step();r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==applied);
    r.rw.step();r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==seq);
    assert(r.motor.executionStats(r.now).io_max_us[0]>0);
  }
  { // A worker snapshot superseded during wheel OFF cannot transmit old MIT.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    t.flags=1;++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto old=r.motor.request(0).batch; r.left.frames.clear();
    r.left.hook=[&](){
      t.target[0]-=.01F;t.target[2]-=.01F;++seq;
      r.motor.apply(t,seq,r.now+20000,r.now,a);
    };
    r.lw.step();assert(r.motor.request(0).batch.token!=old.token);
    for(const auto& frame:r.left.frames) assert(!(frame[4]==3 && frame[5]==0x1a));
    assert(r.motor.diagnostics().error==MotorError::None);
    r.workers();r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==seq);
  }
  { // Replacing queued zero HOME must not bypass the wheel settle condition.
    Rig r;r.home();auto t=r.target();MotorApplied a{};
    r.motor.apply(t,1,r.now+20000,r.now,a);r.workers();
    r.motor.apply(t,2,r.now+20000,r.now,a);
    const auto zero=r.motor.request(0).batch;
    assert(zero.token && !r.motor.diagnostics().leg[0].wheel_ready);
    t.target[4]=t.target[5]=3;
    r.motor.apply(t,3,r.now+20000,r.now,a);
    assert(r.motor.request(0).batch.token==zero.token);
    assert(r.motor.request(0).batch.wheel_rpm==0);
    assert(r.motor.executionStats(r.now).superseded==0);
    r.workers();r.motor.apply(t,3,r.now+20000,r.now,a);
    assert(a.seq==2 && a.target[4]==0 && a.target[5]==0);
  }
  { // One bus claiming freezes reference, token and original deadline for BOTH.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto old=r.motor.request(0).batch;
    r.lw.step();assert(r.motor.executionStats(r.now).batch_flags==5);
    t.target[0]-=.01F;t.target[2]-=.01F;++seq;
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    assert(r.motor.request(1).batch.token==old.token);
    assert(r.motor.request(1).batch.deadline_us==old.deadline_us);
    assert(r.motor.request(0).batch.joint==old.joint);
    // New valid targets cannot rescue an already-started expired batch.
    r.now=old.deadline_us;
    r.rw.step();assert(r.motor.diagnostics().error==MotorError::Expired);
    assert(r.motor.diagnostics().leg[1].fault_context.expired_batch);
    assert(!r.motor.diagnostics().leg[1].fault_context.expired_lease);
    assert(r.motor.executionStats(r.now).batch_flags & (1U<<11));
    r.lw.step();assert(r.motor.disabled());
  }
  { // Duplicate submit cannot renew the lease; disable cancels stale claims.
    Rig r;r.home();auto t=r.target();MotorApplied a{};unsigned seq=1;r.run(t,seq);
    ++seq;const auto deadline=r.now+20000;
    r.motor.apply(t,seq,deadline,r.now,a);const auto batch=r.motor.request(0).batch;
    r.motor.apply(t,seq,deadline+10000,r.now,a);
    assert(r.motor.request(0).lease_us==deadline);
    r.motor.disable();assert(r.motor.claimBatch(0,batch,r.now)==BatchClaim::Cancelled);
    r.workers();assert(r.motor.disabled());
  }
  { // During startup/poll, newer TARGET renews lease BETWEEN bus transactions.
    Rig r;r.home();auto t=r.target();t.flags=1;MotorApplied a{};
    r.now+=4000; // make feedback due for polling
    const auto before=r.now;r.motor.apply(t,1,before+400,before,a);
    r.left.latency=300;
    r.left.hook=[&](){r.motor.apply(t,2,r.now+20000,r.now,a);};
    r.lw.step();
    assert(r.now>before+400 && r.motor.diagnostics().error==MotorError::None);
    assert(r.motor.diagnostics().leg[0].active_io_max_us==300);
  }
  { // A newer TARGET during one I/O cannot extend that I/O's fixed deadline.
    Rig r;r.home();auto t=r.target();t.flags=1;MotorApplied a{};
    r.now+=4000; // make feedback due for polling
    r.motor.apply(t,1,r.now+400,r.now,a);r.left.latency=600;
    r.left.hook=[&](){r.motor.apply(t,2,r.now+20000,r.now,a);};
    r.lw.step();assert(r.motor.diagnostics().error==MotorError::Expired);
    assert(r.motor.diagnostics().leg[0].fault_context.during_io);
    assert(r.motor.diagnostics().leg[0].fault_context.expired_lease);
    r.rw.step();assert(r.motor.disabled());
  }
  { // Service may run while the worker waits for the NEXT motor's reply.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto previous_ack=a.seq;
    auto states=r.motor.diagnostics().leg;
    for(auto& state:states) state.stamp.fill(r.now);
    states[0].stamp[0]=r.now-19000;
    for(unsigned side=0;side<2;++side) r.motor.publish(side,states[side]);
    ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto token=r.motor.request(0).batch.token;
    r.left.latency=700;bool checked=false;
    r.left.hook=[&](){r.left.hook=[&](){
      // Hip was received 700 us ago; the old shared hip stamp is now 20.4 ms old.
      assert(r.motor.feedback(r.now).encoders_valid);
      assert(r.motor.diagnostics().leg[0].stamp[0]==r.now-700);
      assert(r.motor.diagnostics().leg[0].completed!=token);
      r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==previous_ack);
      checked=true;
    };};
    r.lw.step();assert(checked);
    r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==previous_ack);
    r.rw.step();r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==seq);
  }
  { // One fresh sample cannot hide a genuinely stale sample on the other bus.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a);
    ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    auto state=r.motor.diagnostics().leg[1];state.stamp[1]=r.now-20001;
    r.motor.publish(1,state);
    bool checked=false;
    r.left.hook=[&](){r.left.hook=[&](){
      assert(!r.motor.feedback(r.now).encoders_valid);checked=true;
    };};
    r.lw.step();assert(checked);
  }
  { // Corrupt or cancelled replies cannot refresh a sensor timestamp.
    for(bool cancel:{false,true}) {
      Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
      r.motor.apply(t,seq,r.now+20000,r.now,a);
      ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
      const auto stamp=r.motor.diagnostics().leg[0].stamp[0];
      if(cancel) r.left.hook=[&](){r.motor.disable();};
      else r.left.corrupt=true;
      r.lw.step();assert(r.motor.diagnostics().leg[0].stamp[0]==stamp);
      assert(!r.motor.diagnostics().in_flight);
    }
  }
  { // Late sensor publication from before RESET cannot repopulate the backend.
    Rig r;const auto old=r.motor.diagnostics().leg[0];
    r.motor.trip(MotorError::Bus);assert(r.motor.resetFault());
    r.motor.publishSample(0,0,old);
    assert(r.motor.diagnostics().leg[0].stamp[0]==0);
    assert(!r.motor.feedback(r.now).encoders_valid);
  }
  { // Background feedback must yield between I/Os when a TARGET is queued.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a); // no pending batch
    r.now+=4000;r.left.frames.clear();
    r.left.hook=[&](){
      ++seq;r.motor.apply(t,seq,r.now+20000,r.now,a);
    };
    r.lw.step();
    assert(r.left.frames.size()==1); // finish current ping; defer rest of poll
    assert(r.motor.diagnostics().error==MotorError::None);
    r.workers();r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==seq);
  }
  { // Do not start a redundant poll immediately after an applied batch.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);MotorApplied a{};
    r.motor.apply(t,seq,r.now+20000,r.now,a);r.left.frames.clear();
    r.lw.step();assert(r.left.frames.empty());
    // Idle feedback must still be polled and remain subject to driver faults.
    r.now+=4000;r.left.status=0x20;r.lw.step();
    assert(!r.left.frames.empty() && r.motor.diagnostics().error==MotorError::Status);
  }
  { // No clamp: impossible target must fault, disable and never ACK the new seq.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.target[4]=100000;++seq;MotorApplied a{};
    assert(r.motor.apply(t,seq,r.now+20000,r.now,a)==ApplyResult::Fault);
    r.workers();assert(r.motor.disabled());
  }
  { // Worker lease is independent of service execution and repeated submit calls.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.now+=21000;r.workers();assert(r.motor.diagnostics().error==MotorError::Expired);
    assert(r.motor.disabled());for(auto on:r.left.enabled)assert(!on);
  }
  { // Lost speed-write reply is accepted ONLY after matching target readback.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;
    t.target[4]=3;++seq;
    const auto a=r.run(t,seq);close(a.target[4],3.036873F);
    assert(r.motor.diagnostics().error==MotorError::None);
    assert(r.motor.diagnostics().leg[0].recovered_writes>0);
  }
  { // A dropped write must not be ACKed merely because present-state reads work.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.drop_speed_write=true;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Configuration && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].bus_error==BusError::ReadbackMismatch);
    for(auto on:r.left.enabled)assert(!on);
  for(auto on:r.right.enabled)assert(!on);
    r.left.fail_id=0;assert(r.motor.resetFault());r.idle();assert(!r.motor.ready());r.home();
  }
  { // Boot/disarmed must not write speed, including repeated OFF verification.
    Rig r;for(unsigned i=0;i<200;++i)r.idle(); // >1 s, includes periodic verification
    assert(r.motor.diagnostics().error==MotorError::None && r.motor.feedback(r.now).encoders_valid);
    for (auto* bus:{&r.left,&r.right}) for (const auto& f:bus->frames)
      assert(!(f[2]==1 && f[4]==3 && f[5]==0x20));
    r.left.enabled[2]=true; r.motor.emergencyStop();
    assert(!r.motor.disabled()); r.workers();
    assert(r.motor.disabled() && !r.left.enabled[2]);
    r.right.enabled[0]=true;r.now+=100000;r.workers();
    assert(!r.right.enabled[0]);
  }
  { // An ACKed OFF which the driver did not apply cannot claim disabled.
    Rig r;r.left.enabled[2]=true;r.left.ignore_off=true;r.motor.emergencyStop();r.workers();
    assert(!r.motor.disabled() && r.left.enabled[2]);
    const auto d=r.motor.diagnostics();
    assert(d.leg[0].failed_id==1 && d.leg[0].failed_address==0x18);
    assert(d.leg[0].torque_enable[2]==1 && d.leg[0].bus_error==BusError::ReadbackMismatch);
    r.left.ignore_off=false;r.now+=2000;r.workers();assert(r.motor.disabled());
  }
  { // Boot verifies speed-target readback without writing or executing a retained goal.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.wheel_rpm=321;BlMotorWorker worker(motor,0,bus);worker.step();
    const auto d=motor.diagnostics();
    assert(d.leg[0].configured && d.leg[0].startup_wheel_goal_rpm==321);
    assert(!bus.enabled[2] && bus.wheel_rpm==321);
    for(const auto& f:bus.frames)assert(!(f[2]==1 && f[4]==3 && f[5]==0x20));
  }
  { // An unsupported goal READ is found before ARM, not first encountered in motion.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.fail_id=1;bus.fail_addr=0x20;bus.fail_inst=2;
    BlMotorWorker worker(motor,0,bus);worker.step();
    const auto d=motor.diagnostics();
    assert(!d.leg[0].configured && d.error==MotorError::Bus && !bus.enabled[2]);
    assert(d.leg[0].failed_id==1 && d.leg[0].failed_address==0x20);
  }
  { // Maintenance verifies OFF once, then keeps bus quiet during flash writes.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.motor.beginMaintenance();assert(!r.motor.disabled());r.workers();
    assert(r.motor.disabled() && !r.motor.ready() && !r.motor.resetFault());
    const auto left_frames=r.left.frames.size(),right_frames=r.right.frames.size();
    for(unsigned i=0;i<200;++i)r.idle();
    assert(r.left.frames.size()==left_frames && r.right.frames.size()==right_frames);
    MotorApplied a{};assert(r.motor.apply(t,++seq,r.now+20000,r.now,a)==ApplyResult::Fault);
    r.workers();assert(r.motor.disabled());
    assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));
  }
  { // Maintenance cannot freeze a false OFF state when readback failed.
    Rig r;r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=2;
    r.motor.beginMaintenance();r.workers();assert(!r.motor.disabled());
    r.left.fail_id=0;r.now+=2000;r.workers();assert(r.motor.disabled());
  }
  { // Unknown OFF readback stays unknown even if the OFF write was ACKed.
    Rig r;r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=2;
    r.motor.emergencyStop();r.workers();
    assert(!r.motor.disabled() && r.motor.diagnostics().leg[0].torque_enable[2]==255);
  }
  for (auto error:{BusError::EchoTimeout,BusError::EchoMismatch,BusError::ReplyInvalid,BusError::Tx}) {
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.injected_error=error;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].bus_error==error);
  }
  { // Missing speed ACK AND missing readback are a bus fault, never an ACK.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20; // both instructions fail
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].failed_instruction==2);
  }
  { // Readback recovery cannot extend the original 20 ms batch deadline.
    // The wheel write precedes the joint blocks once the legs are held, so the
    // wheel I/O alone must exceed the budget for the deadline to land on it.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.latency=12000;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Expired && r.motor.disabled());
    const auto fault=r.motor.diagnostics().leg[0].fault_context;
    assert(fault.during_io && fault.bus_error==BusError::Deadline && fault.id==1);
    assert(fault.instruction==2 && fault.address==0x20 && fault.batch_seq!=0);
    assert(fault.at_us>=fault.batch_deadline_us);
    r.now+=100000;r.workers();
    assert(r.motor.diagnostics().leg[0].fault_context.at_us==fault.at_us); // OFF cannot replace root cause
  }
  { // Missing wheel enable ACK is accepted only with matching register readback.
    Rig r;r.home();auto t=r.target();MotorApplied a{};
    r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=3;
    r.motor.apply(t,1,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::None && r.left.enabled[2]);
    r.motor.disable();r.workers();assert(r.motor.disabled());
  }
  { // Stop during a MIT transaction: do not send the next joint, turn off all.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);
    r.left.hook=[&](){r.motor.disable();
      assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));};r.workers();assert(r.motor.disabled());
    for(auto on:r.left.enabled)assert(!on);
  }
  { // Driver's gain limits are checked, never silently clamped to another servo.
    Rig r;r.left.gain_cap=36;r.motor.trip(MotorError::Bus);r.motor.resetFault();r.workers();
    assert(r.motor.diagnostics().error==MotorError::Configuration&&!r.motor.ready());
  }
  { Rig r;r.home();r.left.corrupt=true;r.now+=4000;r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus&&!r.motor.feedback(r.now).encoders_valid);
  }
  { // Nonzero startup must wait for wheel settle, even though zero HOME can ACK early.
    Rig r;r.home();auto t=r.target();t.target[4]=1;t.target[5]=-1;
    unsigned seq=1;MotorApplied a{};const auto start=r.now;
    while(r.now-start<23000) {
      const auto result=r.motor.apply(t,seq++,r.now+20000,r.now,a);
      assert(result==ApplyResult::Pending);r.workers();r.now+=500;
    }
    r.run(t,seq);assert(r.left.wheel_rpm==10 && r.right.wheel_rpm==10);
  }
  { // Individual enable bits: disabled wheels report zero, not a decaying filter;
    // disabled legs retain their last latched reference rather than claiming a write.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.profile=HCR_RECOVERY;t.flags=1;t.target[4]=100000;++seq;
    auto a=r.run(t,seq);close(a.target[4],0);assert(!r.left.enabled[2]);
    t.flags=2;t.target[0]=999;t.target[4]=0;++seq;
    a=r.run(t,seq);assert(!r.left.enabled[0]&&!r.left.enabled[1]&&r.left.enabled[2]);
    close(a.target[0],2.35706715F);
  }
  { Rig r;r.home();r.left.status=0x20;r.now+=4000;r.workers();
    assert(r.motor.diagnostics().error==MotorError::Status);assert(r.motor.disabled());
  }
  { // Independent numerical recovery filter: first dt=.005, tau=.075,
    // omega=10 -> .64493 rad/s -> 6 RPM -> applied .6283185 rad/s.
    Lock lock;BlMotorBackend motor(lock);MotorLegState st{};
    st.configured=st.off=st.prepared=st.wheel_ready=true;st.generation=1;
    st.stamp={100000,100000,100000};st.kp_max={65535,65535};st.kd_max={5000,5000};
    motor.publish(0,st);motor.publish(1,st);
    assert(motor.captureHome({2.35706715F,2.35706715F,0,0},100000));
    HcrTarget t{};t.epoch=7;t.profile=HCR_RECOVERY;t.flags=3;
    for(unsigned i=0;i<4;++i)t.target[i]=2.35706715F;
    t.target[4]=10;t.target[5]=20;MotorApplied a{};
    assert(motor.apply(t,1,120000,100000,a)==ApplyResult::Pending);
    assert(motor.request(0).batch.wheel_rpm==6 && motor.request(1).batch.wheel_rpm==-12);
    st.off=false;st.completed=motor.request(0).batch.token;
    motor.publish(0,st);motor.publish(1,st);
    assert(motor.apply(t,1,120000,100500,a)==ApplyResult::Applied);
    close(a.target[4],.6283185F);close(a.target[5],1.256637F);
  }
  { BlMotorConfig c;assert(validMotorConfig(c));c.sign[0][0]=0;assert(!validMotorConfig(c));
    c={};c.id[0]=c.id[1];assert(!validMotorConfig(c));
    c={};c.wheel_max_rpm=1600;assert(!validMotorConfig(c));
    c={};c.profile[0].kd=std::numeric_limits<float>::quiet_NaN();assert(!validMotorConfig(c));
  }
  { // Gains from Lumi fit measured hip limits, while knee gains are independent.
    Rig r;r.left.gain_cap=r.right.gain_cap=120;
    r.motor.trip(MotorError::Bus);r.motor.resetFault();r.idle();r.home();
    auto t=r.target();unsigned seq=1;r.run(t,seq);
    assert(signed16(r.left.last(2,0x1a),8)==36);
    assert(signed16(r.right.last(3,0x1a),8)==50);
  }
  { // OFF stress test writes only wheel zero; it cannot enable or retain HOME.
    Rig r;r.home();assert(!r.motor.startOffTest(0));assert(r.motor.startOffTest(20));
    assert(!r.motor.ready());assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));
    r.left.frames.clear();r.right.frames.clear();
    for(unsigned i=0;i<25;++i){r.workers();r.now+=1000;}
    const auto diag=r.motor.diagnostics();
    assert(diag.leg[0].off_test_done==20 && diag.leg[1].off_test_done==20);
    for(auto* bus:{&r.left,&r.right}) {
      for(bool on:bus->enabled)assert(!on);
      for(const auto& f:bus->frames)if(f[4]==3)
        assert((f[5]==0x18 && f[6]==0) || (f[2]==1 && f[5]==0x20 && f[6]==0 && f[7]==0));
    }
    assert(r.motor.disabled()&&!r.motor.ready());
    r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);assert(!r.motor.startOffTest(20));
  }
  { // A single lost MIT reply is recovered without changing the absolute target.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;
    r.left.fail_remaining=1;r.left.injected_error=BusError::EchoMismatch;
    r.left.frames.clear();++seq;r.run(t,seq);
    const auto d=r.motor.diagnostics();
    assert(d.error==MotorError::None && d.leg[0].retries==1 && d.leg[0].recovered_transactions==1);
    assert(d.leg[0].failed_id==2 && d.leg[0].bus_error==BusError::EchoMismatch);
    assert(r.left.frames.size()>1 && r.left.frames[0]==r.left.frames[1]);
  }
  { // A transient wheel feedback miss is retried, with fresh feedback on success.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x24;r.left.fail_inst=2;r.left.fail_remaining=1;
    ++seq;r.run(t,seq);
    assert(r.motor.diagnostics().leg[0].recovered_transactions==1);
    assert(r.motor.feedback(r.now).encoders_valid);
  }
  { // Cancellation during the failed exchange must suppress the retry, but is
    // a normal DISARM, not a latched driver fault requiring re-homing.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;r.left.fail_remaining=1;
    r.left.hook=[&](){r.motor.disable();};r.left.frames.clear();
    MotorApplied a{};r.motor.apply(t,++seq,r.now+20000,r.now,a);r.workers();
    unsigned mit=0;for(const auto& f:r.left.frames)if(f[4]==3 && f[5]==0x1a)++mit;
    assert(mit==1 && r.motor.disabled() && r.motor.diagnostics().error==MotorError::None);
  }
  { // No retry or next joint TX after the first transaction used the lease.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;r.left.fail_remaining=1;
    r.left.latency=21000;r.left.frames.clear();MotorApplied a{};
    r.motor.apply(t,++seq,r.now+20000,r.now,a);r.workers();
    unsigned mit=0;for(const auto& f:r.left.frames)if(f[4]==3 && f[5]==0x1a)++mit;
    assert(mit==1 && r.motor.disabled() && r.motor.diagnostics().error==MotorError::Expired);
  }
  { // Stable RPM avoids repeated writes but never decimates encoder reads.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.frames.clear();r.run(t,seq,400);
    unsigned writes=0,reads=0;
    for(const auto& f:r.left.frames){
      if(f[2]==1 && f[4]==3 && f[5]==0x20)++writes;
      if(f[2]==1 && f[4]==2 && f[5]==0x24)++reads;
    }
    assert(writes>0 && reads>10*writes && r.motor.diagnostics().leg[0].skipped_writes>10);
    assert(r.motor.feedback(r.now).encoders_valid);
    // A changed RPM bypasses the keepalive interval and is actually sent/ACKed.
    t.target[4]=3;++seq;auto a=r.run(t,seq);close(a.target[4],3.036873F);
    assert(signed16(r.left.last(1,0x20),6)==29);
    t.target[4]=4;++seq;a=r.run(t,seq);assert(signed16(r.left.last(1,0x20),6)==38);
    // A new enabled session must preload/verify zero again.
    r.motor.disable();r.workers();r.left.frames.clear();t.target[4]=0;++seq;r.run(t,seq);
    bool zero=false;for(const auto& f:r.left.frames)
      if(f[2]==1 && f[4]==3 && f[5]==0x20 && signed16(f,6)==0)zero=true;
    assert(zero);
  }
  { // Direct-knee encoder must not move in model feedback when only hip moves.
    Rig r;r.home();const auto initial=r.motor.feedback(r.now);
    r.left.position[0]+=100;r.right.position[0]-=100;r.now+=4000;r.workers();
    const auto f=r.motor.feedback(r.now);close(f.q[2],initial.q[2]);close(f.q[3],initial.q[3]);
    assert(f.qd[0]>0 && f.qd[1]>0);close(f.qd[2],0);close(f.qd[3],0);
    auto t=r.target();t.flags=2;MotorApplied a{};unsigned seq=1;a=r.run(t,seq);
    close(a.target[2],f.q[0]+f.q[2]);close(a.target[3],f.q[1]+f.q[3]);
  }
  { // Legacy physical crank remains an explicit adapter option; HCR1 is identical.
    BlMotorConfig c;c.knee_crank_coupling=1;c.sign={{{{1,-1,1}},{{-1,1,-1}}}};
    Rig r(c);r.home();auto t=r.target();t.profile=HCR_RECOVERY;t.target[0]=t.target[1]=-.5F;
    t.target[2]=t.target[3]=.6F;t.feedforward[0]=.1F;t.feedforward[2]=.3F;
    unsigned seq=1;auto a=r.run(t,seq);
    close(a.target[2],.6F);close(r.motor.feedback(r.now).q[2],1.1F);
    assert(signed16(r.left.last(3,0x1a),12)==-4);
  }
  { // Correct a joint response level while OFF, verify level2 and MIT mode3.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.response_level={1,1,1};BlMotorWorker worker(motor,0,bus);worker.step();
    assert(motor.diagnostics().leg[0].configured);
    assert(bus.response_level[0]==2 && bus.response_level[1]==2);
    assert(bus.mode[0]==3 && bus.mode[1]==3 && !bus.wheel_write_replies);
    for(auto on:bus.enabled)assert(!on);
  }
  { // An ACK without a latched configuration is rejected during disarmed setup.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.ignore_config=true;BlMotorWorker worker(motor,0,bus);worker.step();
    assert(motor.diagnostics().error==MotorError::Configuration);
    assert(motor.diagnostics().leg[0].bus_error==BusError::ReadbackMismatch);
  }
  for (bool drop:{false,true}) { // Wheel no-write-reply mode must verify speed, not merely echo.
    Rig r;r.left.response_level[2]=1;r.motor.trip(MotorError::Bus);assert(r.motor.resetFault());
    r.idle();r.home();assert(!r.left.wheel_write_replies);
    auto t=r.target();unsigned seq=1;r.run(t,seq);r.left.drop_speed_write=drop;t.target[4]=3;
    if (!drop) {
      auto a=r.run(t,++seq);close(a.target[4],3.036873F);
      assert(r.motor.diagnostics().error==MotorError::None);
    } else {
      MotorApplied a{};r.motor.apply(t,++seq,r.now+20000,r.now,a);r.workers();
      assert(r.motor.diagnostics().error==MotorError::Configuration && r.motor.disabled());
    }
  }
  { // An ignored wheel enable cannot be mistaken for applied torque enable.
    Rig r;r.home();r.left.ignore_enable=true;auto t=r.target();MotorApplied a{};
    r.motor.apply(t,1,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Configuration && r.motor.disabled());
  }
  { BlMotorConfig c;c.knee_crank_coupling=.5F;assert(!validMotorConfig(c));
    c={};c.knee_profile[1].kd=std::numeric_limits<float>::quiet_NaN();assert(!validMotorConfig(c));
  }
  std::puts("PASS: Lumi mapping/gains, direct-knee and crank adapters, verified configuration/enable, no-write-reply wheel");
  std::puts("PASS: bounded retry, first failure retained, cancellation/expiry and wheel dedup/keepalive/freshness");
  gatewayIntegration();
  gatewayIntegration(true);
  std::puts("PASS: golden codec, calibrated side/sign/units, physical joint FF, quantized applied targets, startup, both-bus ACK, unwrap/freshness, range, expiry, cancellation, write/enable loss, reset, gain/status/CRC faults");
}
