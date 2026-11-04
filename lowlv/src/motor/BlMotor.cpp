#include "gateway/motor/BlMotor.hpp"
#include "gateway/motor/hal/MotorFrameCodec.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace gateway {
namespace frame = hal::motor_frame;
namespace {
constexpr float pi = frame::kTwoPi/2;
constexpr float count_rad = frame::countsToRad(1.F), rpm_rad = frame::kTwoPi/60.F;
std::uint16_t u16(const std::uint8_t* p) { return p[0] | (std::uint16_t(p[1])<<8); }
void put(std::uint8_t* p, std::int32_t v) { p[0]=v&255; p[1]=(v>>8)&255; }
bool fresh(std::uint64_t stamp, std::uint64_t now, std::uint64_t age) {
  return stamp && now>=stamp && now-stamp<=age;
}
bool quantize(float v, std::int32_t lo, std::int32_t hi, std::int32_t& out) {
  if (!std::isfinite(v) || v<lo || v>hi) return false;
  out=static_cast<std::int32_t>(std::lround(v)); return out>=lo && out<=hi;
}
// Diagnostics only, after a successful reply: echo end to last reply byte, from the bus trace.
void recordReplyLatency(MotorLegState& s, const BlMotorConfig& c, std::uint8_t id, const BusTrace& t) {
  if (!t.rx_count || t.total_us<t.echo_us) return;
  unsigned device=3;
  for (unsigned i=0;i<3;++i) if (c.id[i]==id) device=i;
  if (device>=3) return;
  const std::uint32_t us=t.total_us-t.echo_us;
  std::size_t bin=0;
  while (bin<kReplyBinEdgesUs.size() && us>=kReplyBinEdgesUs[bin]) ++bin;
  ++s.reply_hist[device][bin];
  s.reply_max_us[device]=std::max(s.reply_max_us[device],us);
}
}

std::size_t motorPacket(std::uint8_t id, std::uint8_t instruction,
                        const std::uint8_t* params, std::size_t count,
                        std::array<std::uint8_t,48>& out) noexcept {
  if (!id || id>=0xfe || count>18 || (count && !params)) return 0;
  const auto packet=frame::detail::buildFrame(id,instruction,params,count);
  std::copy_n(packet.bytes.begin(),packet.len,out.begin());
  return packet.len;
}
bool motorReply(BusFormat fmt, std::uint8_t id, std::uint8_t instruction,
                 std::uint8_t address, std::uint8_t request_length,
                 const std::uint8_t* b, std::size_t count,
                 std::size_t read_count, BusReply& reply) noexcept {
  const unsigned echo=fmt==BusFormat::Wheel?5:7;
  if ((fmt!=BusFormat::Joint && fmt!=BusFormat::Wheel) || !b ||
      read_count>reply.data.size() || count!=echo+4+read_count ||
      b[2]!=id || b[echo]!=request_length ||
      b[echo+1]!=instruction || b[echo+2]!=address) return false;
  BusReply decoded{};
  if (fmt==BusFormat::Joint) {
    const auto parsed=frame::parseJointReply(b,count);
    if (!parsed.ok || parsed.data_len!=read_count) return false;
    decoded.status=parsed.status; decoded.position=parsed.pos;
    std::copy_n(parsed.data.begin(),read_count,decoded.data.begin());
  } else {
    const auto parsed=frame::parseDxlResponse(b,count);
    if (!parsed.ok || parsed.payload_len!=read_count) return false;
    decoded.status=parsed.err;
    std::copy_n(parsed.payload.begin(),read_count,decoded.data.begin());
  }
  reply=decoded;
  return true;
}

bool validMotorConfig(const BlMotorConfig& c) noexcept {
  for (const auto& side:c.sign) for (int s:side) if (s!=1 && s!=-1) return false;
  for (unsigned i=0;i<3;++i) {
    if (!c.id[i] || c.id[i]>=0xfe) return false;
    for (unsigned j=0;j<i;++j) if (c.id[i]==c.id[j]) return false;
  }
  for (const auto* profiles:{&c.profile,&c.knee_profile}) for (const auto& g:*profiles)
    if (!std::isfinite(g.kp) || !std::isfinite(g.kd) || g.kp<0 || g.kd<0 ||
        g.kp>65.535F || g.kd>65.535F) return false;
  return (c.knee_crank_coupling==0.F || c.knee_crank_coupling==1.F) &&
    std::isfinite(c.wheel_max_rpm) && c.wheel_max_rpm>0 &&
    c.feedback_age_us>0 && c.feedback_age_us<=20000 &&
    c.wheel_max_rpm*c.feedback_age_us/60000000.F<.5F &&
    std::isfinite(c.recovery_wheel_tau_s) && c.recovery_wheel_tau_s>=0 &&
    std::isfinite(c.hip_min) && std::isfinite(c.hip_max) && c.hip_min<c.hip_max &&
    std::isfinite(c.knee_min) && std::isfinite(c.knee_max) && c.knee_min<c.knee_max &&
    c.wheel_settle_us>=20000;
}

bool BlMotorBackend::sensorsFresh(std::uint64_t now) const noexcept {
  for (const auto& s:state_) {
    if (!s.configured || s.error!=MotorError::None) return false;
    for (unsigned j=0;j<3;++j)
      if (!fresh(s.stamp[j],now,config_.feedback_age_us) ||
          !std::isfinite(s.raw_q[j]) || !std::isfinite(s.raw_qd[j])) return false;
  }
  return true;
}
bool BlMotorBackend::startOffTest(std::uint32_t cycles) noexcept {
  MotorGuard guard(lock_);
  if (!cycles || cycles>10000 || maintenance_ || error_!=MotorError::None) return false;
  for (unsigned side=0;side<2;++side)
    if (request_[side].active || !state_[side].off || !state_[side].configured ||
        request_[side].generation!=state_[side].generation ||
        request_[side].off_test_requested!=state_[side].off_test_done) return false;
  homed_=false; // never let an OFF diagnostic transition into motor operation
  for (auto& req:request_) req.off_test_requested+=cycles;
  return true;
}
bool BlMotorBackend::ready() const noexcept {
  MotorGuard guard(lock_);
  return !maintenance_ && homed_ && error_==MotorError::None && state_[0].configured && state_[1].configured;
}
bool BlMotorBackend::disabled() const noexcept {
  MotorGuard guard(lock_);
  return !request_[0].active && !request_[1].active &&
    state_[0].off && state_[1].off &&
    state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation;
}
MotorFeedback BlMotorBackend::feedback(std::uint64_t now) noexcept {
  MotorGuard guard(lock_);
  MotorFeedback f{}; f.driver_fault=error_!=MotorError::None;
  // Retain the oldest real sample stamp for fault diagnostics even when stale.
  // It does not grant validity: all six samples still pass sensorsFresh below.
  f.stamp_us=state_[0].stamp[0];
  for (const auto& leg:state_) for (auto stamp:leg.stamp)
    f.stamp_us=std::min(f.stamp_us,stamp);
  for (unsigned s=0;s<2;++s) for (unsigned j=0;j<3;++j)
    f.encoder_stamp_us[2*j+s]=state_[s].stamp[j];
  f.encoders_valid=sensorsFresh(now); f.homed=homed_ && f.encoders_valid;
  if (!f.encoders_valid) return f;
  for (unsigned s=0;s<2;++s) {
    const auto& m=state_[s]; const auto& sign=config_.sign[s];
    const float hip=sign[0]*m.raw_q[0]+zero_[s][0];
    const float d=sign[1]*m.raw_q[1]+zero_[s][1];
    f.q[s]=hip; f.q[2+s]=d-config_.knee_crank_coupling*hip; f.q[4+s]=sign[2]*m.raw_q[2]+zero_[s][2];
    f.qd[s]=sign[0]*m.raw_qd[0];
    f.qd[2+s]=sign[1]*m.raw_qd[1]-config_.knee_crank_coupling*f.qd[s]; f.qd[4+s]=sign[2]*m.raw_qd[2];
  }
  return f; // no current-based torque sensor: TORQUE_VALID stays clear
}
void BlMotorBackend::cancel() noexcept {
  if (request_[0].active || request_[1].active) {
    for (auto& r:request_) { r.active=false; r.batch={}; ++r.generation; }
  }
  in_flight_=have_complete_=have_latest_=false; applied_at_=0;
  batch_started_={};
}
void BlMotorBackend::disable() noexcept { MotorGuard guard(lock_); cancel(); }
void BlMotorBackend::emergencyStop() noexcept {
  MotorGuard guard(lock_); cancel();
  // A new explicit E-stop rechecks all drivers even if our cached state is OFF.
  for (auto& r:request_) ++r.generation;
}
void BlMotorBackend::beginMaintenance() noexcept {
  MotorGuard guard(lock_);
  if (maintenance_) return;
  cancel(); maintenance_=true;
  for (auto& r:request_) { r.maintenance=true; ++r.generation; }
}
bool BlMotorBackend::resetFault() noexcept {
  MotorGuard guard(lock_); cancel();
  if (maintenance_) return false;
  if (!validMotorConfig(config_)) return false;
  if (error_==MotorError::None) return true; // LF RESET must not erase a good manual home
  error_=MotorError::None; homed_=false;
  superseded_=applied_latency_max_us_=0;
  for (auto& r:request_) { ++r.reset_generation; ++r.generation; r.off_test_requested=0; }
  state_={}; return true; // workers must force OFF and reconfigure before ready
}
void BlMotorBackend::trip(MotorError e) noexcept {
  MotorGuard guard(lock_);
  if (error_==MotorError::None) error_=e;
  homed_=false; cancel();
}
MotorRequest BlMotorBackend::request(unsigned side) const noexcept {
  MotorGuard guard(lock_); return request_[side];
}
void BlMotorBackend::publish(unsigned side, const MotorLegState& s) noexcept {
  MotorGuard guard(lock_);
  if (s.reset_generation==request_[side].reset_generation) state_[side]=s;
}
void BlMotorBackend::publishSample(unsigned side, unsigned motor, const MotorLegState& s) noexcept {
  MotorGuard guard(lock_);
  if (side>=2 || motor>=3 || s.error!=MotorError::None ||
      s.reset_generation!=request_[side].reset_generation ||
      s.generation!=request_[side].generation) return;
  auto& shared=state_[side];
  shared.stamp[motor]=s.stamp[motor];
  shared.raw_q[motor]=s.raw_q[motor];
  shared.raw_qd[motor]=s.raw_qd[motor];
}
BlMotorDiagnostics BlMotorBackend::diagnostics() const noexcept {
  MotorGuard guard(lock_); return {homed_,request_[0].active,in_flight_,error_,state_};
}
MotorExecutionStats BlMotorBackend::executionStats(std::uint64_t now) const noexcept {
  MotorGuard guard(lock_);
  MotorExecutionStats d{};d.supported=true;d.error=static_cast<std::uint32_t>(error_);
  d.superseded=superseded_;d.applied_latency_max_us=applied_latency_max_us_;
  if (in_flight_) {
    d.batch_flags|=4U;
    d.pending_seq=pending_.seq;
    d.pending_age_us=static_cast<std::uint32_t>(std::min<std::uint64_t>(now>=pending_at_?now-pending_at_:0,UINT32_MAX));
    for (unsigned s=0;s<2;++s) if (batch_started_[s]==token_) d.batch_flags|=1U<<s;
  }
  for (unsigned s=0;s<2;++s) {
    const auto& m=state_[s];
    d.cycle_us[s]=m.active_cycle_us;d.cycle_max_us[s]=m.active_cycle_max_us;
    d.io_us[s]=m.active_io_us;d.io_max_us[s]=m.active_io_max_us;
    d.bus_error[s]=static_cast<std::uint32_t>(m.error);
    d.bus_stage[s]=static_cast<std::uint32_t>(m.fault_context.bus_error);
    if (m.fault_context.expired_lease) d.batch_flags|=1U<<(8+2*s);
    if (m.fault_context.expired_batch) d.batch_flags|=1U<<(9+2*s);
  }
  return d;
}
BatchClaim BlMotorBackend::claimBatch(unsigned side, const MotorBatch& batch, std::uint64_t now) noexcept {
  MotorGuard guard(lock_);
  if (side>=2 || !request_[side].active || request_[side].generation!=batch.generation)
    return BatchClaim::Cancelled;
  if (!in_flight_ || request_[side].batch.token!=batch.token) return BatchClaim::Superseded;
  if (now>=request_[side].lease_us || now>=batch.deadline_us) return BatchClaim::Expired;
  batch_started_[side]=batch.token;
  return BatchClaim::Claimed;
}
bool BlMotorBackend::captureHome(const std::array<float,4>& q, std::uint64_t now) noexcept {
  MotorGuard guard(lock_);
  if (maintenance_ || request_[0].active || request_[1].active || error_!=MotorError::None ||
      request_[0].off_test_requested!=state_[0].off_test_done ||
      request_[1].off_test_requested!=state_[1].off_test_done ||
      !state_[0].off || !state_[1].off ||
      state_[0].generation!=request_[0].generation ||
      state_[1].generation!=request_[1].generation || !sensorsFresh(now)) return false;
  for (unsigned s=0;s<2;++s) {
    if (!std::isfinite(q[s]) || !std::isfinite(q[2+s]) || q[s]<config_.hip_min ||
        q[s]>config_.hip_max || q[2+s]<config_.knee_min || q[2+s]>config_.knee_max) return false;
    for (auto v:state_[s].raw_qd) if (std::abs(v)>.15F) return false;
  }
  for (unsigned s=0;s<2;++s) {
    zero_[s][0]=q[s]-config_.sign[s][0]*state_[s].raw_q[0];
    zero_[s][1]=config_.knee_crank_coupling*q[s]+q[2+s]-config_.sign[s][1]*state_[s].raw_q[1];
    zero_[s][2]=-config_.sign[s][2]*state_[s].raw_q[2];
  }
  homed_=true; return true;
}

bool BlMotorBackend::makeBatch(const HcrTarget& t, std::uint32_t seq,
                                std::uint64_t deadline, std::uint64_t now) noexcept {
  std::array<MotorBatch,2> next{};
  MotorApplied actual{}; actual.epoch=t.epoch; actual.seq=seq;
  if (t.profile<HCR_HOME_PD || t.profile>HCR_POLICY_PD || (t.flags&~3U)) return false;
  for (auto v:t.target) if (!std::isfinite(v)) return false;
  for (auto v:t.feedforward) if (!std::isfinite(v)) return false;
  const float coupling=config_.knee_crank_coupling;
  for (unsigned s=0;s<2;++s) {
    auto& b=next[s]; b.flags=t.flags; b.epoch=t.epoch; b.seq=seq; b.deadline_us=deadline;
    b.generation=request_[s].generation; b.token=token_+1;
    const float hip=t.target[s], knee=t.target[2+s]-hip;
    if ((t.flags&1U) && (hip<config_.hip_min || hip>config_.hip_max ||
                        knee<config_.knee_min || knee>config_.knee_max)) return false;
    for (unsigned j=0;j<2;++j) {
      if (!(t.flags&1U)) {
        actual.target[j*2+s]=have_complete_?completed_.target[j*2+s]:
          config_.sign[s][j]*state_[s].raw_q[j]+zero_[s][j]+
          (j?(1-coupling)*actual.target[s]:0.F);
        continue;
      }
      std::int32_t kp,kd,pos,rpm;
      // HOME/POLICY have zero FF; RECOVERY FF is hip / knee hinge.
      const auto gain=(j?config_.knee_profile:config_.profile)[t.profile-1];
      const float ff=t.profile==HCR_RECOVERY ?
        (j?t.feedforward[2+s]+coupling*t.feedforward[s]:t.feedforward[s]) : 0.F;
      const float target=j?knee+coupling*hip:hip;
      if (!quantize(gain.kp*1000.F,0,state_[s].kp_max[j],kp) ||
          !quantize(gain.kd*1000.F,0,state_[s].kd_max[j],kd) ||
          !quantize(config_.sign[s][j]*(target-zero_[s][j])/count_rad,-32768,32767,pos) ||
          !quantize(config_.sign[s][j]*ff/rpm_rad,-32768,32767,rpm)) return false;
      // Validate first: the source codec's clamping helpers must not hide range errors.
      const frame::MitBlock block{static_cast<std::uint16_t>(kp),static_cast<std::uint16_t>(kd),
        static_cast<std::int16_t>(pos),static_cast<std::int16_t>(rpm),0};
      const auto packet=frame::buildMitWrite(config_.id[j],block);
      std::copy_n(packet.bytes.begin()+6,b.joint[j].size(),b.joint[j].begin());
      actual.target[j*2+s]=config_.sign[s][j]*frame::countsToRad(pos)+zero_[s][j]+
        (j?(1-coupling)*actual.target[s]:0.F);
    }
    float omega=t.target[4+s];
    if ((t.flags&2U) && std::abs(omega/rpm_rad)>config_.wheel_max_rpm) return false; // no hidden clamp
    if (!(t.flags&2U)) omega=0;
    if ((t.flags&2U) && t.profile==HCR_RECOVERY && config_.recovery_wheel_tau_s>0) {
      const float dt=applied_at_ && now>applied_at_ ? (now-applied_at_)*1e-6F : .005F;
      const float previous=have_complete_?completed_.target[4+s]:0.F;
      omega=previous+(omega-previous)*(1-std::exp(-dt/config_.recovery_wheel_tau_s));
    }
    std::int32_t rpm;
    if (!quantize(config_.sign[s][2]*omega/rpm_rad,-32768,32767,rpm)) return false;
    b.wheel_rpm=rpm; actual.target[4+s]=config_.sign[s][2]*rpm*rpm_rad;
  }
  ++token_; pending_=actual; pending_at_=now;
  batch_started_={};
  for (unsigned s=0;s<2;++s) request_[s].batch=next[s];
  in_flight_=true; return true;
}
ApplyResult BlMotorBackend::apply(const HcrTarget& t, std::uint32_t seq,
                                   std::uint64_t deadline, std::uint64_t now,
                                   MotorApplied& out) noexcept {
  MotorGuard guard(lock_);
  if (maintenance_) { cancel(); return ApplyResult::Fault; }
  if (error_!=MotorError::None || !homed_ || !sensorsFresh(now) || now>=deadline) {
    if (error_==MotorError::None) error_=now>=deadline?MotorError::Expired:MotorError::Feedback;
    cancel(); return ApplyResult::Fault;
  }
  if (!have_latest_ || seq!=latest_seq_) {
    if (have_latest_ && (t.epoch!=latest_epoch_ || !hcr_newer(seq,latest_seq_))) {
      error_=MotorError::Configuration; cancel(); return ApplyResult::Fault;
    }
    have_latest_=true; latest_epoch_=t.epoch; latest_seq_=seq;
    for (auto& r:request_) { r.lease_us=deadline; r.active=true; r.wheel_enable=t.flags&2U; }
  }
  if (in_flight_ && state_[0].completed==token_ && state_[1].completed==token_ &&
      state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation) {
    completed_=pending_; have_complete_=true; in_flight_=false; applied_at_=pending_at_;
    applied_latency_max_us_=std::max(applied_latency_max_us_,static_cast<std::uint32_t>(
        std::min<std::uint64_t>(now>=pending_at_?now-pending_at_:0,UINT32_MAX)));
    for (auto& r:request_) r.batch={};
  }
  const bool batch_ready=state_[0].prepared && state_[1].prepared &&
      (!(t.flags&2U) || (state_[0].wheel_ready && state_[1].wheel_ready) ||
       (t.target[4]==0.F && t.target[5]==0.F)) &&
      state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation;
  // Replace only a batch that neither bus has claimed. A started/partially
  // applied batch retains its token, reference and original deadline.
  if (in_flight_ && batch_ready && pending_.seq!=seq && !batch_started_[0] && !batch_started_[1]) {
    if (!makeBatch(t,seq,request_[0].lease_us,now)) {
      error_=MotorError::Range;cancel();return ApplyResult::Fault;
    }
    ++superseded_;
  }
  if (!in_flight_ && (!have_complete_ || completed_.seq!=seq) &&
      batch_ready) {
    if (!makeBatch(t,seq,request_[0].lease_us,now)) {
      error_=MotorError::Range; cancel(); return ApplyResult::Fault;
    }
  }
  if (have_complete_) { out=completed_; return ApplyResult::Applied; }
  return ApplyResult::Pending;
}

void BlMotorWorker::fail(MotorError e) noexcept {
  legs_engaged_=false;
  if (state_.error==MotorError::None) {
    state_.error=e;
    auto& f=state_.fault_context;
    f.at_us=bus_.now(); f.during_io=request_in_progress_;
    f.id=request_id_; f.instruction=request_instruction_; f.address=request_address_;
    f.bus_error=request_error_; f.trace=bus_.trace();
    if (guard_active_) {
      f.lease_us=request_in_progress_ ? guard_request_.lease_us : owner_.request(side_).lease_us;
      f.expired_lease=e==MotorError::Expired && f.at_us>=f.lease_us;
      f.expired_batch=e==MotorError::Expired && batch_io_ &&
        guard_request_.batch.token && f.at_us>=guard_request_.batch.deadline_us;
      f.batch_deadline_us=guard_request_.batch.deadline_us;
      f.batch_seq=guard_request_.batch.seq;
    }
  }
  state_.prepared=false; ++state_.errors; owner_.trip(e);
}
void BlMotorWorker::feedbackFailure(FeedbackError reason, unsigned j, int previous,
                                    int received, std::uint64_t now) noexcept {
  if (state_.feedback_error==FeedbackError::None) {
    state_.feedback_error=reason; state_.feedback_id=owner_.config().id[j];
    state_.feedback_previous=previous; state_.feedback_received=received;
    state_.feedback_gap_us=now>=state_.stamp[j]?now-state_.stamp[j]:0;
    state_.feedback_active=guard_active_;
  }
  fail(MotorError::Feedback);
}
void BlMotorWorker::recordFailure(unsigned j, std::uint8_t inst, std::uint8_t addr, BusError error) noexcept {
  // Preserve the first failed transaction; OFF retries must not obscure it.
  if (state_.failed_id) return;
  state_.failed_id=owner_.config().id[j]; state_.failed_instruction=inst;
  state_.failed_address=addr; state_.bus_error=error;
  state_.failure_trace=bus_.trace();
}
void BlMotorWorker::jointSample(unsigned j, const BusReply& r) noexcept {
  const auto now=bus_.now();
  if (state_.stamp[j] && now>state_.stamp[j]) {
    // Joint encoder is signed multi-turn, not a modulo single-turn wheel.
    // Crossing the int16 boundary is not silently unwrapped into a legal target.
    const int delta=int(r.position)-int(previous_joint_[j]);
    if (std::abs(delta)>16384) {
      feedbackFailure(FeedbackError::JointJump,j,previous_joint_[j],r.position,now); return;
    }
    state_.raw_qd[j]=delta*count_rad/((now-state_.stamp[j])*1e-6F);
  }
  previous_joint_[j]=r.position; state_.raw_q[j]=r.position*count_rad;
  state_.stamp[j]=now;
  owner_.publishSample(side_,j,state_);
}
bool BlMotorWorker::busPermitValid(void* context) noexcept {
  auto& self=*static_cast<BlMotorWorker*>(context);
  const auto latest=self.owner_.request(self.side_);
  return latest.active && latest.generation==self.guard_request_.generation;
}
bool BlMotorWorker::requestBus(BusFormat fmt, std::uint8_t id, std::uint8_t inst,
                               const std::uint8_t* p, std::size_t n,
                               std::size_t read, BusReply& reply) noexcept {
  request_error_=BusError::None;
  request_in_progress_=true;
  request_id_=id; request_instruction_=inst; request_address_=n?p[0]:0;
  BusPermit permit{};
  if (guard_active_) {
    // Refresh the LF lease between I/O calls, never during a call. Startup
    // and feedback polling must not inherit an older step's lease snapshot.
    const auto latest=owner_.request(side_);
    if (!latest.active || latest.generation!=guard_request_.generation) {
      request_error_=BusError::Cancelled;request_in_progress_=false;return false;
    }
    guard_request_.lease_us=latest.lease_us;
    permit.deadline_us=latest.lease_us;
    if (batch_io_)
      permit.deadline_us=std::min(permit.deadline_us,guard_request_.batch.deadline_us);
    permit.valid=busPermitValid; permit.context=this;
  }
  bus_.setPermit(permit);
  if (!bus_.permitted()) {
    request_error_=permit.deadline_us && bus_.now()>=permit.deadline_us?BusError::Deadline:BusError::Cancelled;
    bus_.setPermit({});
    if (request_error_==BusError::Deadline) fail(MotorError::Expired);
    request_in_progress_=false;
    return false;
  }
  ++state_.transactions;
  const auto started=bus_.now();
  const bool ok=bus_.request(fmt,id,inst,p,n,read,reply);
  if (ok) recordReplyLatency(state_,owner_.config(),id,bus_.trace());
  if (guard_active_) {
    state_.active_io_us=static_cast<std::uint32_t>(std::min<std::uint64_t>(bus_.now()-started,UINT32_MAX));
    state_.active_io_max_us=std::max(state_.active_io_max_us,state_.active_io_us);
  }
  // Mock buses and hardware must both reject a completion after cancellation/expiry.
  request_error_=bus_.lastError();
  const bool still_permitted=bus_.permitted();
  bus_.setPermit({});
  if (!still_permitted || bus_.lastError()==BusError::Deadline || bus_.lastError()==BusError::Cancelled) {
    request_error_=(permit.deadline_us && bus_.now()>=permit.deadline_us) || bus_.lastError()==BusError::Deadline ?
         BusError::Deadline:BusError::Cancelled;
    if (request_error_==BusError::Deadline) fail(MotorError::Expired);
    request_in_progress_=false;
    return false;
  }
  request_in_progress_=false;
  return ok;
}
bool BlMotorWorker::transact(unsigned j, std::uint8_t inst,
                              const std::uint8_t* p, std::size_t n,
                              std::size_t read, BusReply* out) noexcept {
  BusReply reply{};
  // Repeat only reads/pings or the identical absolute MIT block. No blind
  // retry of wheel speed/enable writes; their acknowledgement rules stay intact.
  const bool retryable=inst==1 || inst==2 || (j<2 && inst==3 && n==11 && p[0]==0x1a);
  bool ok=false;
  for (unsigned attempt=0;attempt<(retryable?2U:1U);++attempt) {
    if (attempt) ++state_.retries;
    ok=requestBus(j==2?BusFormat::Wheel:BusFormat::Joint,
                  owner_.config().id[j],inst,p,n,read,reply);
    if (ok) { if (attempt) ++state_.recovered_transactions; break; }
    if (request_error_!=BusError::Cancelled) recordFailure(j,inst,n?p[0]:0,request_error_);
    const auto error=request_error_;
    if (state_.error!=MotorError::None || error==BusError::Packet || error==BusError::Unavailable ||
        error==BusError::Deadline || error==BusError::Cancelled || error==BusError::Quiet) break;
  }
  if (!ok) { if (request_error_!=BusError::Cancelled && state_.error==MotorError::None) fail(MotorError::Bus); return false; }
  state_.status[j]=reply.status;
  if (reply.status) { recordFailure(j,inst,n?p[0]:0,BusError::None); fail(MotorError::Status); return false; }
  if (j<2) jointSample(j,reply);
  if (out) *out=reply;
  return state_.error==MotorError::None;
}
bool BlMotorWorker::writeVerified(unsigned j, std::uint8_t address, std::uint8_t value) noexcept {
  const std::uint8_t write[2]={address,value}, read[2]={address,1};
  BusReply reply{};
  if (requestBus(j==2?BusFormat::Wheel:BusFormat::Joint,owner_.config().id[j],3,write,2,0,reply)) {
    state_.status[j]=reply.status;
    if (reply.status) { recordFailure(j,3,address,BusError::None); fail(MotorError::Status); return false; }
    if (j<2) jointSample(j,reply);
  } else {
    if (state_.error!=MotorError::None || request_error_==BusError::Cancelled) return false;
    if (request_error_!=BusError::ReplyTimeout && request_error_!=BusError::WriteEchoOnly) {
      recordFailure(j,3,address,request_error_); fail(MotorError::Bus); return false;
    }
  }
  if (state_.error!=MotorError::None || !transact(j,2,read,2,1,&reply)) return false;
  if (reply.data[0]!=value) {
    recordFailure(j,2,address,BusError::ReadbackMismatch); fail(MotorError::Configuration); return false;
  }
  return true;
}
bool BlMotorWorker::torque(unsigned j, bool enable) noexcept {
  const std::uint8_t p[2]={0x18,static_cast<std::uint8_t>(enable)};
  // OFF works after a latched fault. Verify the actual enable register rather
  // than treating a transmitted packet or a write ACK as physical release.
  if (!enable) {
    const bool guarded=guard_active_; guard_active_=false; // OFF is always permitted.
    BusReply reply{}; const auto fmt=j==2?BusFormat::Wheel:BusFormat::Joint;
    const auto id=owner_.config().id[j];
    const bool wrote=requestBus(fmt,id,3,p,2,0,reply);
    if (!wrote && request_error_!=BusError::WriteEchoOnly) recordFailure(j,3,0x18,request_error_);
    const std::uint8_t read[2]={0x18,1};
    const bool read_ok=requestBus(fmt,id,2,read,2,1,reply);
    guard_active_=guarded;
    state_.torque_enable[j]=read_ok?reply.data[0]:255;
    if (read_ok) state_.status[j]=reply.status;
    if (read_ok && !reply.data[0]) { on_[j]=false; return true; }
    on_[j]=true;
    recordFailure(j,2,0x18,read_ok?BusError::ReadbackMismatch:request_error_);
    fail(read_ok?MotorError::Configuration:MotorError::Bus); return false;
  }
  on_[j]=true; // an unacknowledged enable may still have reached the driver
  state_.torque_enable[j]=255;
  if (j==2) {
    if (!writeVerified(j,0x18,1)) return false;
    state_.torque_enable[j]=1; return true;
  }
  return transact(j,3,p,2);
}
bool BlMotorWorker::wheelSpeed(std::int16_t rpm) noexcept {
  // Reuse only a confirmed goal during the same enabled session. Encoder
  // reads still run for every batch; a changed RPM is never deferred.
  if (on_[2] && wheel_goal_valid_ && rpm==wheel_goal_rpm_ && bus_.now()-wheel_goal_at_<100000) {
    ++state_.skipped_writes; return true;
  }
  wheel_goal_valid_=false;
  std::uint8_t p[3]={0x20,0,0}; put(p+1,rpm);
  BusReply reply{};
  if (requestBus(BusFormat::Wheel,owner_.config().id[2],3,p,3,0,reply)) {
    state_.status[2]=reply.status;
    if (reply.status) { recordFailure(2,3,0x20,BusError::None); fail(MotorError::Status); return false; }
    wheel_goal_valid_=true; wheel_goal_rpm_=rpm; wheel_goal_at_=bus_.now();
    return true;
  }
  if (state_.error!=MotorError::None) return false;
  const auto error=request_error_;
  if (error==BusError::Cancelled) return false;
  // BL4012e does not require a speed-write ACK. HCR additionally requires
  // evidence of the latched target: only a missing reply may use readback.
  if (error!=BusError::ReplyTimeout && error!=BusError::WriteEchoOnly) {
    recordFailure(2,3,0x20,error); fail(MotorError::Bus); return false;
  }
  const std::uint8_t read[2]={0x20,2};
  if (!transact(2,2,read,2,2,&reply)) return false;
  if (static_cast<std::int16_t>(u16(reply.data.data()))!=rpm) {
    recordFailure(2,2,0x20,BusError::ReadbackMismatch); fail(MotorError::Configuration); return false;
  }
  wheel_goal_valid_=true; wheel_goal_rpm_=rpm; wheel_goal_at_=bus_.now();
  ++state_.recovered_writes; return true;
}
bool BlMotorWorker::verifyWheelZero() noexcept {
  const std::uint8_t p[2]={0x20,2}; BusReply r{};
  if (!transact(2,2,p,2,2,&r)) return false;
  if (u16(r.data.data())!=0) { recordFailure(2,2,0x20,BusError::ReadbackMismatch); fail(MotorError::Configuration); return false; }
  wheel_goal_valid_=true; wheel_goal_rpm_=0; wheel_goal_at_=bus_.now();
  wheel_zero_verified_=true; return true;
}
bool BlMotorWorker::wheelRead() noexcept {
  const std::uint8_t p[2]={0x24,6}; BusReply r{};
  if (!transact(2,2,p,2,6,&r)) return false;
  const unsigned pos=u16(r.data.data()); const auto now=bus_.now();
  if (pos>=1024) {
    feedbackFailure(FeedbackError::WheelRange,2,previous_wheel_,pos,now); return false;
  }
  if (wheel_sample_) {
    if (now>=state_.stamp[2]) state_.wheel_gap_max_us=std::max(state_.wheel_gap_max_us,now-state_.stamp[2]);
    if (!fresh(state_.stamp[2],now,owner_.config().feedback_age_us)) {
      feedbackFailure(FeedbackError::WheelGap,2,previous_wheel_,pos,now); return false;
    }
    int delta=int(pos)-int(previous_wheel_);
    if (delta>512) delta-=1024;
    if (delta<-512) delta+=1024;
    if (std::abs(delta)==512) {
      feedbackFailure(FeedbackError::WheelAmbiguous,2,previous_wheel_,pos,now); return false;
    }
    wheel_counts_+=delta;
  } else { wheel_counts_=pos; wheel_sample_=true; }
  previous_wheel_=pos; state_.stamp[2]=now;
  state_.raw_q[2]=wheel_counts_*(2*pi/1024.F);
  state_.raw_qd[2]=static_cast<std::int16_t>(u16(r.data.data()+2))*rpm_rad;
  // Present Load is the third field of the same 6-byte read; kept for diagnosis only, never used for control.
  state_.wheel_load=static_cast<std::int16_t>(u16(r.data.data()+4));
  state_.wheel_load_peak=std::max(state_.wheel_load_peak,
      static_cast<std::uint16_t>(std::abs(static_cast<int>(state_.wheel_load))));
  ++state_.wheel_load_samples;
  owner_.publishSample(side_,2,state_);
  return true;
}
bool BlMotorWorker::off() noexcept {
  legs_engaged_=false;
  guard_active_=false; bus_.setPermit({}); wheel_goal_valid_=false;
  state_.prepared=false; state_.wheel_ready=false; wheel_enabled_at_=0; wheel_zero_verified_=false;
  bool ok=true;
  // Wheel first; every motor is attempted even when the first one is absent.
  for (unsigned j:{2U,0U,1U}) if (on_[j] && !torque(j,false)) ok=false;
  state_.off=ok && !on_[0] && !on_[1] && !on_[2];
  if (state_.off) last_off_verify_=bus_.now();
  return state_.off;
}
bool BlMotorWorker::configure() noexcept {
  if (!off()) return false;
  for (unsigned j=0;j<2;++j) {
    BusReply r{}; const std::uint8_t level[2]={0x10,1}, limits[2]={0x32,4};
    if (!transact(j,2,level,2,1,&r)) return false;
    if (r.data[0]!=2 && !writeVerified(j,0x10,2)) return false;
    if (!transact(j,2,limits,2,4,&r)) return false;
    state_.kp_max[j]=u16(r.data.data()); state_.kd_max[j]=u16(r.data.data()+2);
    for (const auto& g:(j?owner_.config().knee_profile:owner_.config().profile))
      if (!std::isfinite(g.kp) || !std::isfinite(g.kd) || g.kp<0 || g.kd<0 ||
          g.kp*1000>state_.kp_max[j] || g.kd*1000>state_.kd_max[j]) {
        fail(MotorError::Configuration); return false;
      }
    if (!writeVerified(j,0x2d,3)) return false;
  }
  // Lumi-Quantum reads wheel response level at boot. A no-write-reply driver
  // uses echo + settle + verified register readback, without a 9 ms ACK wait.
  const std::uint8_t level[2]={0x10,1}; BusReply wheel_level{};
  if (!transact(2,2,level,2,1,&wheel_level)) return false;
  bus_.setWheelWriteReplies(wheel_level.data[0]>=2);
  // Like the bench: disarmed setup only reads the wheel, never writes speed.
  if (!wheelRead()) return false;
  // Prove that the goal-register readback required by HCR startup is supported
  // while still OFF. A retained nonzero goal is observed, never executed here.
  const std::uint8_t goal[2]={0x20,2}; BusReply reply{};
  if (!transact(2,2,goal,2,2,&reply)) return false;
  state_.startup_wheel_goal_rpm=static_cast<std::int16_t>(u16(reply.data.data()));
  state_.configured=true; return true;
}
bool BlMotorWorker::poll(const MotorRequest* active) noexcept {
  const auto pending_batch=[&]() {
    if (!active || !state_.prepared) return false;
    const auto latest=owner_.request(side_);
    return latest.active && latest.generation==active->generation &&
      latest.batch.token && latest.batch.token!=state_.completed;
  };
  for (unsigned j=0;j<2;++j) {
    if (active && !allowed(*active,false)) return false;
    // A feedback sweep is background work. Finish the current transaction,
    // then give a newly queued TARGET the next worker step, without faulting.
    if (pending_batch()) return true;
    if (!transact(j,1,nullptr,0)) return false;
  }
  if (active && !allowed(*active,false)) return false;
  if (pending_batch()) return true;
  return wheelRead();
}
bool BlMotorWorker::allowed(const MotorRequest& r, bool batch) noexcept {
  const auto latest=owner_.request(side_); const auto now=bus_.now();
  if (!latest.active || latest.generation!=r.generation) return false;
  if (now>=latest.lease_us || (batch && now>=r.batch.deadline_us)) {
    fail(MotorError::Expired); return false;
  }
  return true;
}
void BlMotorWorker::step() noexcept {
  const auto started=bus_.now(); auto r=owner_.request(side_);
  guard_request_=r; guard_active_=r.active;
  batch_io_=false;
  if (r.active && !was_active_) {
    state_.active_cycle_us=state_.active_cycle_max_us=state_.active_io_us=state_.active_io_max_us=0;
  }
  was_active_=r.active;
  if (!r.active) legs_engaged_=false;
  if (!validMotorConfig(owner_.config())) { fail(MotorError::Configuration); return; }
  if (r.reset_generation!=state_.reset_generation) {
    state_={}; state_.reset_generation=r.reset_generation; legs_engaged_=false;
    on_={true,true,true}; wheel_goal_valid_=false; wheel_sample_=false; wheel_enabled_at_=0; wheel_zero_verified_=false;
  }
  if (!r.active && state_.generation!=r.generation) { on_={true,true,true}; state_.off=false; }
  state_.generation=r.generation;
  if (!r.active) {
    // Detect a driver reboot/external enable even when cached OFF was true.
    if (!r.maintenance && state_.off && started-last_off_verify_>=100000) { on_={true,true,true}; state_.off=false; }
    if ((!state_.off || on_[0] || on_[1] || on_[2]) &&
        (!last_off_attempt_ || started-last_off_attempt_>=1000)) {
      off(); last_off_attempt_=started;
    }
    // Once maintenance OFF is verified, stop all bus traffic during flash
    // writes. The backend's sticky latch rejects apply/reset until reboot.
    if (!r.maintenance && state_.error==MotorError::None) {
      if (!state_.configured) configure();
      else if (state_.off && state_.off_test_done<r.off_test_requested) {
        // Zero wheel register only, while verified OFF; never MIT or enable.
        if (wheelSpeed(0) && verifyWheelZero() && poll()) ++state_.off_test_done;
      } else if (started-last_idle_poll_>=3000) { poll(); last_idle_poll_=started; }
    }
  } else if (allowed(r,false) && state_.configured && state_.error==MotorError::None) {
    state_.off=false;
    // Startup is not a TARGET batch. Keep zero speed while the wheel's known
    // enable-settle delay expires; new TARGET packets renew only the lease.
    const bool want_wheel=r.batch.token && state_.completed!=r.batch.token ?
      (r.batch.flags&2U)!=0 : r.wheel_enable;
    if (want_wheel && !on_[2]) {
      state_.prepared=false;
      wheel_zero_verified_=false;
      if (allowed(r,false) && wheelSpeed(0) && allowed(r,false) && torque(2,true)) {
        wheel_enabled_at_=bus_.now();
        if (allowed(r,false)) verifyWheelZero();
      }
    }
    if (!want_wheel && on_[2]) torque(2,false);
    const bool settled=!want_wheel ||
      (on_[2] && wheel_enabled_at_ && bus_.now()-wheel_enabled_at_>=owner_.config().wheel_settle_us);
    if (!state_.prepared && allowed(r,false)) {
      // HOME can ACK its zero wheel reference during settle, but only after
      // preload-0, enable and a target-register READBACK proving zero latched.
      // Nonzero speed is deferred until settle; it is never falsely ACKed.
      if (!want_wheel || wheel_zero_verified_) state_.prepared=true;
    }
    state_.wheel_ready=want_wheel && settled && state_.prepared;
    if (state_.prepared && r.batch.token && state_.completed!=r.batch.token) {
      const auto claim=owner_.claimBatch(side_,r.batch,bus_.now());
      if (claim==BatchClaim::Expired) { batch_io_=true; fail(MotorError::Expired); }
      batch_io_=claim==BatchClaim::Claimed;
      const bool claimed=batch_io_;
      bool ok=batch_io_ && allowed(r,true);
      // The wheel command is the latency-critical write: its response is the
      // slowest in the loop. Once an earlier MIT batch on this bus has completed,
      // the legs are already held, so send the wheel first instead of after the
      // two joint blocks. The first batch of a session, and any batch with the
      // legs released, keep the legs-before-wheel order.
      const bool wheel_first=ok && legs_engaged_ && (r.batch.flags&1U) && (r.batch.flags&2U) && settled;
      if (wheel_first) {
        ok=wheelSpeed(r.batch.wheel_rpm);
        if (ok && r.batch.wheel_rpm!=0) wheel_zero_verified_=false;
      }
      for (unsigned j=0;j<2 && ok;++j) {
        if (!allowed(r,true)) { ok=false; break; }
        if (r.batch.flags&1U) {
          std::uint8_t p[11]={0x1a}; std::memcpy(p+1,r.batch.joint[j].data(),10);
          // A full MIT block auto-enables the joint. Mark unknown/on BEFORE
          // writing, so timeout/CRC failure still causes an OFF attempt.
          on_[j]=true; state_.torque_enable[j]=255; ok=transact(j,3,p,11);
        } else {
          if (on_[j]) ok=torque(j,false);
          // Released joints still need real encoder feedback. Continuous
          // batches can leave no background-poll slot (notably while LF releases the joints).
          // PING reads position without enabling torque or writing a target.
          if (ok && allowed(r,true)) ok=transact(j,1,nullptr,0);
          else ok=false;
        }
      }
      if (ok && allowed(r,true)) {
        if (r.batch.flags&2U) {
          if (!wheel_first) {
            if (settled) ok=wheelSpeed(r.batch.wheel_rpm);
            else ok=wheel_zero_verified_ && r.batch.wheel_rpm==0;
            if (ok && r.batch.wheel_rpm!=0) wheel_zero_verified_=false;
          }
        } else if (on_[2]) ok=torque(2,false);
      } else ok=false;
      if (ok && allowed(r,true)) ok=wheelRead();
      if (ok && allowed(r,true)) {
        state_.completed=r.batch.token;
        legs_engaged_=(r.batch.flags&1U)!=0;
      } else if (claimed) legs_engaged_=false; // a superseded batch changes nothing on the bus
      batch_io_=false;
    } else if (allowed(r,false)) {
      // MIT replies and the batch wheel read already refresh feedback. Do not
      // immediately launch another three-I/O sweep while the peer bus finishes.
      // Poll stale samples independently of TARGET arrival; validity limits
      // and the mandatory per-batch wheel read remain unchanged.
      const auto poll_age=std::min<std::uint64_t>(3000,
          std::max<std::uint64_t>(1,owner_.config().feedback_age_us/2));
      bool recent=true;
      for (auto stamp:state_.stamp) recent&=fresh(stamp,bus_.now(),poll_age);
      if (!recent) poll(&r);
    }
  }
  // Cancellation may arrive during any bus wait; disable before publishing.
  const auto after=owner_.request(side_);
  if (!after.active || after.generation!=r.generation || state_.error!=MotorError::None) {
    if (after.generation!=r.generation) on_={true,true,true};
    if (on_[0] || on_[1] || on_[2]) off();
    state_.prepared=false; state_.generation=after.generation;
  }
  const auto duration=bus_.now()-started;
  if (r.active) {
    state_.active_cycle_us=static_cast<std::uint32_t>(std::min<std::uint64_t>(duration,UINT32_MAX));
    state_.active_cycle_max_us=std::max(state_.active_cycle_max_us,state_.active_cycle_us);
  }
  state_.cycle_max_us=std::max(state_.cycle_max_us,static_cast<std::uint32_t>(duration));
  state_.bus_stats=bus_.stats();
  owner_.publish(side_,state_);
}
} // namespace gateway
