// Exercise the shipped page script. All endpoints, timers and DOM are local mocks.
const assert = require('node:assert/strict'), fs = require('node:fs'), vm = require('node:vm');
const html = fs.readFileSync(process.argv[2], 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map(), pending = [], sent = [], timers = new Map(), events = {};
let timerID = 0, clock = 0;
function element(id) {
  return {value:'',textContent:'',disabled:false,className:'',classList:{add(){},remove(){}}};
}
const buttons = ['ARM','DISARM','RESET','ESTOP','RESET_ENCODERS'].map(command => ({...element(),dataset:{command}}));
const document = {hidden:false,
  getElementById(id) { if (!elements.has(id)) elements.set(id, element(id)); return elements.get(id); },
  querySelectorAll() { return buttons; },
  addEventListener(event, callback) { events[event] = callback; }
};
const context = vm.createContext({document,localStorage:{getItem(){return 'mock-token';},setItem(){}},
  window:{addEventListener(event, callback){ events[event] = callback; }},confirm(){return true;},
  AbortController, performance:{now(){return clock;}}, setTimeout(callback, delay){const id=++timerID;timers.set(id,{callback,delay});return id;},
  clearTimeout(id){timers.delete(id);},
  fetch(url,options){
    return new Promise((resolve,reject) => {
      const entry = {url,options,resolve,reject,ignoreAbort:false};pending.push(entry);sent.push(entry);
      options.signal.addEventListener('abort', () => { if (!entry.ignoreAbort) reject(Error('aborted')); });
    });
  }
});
const flush = () => new Promise(resolve => setImmediate(resolve));
const run = code => vm.runInContext(code, context);
const click = command => buttons.find(button => button.dataset.command === command).onclick();
function state(mode='DISARMED') { return {runtime_online:true,mode,link:true,last_telemetry_flags:7,
  updated_monotonic:10,status_age_s:0,phase:'Folding',epoch:7,counters:{},fault_code:0}; }
function answer(request, data, code=200) { request.resolve({ok:code===200,status:code,json:async()=>data}); }
function timeoutRequest() {
  const timeout=[...timers.entries()].find(([,t])=>t.delay===3000);
  assert.ok(timeout,'request has no timeout');timers.delete(timeout[0]);timeout[1].callback();
}
(async () => {
  vm.runInContext(script, context);
  assert.equal(pending.length,1);const first=pending.shift();assert.equal(first.url,'/api/status');
  run('refresh()');run('refresh()');assert.equal(pending.length,0,'overlapping GET');
  assert.equal(buttons[0].disabled,true,'ARM available before first status');
  answer(first,state());await flush();assert.equal(buttons[0].disabled,false);
  assert.equal(elements.get('link').textContent,'Đã bắt tay');
  // Separate diagnostic display: fresh/fault, stale and unsupported firmware.
  const d={version:1,age_s:.01,transport_age_s:0,flags:0x80000004,local_fault:3,motor_error:5,
    target_rx_age_us:200,pending_age_us:1500,cycle_l_us:2500,cycle_r_us:2600,
    io_l_max_us:900,io_r_max_us:1000,superseded:4,rx_queue_max_us:500,error_l:5,error_r:0,stage_l:9,stage_r:0};
  run('renderMotorDiagnostics('+JSON.stringify({...state(),mcu_diagnostics:d})+')');
  assert.match(elements.get('motor-diagnosis').textContent,/quá hạn thực thi/);
  assert.equal(elements.get('bus-io').textContent,'0.90 ms / 1.00 ms');
  assert.equal(elements.get('target-age').textContent,'0.20 ms');
  run('renderMotorDiagnostics('+JSON.stringify({...state(),mcu_diagnostics:{...d,flags:0x80000804}})+')');
  assert.match(elements.get('motor-diagnosis').textContent,/batch motor vượt deadline gốc/);

  run('renderMotorDiagnostics('+JSON.stringify({...state(),mcu_diagnostics:{...d,age_s:2}})+')');
  assert.match(elements.get('motor-diagnosis').textContent,/đã cũ/);
  assert.equal(elements.get('bus-io').textContent,'—');
  run('renderMotorDiagnostics('+JSON.stringify({...state(),mcu_diagnostics:{...d,flags:0}})+')');
  assert.equal(elements.get('bus-io').textContent,'không hỗ trợ');
  run('renderMotorDiagnostics('+JSON.stringify({...state(),mcu_diagnostics:{...d,transport_age_s:3}})+')');
  assert.equal(elements.get('bus-io').textContent,'—');
  // A resumed/throttled tab cannot use an arbitrarily old successful read.
  clock += 4000; await click('ARM');assert.equal(pending.length,0);
  const recent=run('refresh()');answer(pending.shift(),state());await recent;
  // Hung HTTP request is aborted, marked stale and retried without any command.
  const hanging=run('refresh()');pending.shift();timeoutRequest();await hanging;
  assert.equal(buttons[0].disabled,true);assert.equal(buttons[2].disabled,true);
  assert.equal(buttons[1].disabled,false);assert.equal(buttons[3].disabled,false);
  assert.match(elements.get('view').textContent,/dữ liệu cũ/);
  assert.ok([...timers.values()].some(t=>t.delay>=1000&&t.delay<=5000),'no automatic reconnect');
  assert.equal(sent.filter(r=>r.url==='/api/command').length,0,'display timeout sent command');
  const recovered=run('refresh()');answer(pending.shift(),state());await recovered;
  assert.equal(buttons[0].disabled,false);assert.equal(pending.length,0);
  // One ARM click -> one POST. An older status arriving after the click is ignored.
  const oldRead=run('refresh()'), oldPoll=pending.shift();oldPoll.ignoreAbort=true;
  const arming=click('ARM'), arm=pending.shift();assert.equal(arm.url,'/api/command');
  assert.equal(JSON.parse(arm.options.body).command,'ARM');
  await click('ARM');assert.equal(pending.length,0,'duplicate in-flight ARM');
  answer(oldPoll,state());await oldRead;assert.equal(buttons[0].disabled,true,'stale GET unlocked ARM');
  answer(arm,{accepted:true,mode:'HOME'});await arming;
  answer(pending.shift(),state('HOME'));await flush();
  assert.equal(elements.get('mode').textContent,'Giữ tư thế home');
  assert.equal(sent.filter(r=>r.url==='/api/command').length,1);
  // Stop attempts are available when display polling fails.
  const offline=run('refresh()');pending.shift().reject(Error('network offline'));await offline;
  const stop=click('DISARM'), stopPost=pending.shift();
  assert.equal(JSON.parse(stopPost.options.body).command,'DISARM');
  stopPost.reject(Error('response lost after send'));await stop;
  assert.match(elements.get('command-result').textContent,/chưa xác nhận/);
  const resuming=pending.shift();assert.equal(resuming.url,'/api/status');
  answer(resuming,state());await flush();
  assert.equal(sent.filter(r=>r.url==='/api/command').length,2,'unconfirmed command was replayed');
  // Token changes invalidate a pending GET even if its response survives abort.
  const beforeToken=run('refresh()'), stale=pending.shift();stale.ignoreAbort=true;
  elements.get('token').value='new-token';elements.get('save').onclick();
  answer(stale,state('MODEL'));await beforeToken;
  assert.notEqual(elements.get('mode').textContent,'Model điều khiển');
  run('refresh()');const auth=pending.shift();assert.equal(auth.options.headers.Authorization,'Bearer new-token');
  answer(auth,{error:'access token required'},401);await flush();
  assert.equal(buttons[0].disabled,true);assert.match(elements.get('connection').textContent,/token/);
  // Reload/BFCache/tab reconnection restores reads only, never ARM, RESET or STOP.
  events.pagehide();assert.equal(pending.length,0);
  events.pageshow();answer(pending.shift(),state('MODEL'));await flush();
  document.hidden=false;events.visibilitychange();answer(pending.shift(),state('MODEL'));await flush();
  events.online();answer(pending.shift(),state('MODEL'));await flush();
  assert.equal(sent.filter(r=>r.url==='/api/command').length,2);
  // HTTP errors are not rendered as a healthy status object.
  const bad=run('refresh()');answer(pending.shift(),{error:'busy'},503);await bad;
  assert.equal(run('viewFresh'),false);
  // Explicit E-stop remains possible with stale display and is sent once.
  const estop=click('ESTOP'), emergency=pending.shift();
  assert.equal(JSON.parse(emergency.options.body).command,'ESTOP');
  answer(emergency,{accepted:true,mode:'FAULT'});await estop;
  answer(pending.shift(),state('FAULT'));await flush();
  assert.equal(sent.filter(r=>r.url==='/api/command').length,3);
  // Missing IMU/HOME does not prevent a stationary, disabled encoder capture.
  const unhomed={...state('FAULT'),last_telemetry_flags:2,fault_code:2};
  const homeRead=run('refresh()');answer(pending.shift(),unhomed);await homeRead;
  assert.equal(buttons[0].disabled,true);
  assert.equal(buttons[4].disabled,false);
  const capture=click('RESET_ENCODERS'), capturePost=pending.shift();
  assert.equal(JSON.parse(capturePost.options.body).command,'RESET_ENCODERS');
  await click('RESET_ENCODERS');assert.equal(pending.length,0);
  answer(capturePost,{accepted:true,mode:'FAULT'});await capture;
  assert.match(elements.get('command-result').textContent,/chờ ESP/);
  answer(pending.shift(),{...unhomed,encoder_reset:{state:'pending',message:'Đang chờ ESP'}});await flush();
  assert.equal(buttons[4].disabled,true);assert.equal(buttons[0].disabled,true);
  // Reconnect only reads the result; capture and session RESET stay separate.
  events.online();answer(pending.shift(),{...state(),encoder_reset:{state:'done',message:'ESP đã chụp HOME'}});await flush();
  assert.match(elements.get('encoder-reset').textContent,/đã chụp HOME/);
  assert.equal(buttons[0].disabled,false);
  assert.equal(sent.filter(r=>r.url==='/api/command').length,4);
  for (const flags of [2|8,2|64,0]) {
    const blocked=run('refresh()');answer(pending.shift(),{...unhomed,last_telemetry_flags:flags});await blocked;
    assert.equal(buttons[4].disabled,true);
  }
  console.log('PASS: dashboard page; bounded single poll, stale response/token fencing, timeout/reconnect, one command per click, uncertain delivery no replay, stale-view stop, reload/tab recovery no commands');
})().catch(error => { console.error(error);process.exitCode=1; });
