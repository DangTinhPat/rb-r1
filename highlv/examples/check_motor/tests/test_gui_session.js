// Execute the actual GUI script with a mocked DOM/fetch, never a motor endpoint.
const assert=require('node:assert/strict'), fs=require('node:fs'), path=require('node:path'), vm=require('node:vm');
const html=fs.readFileSync(path.join(__dirname,'../index.html'),'utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements=new Map(), pending=[], workerMessages=[];
class Worker {postMessage(message){workerMessages.push(message);}}
function element(id=''){return {value:'3',textContent:'',disabled:false,innerHTML:'',appendChild(){},setPointerCapture(){}};}
const document={hidden:false,getElementById(id){if(!elements.has(id))elements.set(id,element(id));return elements.get(id);},createElement:element,querySelectorAll(){return [];},addEventListener(){}};
const context=vm.createContext({Worker,document,window:{addEventListener(){}},location:{hash:'#mock-token'},crypto:require('node:crypto').webcrypto,AbortSignal,
 setInterval(){return 1;},clearInterval(){},fetch(url,options){return new Promise((resolve,reject)=>pending.push({url,options,resolve,reject}));}});
function state(controlling=false){return {connected:true,controlling,fault:true,homed:0,off:63,motors:[],gains:{},counters:[],logs:[],online_count:6,stop_reason:'startup',ack:'',commands_sent:0,acks_received:0};}
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function answer(request,body){request.resolve({ok:true,json:async()=>body});}
(async()=>{
 vm.runInContext(script,context);
 assert.equal(pending.length,1);const oldPoll=pending.shift();assert.equal(oldPoll.url,'/api/state');
 // Connect completes before an older GET. Its stale controlling=false must be ignored.
 const connecting=elements.get('connect').onclick();const pulse=pending.shift();assert.equal(pulse.url,'/api/connect');answer(pulse,state(true));await connecting;
 assert.equal(vm.runInContext('controlling',context),true);
 answer(oldPoll,state(false));await flush();await flush();
 assert.equal(vm.runInContext('controlling',context),true,'old state response revoked new Connect');
 assert.equal(elements.get('online').textContent,'Online 6/6');
 // Display reads and the heartbeat worker are separate channels.
 const tick=vm.runInContext('tick()',context);const active=pending.shift();assert.equal(active.url,'/api/state');assert.equal(workerMessages.at(-1).type,'start');
 // STOP wins over an older in-flight successful heartbeat response.
 const stop=vm.runInContext("stop('test STOP')",context);const stopping=pending.shift();answer(stopping,{});await stop;
 answer(active,state(true));await tick;assert.equal(vm.runInContext('controlling',context),false);
 assert.equal(JSON.parse(stopping.options.body).command,'STOP');assert.equal(workerMessages.at(-1).type,'stop');
 // A later Connect is allowed again; do not auto-reset or enable motors.
 const again=elements.get('connect').onclick();const p=pending.shift();answer(p,state(true));await again;
 assert.equal(vm.runInContext('controlling',context),true);
 assert.equal(pending.length,0);
 // Exactly one HTTP motor command per jog click; no hold timers or repeat loop.
 const before=workerMessages.length;
 elements.get('forward1').onclick();assert.equal(pending.length,1);
 const jog=pending.shift();assert.equal(JSON.parse(jog.options.body).command,'JOG 1 1 3');
 answer(jog,{});await flush();assert.equal(pending.length,0);
 assert.equal(workerMessages.length,before);
 // A display outage must not stop a healthy control lease or replay commands.
 const version=vm.runInContext('sessionVersion',context);
 for(let i=0;i<3;i++){
  const reading=vm.runInContext('tick()',context);pending.shift().reject(Error('display timeout'));await reading;
  assert.equal(vm.runInContext('controlling',context),true);
  assert.equal(vm.runInContext('sessionVersion',context),version);
  assert.equal(workerMessages.length,before,'display failure stopped heartbeat');
  assert.equal(pending.length,0,'display failure sent a command');
 }
 assert.equal(elements.get('reset').disabled,true);assert.equal(elements.get('capture').disabled,true);
 assert.match(elements.get('view').textContent,/dữ liệu cũ/);
 elements.get('forward1').onclick();await flush();assert.equal(pending.length,0,'motion allowed with stale display');
 const recovered=vm.runInContext('tick()',context);answer(pending.shift(),state(true));await recovered;
 assert.equal(vm.runInContext('controlling',context),true);
 assert.equal(vm.runInContext('viewFresh',context),true);
 assert.equal(elements.get('reset').disabled,false);
 assert.match(elements.get('view').textContent,/dữ liệu mới/);
 assert.equal(pending.length,0,'display reconnect replayed a motor command');
 // Real heartbeat loss still stops, even while the display is unavailable.
 const unavailable=vm.runInContext('tick()',context);pending.shift().reject(Error('offline'));await unavailable;
 vm.runInContext("heartbeat.onmessage({data:{type:'lost',reason:'test heartbeat lost'}})",context);
 const lost=pending.shift();assert.equal(JSON.parse(lost.options.body).command,'STOP');
 answer(lost,{});await flush();assert.equal(vm.runInContext('controlling',context),false);
 const viewing=vm.runInContext('tick()',context);answer(pending.shift(),state(false));await viewing;
 assert.equal(vm.runInContext('controlling',context),false,'display reconnect rearmed control');
 assert.equal(elements.get('reset').disabled,true);
 assert.equal(pending.length,0);
 console.log('PASS: actual GUI; display outage/recovery preserves healthy heartbeat, stale display blocks motion, heartbeat loss still STOPs, no replay or auto-rearm, stale responses fenced, one command per click');
})().catch(error=>{console.error(error);process.exitCode=1;});
