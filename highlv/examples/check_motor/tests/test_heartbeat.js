const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const script=fs.readFileSync(path.join(__dirname,'../heartbeat.js'),'utf8');
let now=0,timer=null;const sent=[],events=[],sockets=[];
class Socket{
 static OPEN=1;
 constructor(){this.readyState=1;this.bufferedAmount=0;sockets.push(this);}
 send(data){sent.push({at:now,message:JSON.parse(data)});}
 close(){this.readyState=3;}
}
const ctx=vm.createContext({Date:{now:()=>now},WebSocket:Socket,location:{protocol:'http:',host:'mock'},postMessage:m=>events.push(m),setInterval(f){timer=f;return 1;},clearInterval(){timer=null;}});
vm.runInContext(script,ctx);ctx.onmessage({data:{type:'start',token:'mock',client:'a'}});
const stream=sockets[0];stream.onopen();assert.equal(sent[0].message.type,'auth');
stream.onmessage({data:JSON.stringify({type:'ready'})});
let answered=0;
// Delay every pong by 250ms. Outgoing stream must keep its fixed50ms cadence.
for(now=50;now<=2000;now+=50){
 const due=sent.filter(r=>r.message.type==='heartbeat'&&r.at<=now-250);
 if(due.length>answered){answered=due.length;stream.onmessage({data:JSON.stringify({type:'pong',seq:due.at(-1).message.seq})});}
 timer();
}
assert.equal(events.length,0);
const beats=sent.filter(r=>r.message.type==='heartbeat');assert.equal(beats.length,40);
assert(beats.slice(1).every((r,i)=>r.at-beats[i].at===50));
now=2500;timer();assert.equal(events.at(-1).type,'lost');assert.equal(timer,null);assert.equal(stream.readyState,3);
ctx.onmessage({data:{type:'start',token:'mock',client:'b'}});const next=sockets.at(-1);next.onopen();
// Old stream callbacks must not disturb new generation.
stream.onmessage({data:'bad old packet'});assert.equal(events.length,1);
ctx.onmessage({data:{type:'stop'}});assert.equal(next.readyState,3);assert.equal(timer,null);
console.log('PASS: actual WebSocket worker;50ms cadence despite250ms pong RTT,300ms loss STOP, stale stream ignored');
