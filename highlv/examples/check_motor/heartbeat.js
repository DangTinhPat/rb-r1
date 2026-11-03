'use strict';
// Dedicated browser worker and one full-duplex stream, independent of HTTP/UI RTT.
let generation=0, timer=null, lastOK=0, active=false, socket=null, sequence=0;
function halt(){
 active=false;generation++;if(timer!==null)clearInterval(timer);timer=null;
 if(socket){socket.onclose=null;socket.onerror=null;socket.close();socket=null;}
}
function lost(reason){halt();postMessage({type:'lost',reason});}
onmessage=event=>{
 if(event.data.type==='stop'){halt();return;}
 if(event.data.type!=='start')return;
 halt();active=true;lastOK=Date.now();sequence=0;const version=generation,options=event.data;
 socket=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/api/stream');
 socket.onopen=()=>{if(active&&generation===version)socket.send(JSON.stringify({type:'auth',token:options.token,client:options.client}));};
 socket.onmessage=e=>{
  if(!active||generation!==version)return;
  try{const message=JSON.parse(e.data);if(message.type==='ready'||message.type==='pong')lastOK=Date.now();}
  catch(_){lost('Phản hồi heartbeat không hợp lệ → STOP');}
 };
 socket.onerror=()=>{if(active&&generation===version)lost('Lỗi kênh heartbeat GUI → STOP');};
 socket.onclose=()=>{if(active&&generation===version)lost('Kênh heartbeat GUI đã đóng → STOP');};
 timer=setInterval(()=>{
  if(!active||generation!==version)return;
  if(Date.now()-lastOK>=300){lost('Mất phản hồi heartbeat GUI 300 ms → STOP');return;}
  if(socket.readyState!==WebSocket.OPEN)return;
  if(socket.bufferedAmount>1024){lost('Heartbeat GUI bị nghẽn → STOP');return;}
  socket.send(JSON.stringify({type:'heartbeat',seq:++sequence}));
 },50);
};
