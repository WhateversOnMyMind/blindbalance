import { MotionTracker, sensorRowToBoard, multiply, conjugate, rotate, eulerDegrees, clamp } from './motion.js';
import { BoardRenderer, renderChart } from './renderer.js';
import { SerialConnection } from './serial.js';

const $=id=>document.getElementById(id);
const renderer=new BoardRenderer($('scene'));
let tracker=new MotionTracker(), mode='live', generation=0, controller=null, timer=null;
let lastReceived=0, rate=0, rateCount=0, rateTime=performance.now(), boot=null, error='';
let history=[], lastChart=0, dropped=0, simTime=0, simSeq=0, simAccumulator=0;
let simPrevious=[1,0,0,0];
let usb=null;

function receivePacket(packet){
  if(packet.error)throw Error(packet.error);
  if(!Array.isArray(packet.samples)||!Number.isInteger(packet.boot))throw Error('Unexpected motion data. Upload the updated demo.ino.');
  if(boot!==null&&packet.boot!==boot)resetTracking();
  boot=packet.boot;
  if(packet.dropped){dropped++;tracker.startCalibration();}
  for(const row of packet.samples)accept(row);
  error='';
}

async function connectUSB(){
  if(usb){setMode('usb');return;}
  setMode('usb');
  if(!navigator.serial){error='USB serial is not available here. Open http://127.0.0.1:8127/ in desktop Chrome or Edge.';return;}
  const token=generation;
  const link=new SerialConnection(packet=>{if(token===generation)receivePacket(packet);},e=>{
    if(token===generation){usb=null;error=e?.message||'USB disconnected.';}
  });
  usb=link;
  try{await link.open(navigator.serial);}
  catch(e){if(token===generation){usb=null;error=e.name==='NotFoundError'?'No port selected. Click Connect USB to try again.':`USB: ${e.message} Close Serial Monitor and try again.`;}}
}

function resetTracking(){
  tracker=new MotionTracker();lastReceived=0;history=[];rate=0;rateCount=0;rateTime=performance.now();boot=null;dropped=0;error='';
}
function accept(row){
  // Simulation already generates board coordinates; USB/Wi-Fi carry chip axes.
  if(tracker.ingest(mode==='simulated'?row:sensorRowToBoard(row))){lastReceived=performance.now();rateCount++;return true;}
  return false;
}
async function poll(token){
  if(token!==generation||mode!=='live')return;
  const request=new AbortController();controller=request;
  const timeout=setTimeout(()=>request.abort(),2000);
  try{
    const response=await fetch(`/samples?after=${tracker.lastSeq}`,{signal:request.signal,cache:'no-store'});
    const packet=await response.json();
    if(token!==generation)return;
    if(!response.ok)throw new Error(packet.error||`Board returned HTTP ${response.status}`);
    if(!Array.isArray(packet.samples)||!Number.isInteger(packet.boot))throw new Error('Unexpected motion data. Flash demo/demo.ino to this board.');
    if(boot!==null&&packet.boot!==boot){resetTracking();}
    boot=packet.boot;
    if(packet.dropped){dropped++;tracker.startCalibration();}
    let accepted=0;
    for(const row of packet.samples)if(accept(row))accepted++;
    if(packet.samples.length&&!accepted)throw new Error('The board returned invalid or duplicate samples.');
    error='';
  }catch(e){
    if(token===generation)error=e.name==='AbortError'?'Board timed out. Check your Wi-Fi connection.':e.message;
  }finally{
    clearTimeout(timeout);
    if(token===generation&&mode==='live')timer=setTimeout(()=>poll(token),error?800:30);
  }
}
function setMode(next){
  if(usb){const old=usb;usb=null;void old.close();}
  generation++;clearTimeout(timer);controller?.abort();mode=next;resetTracking();
  $('usb').classList.toggle('selected',mode==='usb');
  $('usb').setAttribute('aria-pressed',String(mode==='usb'));
  $('live').classList.toggle('selected',mode==='live');$('simulate').classList.toggle('selected',mode==='simulated');
  $('live').setAttribute('aria-pressed',String(mode==='live'));$('simulate').setAttribute('aria-pressed',String(mode==='simulated'));
  $('source-label').textContent=mode==='usb'?'USB serial':mode==='live'?'Wi-Fi sensor data':'Generated test data';
  simTime=0;simSeq=0;simAccumulator=0;simPrevious=[1,0,0,0];
  if(mode==='live')poll(generation);
}
function simulationStep(){
  simTime+=.004;
  const t=Math.max(0,simTime-2.8);
  const roll=.72*Math.sin(t*.9),pitch=.5*Math.sin(t*.63),yaw=.95*Math.sin(t*.37);
  const q=multiply(multiply([Math.cos(yaw/2),0,0,Math.sin(yaw/2)],[Math.cos(pitch/2),0,Math.sin(pitch/2),0]),[Math.cos(roll/2),Math.sin(roll/2),0,0]);
  const delta=multiply(conjugate(simPrevious),q),length=Math.hypot(...delta.slice(1));
  const angle=2*Math.atan2(length,delta[0]);
  const gyro=delta.slice(1).map(v=>length>1e-12?v/length*angle/.004*180/Math.PI:0);
  const world=t>0?[.9*Math.sin(t*1.9),.6*Math.sin(t*1.3),1+.4*Math.sin(t*1.6)]:[0,0,1];
  // world X/Y and the Z excess above gravity are specified in m/s².
  if(t>0){world[0]/=9.80665;world[1]/=9.80665;world[2]=1+(world[2]-1)/9.80665;}
  const a=rotate(conjugate(q),world);
  accept([++simSeq,Math.round(simTime*1e6)>>>0,...a,...gyro.map((v,i)=>v+[.3,-.2,.4][i]),26.4]);
  simPrevious=q;
}
$('live').addEventListener('click',()=>setMode('live'));
$('usb').addEventListener('click',connectUSB);
$('simulate').addEventListener('click',()=>setMode('simulated'));
$('camera-reset').addEventListener('click',()=>renderer.resetCamera());
$('recenter').addEventListener('click',()=>tracker.recenter());
$('calibrate').addEventListener('click',()=>{
  if(mode==='simulated'){setMode('simulated');return;}
  tracker.startCalibration();history=[];
});
$('gain').addEventListener('input',()=>{$('gain-value').textContent=Number($('gain').value).toFixed(1)+'×';});

function updateUI(now){
  const age=lastReceived?now-lastReceived:Infinity, fresh=age<800;
  const ready=fresh&&tracker.ready;
  const status=$('status');
  $('usb').textContent=usb?'Disconnect USB':'Connect USB';
  status.className='badge '+(mode==='simulated'?'simulated':ready?'':'waiting');
  status.textContent=mode==='simulated'?'SIMULATED':!fresh?(lastReceived?'DISCONNECTED':'CONNECTING'):tracker.ready?'LIVE':'CALIBRATING';
  let message;
  if(mode==='simulated')message=tracker.ready?'Simulated motion · This animation is generated, not live sensor data.':'Simulated motion · Demonstrating the 2-second still calibration…';
  else if(!fresh)message=error||(mode==='usb'?(usb?'Waiting for USB samples. Upload the updated demo.ino, enable USB CDC On Boot for native USB, and close Serial Monitor.':'Connect the board with a data cable, then click Connect USB. Use desktop Chrome or Edge.'):'Wi-Fi mode requires DEMO_WIFI=1 and the page at http://192.168.4.1/.');
  else if(!tracker.ready)message='Keep the board completely still for 2 seconds. Movement restarts calibration.';
  else message=dropped?'Live motion resumed after a stream gap. Recenter if needed.':'Live motion · Tilt and rotate the board, or move it gently in any direction.';
  $('message').textContent=message;
  $('calibration-progress').hidden=!fresh||tracker.ready;
  $('calibration-progress').value=tracker.progress;
  $('recenter').disabled=!ready;$('calibrate').disabled=!fresh;
  const angles=eulerDegrees(tracker.viewQuaternion());
  ['roll','pitch','yaw'].forEach((id,i)=>{$(id).textContent=ready?angles[i].toFixed(1)+'°':'—';});
  ['ax','ay','az'].forEach((id,i)=>{$(id).textContent=ready?tracker.linear[i].toFixed(2):'—';});
  $('rate').textContent=fresh?`${rate} Hz`:'— Hz';
  $('temperature').textContent=fresh?tracker.temperature.toFixed(1)+' °C':'— °C';
  $('age').textContent=lastReceived?(age>1000?(age/1000).toFixed(1)+' s':Math.round(age)+' ms'):'—';
  return {fresh,ready};
}
let previousFrame=performance.now(),lastUI=0;
function frame(now){
  const dt=clamp((now-previousFrame)/1000,0,.05);previousFrame=now;
  if(mode==='simulated'){
    simAccumulator+=dt;
    while(simAccumulator>=.004){simulationStep();simAccumulator-=.004;}
  }
  if(now-rateTime>=1000){rate=Math.round(rateCount*1000/(now-rateTime));rateCount=0;rateTime=now;}
  // RAF's timestamp predates samples generated inside this callback.
  // Use the current clock so simulated samples never show a negative age.
  if(now-lastUI>80){updateUI(performance.now());lastUI=now;}
  const fresh=lastReceived&&now-lastReceived<800;
  const movement=$('movement').checked&&tracker.ready;
  const position=movement?tracker.viewPosition().map(v=>v*2.7*Number($('gain').value)):[0,0,0];
  renderer.render(tracker.viewQuaternion(),position,Boolean(fresh));
  if(now-lastChart>32){
    if(fresh&&tracker.ready)history.push({time:now,values:[...tracker.linear]});
    history=history.filter(p=>now-p.time<=8000);
    renderChart($('chart'),history,now);lastChart=now;
  }
  requestAnimationFrame(frame);
}
setMode(['localhost','127.0.0.1','[::1]'].includes(location.hostname)?'usb':'live');requestAnimationFrame(frame);
