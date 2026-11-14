import test from 'node:test';
import assert from 'node:assert/strict';
import { JsonLines, SerialConnection } from '../web/serial.js';
test('serial framing handles split packets, boot logs and CRLF',()=>{
  const packets=[], parser=new JsonLines(p=>packets.push(p));
  parser.push('Booting\r\n{"boot":7,"sam');
  parser.push('ples":[]}\r\n{"error":"IMU not found"}\n');
  assert.deepEqual(packets,[{boot:7,samples:[]},{error:'IMU not found'}]);
});
test('oversized and damaged serial lines recover at the next newline',()=>{
  const packets=[], parser=new JsonLines(p=>packets.push(p));
  parser.push('x'.repeat(70000)); parser.push('\n{bad}\n{"boot":8,"samples":[]}\n');
  assert.deepEqual(packets,[{boot:8,samples:[]}]);
});
test('USB reader delivers packets and releases its stream before closing',async()=>{
  let control, closed=false;
  const readable=new ReadableStream({start(c){control=c;}});
  const packets=[];
  const port={readable,async open(options){assert.equal(options.baudRate,460800);},async setSignals(signals){assert.equal(signals.dataTerminalReady,true);},async close(){assert.equal(readable.locked,false);closed=true;}};
  const link=new SerialConnection(p=>packets.push(p),()=>{});
  await link.open({requestPort:async()=>port});
  control.enqueue(new TextEncoder().encode('{"boot":9,"samples":[]}\n'));
  await new Promise(r=>setImmediate(r));
  await link.close();
  assert.deepEqual(packets,[{boot:9,samples:[]}]);assert.equal(closed,true);
});
test('switching source while device picker is open does not open the stale port',async()=>{
  let choose,opened=false;
  const link=new SerialConnection(()=>{},()=>{});
  const opening=link.open({requestPort:()=>new Promise(r=>choose=r)});
  await link.close();
  choose({open:async()=>{opened=true;}});
  await opening;
  assert.equal(opened,false);
});
