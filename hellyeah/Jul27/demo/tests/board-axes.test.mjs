import test from 'node:test';
import assert from 'node:assert/strict';
import { MotionTracker, sensorRowToBoard, rotate, conjugate } from '../web/motion.js';

// Board coordinates: photo right, photo top (away from USB), component side.
// The mounted chip's +X points toward USB, +Y toward photo right.
const toSensor = ([x,y,z]) => [-y,x,z];
test('mounted IMU follows board rotation around each physical axis', () => {
  for (let axis=0; axis<3; axis++) {
    const tracker=new MotionTracker();
    let seq=0;
    const feed=(a,g)=>tracker.ingest(sensorRowToBoard([++seq,seq*4000,...toSensor(a),...toSensor(g),25]));
    for(let i=0;i<600;i++)feed([0,0,1],[0,0,0]);
    for(let i=1;i<=250;i++) {
      const q=[Math.cos(i*Math.PI/1000),0,0,0];
      q[axis+1]=Math.sin(i*Math.PI/1000);
      const gyro=[0,0,0];gyro[axis]=90;
      feed(rotate(conjugate(q),[0,0,1]),gyro);
    }
    const expected=[Math.SQRT1_2,0,0,0];expected[axis+1]=Math.SQRT1_2;
    tracker.q.forEach((v,i)=>assert.ok(Math.abs(v-expected[i])<0.015));
    tracker.linear.forEach(v=>assert.ok(Math.abs(v)<0.06));
  }
});

test('mapping preserves metadata, rejects malformed rows downstream, and does not mutate input',()=>{
  const row=[7,28000,1,2,3,4,5,6,27];
  assert.deepEqual(sensorRowToBoard(row),[7,28000,2,-1,3,5,-4,6,27]);
  assert.deepEqual(row,[7,28000,1,2,3,4,5,6,27]);
  assert.equal(sensorRowToBoard(null),null);
});
