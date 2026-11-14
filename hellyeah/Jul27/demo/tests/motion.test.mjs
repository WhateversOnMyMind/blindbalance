import test from 'node:test';
import assert from 'node:assert/strict';
import { MotionTracker, rotate, conjugate } from '../web/motion.js';

const close = (got, want, tolerance = 0.02) => assert.ok(Math.abs(got - want) < tolerance, `${got} should be near ${want}`);
function source(tracker) {
  let seq = 0, us = 0;
  return (a = [0, 0, 1], g = [0, 0, 0], count = 1, step = 4000) => {
    for (let i = 0; i < count; i++) {
      us = (us + step) >>> 0;
      tracker.ingest([++seq, us, ...a, ...g, 25]);
    }
  };
}
function ready(a = [0, 0, 1], g = [0, 0, 0]) {
  const tracker = new MotionTracker();
  const feed = source(tracker);
  feed(a, g, 600);
  assert.equal(tracker.ready, true);
  return { tracker, feed };
}

test('stationary calibration removes all three gyro biases', () => {
  const { tracker, feed } = ready([0, 0, 1], [0.6, -0.8, 1.2]);
  feed([0, 0, 1], [0.6, -0.8, 1.2], 2500);
  tracker.bias.forEach((v, i) => close(v, [0.6, -0.8, 1.2][i], 0.001));
  tracker.linear.forEach(v => close(v, 0, 0.001));
  tracker.position.forEach(v => close(v, 0, 0.001));
  close(tracker.q[0], 1, 0.001);
});

test('calibration cannot finish while board is rotating or accelerating', () => {
  const t = new MotionTracker(), feed = source(t);
  feed([0, 0, 1], [0, 20, 0], 1000);
  assert.equal(t.ready, false);
  for (let i = 0; i < 1200; i++) feed([i % 2 ? 0.3 : -0.3, 0, 1]);
  assert.equal(t.ready, false);
  feed([0, 0, 1], [0, 0, 0], 600);
  assert.equal(t.ready, true);
});

test('positive 90 degree yaw maps sensor X onto world Y', () => {
  const { tracker, feed } = ready();
  feed([0, 0, 1], [0, 0, 90], 250);
  const x = rotate(tracker.q, [1, 0, 0]);
  close(x[0], 0); close(x[1], 1); close(x[2], 0);
  close(Math.hypot(...tracker.q), 1, 1e-9);
});

test('gravity is removed at tilted and upside-down startup poses', () => {
  for (const a of [[0, 1, 0], [0, 0, -1], [0.6, 0, 0.8]]) {
    const { tracker, feed } = ready(a);
    feed(a, [0, 0, 0], 1000);
    tracker.linear.forEach(v => close(v, 0, 0.005));
    tracker.position.forEach(v => close(v, 0, 0.005));
  }
});

test('roll follows gyro through 90 degrees without mistaking gravity for translation', () => {
  const { tracker, feed } = ready();
  for (let i = 1; i <= 250; i++) {
    const angle = i * Math.PI / 500;
    feed([0, Math.sin(angle), Math.cos(angle)], [90, 0, 0]);
  }
  const y = rotate(tracker.q, [0, 1, 0]);
  close(y[1], 0); close(y[2], 1);
  assert.ok(Math.hypot(...tracker.linear) < 0.15);
});

test('translation responds to acceleration then settles instead of drifting', () => {
  const { tracker, feed } = ready();
  feed([0.3, 0, 1], [0, 0, 0], 30);
  assert.ok(tracker.position[0] > 0.005);
  assert.ok(tracker.linear[0] > 1);
  feed([0, 0, 1], [0, 0, 0], 4000);
  assert.ok(Math.hypot(...tracker.position) < 0.02);
});

test('world acceleration uses current orientation', () => {
  const { tracker, feed } = ready();
  feed([0, 0, 1], [0, 0, 90], 250);
  feed([0.3, 0, 1], [0, 0, 0], 10);
  assert.ok(tracker.linear[1] > 2);
  assert.ok(Math.abs(tracker.linear[0]) < 0.1);
});

test('duplicate and malformed samples do not contaminate the pose', () => {
  const { tracker } = ready();
  const before = [...tracker.q];
  for (const sample of [null, [], [900, 3, NaN, 0, 1, 0, 0, 0, 25], [1, 4000, 0, 0, 1, 900, 0, 0, 25]]) {
    assert.equal(tracker.ingest(sample), false);
  }
  assert.deepEqual(tracker.q, before);
});

test('micros rollover preserves small positive time steps', () => {
  const { tracker } = ready();
  tracker.lastUs = 4294965296;
  tracker.ingest([601, 2000, 0, 0, 1, 0, 0, 90, 25]);
  assert.equal(tracker.ready, true);
  close(rotate(tracker.q, [1, 0, 0])[1], 0.006283, 0.0001);
});

test('long sample gap requires fresh calibration instead of integrating stale gyro', () => {
  const { tracker, feed } = ready();
  feed([0, 0, 1], [0, 0, 90], 1, 500000);
  assert.equal(tracker.ready, false);
  close(tracker.q[0], 1, 0.001);
});

test('recenter resets relative view and translation while preserving gravity reference', () => {
  const { tracker, feed } = ready();
  feed([0, 0, 1], [0, 0, 90], 250);
  tracker.recenter();
  const relative = tracker.viewQuaternion();
  close(relative[0], 1, 0.001);
  feed([0.3, 0, 1], [0, 0, 0], 10);
  assert.ok(tracker.linear[1] > 2);
  const displayAcceleration = rotate(conjugate(tracker.reference), tracker.linear);
  assert.ok(displayAcceleration[0] > 2);
});
