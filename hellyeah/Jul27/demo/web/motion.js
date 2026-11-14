// Right-handed board/world coordinates. Quaternions are [w,x,y,z].
// TDK DS-000347 section 10.1, Figure 16 + the supplied PCB top view:
// pin-1 dot at upper right -> sensor +X toward USB, +Y right, +Z outward.
// Board +X right, +Y away from USB, +Z out of the component side.
export function sensorRowToBoard(row) {
  if (!Array.isArray(row) || row.length !== 9) return row;
  const [seq,us,ax,ay,az,gx,gy,gz,temp]=row;
  return [seq,us,ay,-ax,az,gy,-gx,gz,temp];
}
export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
export const conjugate = q => [q[0], -q[1], -q[2], -q[3]];
export function multiply(a, b) {
  const [w,x,y,z] = a, [v,i,j,k] = b;
  return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
    w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v];
}
export function rotate(q, v) {
  return multiply(multiply(q, [0, ...v]), conjugate(q)).slice(1);
}
const normalize = v => { const n = Math.hypot(...v); return v.map(x => x/n); };
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
function fromGravity(a) {
  const v = normalize(a);
  if (v[2] < -0.999999) return [0,1,0,0];
  return normalize([1+v[2], v[1], -v[0], 0]);
}
function deltaQuaternion(omega, dt) {
  const speed = Math.hypot(...omega);
  if (speed < 1e-12) return [1,0,0,0];
  const half = speed * dt / 2, s = Math.sin(half) / speed;
  return [Math.cos(half), ...omega.map(x => x*s)];
}
export function eulerDegrees(q) {
  const [w,x,y,z] = q, d = 180/Math.PI;
  return [Math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))*d,
    Math.asin(clamp(2*(w*y-z*x), -1, 1))*d,
    Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))*d];
}

export class MotionTracker {
  constructor() {
    this.lastSeq = 0;
    this.lastUs = null;
    this.startCalibration();
  }
  startCalibration() {
    this.ready = false;
    this.progress = 0;
    this.q = [1,0,0,0];
    this.reference = [1,0,0,0];
    this.bias = [0,0,0];
    this.gravity = 1;
    this.gyro = [0,0,0];
    this.accel = [0,0,1];
    this.temperature = 0;
    this.lastUs = null;
    this.resetWindow();
    this.resetMovement();
  }
  resetWindow() {
    this.calibrationTime = 0;
    this.calibrationCount = 0;
    this.sumA = [0,0,0];
    this.sumG = [0,0,0];
    this.anchor = null;
    this.progress = 0;
  }
  resetMovement() {
    this.position = [0,0,0];
    this.velocity = [0,0,0];
    this.linear = [0,0,0];
  }
  recenter() {
    this.reference = [...this.q];
    this.resetMovement();
  }
  viewQuaternion() { return multiply(conjugate(this.reference), this.q); }
  viewPosition() { return rotate(conjugate(this.reference), this.position); }
  ingest(row) {
    if (!Array.isArray(row) || row.length !== 9 || !row.every(Number.isFinite)) return false;
    const [seq, us, ...values] = row;
    if (!Number.isInteger(seq) || seq <= this.lastSeq || !Number.isInteger(us) || us < 0 || us > 0xffffffff) return false;
    const a = values.slice(0,3), g = values.slice(3,6);
    if (a.some(x => Math.abs(x) > 16) || g.some(x => Math.abs(x) > 2000)) return false;
    let dt = this.lastUs === null ? 0.004 : ((us-this.lastUs) >>> 0) / 1e6;
    if (dt <= 0) return false;
    if (dt > 0.1) { this.startCalibration(); dt = 0.004; }
    this.lastSeq = seq;
    this.lastUs = us;
    this.accel = a;
    this.temperature = values[6];
    if (!this.ready) {
      const magnitude = Math.hypot(...a);
      const moved = this.anchor && Math.hypot(...a.map((v,i) => v-this.anchor[i])) > 0.035;
      if (Math.hypot(...g) > 4 || Math.abs(magnitude-1) > 0.12 || moved) {
        this.resetWindow();
        return true;
      }
      if (!this.anchor) this.anchor = [...a];
      a.forEach((v,i) => { this.sumA[i] += v; this.sumG[i] += g[i]; });
      this.calibrationCount++;
      this.calibrationTime += dt;
      this.progress = clamp(this.calibrationTime/2, 0, 1);
      if (this.calibrationTime >= 2) {
        const meanA = this.sumA.map(v => v/this.calibrationCount);
        this.gravity = Math.hypot(...meanA);
        this.bias = this.sumG.map(v => v/this.calibrationCount);
        this.q = fromGravity(meanA);
        this.ready = true;
        // Keep world Z vertical at startup. Full relative pose is optional via Recenter.
        this.reference = [1,0,0,0];
      }
      return true;
    }
    this.gyro = g.map((v,i) => v-this.bias[i]);
    const omega = this.gyro.map(v => v*Math.PI/180);
    this.q = normalize(multiply(this.q, deltaQuaternion(omega, dt)));
    const mag = Math.hypot(...a);
    if (mag > 0.01) {
      // Reject accelerometer correction under obvious linear acceleration.
      // Small sustained acceleration is still indistinguishable from tilt in a 6-axis IMU.
      const trust = clamp(1-Math.abs(mag/this.gravity-1)/0.025, 0, 1);
      const estimatedGravity = rotate(conjugate(this.q), [0,0,1]);
      const error = cross(a.map(v => v/mag), estimatedGravity).map(v => v*1.8*trust);
      this.q = normalize(multiply(this.q, deltaQuaternion(error, dt)));
    }
    const world = rotate(this.q, a.map(v => v/this.gravity));
    const alpha = 1-Math.exp(-dt*35);
    for (let i = 0; i < 3; i++) {
      let acceleration = (world[i]-(i === 2 ? 1 : 0))*9.80665;
      if (Math.abs(acceleration) < 0.06) acceleration = 0;
      this.linear[i] += alpha*(acceleration-this.linear[i]);
      // Spring + damping deliberately returns to center: this is NOT odometry.
      this.velocity[i] += (this.linear[i]-12*this.position[i]-5*this.velocity[i])*dt;
      const next = this.position[i]+this.velocity[i]*dt;
      this.position[i] = clamp(next, -0.35, 0.35);
      if (next !== this.position[i]) this.velocity[i] = 0;
    }
    return true;
  }
}
