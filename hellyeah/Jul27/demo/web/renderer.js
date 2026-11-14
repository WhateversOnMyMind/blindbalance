import { rotate, clamp } from './motion.js';
const dot = (a,b) => a.reduce((s,v,i) => s+v*b[i],0);
const add = (a,b) => a.map((v,i) => v+b[i]);
const colors = ['#f5ab8e','#c4efa8','#90c9ed'];

// A painter's sort cannot correctly order a large PCB face and smaller raised
// components. Resolve visibility at each pixel using perspective-correct depth.
export function rasterizeFaces(faces, project, width, height, pixels, depths) {
  pixels.fill(0);
  depths.fill(0); // inverse depth: larger values are closer to the camera
  for (const face of faces) {
    const points = face.points.map(project);
    const rgb = Number.parseInt(face.fill.slice(1), 16);
    const red = rgb >> 16, green = (rgb >> 8) & 255, blue = rgb & 255;
    for (let triangle = 1; triangle < points.length - 1; triangle++) {
      const [a,b,c] = [points[0],points[triangle],points[triangle+1]];
      if (![...a,...b,...c].every(Number.isFinite) || Math.min(a[2],b[2],c[2]) <= 0) continue;
      const area = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
      if (Math.abs(area) < 1e-7) continue;
      const minX = Math.max(0,Math.floor(Math.min(a[0],b[0],c[0])));
      const maxX = Math.min(width-1,Math.ceil(Math.max(a[0],b[0],c[0])));
      const minY = Math.max(0,Math.floor(Math.min(a[1],b[1],c[1])));
      const maxY = Math.min(height-1,Math.ceil(Math.max(a[1],b[1],c[1])));
      const dx0 = (b[1]-c[1])/area, dx1 = (c[1]-a[1])/area;
      const za = 1/a[2], zb = 1/b[2], zc = 1/c[2];
      for (let y = minY; y <= maxY; y++) {
        const py = y+.5, px = minX+.5;
        let w0 = ((b[0]-px)*(c[1]-py)-(b[1]-py)*(c[0]-px))/area;
        let w1 = ((c[0]-px)*(a[1]-py)-(c[1]-py)*(a[0]-px))/area;
        for (let x = minX; x <= maxX; x++, w0 += dx0, w1 += dx1) {
          const w2 = 1-w0-w1;
          if (w0 < -1e-7 || w1 < -1e-7 || w2 < -1e-7) continue;
          const inverseDepth = w0*za+w1*zb+w2*zc, index = y*width+x;
          if (inverseDepth <= depths[index]) continue;
          depths[index] = inverseDepth;
          const offset = index*4;
          pixels[offset] = red; pixels[offset+1] = green;
          pixels[offset+2] = blue; pixels[offset+3] = 255;
        }
      }
    }
  }
}

export class BoardRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.meshCanvas = document.createElement('canvas');
    this.meshContext = this.meshCanvas.getContext('2d');
    this.resetCamera();
    let drag = null;
    canvas.addEventListener('pointerdown', e => { drag = [e.clientX,e.clientY]; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener('pointermove', e => {
      if (!drag) return;
      this.azimuth -= (e.clientX-drag[0])*0.008;
      this.elevation = clamp(this.elevation+(e.clientY-drag[1])*0.006, 0.12, 1.45);
      drag = [e.clientX,e.clientY];
    });
    for (const event of ['pointerup','pointercancel','lostpointercapture']) canvas.addEventListener(event, () => { drag = null; });
    canvas.addEventListener('wheel', e => { e.preventDefault(); this.distance = clamp(this.distance+e.deltaY*0.006, 5, 12); }, {passive:false});
  }
  resetCamera() { this.azimuth = -0.9; this.elevation = 0.67; this.distance = 7.8; }
  resize() {
    const {width,height} = this.canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.round(width*dpr) || this.canvas.height !== Math.round(height*dpr)) {
      this.canvas.width = Math.round(width*dpr); this.canvas.height = Math.round(height*dpr);
    }
    this.ctx.setTransform(dpr,0,0,dpr,0,0);
    this.width = width; this.height = height;
    const ca = Math.cos(this.azimuth), sa = Math.sin(this.azimuth), ce = Math.cos(this.elevation), se = Math.sin(this.elevation);
    this.right = [-sa,ca,0]; this.up = [-ca*se,-sa*se,ce]; this.towardCamera = [ca*ce,sa*ce,se];
  }
  project(p) {
    const depth = Math.max(1,this.distance-dot(p,this.towardCamera));
    const scale = Math.min(this.width,this.height)*2.2/depth;
    return [this.width*.5+dot(p,this.right)*scale, this.height*.46-dot(p,this.up)*scale, depth];
  }
  path(points, stroke, fill, lineWidth=1) {
    const c = this.ctx;
    c.beginPath();
    points.forEach((p,i) => { const v=this.project(p); if(i) c.lineTo(v[0],v[1]); else c.moveTo(v[0],v[1]); });
    if (fill) { c.closePath(); c.fillStyle=fill; c.fill(); }
    if (stroke) { c.strokeStyle=stroke; c.lineWidth=lineWidth; c.stroke(); }
  }
  render(q, position, active) {
    this.resize();
    const c=this.ctx;
    c.clearRect(0,0,this.width,this.height);
    for (let i=-7;i<=7;i++) {
      this.path([[i*.5,-3.5,-.85],[i*.5,3.5,-.85]], i===0?'#446046':'#2c4035');
      this.path([[-3.5,i*.5,-.85],[3.5,i*.5,-.85]], i===0?'#446046':'#2c4035');
    }
    const shadow=[];
    for(let i=0;i<48;i++){const t=i*Math.PI/24;shadow.push([position[0]+1.25*Math.cos(t),position[1]+.72*Math.sin(t),-.84]);}
    this.path(shadow,null,'#070f0b55');
    const transform = v => add(rotate(q,v),position);
    const faces=[];
    const box=(center,size,palette)=>{
      const vertices=[];
      for(let i=0;i<8;i++)vertices.push(transform(center.map((v,k)=>v+((i>>k)&1?1:-1)*size[k]/2)));
      const indices=[[0,2,3,1],[0,1,5,4],[0,4,6,2],[1,3,7,5],[2,6,7,3],[4,5,7,6]];
      indices.forEach((ids,i)=>{const points=ids.map(j=>vertices[j]);faces.push({points,fill:palette[i]||palette[0]});});
    };
    // Approximate proportions/positions from the supplied board images.
    // Photo right is +X; photo top is +Y. No antenna trace or tiny passives.
    const metal=['#707777','#889090','#a6aaaa','#bbc0c0','#9ca3a3','#dedede'];
    const black=['#171717','#222222','#343434','#292929','#303030','#444444'];
    box([0,0,0],[2.4,2.08,.08],['#173b28','#244d35','#28543b','#224c33','#2e5940','#305e43']);
    box([-.2,.15,.17],[.98,.94,.26],metal); // ESP32 shield
    box([-.2,-.92,.16],[.62,.58,.24],metal); // USB-C at bottom edge
    box([-.2,-1.215,.16],[.47,.012,.12],black); // socket opening
    box([-.87,-.18,.09],[.2,.23,.1],black); // IMU on left
    for(const [x,y] of [[-.97,.45],[.5,.51]]) {
      box([x,y,.09],[.25,.23,.1],metal);
      box([x,y,.16],[.14,.13,.05],black);
    }
    box([.68,-.05,.16],[.28,.5,.24],['#777267','#aaa697','#bdb9aa','#cbc7b8','#b2ae9e','#d7d2c4']); // battery connector
    this.drawMesh(faces);
    if(dot(rotate(q,[0,0,1]),this.towardCamera)>0) {
      this.labelOnBoard('ESP32-S3',[-.55,.17,.301],q,position,.12,'#444444');
    }
    const origin=transform([0,0,.38]);
    for(let i=0;i<3;i++){
      const axis=[0,0,.38];axis[i]+=1.15;
      const end=transform(axis);
      this.path([origin,end],colors[i],null,1.7);
      const p=this.project(end); c.fillStyle=colors[i]; c.font='11px monospace';c.fillText('XYZ'[i],p[0]+6,p[1]-4);
    }
    if(!active){c.fillStyle='#bfd0c055';c.font='10px monospace';c.textAlign='center';c.fillText('WAITING FOR SENSOR DATA',this.width*.5,this.height*.73);c.textAlign='left';}
  }
  drawMesh(faces) {
    const width=Math.max(1,Math.round(this.width)),height=Math.max(1,Math.round(this.height));
    if(this.meshCanvas.width!==width || this.meshCanvas.height!==height || !this.meshImage) {
      this.meshCanvas.width=width; this.meshCanvas.height=height;
      this.meshImage=this.meshContext.createImageData(width,height);
      this.meshDepths=new Float32Array(width*height);
    }
    rasterizeFaces(faces,p=>this.project(p),width,height,this.meshImage.data,this.meshDepths);
    this.meshContext.putImageData(this.meshImage,0,0);
    this.ctx.drawImage(this.meshCanvas,0,0,this.width,this.height);
  }
  labelOnBoard(text,point,q,position,size,color){
    const c=this.ctx,project=v=>this.project(add(rotate(q,v),position));
    const p=project(point),x=project(add(point,[1,0,0])),y=project(add(point,[0,-1,0]));
    // Don't put the label through another surface at an oblique angle.
    const ix=Math.floor(p[0]),iy=Math.floor(p[1]);
    if(ix<0 || iy<0 || ix>=this.meshCanvas.width || iy>=this.meshCanvas.height) return;
    if(this.meshDepths[iy*this.meshCanvas.width+ix] > 1/p[2]+.001) return;
    c.save();c.transform(x[0]-p[0],x[1]-p[1],y[0]-p[0],y[1]-p[1],p[0],p[1]);
    c.font=`600 ${size}px monospace`;c.fillStyle=color;c.fillText(text,0,0);c.restore();
  }
}

export function renderChart(canvas,history,now){
  const chartColors=['#b64d24','#38752e','#226fa5'];
  const c=canvas.getContext('2d'),rect=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
  if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);}
  c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,rect.width,rect.height);
  const w=rect.width-32,h=rect.height-20,mid=h/2;
  const max=Math.max(2,...history.flatMap(p=>p.values.map(Math.abs)));
  const range=Math.ceil(max/2)*2;
  c.font='10px monospace';c.fillStyle='#555';c.lineWidth=1;
  for(const v of [-range,0,range]){
    const y=mid-v/range*(mid-5);c.strokeStyle=v===0?'#999':'#ddd';c.beginPath();c.moveTo(29,y);c.lineTo(rect.width,y);c.stroke();c.fillText(String(v),0,y+3);
  }
  for(let i=0;i<3;i++){
    c.beginPath();c.strokeStyle=chartColors[i];c.lineWidth=1.5;
    history.forEach((p,j)=>{const x=32+w*(1-(now-p.time)/8000),y=mid-p.values[i]/range*(mid-5);if(j)c.lineTo(x,y);else c.moveTo(x,y);});c.stroke();
  }
  c.fillStyle='#555';c.fillText('−8 s',32,rect.height-1);c.fillText('now',rect.width-21,rect.height-1);
}
