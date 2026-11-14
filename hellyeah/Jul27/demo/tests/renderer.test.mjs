import test from 'node:test';
import assert from 'node:assert/strict';
import { rasterizeFaces } from '../web/renderer.js';
const project = p => p;
const face = (points, fill) => ({points, fill});
const quad = (z, fill) => face([[1,1,z],[15,1,z],[15,15,z],[1,15,z]],fill);
function draw(faces) {
  const pixels = new Uint8ClampedArray(16*16*4), depths = new Float32Array(16*16);
  rasterizeFaces(faces, project, 16, 16, pixels, depths);
  return (x,y) => [...pixels.slice((y*16+x)*4,(y*16+x)*4+4)];
}
test('near shield stays visible regardless of PCB submission order', () => {
  for (const faces of [[quad(4,'#cccccc'),quad(6,'#226644')],[quad(6,'#226644'),quad(4,'#cccccc')]]) {
    assert.deepEqual(draw(faces)(8,8),[204,204,204,255]);
  }
});
test('slanted surfaces resolve depth per pixel, not by average face distance', () => {
  const tilted = face([[1,1,2],[15,1,10],[15,15,10],[1,15,2]],'#226644');
  for (const faces of [[tilted,quad(4,'#cccccc')],[quad(4,'#cccccc'),tilted]]) {
    const pixel = draw(faces);
    assert.deepEqual(pixel(3,8),[34,102,68,255]);
    assert.deepEqual(pixel(13,8),[204,204,204,255]);
  }
});
test('edge-on triangles and geometry outside the viewport stay bounded', () => {
  const pixel = draw([face([[-50,-50,3],[-40,-50,3],[-40,-40,3]],'#ffffff'),face([[2,2,4],[4,4,4],[6,6,4]],'#ffffff')]);
  assert.deepEqual(pixel(3,3),[0,0,0,0]);
});
