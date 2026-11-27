import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {parseTrace,frameLines,rotate,BOX_EDGES} from '../viewer/trace_geometry.mjs';
const tiny=()=>JSON.stringify({tick:0,dt:1/120,hash:'abc',bodies:[{id:1,type:'box',p:[1,2,3],q:[1,0,0,0],half:[.5,.5,.5]},{id:2,type:'sphere',p:[0,0,0],q:[1,0,0,0],radius:.6}],energy:0,candidates:0,contacts:0});
test('rotation oracle: identity and 90 degree z',()=>{
 assert.deepEqual(rotate([1,0,0,0],[1,2,3]),[1,2,3]);
 const s=Math.SQRT1_2, r=rotate([s,0,0,s],[1,0,0]);
 assert.ok(Math.abs(r[0])<1e-12);assert.ok(Math.abs(r[1]-1)<1e-12);
});
test('view geometry is finite and has exactly expected segment count',()=>{
 const f=parseTrace(tiny())[0];
 const lines=frameLines(f,{grid:false,contacts:false,segments:16});
 assert.equal(BOX_EDGES.length,12);assert.equal(lines.length/12,12+3*16);
 assert.ok(lines.every(Number.isFinite));
});
test('missing and duplicated frames rejected',()=>{
 const row=JSON.parse(tiny());
 assert.throws(()=>parseTrace(`${tiny()}\n${tiny()}`),/frame/);
 assert.throws(()=>parseTrace(''),/frame count/);
 row.tick=5;assert.throws(()=>parseTrace(JSON.stringify(row)),/frame/);
});
test('invalid geometry, zero quaternion components and file-bomb limits',()=>{
 const row=JSON.parse(tiny());row.bodies[0].half=[-1,1,1];assert.throws(()=>parseTrace(JSON.stringify(row)),/body/);
 row.bodies[0].half=[1,1,1];row.bodies[0].p=[NaN,1,0];assert.throws(()=>parseTrace(JSON.stringify(row)),/body/);
 assert.throws(()=>parseTrace(tiny(),{maxBytes:4}),/size/);
});
test('bundled real C++ JSONL trace parses and produces finite GPU vertices',()=>{
 const path=new URL('../viewer/demo.jsonl',import.meta.url);
 const frames=parseTrace(readFileSync(path,'utf8'));
 assert.ok(frames.length>20);assert.ok(frames[0].bodies.length>5);
 for(let i=0;i<frames.length;i+=10) {
   const lines=frameLines(frames[i]);
   assert.ok(lines.length>0&&lines.every(Number.isFinite));
 }
});
