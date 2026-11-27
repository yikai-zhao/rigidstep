import {parseTrace,frameLines} from './trace_geometry.mjs';
const $=id=>document.getElementById(id);
const canvas=$('view');
const gl=canvas.getContext('webgl2',{alpha:false,antialias:true,depth:true,preserveDrawingBuffer:true});
if(!gl){$('status').textContent='WebGL2 unavailable in this browser. Open the Canvas viewer instead.';throw Error('WebGL2 is required');}
const vert=`#version 300 es
precision highp float;
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;
void main(){gl_Position=uMVP*vec4(aPos,1.0);vColor=aColor;}`;
const frag=`#version 300 es
precision highp float;
in vec3 vColor;
out vec4 color;
void main(){color=vec4(vColor,1.0);}`;
function shader(type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);
 if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
const program=gl.createProgram();gl.attachShader(program,shader(gl.VERTEX_SHADER,vert));gl.attachShader(program,shader(gl.FRAGMENT_SHADER,frag));gl.linkProgram(program);
if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
gl.useProgram(program);const mvpLocation=gl.getUniformLocation(program,'uMVP');
const vao=gl.createVertexArray(),buffer=gl.createBuffer();gl.bindVertexArray(vao);gl.bindBuffer(gl.ARRAY_BUFFER,buffer);
gl.enableVertexAttribArray(0);gl.vertexAttribPointer(0,3,gl.FLOAT,false,24,0);
gl.enableVertexAttribArray(1);gl.vertexAttribPointer(1,3,gl.FLOAT,false,24,12);
gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.clearColor(.033,.063,.114,1);
const sub=(a,b)=>a.map((x,i)=>x-b[i]);const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const s=Math.hypot(...a)||1;return a.map(v=>v/s)};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
function cameraMatrix(aspect,yaw,pitch,dist){
 const center=[0,1.3,0],eye=[dist*Math.sin(yaw)*Math.cos(pitch),1.3+dist*Math.sin(pitch),dist*Math.cos(yaw)*Math.cos(pitch)];
 const forward=norm(sub(center,eye)),right=norm(cross(forward,[0,1,0])),up=cross(right,forward);
 const view=[right[0],up[0],-forward[0],0,right[1],up[1],-forward[1],0,right[2],up[2],-forward[2],0,
  -dot(right,eye),-dot(up,eye),dot(forward,eye),1];
 const near=.1,far=500,f=1/Math.tan(Math.PI/6);const proj=[f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0];
 const out=new Float32Array(16);
 // Column-major multiplication proj * view.
 for(let col=0;col<4;col++)for(let row=0;row<4;row++)for(let k=0;k<4;k++)out[col*4+row]+=proj[k*4+row]*view[col*4+k];
 return out;
}
let frames=[],index=0,playing=false,speed=1,last=0,carry=0,yaw=.8,pitch=.38,distance=22,drag=null;
let currentGeometry=new Float32Array(0);
function setFrame(i){index=Math.max(0,Math.min(frames.length-1,i));const f=frames[index];if(!f)return;
 currentGeometry=frameLines(f,{contacts:$('showContacts').checked});
 gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,currentGeometry,gl.DYNAMIC_DRAW);
 $('seek').value=String(index);$('seek').max=String(frames.length-1);$('tick').textContent=`${f.tick} / ${frames.length-1}`;
 $('bodyCount').textContent=f.bodies.length;$('candidates').textContent=f.candidates;
 $('contacts').textContent=f.contacts;$('energy').textContent=Number(f.energy).toFixed(3);
 $('hash').textContent=f.hash;$('drawLines').textContent=currentGeometry.length/12;
 $('gpu').textContent='WebGL2';render();}
function render(){const ratio=Math.min(devicePixelRatio||1,2),w=Math.max(1,Math.floor(canvas.clientWidth*ratio)),h=Math.max(1,Math.floor(canvas.clientHeight*ratio));
 if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}gl.viewport(0,0,w,h);
 gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.uniformMatrix4fv(mvpLocation,false,cameraMatrix(w/h,yaw,pitch,distance));
 gl.bindVertexArray(vao);gl.drawArrays(gl.LINES,0,currentGeometry.length/6);}
function load(text,name){const parsed=parseTrace(text);frames=parsed;playing=false;carry=0;$('play').textContent='▶ Play';
 $('status').textContent=`${name}: ${frames.length} validated frames. GPU draws recorded states only.`;setFrame(0);}
$('file').addEventListener('change',async e=>{const file=e.target.files[0];if(!file)return;
 try{if(file.size>40_000_000)throw Error('Trace too large');load(await file.text(),file.name)}catch(err){$('status').textContent=err.message;}});
$('seek').addEventListener('input',()=>setFrame(Number($('seek').value)));
$('showContacts').addEventListener('change',()=>{if(frames.length)setFrame(index)});
$('play').addEventListener('click',()=>{playing=!playing;$('play').textContent=playing?'Ⅱ Pause':'▶ Play'});
$('speed').addEventListener('click',()=>{speed=speed===1?2:speed===2?.25:speed===.25?.5:1;$('speed').textContent=speed+'×'});
canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)});
canvas.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.007;pitch=Math.max(-1.3,Math.min(1.3,pitch+(e.clientY-drag[1])*.005));drag=[e.clientX,e.clientY];render();});
canvas.addEventListener('pointerup',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();distance=Math.max(3,Math.min(100,distance*Math.exp(e.deltaY*.001)));render();},{passive:false});
window.addEventListener('resize',render);
function loop(time){if(last&&playing&&frames.length){carry+=Math.min(.15,(time-last)/1000)*speed;
 let steps=0;while(carry>=frames[0].dt&&steps++<32){carry-=frames[0].dt;if(index>=frames.length-1){playing=false;$('play').textContent='▶ Play';break;}setFrame(index+1);}}
 last=time;requestAnimationFrame(loop);}requestAnimationFrame(loop);
fetch('demo.jsonl').then(r=>{if(!r.ok)throw Error('Demo unavailable');return r.text()}).then(t=>load(t,'Bundled demo'))
 .catch(err=>$('status').textContent=`${err.message}. Select a JSONL file or start python3 -m http.server 8000.`);
