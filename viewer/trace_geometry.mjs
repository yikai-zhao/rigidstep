// Pure geometry and validation module, usable from the browser and Node tests.
export const BOX_EDGES = [[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
const finite = x => typeof x === 'number' && Number.isFinite(x);
const v3 = a => Array.isArray(a) && a.length === 3 && a.every(finite);
export function rotate(q, v) {
  const [w,x,y,z] = q;
  const tx = 2*(y*v[2]-z*v[1]), ty=2*(z*v[0]-x*v[2]), tz=2*(x*v[1]-y*v[0]);
  return [v[0]+w*tx+(y*tz-z*ty), v[1]+w*ty+(z*tx-x*tz), v[2]+w*tz+(x*ty-y*tx)];
}
export function parseTrace(text, {maxBytes=40_000_000, maxFrames=50_000, maxBodies=5_000}={}) {
  if (text.length > maxBytes) throw Error('Trace exceeds supported input size');
  const lines=text.split(/\r?\n/).filter(Boolean);
  if(!lines.length||lines.length>maxFrames) throw Error('Trace frame count outside supported range');
  const out=[];let ids=null,dt=null;
  for (let i=0;i<lines.length;i++) {
    const f=JSON.parse(lines[i]);
    if (f.tick!==i || !finite(f.dt) || f.dt<=0 || (dt!==null && f.dt!==dt)) throw Error(`Invalid frame/timestep at ${i}`);
    if (!Array.isArray(f.bodies)||f.bodies.length>maxBodies)throw Error(`Invalid body count at ${i}`);
    const current=[];
    for(const b of f.bodies) {
      if(!Number.isInteger(b.id)||!v3(b.p)||!Array.isArray(b.q)||b.q.length!==4||!b.q.every(finite)||
          !(b.type==='sphere'||b.type==='box') ||
          (b.type==='box'&&(!v3(b.half)||b.half.some(x=>x<=0))) ||
          (b.type==='sphere'&&(!finite(b.radius)||b.radius<=0)))throw Error(`Invalid body at ${i}`);
      current.push(b.id);
    }
    if(new Set(current).size!==current.length)throw Error(`Duplicate IDs at ${i}`);
    if(ids!==null&&(current.length!==ids.length||current.some((n,j)=>n!==ids[j])))throw Error(`Body identities changed at ${i}`);
    if(f.contact_snapshot!==undefined) {
      if(!Array.isArray(f.contact_snapshot)||f.contact_snapshot.length>20_000)throw Error(`Contact count too large at ${i}`);
      for(const c of f.contact_snapshot)if(!v3(c.p)||!v3(c.n)||!finite(c.depth)||c.depth<0)throw Error(`Bad contact at ${i}`);
    }
    ids=current;dt=f.dt;out.push(f);
  }
  return out;
}

// Interleaved float32 position RGB per vertex, emitted as GL_LINES.
export function frameLines(frame, {contacts=true,grid=true,segments=20}={}) {
  const data=[];
  const line=(a,b,rgb)=>{data.push(...a,...rgb,...b,...rgb);};
  if(grid) {
    for(let i=-16;i<=16;i+=2) {
      line([i,0,-16],[i,0,16],[.14,.23,.32]);
      line([-16,0,i],[16,0,i],[.14,.23,.32]);
    }
    line([0,0,0],[2,0,0],[.9,.39,.34]);
    line([0,0,0],[0,2,0],[.35,.85,.52]);
    line([0,0,0],[0,0,2],[.35,.54,.93]);
  }
  for(const b of frame.bodies) {
    if(b.type==='sphere') {
      // Three orthogonal great circles; quaternion rotates points for consistency.
      for (let axis=0;axis<3;axis++)for(let i=0;i<segments;i++) {
        const angle=2*Math.PI*i/segments,next=2*Math.PI*(i+1)/segments;
        const point=t=>axis===0?[0,Math.cos(t)*b.radius,Math.sin(t)*b.radius]:
          axis===1?[Math.cos(t)*b.radius,0,Math.sin(t)*b.radius]:
                   [Math.cos(t)*b.radius,Math.sin(t)*b.radius,0];
        const w=t=>rotate(b.q,point(t)).map((x,k)=>x+b.p[k]);
        line(w(angle),w(next),[.44,.71,1]);
      }
    } else {
      const verts=[];
      for(let mask=0;mask<8;mask++) {
        const local=[(mask&1)?b.half[0]:-b.half[0],(mask&2)?b.half[1]:-b.half[1],
                     (mask&4)?b.half[2]:-b.half[2]];
        verts.push(rotate(b.q,local).map((x,k)=>x+b.p[k]));
      }
      for(const [i,j] of BOX_EDGES) line(verts[i],verts[j],[.32,.89,.74]);
    }
  }
  if(contacts&&Array.isArray(frame.contact_snapshot)) for(const c of frame.contact_snapshot) {
    const len=Math.max(.2,Math.min(.6,.2+c.depth));
    const end=c.p.map((x,i)=>x+c.n[i]*len);
    const rgb=c.feature==='box-edge'?[1,.69,.31]:c.feature==='ground'?[.76,.49,1]:[1,.35,.41];
    line(c.p,end,rgb);
    const r=.035;
    line([c.p[0]-r,c.p[1],c.p[2]],[c.p[0]+r,c.p[1],c.p[2]],rgb);
    line([c.p[0],c.p[1]-r,c.p[2]],[c.p[0],c.p[1]+r,c.p[2]],rgb);
  }
  return new Float32Array(data);
}
