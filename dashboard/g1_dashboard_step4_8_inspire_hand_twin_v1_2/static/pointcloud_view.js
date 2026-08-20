/* Browser-side latest-only D435i point-cloud renderer.
 *
 * Geometry arrives as compact camera-relative XYZ+RGB snapshots from the
 * dashboard bridge. Orbit/zoom is rendered locally at requestAnimationFrame
 * rate, so changing perspective never waits for Teleimager/H.264 round-trips.
 */
import * as THREE from './vendor/three.module.min.js';

const DEFAULT_VIEW={yaw_deg:22,pitch_deg:14,distance_m:3.16,target_z_m:2.0};
const clamp=(x,lo,hi)=>Math.max(lo,Math.min(hi,Number(x)));

class PointCloudViewer{
  constructor(canvas,onViewChange=null){
    this.canvas=canvas;
    this.onViewChange=onViewChange;
    this.scene=new THREE.Scene();
    this.scene.background=new THREE.Color(0x071018);
    this.camera=new THREE.PerspectiveCamera(58,1,.03,30);
    this.camera.up.set(0,1,0);
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:false,powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5));
    this.geometry=new THREE.BufferGeometry();
    this.material=new THREE.PointsMaterial({size:.022,vertexColors:true,sizeAttenuation:true});
    this.points=new THREE.Points(this.geometry,this.material);
    this.points.frustumCulled=false;
    this.scene.add(this.points);

    // Camera-relative helpers only; the grid is not claimed to be the floor.
    const grid=new THREE.GridHelper(6,24,0x355066,0x1a2b38);
    grid.position.y=-1.0;
    grid.material.transparent=true;
    grid.material.opacity=.42;
    this.scene.add(grid);
    const axes=new THREE.AxesHelper(.35);
    this.scene.add(axes);

    this.view={...DEFAULT_VIEW};
    this.visible=false;
    this.drag=null;
    this.lastSeq=null;
    this.pointCount=0;
    this.dataHz=0;
    this.lastDataPerf=null;
    this.lastDataUnix=0;
    this._bind();
    this._resizeObserver=new ResizeObserver(()=>this.resize());
    this._resizeObserver.observe(canvas);
    this.resize();
    requestAnimationFrame(t=>this._loop(t));
  }
  _bind(){
    this.canvas.addEventListener('contextmenu',e=>e.preventDefault());
    this.canvas.addEventListener('pointerdown',e=>{
      if(!this.visible||e.button!==0)return;
      this.drag={x:e.clientX,y:e.clientY};
      try{this.canvas.setPointerCapture(e.pointerId);}catch{}
      this.canvas.classList.add('dragging');
      e.preventDefault();
    });
    this.canvas.addEventListener('pointermove',e=>{
      if(!this.drag)return;
      const dx=e.clientX-this.drag.x,dy=e.clientY-this.drag.y;
      this.drag.x=e.clientX;this.drag.y=e.clientY;
      this.view.yaw_deg=clamp(this.view.yaw_deg-dx*.34,-180,180);
      this.view.pitch_deg=clamp(this.view.pitch_deg+dy*.28,-82,82);
      this._emitView(false);
      e.preventDefault();
    });
    const finish=e=>{
      if(!this.drag)return;
      this.drag=null;this.canvas.classList.remove('dragging');
      try{this.canvas.releasePointerCapture(e.pointerId);}catch{}
      this._emitView(true);
    };
    this.canvas.addEventListener('pointerup',finish);
    this.canvas.addEventListener('pointercancel',finish);
    this.canvas.addEventListener('wheel',e=>{
      if(!this.visible)return;
      this.view.distance_m=clamp(this.view.distance_m*Math.exp(e.deltaY*.0011),1,8);
      this._emitView(false);
      clearTimeout(this._wheelEnd);
      this._wheelEnd=setTimeout(()=>this._emitView(true),130);
      e.preventDefault();
    },{passive:false});
    this.canvas.addEventListener('dblclick',()=>{if(this.visible){this.setView(DEFAULT_VIEW,true);}});
  }
  _emitView(final){
    if(typeof this.onViewChange==='function')this.onViewChange(this.getView(),!!final);
  }
  setVisible(v){this.visible=!!v;this.canvas.classList.toggle('hidden',!this.visible);if(this.visible)this.resize();}
  getView(){return {...this.view};}
  setView(raw={},emit=false){
    this.view={
      yaw_deg:clamp(Number.isFinite(Number(raw.yaw_deg))?raw.yaw_deg:DEFAULT_VIEW.yaw_deg,-180,180),
      pitch_deg:clamp(Number.isFinite(Number(raw.pitch_deg))?raw.pitch_deg:DEFAULT_VIEW.pitch_deg,-82,82),
      distance_m:clamp(Number.isFinite(Number(raw.distance_m))?raw.distance_m:DEFAULT_VIEW.distance_m,1,8),
      target_z_m:clamp(Number.isFinite(Number(raw.target_z_m))?raw.target_z_m:DEFAULT_VIEW.target_z_m,.5,5),
    };
    if(emit)this._emitView(true);
  }
  resize(){
    const r=this.canvas.getBoundingClientRect();
    const w=Math.max(1,Math.round(r.width)),h=Math.max(1,Math.round(r.height));
    this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();
  }
  _updateCamera(){
    const yaw=THREE.MathUtils.degToRad(this.view.yaw_deg),pitch=THREE.MathUtils.degToRad(this.view.pitch_deg);
    const cp=Math.cos(pitch),r=this.view.distance_m;
    // Snapshot coordinates use Three.js conventional -Z as physical forward,
    // which keeps camera +X visually on the right instead of mirroring it.
    const target=new THREE.Vector3(0,0,-this.view.target_z_m);
    this.camera.position.set(
      target.x+r*cp*Math.sin(yaw),
      target.y+r*Math.sin(pitch),
      target.z+r*cp*Math.cos(yaw),
    );
    this.camera.lookAt(target);
  }
  setSnapshot(buffer){
    if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<32)return false;
    const dv=new DataView(buffer);
    if(String.fromCharCode(dv.getUint8(0),dv.getUint8(1),dv.getUint8(2),dv.getUint8(3))!=='G1PC')return false;
    const version=dv.getUint16(4,true),stride=dv.getUint16(6,true),count=dv.getUint32(8,true),seq=dv.getUint32(12,true);
    const unix=dv.getFloat64(16,true);
    if(version!==1||stride!==9||count>100000||32+count*stride>buffer.byteLength)return false;
    if(this.lastSeq===seq)return true;
    this.lastSeq=seq;
    let posAttr=this.geometry.getAttribute('position');
    let colAttr=this.geometry.getAttribute('color');
    if(!posAttr||posAttr.array.length!==count*3){
      posAttr=new THREE.BufferAttribute(new Float32Array(count*3),3);
      colAttr=new THREE.BufferAttribute(new Uint8Array(count*3),3,true);
      this.geometry.setAttribute('position',posAttr);
      this.geometry.setAttribute('color',colAttr);
    }
    const p=posAttr.array,c=colAttr.array;
    let o=32,j=0;
    for(let i=0;i<count;i++,o+=stride,j+=3){
      p[j]=dv.getInt16(o,true)*.001;
      p[j+1]=dv.getInt16(o+2,true)*.001;
      p[j+2]=dv.getInt16(o+4,true)*.001;
      c[j]=dv.getUint8(o+6);c[j+1]=dv.getUint8(o+7);c[j+2]=dv.getUint8(o+8);
    }
    posAttr.needsUpdate=true;colAttr.needsUpdate=true;
    this.geometry.setDrawRange(0,count);
    this.pointCount=count;this.lastDataUnix=unix;
    const now=performance.now();
    if(this.lastDataPerf!==null){const dt=(now-this.lastDataPerf)/1000;if(dt>0&&dt<2){const hz=1/dt;this.dataHz=this.dataHz?this.dataHz*.82+hz*.18:hz;}}
    this.lastDataPerf=now;
    return true;
  }
  getStats(){return {points:this.pointCount,dataHz:this.dataHz,lastDataUnix:this.lastDataUnix,seq:this.lastSeq};}
  _loop(){
    if(this.visible){this._updateCamera();this.renderer.render(this.scene,this.camera);}
    requestAnimationFrame(()=>this._loop());
  }
}

let viewer=null;
export const PointCloud3D={
  init(canvas,onViewChange){if(!viewer)viewer=new PointCloudViewer(canvas,onViewChange);return viewer;},
  setVisible(v){viewer?.setVisible(v);},
  setSnapshot(b){return viewer?.setSnapshot(b)||false;},
  setView(v,emit=false){viewer?.setView(v,emit);},
  getView(){return viewer?.getView()||{...DEFAULT_VIEW};},
  getStats(){return viewer?.getStats()||{points:0,dataHz:0,lastDataUnix:0,seq:null};},
  defaults:{...DEFAULT_VIEW},
};
