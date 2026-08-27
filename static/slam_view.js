import * as THREE from './vendor/three.module.min.js';

const clamp=(value,minimum,maximum)=>
  Math.max(minimum,Math.min(maximum,Number(value)));

class SlamViewer {
  constructor(canvas){
    this.canvas=canvas;
    this.visible=false;
    this.mapEnabled=true;
    this.liveEnabled=true;
    this.robotEnabled=true;
    this.liveFresh=false;
    this.liveHasCloud=false;
    this.robotLocalized=false;
    this.robotHasPose=false;
    this.drag=null;

    this.initialPoseSelecting=false;
    this.initialPoseDraft=null;
    this.initialPoseCallback=null;
    this.raycaster=new THREE.Raycaster();
    this.pointerNdc=new THREE.Vector2();
    this.selectionPlane=new THREE.Plane(
      new THREE.Vector3(0,0,1),
      0
    );

    this.scene=new THREE.Scene();
    this.scene.background=new THREE.Color(0x061019);

    this.camera=new THREE.PerspectiveCamera(52,1,0.05,250);
    this.camera.up.set(0,0,1);

    this.renderer=new THREE.WebGLRenderer({
      canvas,
      antialias:false,
      powerPreference:'high-performance'
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5));

    this.target=new THREE.Vector3(3,-2,0.7);
    this.yaw=-0.72;
    this.pitch=0.78;
    this.distance=24;
    this.fitView={target:this.target.clone(),yaw:this.yaw,pitch:this.pitch,distance:this.distance};

    this.mapGeometry=new THREE.BufferGeometry();
    this.mapMaterial=new THREE.PointsMaterial({
      size:0.045,
      vertexColors:true,
      sizeAttenuation:true,
      transparent:true,
      opacity:0.9
    });
    this.mapPoints=new THREE.Points(this.mapGeometry,this.mapMaterial);
    this.mapPoints.frustumCulled=false;
    this.scene.add(this.mapPoints);

    this.liveGeometry=new THREE.BufferGeometry();
    this.liveMaterial=new THREE.PointsMaterial({
      size:0.065,
      color:0xff3bd4,
      sizeAttenuation:true,
      transparent:true,
      opacity:0.95
    });
    this.livePoints=new THREE.Points(this.liveGeometry,this.liveMaterial);
    this.livePoints.frustumCulled=false;
    this.scene.add(this.livePoints);

    this.grid=null;
    this.robot=this.buildRobot();
    this.scene.add(this.robot);

    this.initialPoseMarker=this.buildInitialPoseMarker();
    this.scene.add(this.initialPoseMarker);

    this.scene.add(new THREE.HemisphereLight(0xcce8ff,0x102030,1.7));
    const light=new THREE.DirectionalLight(0xffffff,1.2);
    light.position.set(5,-4,10);
    this.scene.add(light);

    this.bind();
    this.resizeObserver=new ResizeObserver(()=>this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
    this.updateVisibility();
    requestAnimationFrame(()=>this.loop());
  }

  buildRobot(){
    const group=new THREE.Group();
    const light=new THREE.MeshStandardMaterial({
      color:0xc8d2db,roughness:0.55,metalness:0.18
    });
    const dark=new THREE.MeshStandardMaterial({
      color:0x25303a,roughness:0.7
    });

    const part=(geometry,material,position)=>{
      const mesh=new THREE.Mesh(geometry,material);
      mesh.position.set(...position);
      group.add(mesh);
      return mesh;
    };

    part(new THREE.BoxGeometry(0.38,0.24,0.48),light,[0,0,0.93]);
    part(new THREE.BoxGeometry(0.26,0.25,0.23),dark,[0.02,0,1.31]);
    part(new THREE.BoxGeometry(0.30,0.22,0.20),dark,[0,0,0.64]);

    for(const side of [-1,1]){
      part(new THREE.BoxGeometry(0.11,0.10,0.54),light,[0,side*0.12,0.34]);
      part(new THREE.BoxGeometry(0.26,0.12,0.08),dark,[0.07,side*0.12,0.04]);
      part(new THREE.BoxGeometry(0.10,0.10,0.48),light,[0,side*0.25,0.88]);
    }

    const heading=new THREE.ArrowHelper(
      new THREE.Vector3(1,0,0),
      new THREE.Vector3(0,0,0.12),
      0.9,
      0xff5364,
      0.24,
      0.15
    );
    group.add(heading);
    group.visible=false;
    return group;
  }

  buildInitialPoseMarker(){
    const group=new THREE.Group();

    const ring=new THREE.Mesh(
      new THREE.TorusGeometry(0.34,0.045,8,40),
      new THREE.MeshBasicMaterial({
        color:0xffd84a,
        transparent:true,
        opacity:0.98,
        depthTest:false
      })
    );
    ring.renderOrder=20;
    group.add(ring);

    const arrow=new THREE.ArrowHelper(
      new THREE.Vector3(1,0,0),
      new THREE.Vector3(0,0,0.03),
      1.05,
      0xffd84a,
      0.27,
      0.18
    );
    arrow.line.material.depthTest=false;
    arrow.cone.material.depthTest=false;
    arrow.line.renderOrder=20;
    arrow.cone.renderOrder=20;
    group.add(arrow);

    group.visible=false;
    return group;
  }

  screenToGround(clientX,clientY){
    const bounds=this.canvas.getBoundingClientRect();
    if(bounds.width<=0||bounds.height<=0)return null;

    this.pointerNdc.set(
      ((clientX-bounds.left)/bounds.width)*2-1,
      -((clientY-bounds.top)/bounds.height)*2+1
    );
    this.raycaster.setFromCamera(this.pointerNdc,this.camera);

    const point=new THREE.Vector3();
    return this.raycaster.ray.intersectPlane(
      this.selectionPlane,
      point
    )?point:null;
  }

  bind(){
    this.canvas.addEventListener('contextmenu',event=>event.preventDefault());

    const move=event=>{
      if(!this.drag)return;
      if(event.pointerId!==this.drag.pointerId)return;

      if(this.drag.mode==='initial-pose'){
        const point=this.screenToGround(event.clientX,event.clientY);
        if(point){
          this.updateInitialPoseDraft(
            this.drag.initialStart,
            point
          );
        }
        event.preventDefault();
        return;
      }

      const dx=event.clientX-this.drag.x;
      const dy=event.clientY-this.drag.y;
      this.drag.x=event.clientX;
      this.drag.y=event.clientY;

      if(this.drag.mode==='orbit'){
        this.yaw-=dx*0.006;
        this.pitch=clamp(this.pitch+dy*0.005,0.10,1.48);
      }else{
        const scale=this.distance/650;
        this.target.x+=(
          Math.sin(this.yaw)*dx-Math.cos(this.yaw)*dy
        )*scale;
        this.target.y+=(
          -Math.cos(this.yaw)*dx-Math.sin(this.yaw)*dy
        )*scale;
      }

      event.preventDefault();
    };

    const finish=event=>{
      if(!this.drag)return;
      if(
        event
        && Number.isFinite(event.pointerId)
        && event.pointerId!==this.drag.pointerId
      ){
        return;
      }

      this.drag=null;
      this.canvas.classList.remove('dragging');
    };

    this.canvas.addEventListener('pointerdown',event=>{
      if(!this.visible||this.drag)return;
      if(event.button!==0&&event.button!==2)return;

      if(this.initialPoseSelecting&&event.button===0){
        const point=this.screenToGround(event.clientX,event.clientY);
        if(!point)return;

        this.drag={
          pointerId:event.pointerId,
          x:event.clientX,
          y:event.clientY,
          mode:'initial-pose',
          initialStart:point.clone()
        };
        this.updateInitialPoseDraft(point,point);
      }else{
        this.drag={
          pointerId:event.pointerId,
          x:event.clientX,
          y:event.clientY,
          mode:event.button===0?'pan':'orbit'
        };
      }

      this.canvas.classList.add('dragging');
      event.preventDefault();
    });

    // Track an active drag at window level. This keeps movement continuous
    // when the cursor crosses the canvas boundary or Firefox drops capture.
    window.addEventListener('pointermove',move,{
      capture:true,
      passive:false
    });
    window.addEventListener('pointerup',finish,true);
    window.addEventListener('pointercancel',finish,true);
    window.addEventListener('blur',()=>finish(null));

    this.canvas.addEventListener('wheel',event=>{
      if(!this.visible)return;
      this.distance=clamp(
        this.distance*Math.exp(event.deltaY*0.001),
        1.5,
        120
      );
      event.preventDefault();
    },{passive:false});

    this.canvas.addEventListener('dblclick',event=>{
      if(this.initialPoseSelecting){
        event.preventDefault();
        return;
      }
      this.resetView();
    });
  }

  resize(){
    const bounds=this.canvas.getBoundingClientRect();
    const width=Math.max(1,Math.round(bounds.width));
    const height=Math.max(1,Math.round(bounds.height));
    this.renderer.setSize(width,height,false);
    this.camera.aspect=width/height;
    this.camera.updateProjectionMatrix();
  }

  updateCamera(){
    const horizontal=this.distance*Math.cos(this.pitch);
    this.camera.position.set(
      this.target.x+horizontal*Math.cos(this.yaw),
      this.target.y+horizontal*Math.sin(this.yaw),
      this.target.z+this.distance*Math.sin(this.pitch)
    );
    this.camera.lookAt(this.target);
  }

  loop(){
    if(this.visible){
      this.updateCamera();
      this.renderer.render(this.scene,this.camera);
    }
    requestAnimationFrame(()=>this.loop());
  }

  setVisible(value){
    this.visible=Boolean(value);
    if(this.visible)this.resize();
  }

  resetView(){
    this.target.copy(this.fitView.target);
    this.yaw=this.fitView.yaw;
    this.pitch=this.fitView.pitch;
    this.distance=this.fitView.distance;
  }

  updateVisibility(){
    this.mapPoints.visible=this.mapEnabled;
    this.livePoints.visible=this.liveEnabled&&this.liveHasCloud;
    this.robot.visible=this.robotEnabled&&this.robotHasPose;
  }

  setMapEnabled(value){
    this.mapEnabled=Boolean(value);
    this.updateVisibility();
  }

  setLiveEnabled(value){
    this.liveEnabled=Boolean(value);
    this.updateVisibility();
  }

  setRobotEnabled(value){
    this.robotEnabled=Boolean(value);
    this.updateVisibility();
  }

  setLiveFresh(value){
    this.liveFresh=Boolean(value);
    this.liveMaterial.opacity=this.liveFresh?0.95:0.30;
    this.updateVisibility();
  }

  floorHeight(x,y){
    return -0.092484490*x+0.025026605*y-1.235811371;
  }

  levelPoint(x,y,z,target){
    if(!this.floorRotation){
      const floorNormal=new THREE.Vector3(
        0.092484490,
        -0.025026605,
        1
      ).normalize();

      this.floorRotation=new THREE.Quaternion().setFromUnitVectors(
        floorNormal,
        new THREE.Vector3(0,0,1)
      );
      this.floorOriginZ=-1.235811371;
    }

    return target
      .set(x,y,z-this.floorOriginZ)
      .applyQuaternion(this.floorRotation);
  }

  unlevelPoint(point,target){
    if(!this.floorRotation){
      this.levelPoint(
        0,
        0,
        this.floorHeight(0,0),
        new THREE.Vector3()
      );
    }

    target.copy(point);
    target.applyQuaternion(
      this.floorRotation.clone().invert()
    );
    target.z+=this.floorOriginZ;
    return target;
  }

  updateInitialPoseDraft(startVisual,endVisual){
    const startNative=this.unlevelPoint(
      startVisual,
      new THREE.Vector3()
    );
    const endNative=this.unlevelPoint(
      endVisual,
      new THREE.Vector3()
    );

    const nativeDx=endNative.x-startNative.x;
    const nativeDy=endNative.y-startNative.y;
    const nativeLength=Math.hypot(nativeDx,nativeDy);
    const yaw=nativeLength>0.02?
      Math.atan2(nativeDy,nativeDx):
      Number(this.initialPoseDraft?.yaw||0);

    const visualDx=endVisual.x-startVisual.x;
    const visualDy=endVisual.y-startVisual.y;
    const visualLength=Math.hypot(visualDx,visualDy);
    const visualYaw=visualLength>0.02?
      Math.atan2(visualDy,visualDx):
      0;

    this.initialPoseDraft={
      x:startNative.x,
      y:startNative.y,
      yaw
    };

    this.initialPoseMarker.position.set(
      startVisual.x,
      startVisual.y,
      0.06
    );
    this.initialPoseMarker.rotation.z=visualYaw;
    this.initialPoseMarker.visible=true;

    if(typeof this.initialPoseCallback==='function'){
      this.initialPoseCallback({...this.initialPoseDraft});
    }
  }

  beginInitialPoseSelection(callback){
    this.initialPoseSelecting=true;
    this.initialPoseDraft=null;
    this.initialPoseCallback=
      typeof callback==='function'?callback:null;
    this.initialPoseMarker.visible=false;
    this.canvas.classList.add('selecting-pose');
  }

  finishInitialPoseSelection(keepMarker=true){
    this.initialPoseSelecting=false;
    this.initialPoseCallback=null;
    this.drag=null;
    this.canvas.classList.remove('selecting-pose','dragging');
    if(!keepMarker)this.initialPoseMarker.visible=false;
  }

  clearInitialPoseSelection(){
    this.initialPoseDraft=null;
    this.finishInitialPoseSelection(false);
  }

  setRobotPose(pose,localized){
    this.robotLocalized=Boolean(localized);
    const valid=Boolean(
      pose
      && Number.isFinite(Number(pose.x))
      && Number.isFinite(Number(pose.y))
      && Number.isFinite(Number(pose.yaw))
    );
    this.robotHasPose=valid;
    if(valid){
      const x=Number(pose.x);
      const y=Number(pose.y);
      const yaw=Number(pose.yaw);

      const robotPosition=this.levelPoint(
        x,
        y,
        this.floorHeight(x,y),
        new THREE.Vector3()
      );

      const headingX=x+Math.cos(yaw);
      const headingY=y+Math.sin(yaw);
      const robotHeading=this.levelPoint(
        headingX,
        headingY,
        this.floorHeight(headingX,headingY),
        new THREE.Vector3()
      ).sub(robotPosition);

      this.robot.position.copy(robotPosition);
      this.robot.rotation.z=Math.atan2(
        robotHeading.y,
        robotHeading.x
      );
    }
    this.updateVisibility();
  }

  loadAsciiPcd(text){
    const lines=String(text).split(/\r?\n/);
    let fields=[];
    let dataIndex=-1;

    for(let index=0;index<lines.length;index++){
      const line=lines[index].trim();
      if(line.toUpperCase().startsWith('FIELDS ')){
        fields=line.split(/\s+/).slice(1);
      }
      if(line.toUpperCase()==='DATA ASCII'){
        dataIndex=index+1;
        break;
      }
    }

    const xIndex=fields.indexOf('x');
    const yIndex=fields.indexOf('y');
    const zIndex=fields.indexOf('z');
    if(dataIndex<0||xIndex<0||yIndex<0||zIndex<0){
      throw new Error('Expected an ASCII PCD containing x, y and z');
    }

    const positions=[];
    const colors=[];
    const leveled=new THREE.Vector3();
    const heightColor=new THREE.Color();
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;

    for(let index=dataIndex;index<lines.length;index++){
      const parts=lines[index].trim().split(/\s+/);
      if(parts.length<fields.length)continue;

      const x=Number(parts[xIndex]);
      const y=Number(parts[yIndex]);
      const z=Number(parts[zIndex]);
      if(![x,y,z].every(Number.isFinite))continue;

      // Keep every finite point from the saved PCD. This map is not
      // level in world Z, so fixed Z clipping removes valid lab geometry.

      this.levelPoint(x,y,z,leveled);

      positions.push(leveled.x,leveled.y,leveled.z);
      minX=Math.min(minX,leveled.x);
      maxX=Math.max(maxX,leveled.x);
      minY=Math.min(minY,leveled.y);
      maxY=Math.max(maxY,leveled.y);

      // Violet floor -> cyan -> green -> yellow -> red ceiling.
      const height=clamp((leveled.z+0.10)/2.60,0,1);
      heightColor.setHSL(
        0.72-height*0.72,
        0.88,
        0.60
      );
      colors.push(
        heightColor.r,
        heightColor.g,
        heightColor.b
      );
    }

    if(!positions.length)throw new Error('The PCD has no visible points');

    this.mapGeometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(positions,3)
    );
    this.mapGeometry.setAttribute(
      'color',
      new THREE.Float32BufferAttribute(colors,3)
    );
    this.mapGeometry.computeBoundingSphere();

    const centerX=(minX+maxX)/2;
    const centerY=(minY+maxY)/2;
    const span=Math.max(maxX-minX,maxY-minY,5);

    if(this.grid){
      this.scene.remove(this.grid);
      this.grid.geometry.dispose();
      this.grid.material.dispose();
    }

    this.grid=new THREE.GridHelper(
      Math.ceil(span*1.25),
      Math.max(10,Math.ceil(span)),
      0x31516d,
      0x172a3a
    );
    this.grid.rotation.x=Math.PI/2;
    this.grid.position.set(centerX,centerY,-0.03);
    this.grid.material.transparent=true;
    this.grid.material.opacity=0.45;
    this.scene.add(this.grid);

    this.fitView={
      target:new THREE.Vector3(centerX,centerY,0.65),
      yaw:-0.72,
      pitch:0.86,
      distance:Math.max(10,span*0.95)
    };
    this.resetView();

    return {
      points:positions.length/3,
      spanX:maxX-minX,
      spanY:maxY-minY
    };
  }

  setLiveCloud(buffer){
    if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<12||buffer.byteLength%12){
      return 0;
    }
    const source=new Float32Array(buffer);
    const copy=new Float32Array(source.length);
    const leveled=new THREE.Vector3();

    for(let index=0;index<source.length;index+=3){
      this.levelPoint(
        source[index],
        source[index+1],
        source[index+2],
        leveled
      );
      copy[index]=leveled.x;
      copy[index+1]=leveled.y;
      copy[index+2]=leveled.z;
    }

    this.liveGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(copy,3)
    );
    this.liveGeometry.computeBoundingSphere();
    this.liveHasCloud=true;
    this.updateVisibility();
    return copy.length/3;
  }
}

let viewer=null;

export const SlamWorld3D={
  init(canvas){
    if(!viewer)viewer=new SlamViewer(canvas);
    return viewer;
  }
};
