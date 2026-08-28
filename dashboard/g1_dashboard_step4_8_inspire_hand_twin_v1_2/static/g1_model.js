/*
 * G1 mesh digital twin — Step 4.2.
 *
 * Kinematic origins/axes are retained from the validated rev_1_0 telemetry
 * renderer. Visual geometry is loaded from Unitree's official G1 STL mesh set.
 * Three.js r180 and STLLoader are vendored locally under static/vendor.
 */
import * as THREE from './vendor/three.module.min.js';
import { STLLoader } from './vendor/STLLoader.js';
import { INSPIRE_HAND, INSPIRE_MESH_BASE, INSPIRE_MESH_COUNT, normalizedHandToJointMap } from './inspire_hand_model.js';

const G1_KINEMATIC_JOINTS = [{"name":"pelvis_contour_joint","type":"fixed","parent":"pelvis","child":"pelvis_contour_link","xyz":[0,0,0],"rpy":[0,0,0],"axis":[0,0,1],"index":null},{"name":"left_hip_pitch_joint","type":"revolute","parent":"pelvis","child":"left_hip_pitch_link","xyz":[0.0,0.064452,-0.1027],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":0},{"name":"left_hip_roll_joint","type":"revolute","parent":"left_hip_pitch_link","child":"left_hip_roll_link","xyz":[0.0,0.052,-0.030465],"rpy":[0.0,-0.1749,0.0],"axis":[1.0,0.0,0.0],"index":1},{"name":"left_hip_yaw_joint","type":"revolute","parent":"left_hip_roll_link","child":"left_hip_yaw_link","xyz":[0.025001,0.0,-0.12412],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":2},{"name":"left_knee_joint","type":"revolute","parent":"left_hip_yaw_link","child":"left_knee_link","xyz":[-0.078273,0.0021489,-0.17734],"rpy":[0.0,0.1749,0.0],"axis":[0.0,1.0,0.0],"index":3},{"name":"left_ankle_pitch_joint","type":"revolute","parent":"left_knee_link","child":"left_ankle_pitch_link","xyz":[0.0,-9.4445e-05,-0.30001],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":4},{"name":"left_ankle_roll_joint","type":"revolute","parent":"left_ankle_pitch_link","child":"left_ankle_roll_link","xyz":[0.0,0.0,-0.017558],"rpy":[0.0,0.0,0.0],"axis":[1.0,0.0,0.0],"index":5},{"name":"right_hip_pitch_joint","type":"revolute","parent":"pelvis","child":"right_hip_pitch_link","xyz":[0.0,-0.064452,-0.1027],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":6},{"name":"right_hip_roll_joint","type":"revolute","parent":"right_hip_pitch_link","child":"right_hip_roll_link","xyz":[0.0,-0.052,-0.030465],"rpy":[0.0,-0.1749,0.0],"axis":[1.0,0.0,0.0],"index":7},{"name":"right_hip_yaw_joint","type":"revolute","parent":"right_hip_roll_link","child":"right_hip_yaw_link","xyz":[0.025001,0.0,-0.12412],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":8},{"name":"right_knee_joint","type":"revolute","parent":"right_hip_yaw_link","child":"right_knee_link","xyz":[-0.078273,-0.0021489,-0.17734],"rpy":[0.0,0.1749,0.0],"axis":[0.0,1.0,0.0],"index":9},{"name":"right_ankle_pitch_joint","type":"revolute","parent":"right_knee_link","child":"right_ankle_pitch_link","xyz":[0.0,9.4445e-05,-0.30001],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":10},{"name":"right_ankle_roll_joint","type":"revolute","parent":"right_ankle_pitch_link","child":"right_ankle_roll_link","xyz":[0.0,0.0,-0.017558],"rpy":[0.0,0.0,0.0],"axis":[1.0,0.0,0.0],"index":11},{"name":"waist_yaw_joint","type":"revolute","parent":"pelvis","child":"waist_yaw_link","xyz":[0.0,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":12},{"name":"waist_roll_joint","type":"revolute","parent":"waist_yaw_link","child":"waist_roll_link","xyz":[-0.0039635,0.0,0.044],"rpy":[0.0,0.0,0.0],"axis":[1.0,0.0,0.0],"index":13},{"name":"waist_pitch_joint","type":"revolute","parent":"waist_roll_link","child":"torso_link","xyz":[0.0,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":14},{"name":"logo_joint","type":"fixed","parent":"torso_link","child":"logo_link","xyz":[0.0039635,0.0,-0.044],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null},{"name":"head_joint","type":"fixed","parent":"torso_link","child":"head_link","xyz":[0.0039635,0.0,-0.044],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null},{"name":"imu_in_torso_joint","type":"fixed","parent":"torso_link","child":"imu_in_torso","xyz":[-0.03959,-0.00224,0.14792],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null},{"name":"imu_in_pelvis_joint","type":"fixed","parent":"pelvis","child":"imu_in_pelvis","xyz":[0.04525,0.0,-0.08339],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null},{"name":"d435_joint","type":"fixed","parent":"torso_link","child":"d435_link","xyz":[0.0576235,0.01753,0.42987],"rpy":[0.0,0.8307767239493009,0.0],"axis":[0,0,1],"index":null},{"name":"mid360_joint","type":"fixed","parent":"torso_link","child":"mid360_link","xyz":[0.0002835,3e-05,0.428434],"rpy":[3.141592653589793,0.05112069379091391,0.0],"axis":[0,0,1],"index":null},{"name":"left_shoulder_pitch_joint","type":"revolute","parent":"torso_link","child":"left_shoulder_pitch_link","xyz":[0.0039563,0.10022,0.24778],"rpy":[0.27931,5.4949e-05,-0.00019159],"axis":[0.0,1.0,0.0],"index":15},{"name":"left_shoulder_roll_joint","type":"revolute","parent":"left_shoulder_pitch_link","child":"left_shoulder_roll_link","xyz":[0.0,0.038,-0.013831],"rpy":[-0.27925,0.0,0.0],"axis":[1.0,0.0,0.0],"index":16},{"name":"left_shoulder_yaw_joint","type":"revolute","parent":"left_shoulder_roll_link","child":"left_shoulder_yaw_link","xyz":[0.0,0.00624,-0.1032],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":17},{"name":"left_elbow_joint","type":"revolute","parent":"left_shoulder_yaw_link","child":"left_elbow_link","xyz":[0.015783,0.0,-0.080518],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":18},{"name":"left_wrist_roll_joint","type":"revolute","parent":"left_elbow_link","child":"left_wrist_roll_link","xyz":[0.1,0.00188791,-0.01],"rpy":[0.0,0.0,0.0],"axis":[1.0,0.0,0.0],"index":19},{"name":"left_wrist_pitch_joint","type":"revolute","parent":"left_wrist_roll_link","child":"left_wrist_pitch_link","xyz":[0.038,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":20},{"name":"left_wrist_yaw_joint","type":"revolute","parent":"left_wrist_pitch_link","child":"left_wrist_yaw_link","xyz":[0.046,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":21},{"name":"left_hand_palm_joint","type":"fixed","parent":"left_wrist_yaw_link","child":"left_rubber_hand","xyz":[0.0415,0.003,0.0],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null},{"name":"right_shoulder_pitch_joint","type":"revolute","parent":"torso_link","child":"right_shoulder_pitch_link","xyz":[0.0039563,-0.10021,0.24778],"rpy":[-0.27931,5.4949e-05,0.00019159],"axis":[0.0,1.0,0.0],"index":22},{"name":"right_shoulder_roll_joint","type":"revolute","parent":"right_shoulder_pitch_link","child":"right_shoulder_roll_link","xyz":[0.0,-0.038,-0.013831],"rpy":[0.27925,0.0,0.0],"axis":[1.0,0.0,0.0],"index":23},{"name":"right_shoulder_yaw_joint","type":"revolute","parent":"right_shoulder_roll_link","child":"right_shoulder_yaw_link","xyz":[0.0,-0.00624,-0.1032],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":24},{"name":"right_elbow_joint","type":"revolute","parent":"right_shoulder_yaw_link","child":"right_elbow_link","xyz":[0.015783,0.0,-0.080518],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":25},{"name":"right_wrist_roll_joint","type":"revolute","parent":"right_elbow_link","child":"right_wrist_roll_link","xyz":[0.1,-0.00188791,-0.01],"rpy":[0.0,0.0,0.0],"axis":[1.0,0.0,0.0],"index":26},{"name":"right_wrist_pitch_joint","type":"revolute","parent":"right_wrist_roll_link","child":"right_wrist_pitch_link","xyz":[0.038,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,1.0,0.0],"index":27},{"name":"right_wrist_yaw_joint","type":"revolute","parent":"right_wrist_pitch_link","child":"right_wrist_yaw_link","xyz":[0.046,0.0,0.0],"rpy":[0.0,0.0,0.0],"axis":[0.0,0.0,1.0],"index":28},{"name":"right_hand_palm_joint","type":"fixed","parent":"right_wrist_yaw_link","child":"right_rubber_hand","xyz":[0.0415,-0.003,0.0],"rpy":[0.0,0.0,0.0],"axis":[0,0,1],"index":null}];

const G1_TELEMETRY_NAMES = [
  'L_hip_pitch','L_hip_roll','L_hip_yaw','L_knee','L_ankle_pitch','L_ankle_roll',
  'R_hip_pitch','R_hip_roll','R_hip_yaw','R_knee','R_ankle_pitch','R_ankle_roll',
  'waist_yaw','waist_roll','waist_pitch',
  'L_shoulder_pitch','L_shoulder_roll','L_shoulder_yaw','L_elbow','L_wrist_roll','L_wrist_pitch','L_wrist_yaw',
  'R_shoulder_pitch','R_shoulder_roll','R_shoulder_yaw','R_elbow','R_wrist_roll','R_wrist_pitch','R_wrist_yaw'
];

const MODEL_BASE = '/static/model/g1/meshes/';
const MESH_NAMES = ['pelvis', 'pelvis_contour_link', 'left_hip_pitch_link', 'left_hip_roll_link', 'left_hip_yaw_link', 'left_knee_link', 'left_ankle_pitch_link', 'left_ankle_roll_link', 'right_hip_pitch_link', 'right_hip_roll_link', 'right_hip_yaw_link', 'right_knee_link', 'right_ankle_pitch_link', 'right_ankle_roll_link', 'waist_yaw_link', 'waist_roll_link', 'torso_link', 'logo_link', 'head_link', 'waist_support_link', 'left_shoulder_pitch_link', 'left_shoulder_roll_link', 'left_shoulder_yaw_link', 'left_elbow_link', 'left_wrist_roll_link', 'left_wrist_pitch_link', 'left_wrist_yaw_link', 'left_rubber_hand', 'right_shoulder_pitch_link', 'right_shoulder_roll_link', 'right_shoulder_yaw_link', 'right_elbow_link', 'right_wrist_roll_link', 'right_wrist_pitch_link', 'right_wrist_yaw_link', 'right_rubber_hand'];
const ARM_MESHES = new Set(MESH_NAMES.filter(n => n.startsWith('left_shoulder') || n.startsWith('left_elbow') || n.startsWith('left_wrist') || n === 'left_rubber_hand' || n.startsWith('right_shoulder') || n.startsWith('right_elbow') || n.startsWith('right_wrist') || n === 'right_rubber_hand'));
const DARK_MESHES = new Set(['pelvis','left_hip_pitch_link','right_hip_pitch_link','left_ankle_roll_link','right_ankle_roll_link','logo_link','head_link','left_rubber_hand','right_rubber_hand']);
const CHILD_TO_JOINT = new Map(G1_KINEMATIC_JOINTS.filter(j => j.index !== null && j.index !== undefined).map(j => [j.child, j.index]));
const JOINT_BY_INDEX = new Map(G1_KINEMATIC_JOINTS.filter(j => j.index !== null && j.index !== undefined).map(j => [j.index, j]));

// Official rev_1_0 URDF effort attributes, in telemetry index order (N·m).
// These are used only to normalize the diagnostic torque overlay; they are not
// treated as controller safety limits.
const G1_EFFORT_LIMIT_NM = [
  88,139,88,139,35,35,
  88,139,88,139,35,35,
  88,35,35,
  25,25,25,25,25,5,5,
  25,25,25,25,25,5,5
];
const HEALTH_COLORS = [0x33d39a,0xf1c75b,0xff934d,0xff5364];
const HEALTH_LABELS = ['NOMINAL','WATCH','HIGH','VERY HIGH'];
function maxTempValue(v){
  const xs=Array.isArray(v)?v:[v]; const nums=xs.map(Number).filter(Number.isFinite);
  return nums.length?Math.max(...nums):null;
}
function levelFromTemp(c){
  if(!Number.isFinite(c))return null;
  // Dashboard visualization bands only — not Unitree safety thresholds.
  if(c>=90)return 3; if(c>=80)return 2; if(c>=70)return 1; return 0;
}
function levelFromTorqueUtil(u){
  if(!Number.isFinite(u))return null;
  if(u>=0.90)return 3; if(u>=0.75)return 2; if(u>=0.50)return 1; return 0;
}

function ident(){return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];}
function mul(a,b){const c=new Array(16).fill(0);for(let r=0;r<4;r++)for(let q=0;q<4;q++)for(let k=0;k<4;k++)c[r*4+q]+=a[r*4+k]*b[k*4+q];return c;}
function trans(v){const m=ident();m[3]=v[0];m[7]=v[1];m[11]=v[2];return m;}
function rx(a){const c=Math.cos(a),s=Math.sin(a);return [1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1];}
function ry(a){const c=Math.cos(a),s=Math.sin(a);return [c,0,s,0, 0,1,0,0, -s,0,c,0, 0,0,0,1];}
function rz(a){const c=Math.cos(a),s=Math.sin(a);return [c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1];}
function rpy(v){return mul(mul(rz(v[2]),ry(v[1])),rx(v[0]));}
function axisAngle(axis,a){let [x,y,z]=axis;const n=Math.hypot(x,y,z)||1;x/=n;y/=n;z/=n;const c=Math.cos(a),s=Math.sin(a),t=1-c;return [t*x*x+c,t*x*y-s*z,t*x*z+s*y,0, t*x*y+s*z,t*y*y+c,t*y*z-s*x,0, t*x*z-s*y,t*y*z+s*x,t*z*z+c,0, 0,0,0,1];}
function tp(m,p){return [m[0]*p[0]+m[1]*p[1]+m[2]*p[2]+m[3],m[4]*p[0]+m[5]*p[1]+m[6]*p[2]+m[7],m[8]*p[0]+m[9]*p[1]+m[10]*p[2]+m[11]];}
function finite(x){return Number.isFinite(Number(x));}
function arr29(v){return Array.isArray(v)&&v.length>=29?v.slice(0,29).map(x=>finite(x)?Number(x):0):new Array(29).fill(0);}
function arr14(v){return Array.isArray(v)&&v.length>=14?v.slice(0,14).map(x=>finite(x)?Number(x):0):null;}
function arr6(v){return Array.isArray(v)&&v.length>=6?v.slice(0,6).map(x=>finite(x)?Math.max(0,Math.min(1,Number(x))):1):null;}
function validatedFeedback6(v){
  if(!Array.isArray(v)||v.length<6)return null;
  const out=v.slice(0,6).map(Number);
  if(!out.every(x=>Number.isFinite(x)&&x>=-0.02&&x<=1.02))return null;
  return out.map(x=>Math.max(0,Math.min(1,x)));
}
function forward(q){
  const links={pelvis:trans([0,0,0.793])}, jointFrames={}, jointPoints={};
  let pending=G1_KINEMATIC_JOINTS.slice();
  for(let pass=0;pass<10 && pending.length;pass++){
    const next=[];
    for(const j of pending){
      const parent=links[j.parent]; if(!parent){next.push(j);continue;}
      const origin=mul(trans(j.xyz),rpy(j.rpy)); const jf=mul(parent,origin);
      jointFrames[j.name]=jf; jointPoints[j.name]=tp(jf,[0,0,0]);
      let child=jf;
      if(j.index!==null && j.index!==undefined) child=mul(jf,axisAngle(j.axis,finite(q[j.index])?Number(q[j.index]):0));
      links[j.child]=child;
    }
    pending=next;
  }
  // rev_1_0 visual support shell is fixed to the same torso offset as head/logo.
  if(!links.waist_support_link && links.logo_link) links.waist_support_link=links.logo_link.slice();
  return {links,jointFrames,jointPoints};
}

function handForward(side,wristFrame,normalized){
  const h=INSPIRE_HAND[side];
  if(!h||!wristFrame)return {links:{},jointFrames:{}};
  const mount=mul(wristFrame,mul(trans(h.mount.xyz),rpy(h.mount.rpy)));
  const links={[h.root_link]:mount}, jointFrames={};
  const controls=normalizedHandToJointMap(side,normalized);
  let pending=h.joints.slice();
  for(let pass=0;pass<10&&pending.length;pass++){
    const next=[];
    for(const j of pending){
      const parent=links[j.parent]; if(!parent){next.push(j);continue;}
      const origin=mul(trans(j.xyz),rpy(j.rpy)); const jf=mul(parent,origin); jointFrames[j.name]=jf;
      let child=jf;
      if(j.type==='revolute'){
        let q=controls[j.name];
        if(j.mimic){
          const src=controls[j.mimic.joint];
          q=(finite(src)?Number(src):0)*Number(j.mimic.multiplier??1)+Number(j.mimic.offset??0);
        }
        child=mul(jf,axisAngle(j.axis,finite(q)?Number(q):0));
      }
      links[j.child]=child;
    }
    pending=next;
  }
  return {links,jointFrames};
}
function applyMatrix(obj,m){obj.matrix.set(m[0],m[1],m[2],m[3], m[4],m[5],m[6],m[7], m[8],m[9],m[10],m[11], m[12],m[13],m[14],m[15]); obj.matrixWorldNeedsUpdate=true;}
function stepAngle(cur,target,alpha){let d=target-cur;while(d>Math.PI)d-=Math.PI*2;while(d<-Math.PI)d+=Math.PI*2;return cur+d*alpha;}

class MeshTwin {
  constructor(canvas,onSelect){
    this.canvas=canvas; this.onSelect=onSelect||null; this.selected=18; this.showGhost=true; this.showJoints=false; this.showHealth=true; this.healthMode='composite';
    this.targetQ=new Array(29).fill(0); this.displayQ=new Array(29).fill(0); this.targetPub=null; this.displayPub=new Array(14).fill(0); this.havePose=false;
    this.targetHandCmdLeft=new Array(6).fill(1); this.targetHandCmdRight=new Array(6).fill(1); this.displayHandCmdLeft=new Array(6).fill(1); this.displayHandCmdRight=new Array(6).fill(1);
    this.targetHandFbLeft=new Array(6).fill(1); this.targetHandFbRight=new Array(6).fill(1); this.displayHandFbLeft=new Array(6).fill(1); this.displayHandFbRight=new Array(6).fill(1); this.haveHandPose=false; this.handFeedbackValid={left:false,right:false};
    this.sequence=null; this.sourceAgeMs=null; this.packetAgeMs=null; this.rxHz=0; this.fps=0; this.lastRxPerf=null; this.rxIntervals=[]; this.frameCount=0; this.fpsStamp=performance.now();
    this.loaded=0; this.failed=[]; this.inspireLoaded=0; this.inspireFailed=[]; this.inspireReady=false; this.modelStatus='loading G1 + Inspire RH56DFX meshes';
    this.measuredMeshes=new Map(); this.ghostMeshes=new Map(); this.inspireMeasuredMeshes={left:new Map(),right:new Map()}; this.inspireGhostMeshes={left:new Map(),right:new Map()}; this.pickMeshes=[]; this.markerMeshes=[]; this.lastFk=null; this.sceneReplicas=new Set(); this.jointHealth=new Array(29).fill(null); this.healthSummary={label:'NO DATA',severity:null,watchCount:0,highCount:0,veryHighCount:0,peakTempC:null,peakTempIndex:null,peakTorqueUtil:null,peakTorqueIndex:null};
    this.scene=new THREE.Scene(); this.scene.background=new THREE.Color(0x071018);
    this.camera=new THREE.PerspectiveCamera(34,1,0.02,20); this.camera.up.set(0,0,1);
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,powerPreference:'high-performance'}); this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.5)); this.renderer.outputColorSpace=THREE.SRGBColorSpace;
    this.target=new THREE.Vector3(0,0,0.80); this.yaw=-0.78; this.pitch=0.18; this.distance=2.28;
    this.raycaster=new THREE.Raycaster(); this.pointer=new THREE.Vector2(); this.drag=null;
    this.setupScene(); this.bind(); this.resizeObserver=new ResizeObserver(()=>this.resize()); this.resizeObserver.observe(canvas); this.resize(); this.loadMeshes(); this.loadInspireMeshes();
    this.lastFrame=performance.now(); requestAnimationFrame(t=>this.loop(t));
  }
  setupScene(){
    this.scene.add(new THREE.HemisphereLight(0xcfe9ff,0x17202a,2.1));
    const key=new THREE.DirectionalLight(0xffffff,3.0); key.position.set(2.8,-2.2,4.5); this.scene.add(key);
    const fill=new THREE.DirectionalLight(0x76bfff,1.1); fill.position.set(-2.0,2.5,2.0); this.scene.add(fill);
    const grid=new THREE.GridHelper(1.8,18,0x274158,0x152738); grid.rotation.x=Math.PI/2; grid.position.z=0; grid.material.transparent=true; grid.material.opacity=0.5; this.scene.add(grid);
    const jointGeo=new THREE.SphereGeometry(0.0165,12,10);
    for(let i=0;i<29;i++){const mat=new THREE.MeshBasicMaterial({color:i===this.selected?0xb9f3ff:0x63b9e8,transparent:true,opacity:.92,depthTest:false,depthWrite:false});const s=new THREE.Mesh(jointGeo,mat);s.userData.jointIndex=i;s.visible=this.showJoints||i===this.selected;s.renderOrder=8;this.markerMeshes[i]=s;this.scene.add(s);}
  }
  materialFor(name){
    const dark=DARK_MESHES.has(name); return new THREE.MeshStandardMaterial({color:dark?0x26313b:0xaab4bd,roughness:.72,metalness:.12,emissive:0x000000});
  }
  ghostMaterial(){return new THREE.MeshStandardMaterial({color:0x40cfff,roughness:.5,metalness:.08,transparent:true,opacity:.22,depthWrite:false,side:THREE.DoubleSide});}
  inspireGhostMaterial(){return new THREE.MeshStandardMaterial({color:0x40cfff,roughness:.42,metalness:.05,transparent:true,opacity:.34,depthWrite:false,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:-2,polygonOffsetUnits:-2});}
  inspireMaterial(){return new THREE.MeshStandardMaterial({color:0x35414c,roughness:.70,metalness:.10,emissive:0x000000});}

  createSceneReplica(){
    const root=new THREE.Group();
    root.name='G1 SLAM mesh replica';

    const replica={
      root,
      meshes:new Map()
    };

    this.sceneReplicas.add(replica);

    for(const [name,source] of this.measuredMeshes){
      this.addSceneReplicaMesh(
        replica,
        name,
        source
      );
    }

    this.syncSceneReplicas(
      this.lastFk||forward(this.displayQ)
    );

    return root;
  }

  addSceneReplicaMesh(replica,name,source){
    if(replica.meshes.has(name))return;

    const material=source.material.clone();
    if(material.emissive){
      material.emissive.setHex(0x000000);
    }

    const mesh=new THREE.Mesh(
      source.geometry,
      material
    );

    mesh.name=`slam_${name}`;
    mesh.matrixAutoUpdate=false;
    mesh.castShadow=false;
    mesh.receiveShadow=false;

    replica.meshes.set(name,mesh);
    replica.root.add(mesh);
  }

  syncSceneReplicas(fk){
    if(!fk)return;

    for(const replica of this.sceneReplicas){
      for(const [name,mesh] of replica.meshes){
        const matrix=fk.links[name];
        mesh.visible=Boolean(matrix);
        if(matrix)applyMatrix(mesh,matrix);
      }
    }
  }
  refreshModelStatus(){
    this.inspireReady=this.inspireLoaded===INSPIRE_MESH_COUNT;
    const bodyReady=this.loaded===MESH_NAMES.length;
    if(bodyReady&&this.inspireReady)this.modelStatus='official G1 + Inspire RH56DFX meshes ready';
    else if(this.loaded===0&&this.inspireLoaded===0)this.modelStatus='model assets missing';
    else this.modelStatus=`G1 ${this.loaded}/${MESH_NAMES.length} · Inspire ${this.inspireLoaded}/${INSPIRE_MESH_COUNT}`;
  }
  async loadMeshes(){
    const loader=new STLLoader(); const tasks=MESH_NAMES.map(name=>new Promise(resolve=>{
      loader.load(`${MODEL_BASE}${name}.STL`,geo=>{
        geo.computeVertexNormals(); const measured=new THREE.Mesh(geo,this.materialFor(name)); measured.matrixAutoUpdate=false; measured.userData.linkName=name; measured.userData.jointIndex=CHILD_TO_JOINT.has(name)?CHILD_TO_JOINT.get(name):null; this.measuredMeshes.set(name,measured); this.scene.add(measured); for(const replica of this.sceneReplicas)this.addSceneReplicaMesh(replica,name,measured); this.syncSceneReplicas(this.lastFk||forward(this.displayQ)); if(measured.userData.jointIndex!==null)this.pickMeshes.push(measured);
        if(ARM_MESHES.has(name)){const ghost=new THREE.Mesh(geo,this.ghostMaterial());ghost.matrixAutoUpdate=false;ghost.renderOrder=3;ghost.visible=this.showGhost;this.ghostMeshes.set(name,ghost);this.scene.add(ghost);}
        this.loaded++; this.modelStatus=`loading official G1 meshes ${this.loaded}/${MESH_NAMES.length}`; resolve(true);
      },undefined,()=>{this.failed.push(name);resolve(false);});
    }));
    await Promise.all(tasks);
    this.refreshModelStatus();
    this.applyHealthVisuals();
  }
  async loadInspireMeshes(){
    const loader=new STLLoader(); const tasks=[];
    for(const side of ['left','right']){
      const h=INSPIRE_HAND[side];
      for(const [link,file] of Object.entries(h.meshes)){
        tasks.push(new Promise(resolve=>{
          loader.load(`${INSPIRE_MESH_BASE}${file}`,geo=>{
            geo.computeVertexNormals();
            const measured=new THREE.Mesh(geo,this.inspireMaterial()); measured.matrixAutoUpdate=false; measured.visible=false; measured.userData.inspireSide=side; measured.userData.inspireLink=link; this.inspireMeasuredMeshes[side].set(link,measured); this.scene.add(measured);
            const ghost=new THREE.Mesh(geo,this.inspireGhostMaterial()); ghost.matrixAutoUpdate=false; ghost.renderOrder=4; ghost.visible=false; this.inspireGhostMeshes[side].set(link,ghost); this.scene.add(ghost);
            this.inspireLoaded++; this.refreshModelStatus(); resolve(true);
          },undefined,()=>{this.inspireFailed.push(`${side}:${file}`);this.refreshModelStatus();resolve(false);});
        }));
      }
    }
    await Promise.all(tasks); this.refreshModelStatus();
  }
  bind(){
    this.canvas.addEventListener('contextmenu',e=>e.preventDefault());
    this.canvas.addEventListener('pointerdown',e=>{this.drag={x:e.clientX,y:e.clientY,moved:false};this.canvas.setPointerCapture(e.pointerId);});
    this.canvas.addEventListener('pointermove',e=>{if(!this.drag)return;const dx=e.clientX-this.drag.x,dy=e.clientY-this.drag.y;if(Math.abs(dx)+Math.abs(dy)>2)this.drag.moved=true;this.yaw-=dx*.007;this.pitch=Math.max(-.65,Math.min(.72,this.pitch+dy*.006));this.drag.x=e.clientX;this.drag.y=e.clientY;});
    this.canvas.addEventListener('pointerup',e=>{if(this.drag&&!this.drag.moved)this.pickAt(e);this.drag=null;});
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(1.15,Math.min(4.4,this.distance*Math.exp(e.deltaY*.001)));},{passive:false});
    this.canvas.addEventListener('dblclick',()=>this.resetView());
  }
  resize(){const r=this.canvas.getBoundingClientRect();const w=Math.max(1,Math.round(r.width)),h=Math.max(1,Math.round(r.height));this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}
  resetView(){this.target.set(0,0,.80);this.yaw=-.78;this.pitch=.18;this.distance=2.28;}
  setPose(p){
    const q=arr29(p?.robot?.measured_q_rad); if(!Array.isArray(p?.robot?.measured_q_rad)||p.robot.measured_q_rad.length<29)return;
    const pub=arr14(p?.arms?.published_q_rad); const now=performance.now();
    if(!this.havePose){this.displayQ=q.slice();if(pub)this.displayPub=pub.slice();this.havePose=true;}
    this.targetQ=q; if(pub)this.targetPub=pub;
    const hcL=arr6(p?.hands?.current_left), hcR=arr6(p?.hands?.current_right), fb=Array.isArray(p?.hands?.feedback_state)&&p.hands.feedback_state.length>=12?p.hands.feedback_state.slice(0,12):null;
    const hfR=fb?validatedFeedback6(fb.slice(0,6)):null, hfL=fb?validatedFeedback6(fb.slice(6,12)):null;
    this.handFeedbackValid.left=!!hfL; this.handFeedbackValid.right=!!hfR;
    if(hcL)this.targetHandCmdLeft=hcL; if(hcR)this.targetHandCmdRight=hcR;
    // A known DFX dual-hand feedback failure can publish out-of-range/raw q.
    // Do not clamp that into a fake measured pose. For visualization only,
    // fall back to the controller's current command and hide that side's hand
    // solid hand uses command-estimated pose for visualization when feedback is invalid.
    // The command ghost remains visible as a cyan overlay/shell, even when coincident,
    // so the operator can keep the same command-ghost visual language as the arms.
    // No robot-control path uses this.
    if(hfL)this.targetHandFbLeft=hfL; else if(hcL)this.targetHandFbLeft=hcL;
    if(hfR)this.targetHandFbRight=hfR; else if(hcR)this.targetHandFbRight=hcR;
    if(!this.haveHandPose&&(hcL||hcR||hfL||hfR)){this.displayHandCmdLeft=this.targetHandCmdLeft.slice();this.displayHandCmdRight=this.targetHandCmdRight.slice();this.displayHandFbLeft=this.targetHandFbLeft.slice();this.displayHandFbRight=this.targetHandFbRight.slice();this.haveHandPose=true;}
    const seq=Number(p?.sequence); if(Number.isFinite(seq)&&seq!==this.sequence){if(this.lastRxPerf!==null){const d=(now-this.lastRxPerf)/1000;if(d>0&&d<2){this.rxIntervals.push(d);if(this.rxIntervals.length>40)this.rxIntervals.shift();const mean=this.rxIntervals.reduce((a,b)=>a+b,0)/this.rxIntervals.length;this.rxHz=mean>0?1/mean:0;}}this.lastRxPerf=now;this.sequence=seq;}
    const src=Number(p?.source_unix_time_s), srv=Number(p?.bridge?.server_unix_time_s); this.sourceAgeMs=Number.isFinite(src)&&Number.isFinite(srv)?Math.max(0,(srv-src)*1000):null;
    const pa=Number(p?.bridge?.packet_age_s); this.packetAgeMs=Number.isFinite(pa)?pa*1000:null;
  }
  updateCamera(){const cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),cy=Math.cos(this.yaw),sy=Math.sin(this.yaw);this.camera.position.set(this.target.x+this.distance*cp*cy,this.target.y+this.distance*cp*sy,this.target.z+this.distance*sp);this.camera.lookAt(this.target);}
  updatePose(dt){
    if(!this.havePose)return; const alpha=1-Math.exp(-Math.max(0,dt)/0.040);
    for(let i=0;i<29;i++)this.displayQ[i]=stepAngle(this.displayQ[i],this.targetQ[i],alpha);
    if(this.targetPub)for(let i=0;i<14;i++)this.displayPub[i]=stepAngle(this.displayPub[i],this.targetPub[i],alpha);
    for(let i=0;i<6;i++){
      this.displayHandCmdLeft[i]+= (this.targetHandCmdLeft[i]-this.displayHandCmdLeft[i])*alpha; this.displayHandCmdRight[i]+= (this.targetHandCmdRight[i]-this.displayHandCmdRight[i])*alpha;
      this.displayHandFbLeft[i]+= (this.targetHandFbLeft[i]-this.displayHandFbLeft[i])*alpha; this.displayHandFbRight[i]+= (this.targetHandFbRight[i]-this.displayHandFbRight[i])*alpha;
    }
    const fk=forward(this.displayQ);this.lastFk=fk;
    for(const [name,mesh] of this.measuredMeshes){
      if(this.inspireReady&&(name==='left_rubber_hand'||name==='right_rubber_hand')){mesh.visible=false;continue;}
      const m=fk.links[name];if(m){applyMatrix(mesh,m);mesh.visible=true;}else mesh.visible=false;
    }
    this.syncSceneReplicas(fk);
    const qg=this.displayQ.slice(); if(this.targetPub)for(let i=0;i<14;i++)qg[15+i]=this.displayPub[i]; const gfk=forward(qg);
    for(const [name,mesh] of this.ghostMeshes){
      if(this.inspireReady&&(name==='left_rubber_hand'||name==='right_rubber_hand')){mesh.visible=false;continue;}
      const m=gfk.links[name];mesh.visible=this.showGhost&&!!m;if(m)applyMatrix(mesh,m);
    }
    if(this.inspireReady){
      const measuredHands={
        left:handForward('left',fk.links.left_wrist_yaw_link,this.displayHandFbLeft),
        right:handForward('right',fk.links.right_wrist_yaw_link,this.displayHandFbRight)
      };
      const commandHands={
        left:handForward('left',gfk.links.left_wrist_yaw_link,this.displayHandCmdLeft),
        right:handForward('right',gfk.links.right_wrist_yaw_link,this.displayHandCmdRight)
      };
      for(const side of ['left','right']){
        for(const [link,mesh] of this.inspireMeasuredMeshes[side]){const m=measuredHands[side].links[link];mesh.visible=!!m;if(m)applyMatrix(mesh,m);}
        for(const [link,mesh] of this.inspireGhostMeshes[side]){const m=commandHands[side].links[link];mesh.visible=this.showGhost&&!!m;if(m)applyMatrix(mesh,m);}
      }
    }
    for(let i=0;i<29;i++){const j=JOINT_BY_INDEX.get(i),m=j?fk.jointFrames[j.name]:null,s=this.markerMeshes[i];if(m){s.position.set(m[3],m[7],m[11]);s.visible=this.showJoints||i===this.selected;}else s.visible=false;}
  }
  healthSeverity(h){
    if(!h)return null;
    if(this.healthMode==='temperature')return h.tempLevel;
    if(this.healthMode==='torque')return h.torqueLevel;
    const vals=[h.tempLevel,h.torqueLevel].filter(Number.isFinite);
    return vals.length?Math.max(...vals):null;
  }
  healthForIndex(i){
    const h=this.jointHealth[i]; if(!h)return null;
    const severity=this.healthSeverity(h);
    return {...h,severity,label:Number.isFinite(severity)?HEALTH_LABELS[severity]:'NO DATA'};
  }
  applyHealthVisuals(){
    for(const [name,mesh] of this.measuredMeshes){
      const i=mesh.userData.jointIndex;
      const dark=DARK_MESHES.has(name);
      const base=new THREE.Color(dark?0x26313b:0xaab4bd);
      const h=i===null||i===undefined?null:this.healthForIndex(i);
      if(this.showHealth && h && Number.isFinite(h.severity)){
        const hc=new THREE.Color(HEALTH_COLORS[h.severity]);
        const mix=h.severity===0?(dark?.10:.16):h.severity===1?(dark?.32:.42):h.severity===2?(dark?.46:.56):(dark?.58:.66);
        mesh.material.color.copy(base.clone().lerp(hc,mix));
        mesh.material.emissive.copy(hc.clone().multiplyScalar(h.severity===0?.025:h.severity===1?.10:.18));
      }else{
        mesh.material.color.copy(base); mesh.material.emissive.setHex(0x000000);
      }
      if(i===this.selected){
        const selectedColor=new THREE.Color(0x36c7ff);
        mesh.material.emissive.lerp(selectedColor,.48);
      }
    }
    for(let i=0;i<29;i++){
      const s=this.markerMeshes[i]; if(!s)continue;
      const h=this.healthForIndex(i);
      const c=this.showHealth&&h&&Number.isFinite(h.severity)?HEALTH_COLORS[h.severity]:0x63b9e8;
      s.material.color.setHex(i===this.selected?0xb9f3ff:c);
      s.material.opacity=i===this.selected?1:.86;
      s.scale.setScalar(i===this.selected?1.8:1);
    }
  }
  setHealthData(robot){
    const temps=Array.isArray(robot?.temperatures_c)?robot.temperatures_c:[];
    const taus=Array.isArray(robot?.tau_est)?robot.tau_est:[];
    this.jointHealth=new Array(29).fill(null).map((_,i)=>{
      const tempC=maxTempValue(temps[i]); const tauRaw=taus[i]; const tau=finite(tauRaw)?Number(tauRaw):null; const effort=G1_EFFORT_LIMIT_NM[i];
      const torqueNm=Number.isFinite(tau)?Math.abs(tau):null;
      const torqueUtil=Number.isFinite(torqueNm)&&Number.isFinite(effort)&&effort>0?torqueNm/effort:null;
      return {index:i,tempC,torqueNm,effortLimitNm:effort,torqueUtil,tempLevel:levelFromTemp(tempC),torqueLevel:levelFromTorqueUtil(torqueUtil)};
    });
    let peakTempC=null,peakTempIndex=null,peakTorqueUtil=null,peakTorqueIndex=null;
    const severities=[];
    for(let i=0;i<29;i++){
      const h=this.healthForIndex(i); if(!h)continue;
      if(Number.isFinite(h.severity))severities.push(h.severity);
      if(Number.isFinite(h.tempC)&&(peakTempC===null||h.tempC>peakTempC)){peakTempC=h.tempC;peakTempIndex=i;}
      if(Number.isFinite(h.torqueUtil)&&(peakTorqueUtil===null||h.torqueUtil>peakTorqueUtil)){peakTorqueUtil=h.torqueUtil;peakTorqueIndex=i;}
    }
    const severity=severities.length?Math.max(...severities):null;
    this.healthSummary={label:Number.isFinite(severity)?HEALTH_LABELS[severity]:'NO DATA',severity,
      watchCount:severities.filter(x=>x===1).length,highCount:severities.filter(x=>x===2).length,veryHighCount:severities.filter(x=>x===3).length,
      peakTempC,peakTempIndex,peakTorqueUtil,peakTorqueIndex};
    this.applyHealthVisuals();
  }
  updateHighlight(){this.applyHealthVisuals();}
  pickAt(e){const rect=this.canvas.getBoundingClientRect();this.pointer.x=((e.clientX-rect.left)/rect.width)*2-1;this.pointer.y=-((e.clientY-rect.top)/rect.height)*2+1;this.raycaster.setFromCamera(this.pointer,this.camera);const hits=this.raycaster.intersectObjects(this.pickMeshes,false);if(hits.length){const i=hits[0].object.userData.jointIndex;if(i!==null&&i!==undefined){this.selectJoint(i);if(this.onSelect)this.onSelect(i);}}}
  selectJoint(i){this.selected=Math.max(0,Math.min(28,Number(i)||0));this.updateHighlight();}
  setGhostVisible(v){this.showGhost=!!v;for(const m of this.ghostMeshes.values())m.visible=this.showGhost;for(const side of ['left','right'])for(const m of this.inspireGhostMeshes[side].values())m.visible=this.showGhost;}
  setJointsVisible(v){this.showJoints=!!v;for(let i=0;i<this.markerMeshes.length;i++){const s=this.markerMeshes[i];if(s)s.visible=this.showJoints||i===this.selected;}}
  setHealthVisible(v){this.showHealth=!!v;this.applyHealthVisuals();}
  setHealthMode(v){this.healthMode=['composite','temperature','torque'].includes(v)?v:'composite';
    const severities=this.jointHealth.map(h=>h?this.healthSeverity(h):null).filter(Number.isFinite);
    const severity=severities.length?Math.max(...severities):null; this.healthSummary={...this.healthSummary,label:Number.isFinite(severity)?HEALTH_LABELS[severity]:'NO DATA',severity,watchCount:severities.filter(x=>x===1).length,highCount:severities.filter(x=>x===2).length,veryHighCount:severities.filter(x=>x===3).length}; this.applyHealthVisuals();}
  loop(now){
    const dt=Math.min(.1,Math.max(0,(now-this.lastFrame)/1000));this.lastFrame=now;this.frameCount++;
    if(now-this.fpsStamp>=500){this.fps=this.frameCount*1000/(now-this.fpsStamp);this.frameCount=0;this.fpsStamp=now;}
    this.updateCamera();this.updatePose(dt);this.renderer.render(this.scene,this.camera);requestAnimationFrame(t=>this.loop(t));
  }
  getStats(){return {fps:this.fps,rxHz:this.rxHz,sequence:this.sequence,sourceAgeMs:this.sourceAgeMs,packetAgeMs:this.packetAgeMs,modelStatus:this.modelStatus,loadedMeshes:this.loaded+this.inspireLoaded,totalMeshes:MESH_NAMES.length+INSPIRE_MESH_COUNT,missingMeshes:[...this.failed,...this.inspireFailed],handFeedbackValid:{...this.handFeedbackValid}};}
}

let twin=null;
export const G1Twin={
  init(canvas,onSelect){if(!twin)twin=new MeshTwin(canvas,onSelect);return twin;},
  updatePose(p){twin?.setPose(p);},
  createSceneReplica(){return twin?.createSceneReplica()||null;},
  setGhostVisible(v){twin?.setGhostVisible(v);},
  setJointsVisible(v){twin?.setJointsVisible(v);},
  setHealthVisible(v){twin?.setHealthVisible(v);},
  setHealthMode(v){twin?.setHealthMode(v);},
  setHealthData(robot){twin?.setHealthData(robot);},
  getJointHealth(i){return twin?.healthForIndex(Math.max(0,Math.min(28,Number(i)||0)))||null;},
  getHealthSummary(){return twin?.healthSummary||{label:'NO DATA',severity:null};},
  resetView(){twin?.resetView();},
  selectJoint(i){twin?.selectJoint(i);},
  getSelectedIndex(){return twin?.selected??18;},
  getStats(){return twin?.getStats()||{modelStatus:'renderer not initialized',fps:0,rxHz:0,sequence:null,sourceAgeMs:null,packetAgeMs:null,loadedMeshes:0,totalMeshes:MESH_NAMES.length+INSPIRE_MESH_COUNT,missingMeshes:[]};},
  jointNames:G1_TELEMETRY_NAMES.slice()
};
