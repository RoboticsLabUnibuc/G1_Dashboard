import { G1Twin } from './g1_model.js';

(()=>{
'use strict';

const $ = (id) => document.getElementById(id);
const val = (obj, path, fallback=null) => {
  let cur=obj;
  for(const k of path){ if(cur==null || typeof cur!=='object' || !(k in cur)) return fallback; cur=cur[k]; }
  return cur==null ? fallback : cur;
};
const finite = (x) => Number.isFinite(Number(x));
const n = (x,d=2,suffix='') => finite(x) ? `${Number(x).toFixed(d)}${suffix}` : '—';
const pct = (x) => Math.max(0,Math.min(100,finite(x)?Number(x):0));
const boolWord = (x) => x ? 'YES' : 'NO';
const setTone = (el,tone) => { if(!el)return; el.classList.remove('good-text','warn-text','bad-text'); if(tone) el.classList.add(`${tone}-text`); };
const setChip = (el,text,tone) => { if(!el)return; el.textContent=text; el.className=`chip${tone?` ${tone}`:''}`; };
const fmtTime = (unix) => { try{return new Date(Number(unix)*1000).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'});}catch{return '—';} };

let latestEnv=null;
let pollBusy=false;
let currentView='live';
let cameraPc=null;
let cameraFallback=false;
let cameraUrl=null;
let cameraFrameWatch=null;
let cameraConnecting=false;
let selectedJointIndex=18;

function switchView(name){
  if(!['live','inspect','status'].includes(name)) return;
  currentView=name;
  document.querySelectorAll('.view').forEach(v=>v.classList.toggle('active',v.id===`view-${name}`));
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.view===name));
}
document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>switchView(b.dataset.view)));
document.querySelectorAll('[data-view-jump]').forEach(b=>b.addEventListener('click',()=>switchView(b.dataset.viewJump)));
$('faultDetailsBtn').addEventListener('click',()=>switchView('status'));

/* ---------- Camera / teleimager ---------- */
function fallbackCameraOffer(){
  const host=window.location.hostname;
  return host ? `https://${host}:60001/offer` : null;
}
function cameraBaseFromTelemetry(t){
  const offer=val(t,['camera','webrtc_offer_url']) || fallbackCameraOffer();
  if(!offer) return null;
  try{ const u=new URL(offer); u.pathname='/'; u.search=''; u.hash=''; return u.toString().replace(/\/$/,''); }catch{return null;}
}
function cameraOfferFromTelemetry(t){ return val(t,['camera','webrtc_offer_url']) || fallbackCameraOffer(); }
function updateCameraButtons(t){
  const base=cameraBaseFromTelemetry(t);
  cameraUrl=cameraOfferFromTelemetry(t);
  const link=$('cameraTrustLink');
  if(base){ link.href=base; link.classList.remove('hidden'); } else { link.removeAttribute('href'); link.classList.add('hidden'); }
  const configured=val(t,['camera','webrtc_enabled'], cameraUrl?true:false);
  $('cameraConnectBtn').disabled=!configured || !cameraUrl || cameraConnecting;
}
function cameraState(text,tone,detail){
  $('cameraStateText').textContent=text;
  $('cameraStateDot').className=`dot${tone?` ${tone}`:''}`;
  setChip($('cameraChip'),`CAMERA ${text}`, tone==='good'?'good':tone==='warn'?'warn':tone==='bad'?'bad':null);
  if(detail) $('cameraOverlayText').textContent=detail;
}
function stopCamera({silent=false}={}){
  if(cameraFrameWatch){ clearTimeout(cameraFrameWatch); cameraFrameWatch=null; }
  if(cameraPc){ try{cameraPc.close();}catch{} cameraPc=null; }
  const v=$('cameraVideo'); try{ if(v.srcObject) v.srcObject.getTracks().forEach(t=>t.stop()); }catch{} v.srcObject=null;
  cameraConnecting=false; cameraFallback=false;
  $('cameraOverlay').classList.remove('hidden');
  $('cameraOverlayTitle').textContent='Camera not connected';
  $('cameraOverlayText').textContent='Start teleimager, then connect.';
  $('cameraConnectBtn').classList.remove('hidden'); $('cameraStopBtn').classList.add('hidden');
  if(!silent) cameraState('OFFLINE',null);
}
async function waitIceComplete(pc,timeoutMs=3500){
  if(pc.iceGatheringState==='complete') return;
  await new Promise((resolve)=>{
    const done=()=>{pc.removeEventListener('icegatheringstatechange',onChange);clearTimeout(timer);resolve();};
    const onChange=()=>{if(pc.iceGatheringState==='complete')done();};
    const timer=setTimeout(done,timeoutMs); pc.addEventListener('icegatheringstatechange',onChange);
  });
}
async function connectCamera(codec=null){
  if(cameraConnecting) return;
  const t=latestEnv?.telemetry;
  const offerUrl=cameraOfferFromTelemetry(t||{});
  if(!offerUrl){cameraState('NO CONFIG','warn','No WebRTC offer URL in telemetry.');return;}
  stopCamera({silent:true});
  cameraConnecting=true; cameraFallback=codec==='vp8';
  $('cameraConnectBtn').disabled=true;
  $('cameraOverlay').classList.remove('hidden');
  $('cameraOverlayTitle').textContent='Connecting camera…';
  $('cameraOverlayText').textContent='Negotiating WebRTC with teleimager.';
  cameraState('CONNECTING','warn');
  try{
    const pc=new RTCPeerConnection({sdpSemantics:'unified-plan'}); cameraPc=pc;
    pc.addTransceiver('video',{direction:'recvonly'});
    pc.addEventListener('connectionstatechange',()=>{
      if(pc!==cameraPc)return;
      if(pc.connectionState==='connected') cameraState('LIVE','good');
      else if(['failed','disconnected'].includes(pc.connectionState)) cameraState('LOST','bad');
    });
    pc.addEventListener('track',(evt)=>{
      if(pc!==cameraPc || evt.track.kind!=='video') return;
      const v=$('cameraVideo'); v.srcObject=evt.streams[0]; v.play().catch(()=>{});
      $('cameraOverlay').classList.add('hidden');
      $('cameraConnectBtn').classList.add('hidden'); $('cameraStopBtn').classList.remove('hidden');
      cameraState('LIVE','good');
      const t0=v.currentTime;
      cameraFrameWatch=setTimeout(()=>{
        if(pc===cameraPc && v.currentTime===t0 && !cameraFallback){ stopCamera({silent:true}); connectCamera('vp8'); }
      },5000);
    });
    const offer=await pc.createOffer(); await pc.setLocalDescription(offer); await waitIceComplete(pc);
    const response=await fetch(offerUrl,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({sdp:pc.localDescription.sdp,type:pc.localDescription.type,codec:codec||null})});
    if(!response.ok) throw new Error(`teleimager /offer HTTP ${response.status}`);
    const answer=await response.json(); if(answer.error) throw new Error(answer.error); await pc.setRemoteDescription(answer);
  }catch(err){
    console.error('camera connection failed',err); stopCamera({silent:true});
    $('cameraOverlayTitle').textContent='Camera connection failed';
    $('cameraOverlayText').textContent='Camera may be occupied, teleimager may be down, or its certificate may need trust.';
    cameraState('ERROR','bad');
  }finally{ cameraConnecting=false; $('cameraConnectBtn').disabled=false; }
}
$('cameraConnectBtn').addEventListener('click',()=>connectCamera());
$('cameraStopBtn').addEventListener('click',()=>stopCamera());

/* ---------- Robot twin ---------- */
function jointGroup(i){
  if(i<6)return 'LEFT LEG'; if(i<12)return 'RIGHT LEG'; if(i<15)return 'WAIST'; if(i<22)return 'LEFT ARM'; return 'RIGHT ARM';
}
function arrAt(a,i){return Array.isArray(a)&&i>=0&&i<a.length?a[i]:null;}
function armLocalIndex(i){return i>=15&&i<=28?i-15:null;}
function formatTemps(pair){
  if(!Array.isArray(pair))return '—'; const vals=pair.filter(finite).map(x=>Number(x).toFixed(0)); return vals.length?`${vals.join('/')} °C`:'—';
}
function maxAbs(arr){const v=(Array.isArray(arr)?arr:[]).filter(finite).map(x=>Math.abs(Number(x)));return v.length?Math.max(...v):null;}
function maxTemp(pairs){const v=[];for(const p of Array.isArray(pairs)?pairs:[])if(Array.isArray(p))for(const x of p)if(finite(x))v.push(Number(x));return v.length?Math.max(...v):null;}
function renderSelectedJoint(t){
  const r=t?.robot||{},a=t?.arms||{},i=Math.max(0,Math.min(28,selectedJointIndex));
  const names=Array.isArray(r.joint_names)&&r.joint_names.length>=29?r.joint_names:(G1Twin.jointNames||[]);
  const name=names[i]||`joint_${i}`; const local=armLocalIndex(i);
  $('selectedJointName').textContent=name; $('selectedJointIndex').textContent=`#${i}`; $('selectedJointGroup').textContent=jointGroup(i);
  $('selectedMeasuredQ').textContent=n(arrAt(r.measured_q_rad,i),4,' rad'); $('selectedDq').textContent=n(arrAt(r.measured_dq_rps,i),3,' rad/s'); $('selectedTau').textContent=n(arrAt(r.tau_est,i),2);
  $('selectedTemp').textContent=formatTemps(arrAt(r.temperatures_c,i)); const ms=arrAt(r.motor_state,i); $('selectedMotorState').textContent=ms==null?'motor —':`motor ${ms}`;
  const h=G1Twin.getJointHealth(i);
  const sev=Number(h?.severity); const healthTone=sev>=3?'bad':sev>=1?'warn':Number.isFinite(sev)?'good':null;
  setChip($('selectedHealthChip'),h?.label||'HEALTH —',healthTone);
  $('selectedTorqueUtil').textContent=finite(h?.torqueUtil)?`${(Number(h.torqueUtil)*100).toFixed(0)}%`:'—';
  $('selectedEffortLimit').textContent=finite(h?.effortLimitNm)?`${Number(h.effortLimitNm).toFixed(0)} N·m`:'—';
  $('selectedTempBar').style.width=`${pct(finite(h?.tempC)?(Number(h.tempC)/100)*100:0)}%`;
  $('selectedTorqueBar').style.width=`${pct(finite(h?.torqueUtil)?Number(h.torqueUtil)*100:0)}%`;
  $('selectedTempBar').dataset.level=finite(h?.tempLevel)?String(h.tempLevel):'';
  $('selectedTorqueBar').dataset.level=finite(h?.torqueLevel)?String(h.torqueLevel):'';
  if(local!==null){
    $('selectedPublishedQ').textContent=n(arrAt(a.published_q_rad,local),4,' rad'); $('selectedTargetQ').textContent=n(arrAt(a.target_q_rad,local),4,' rad'); $('selectedIkQ').textContent=n(arrAt(a.ik_q_rad,local),4,' rad'); $('selectedErrorQ').textContent=n(arrAt(a.published_error_rad,local),4,' rad');
    $('selectedJointNote').textContent='Arm command fields compare measured LowState against the published arm command, target, and latest IK solution.';
  }else{
    $('selectedPublishedQ').textContent='—'; $('selectedTargetQ').textContent='—'; $('selectedIkQ').textContent='—'; $('selectedErrorQ').textContent='—';
    $('selectedJointNote').textContent='Leg and waist telemetry is read-only LowState feedback. The dashboard does not synthesize lower-body command targets.';
  }
}
function renderRobot(t){
  const r=t?.robot||{},a=t?.arms||{}; const hasFull=Array.isArray(r.measured_q_rad)&&r.measured_q_rad.length>=29;
  const mm=r.mode_machine;
  const meshStats=G1Twin.getStats();
  const kin=!hasFull?'controller V1.4 full-body telemetry required':mm===5?'mode 5 · rev_1_0 kinematics':mm==null?'rev_1_0 kinematics · mode unknown':`mode ${mm} · rev_1_0 kinematics preview`;
  $('robotModelStatus').textContent=`${kin} · ${meshStats.modelStatus}`;
  $('robotModeMachine').textContent=r.mode_machine==null?'—':String(r.mode_machine); $('statusModeMachine').textContent=r.mode_machine==null?'—':String(r.mode_machine);
  G1Twin.setHealthData(r);
  const armErr=finite(a.max_abs_published_error_rad)?Number(a.max_abs_published_error_rad):maxAbs(a.published_error_rad);
  const bodyTemp=maxTemp(r.temperatures_c); const hs=G1Twin.getHealthSummary();
  $('robotMaxError').textContent=n(armErr,4,' rad');
  $('robotPeakTorque').textContent=finite(hs?.peakTorqueUtil)?`${(Number(hs.peakTorqueUtil)*100).toFixed(0)}%`:'—';
  $('robotMaxTemp').textContent=n(bodyTemp,0,'°C'); $('robotHealthSummary').textContent=hs?.label||'—';
  setTone($('robotHealthSummary'),hs?.severity>=3?'bad':hs?.severity>=1?'warn':Number.isFinite(hs?.severity)?'good':null);
  renderSelectedJoint(t||{});
}

G1Twin.init($('robotTwinCanvas'),(i)=>{selectedJointIndex=i;renderSelectedJoint(latestEnv?.telemetry||{});});
G1Twin.selectJoint(selectedJointIndex);
$('ghostToggle').addEventListener('change',e=>G1Twin.setGhostVisible(e.target.checked));
$('jointToggle').addEventListener('change',e=>G1Twin.setJointsVisible(e.target.checked));
$('healthToggle').addEventListener('change',e=>{G1Twin.setHealthVisible(e.target.checked);$('healthLegend').classList.toggle('hidden',!e.target.checked);renderSelectedJoint(latestEnv?.telemetry||{});});
$('healthModeSelect').addEventListener('change',e=>{G1Twin.setHealthMode(e.target.value);renderSelectedJoint(latestEnv?.telemetry||{});const hs=G1Twin.getHealthSummary();$('robotHealthSummary').textContent=hs?.label||'—';setTone($('robotHealthSummary'),hs?.severity>=3?'bad':hs?.severity>=1?'warn':Number.isFinite(hs?.severity)?'good':null);});
$('twinResetBtn').addEventListener('click',()=>G1Twin.resetView());

/* ---------- Hands / events / status ---------- */
function validatedHandFeedback(values){
  if(!Array.isArray(values)||values.length<6)return null;
  const out=values.slice(0,6).map(Number);
  // Unitree documents rt/inspire/state q in the same normalized [0,1]
  // convention as rt/inspire/cmd. Reject a whole side if the service returns
  // out-of-range/raw values; clipping those values would falsely look valid.
  if(!out.every(v=>Number.isFinite(v)&&v>=-0.02&&v<=1.02))return null;
  return out.map(v=>Math.max(0,Math.min(1,v)));
}
function handFeedbackSides(feedbackState){
  const fb=Array.isArray(feedbackState)?feedbackState:[];
  return {
    right:validatedHandFeedback(fb.slice(0,6)),
    left:validatedHandFeedback(fb.slice(6,12)),
  };
}
function renderHandMatrix(currentLeft,currentRight,feedbackState){
  const body=$('handMatrixBody'); if(!body)return;
  const names=['Pinky','Ring','Middle','Index','Thumb','Thumb rot'];
  const left=Array.isArray(currentLeft)?currentLeft:[];
  const right=Array.isArray(currentRight)?currentRight:[];
  // Inspire motor order is right 0..5, left 6..11 in the controller.
  const sides=handFeedbackSides(feedbackState);
  const fbRight=sides.right||[], fbLeft=sides.left||[];
  body.innerHTML='';
  const channel=(cmd,feedback)=>{
    const c=finite(cmd)?Number(cmd):null, f=finite(feedback)?Number(feedback):null;
    const cPct=c==null?0:pct(c*100), fPct=f==null?0:pct(f*100);
    return `<div class="hand-channel">
      <div class="hand-bar"><i class="hand-command-fill" style="width:${cPct}%"></i>${f==null?'':`<b class="hand-feedback-marker" style="left:${fPct}%"></b>`}</div>
      <span class="hand-readout"><strong>${c==null?'—':c.toFixed(2)}</strong><em>${f==null?'—':f.toFixed(2)}</em></span>
    </div>`;
  };
  for(let i=0;i<6;i++){
    const row=document.createElement('div'); row.className='hand-matrix-row';
    row.innerHTML=`<span class="hand-finger-name">${names[i]}</span>${channel(left[i],fbLeft[i])}${channel(right[i],fbRight[i])}`;
    body.appendChild(row);
  }
}
let lastEventSignature='';
function renderEvents(env){
  const events=Array.isArray(env.events)?env.events:[]; const sig=events.slice(0,80).map(e=>`${e.id||''}:${e.message||''}`).join('|'); if(sig===lastEventSignature)return; lastEventSignature=sig; const all=$('allEvents'); if(!all)return; all.innerHTML='';
  if(!events.length){all.innerHTML='<div class="empty">No events yet</div>';return;}
  events.slice(0,80).forEach(e=>{const d=document.createElement('div');d.className=`event ${e.level||'info'}`;d.innerHTML=`<span class="time">${fmtTime(e.unix_time_s)}</span><span class="cat">${e.category||''}</span><span>${e.message||''}</span>`;all.appendChild(d);});
}
function updateFaultBanner(env,t){
  const online=!!val(env,['bridge','telemetry_online']); const safety=val(t,['mode','safety_fault_reason']); const hold=val(t,['mode','tracking_hold_reason']);
  const lowOk=!!val(t,['health','lowstate','ok']); const fingerFault=!!val(t,['health','finger_worker','feedback_fault']); const workerError=val(t,['health','finger_worker','error']);
  let title=null,text=null,kind='ATTENTION';
  if(!online){title='Telemetry stale';text='Dashboard data is no longer current.';}
  else if(safety){title='Safety fault';text=safety;kind='SAFETY';}
  else if(!lowOk){title='LowState unavailable';text='Fresh robot feedback is missing.';kind='ROBOT';}
  else if(workerError){title='Finger worker error';text=workerError;kind='HANDS';}
  else if(fingerFault){title='Inspire feedback fault';text='Finger feedback is stale.';kind='HANDS';}
  else if(hold){title='Tracking hold';text=hold;kind='TRACKING';}
  const b=$('faultBanner'); if(title){$('faultKind').textContent=kind;$('faultTitle').textContent=title;$('faultText').textContent=text||'';b.classList.remove('hidden');}else b.classList.add('hidden');
}

function renderActionReadiness(t){
  const actions=t?.actions||{}, handover=actions?.xr_handover||{}, cond=actions?.engagement_conditions||{};
  const has=!!actions?.schema;
  const available=has && handover?.available===true;
  const label=has?(handover?.label||'No action'):'Controller V1.8 required';
  const operation=has?(handover?.operation||'NONE'):'—';
  const next=has?(handover?.would_enter_state||'—'):'—';
  const reason=has?(handover?.reason||'—'):'Read-only action readiness requires controller V1.8.';
  const channel=actions?.request_channel_enabled===true;

  $('actionLabel').textContent=label;
  $('actionOperation').textContent=operation;
  $('actionNextState').textContent=next;
  $('actionReason').textContent=reason;
  setChip($('actionReadinessChip'),has?(available?'READY':'BLOCKED'):'READ ONLY',has?(available?'good':'warn'):null);

  const condChip=(id,text,ok,neutral=false)=>{const el=$(id); el.textContent=text; el.classList.toggle('cond-good',!!ok); el.classList.toggle('cond-warn',!ok&&!neutral); el.classList.toggle('cond-neutral',!!neutral);};
  condChip('actionCondLowstate',`LOW ${cond.lowstate_ok?'OK':'BAD'}`,cond.lowstate_ok===true,!has);
  condChip('actionCondXr',`XR ${cond.xr_ok?'OK':'WAIT'}`,cond.xr_ok===true,!has);
  condChip('actionCondStop',`STOP ${cond.stop_gate_ready?'READY':cond.stop_gate_instant?'TIMING':'WAIT'}`,cond.stop_gate_ready===true,!has);
  condChip('actionCondFault',`FAULT ${cond.safety_fault_clear?'CLEAR':'HOLD'}`,cond.safety_fault_clear===true,!has);

  setChip($('statusActionChip'),has?(available?'READY':'BLOCKED'):'READ ONLY',has?(available?'good':'warn'):null);
  $('statusActionLabel').textContent=label;
  $('statusActionOperation').textContent=operation;
  $('statusActionNextState').textContent=next;
  $('statusActionChannel').textContent=channel?'ENABLED':'DISABLED';
  setTone($('statusActionChannel'),channel?'good':null);
  $('statusActionReason').textContent=reason+(channel?'':' Browser requests are intentionally disabled in this revision.');
}

function render(env){
  latestEnv=env; const t=env.telemetry; const bridge=env.bridge||{};
  setChip($('bridgeChip'),'BRIDGE CONNECTED','good'); setChip($('telemetryChip'),bridge.telemetry_online?'TELEMETRY LIVE':'TELEMETRY STALE',bridge.telemetry_online?'good':'warn');
  $('packetAge').textContent=finite(bridge.packet_age_s)?`${Math.round(bridge.packet_age_s*1000)} ms`:'—'; $('statusPacketAge').textContent=$('packetAge').textContent;
  if(!t){
    $('versionText').textContent='waiting for controller'; updateCameraButtons({}); if(!cameraPc&&!cameraConnecting)setChip($('cameraChip'),'CAMERA OFFLINE','warn');
    $('robotModelStatus').textContent='waiting for controller'; renderRobot({}); renderActionReadiness({}); updateFaultBanner(env,{}); renderEvents(env); return;
  }
  $('versionText').textContent=val(t,['controller','version'],'—');
  const state=val(t,['mode','state'],'—'), weight=Number(val(t,['mode','arm_ownership_weight'],0));
  $('modeValue').textContent=state; $('ownershipValue').textContent=n(weight,3); $('ownershipBar').style.width=`${pct(weight*100)}%`; $('robotWeight').textContent=n(weight,3); $('statusOwnership').textContent=n(weight,3); setChip($('statusModeChip'),state,state==='SAFETY_FAULT_HOLD'?'bad':state==='XR_TRACKING_HOLD'?'warn':null);
  const xrOk=!!val(t,['health','xr','ok']), xrReason=val(t,['health','xr','reason'],'—'); $('xrValue').textContent=xrOk?'OK':'BAD'; $('xrReason').textContent=xrReason; setTone($('xrValue'),xrOk?'good':'warn'); setChip($('statusXrChip'),xrOk?'XR OK':'XR BAD',xrOk?'good':'warn'); $('statusXrReason').textContent=xrReason;
  const lowOk=!!val(t,['health','lowstate','ok']), lowAge=val(t,['health','lowstate','age_s']); $('lowstateValue').textContent=lowOk?'OK':'BAD'; $('lowstateAge').textContent=`age ${finite(lowAge)?(Number(lowAge)*1000).toFixed(1):'—'} ms`; setTone($('lowstateValue'),lowOk?'good':'bad'); setChip($('statusLowstateChip'),lowOk?'LOWSTATE OK':'LOWSTATE BAD',lowOk?'good':'bad');
  const hm=val(t,['hands','mode'],'—'), htrack=!!val(t,['hands','tracking_valid']); $('handsValue').textContent=hm; $('handsReason').textContent=htrack?'tracking OK':val(t,['hands','tracking_reason'],'—'); setTone($('handsValue'),hm==='FOLLOW'?'good':hm==='HOLD'||hm==='REACQUIRE'?'warn':null);
  const guard=!!val(t,['health','tracking_guard','active']); $('guardValue').textContent=guard?'PAIR HOLD':'CLEAR'; $('guardReason').textContent=guard?`L=${+!!val(t,['health','tracking_guard','rejected_left'])} R=${+!!val(t,['health','tracking_guard','rejected_right'])}`:'coherent wrist pair'; setTone($('guardValue'),guard?'warn':'good'); $('statusGuard').textContent=guard?$('guardReason').textContent:'CLEAR';
  const gate=val(t,['motion','stop_gate'],{}); const gateText=gate.ready?`READY ${n(gate.elapsed_s,1,'s')}`:gate.instant?`TIMING ${n(gate.elapsed_s,1,'s')}`:'MOVING'; $('stopGate').textContent=gateText; $('statusStopGate').textContent=gateText;
  const fp=val(t,['health','finger_worker','phase'],'—'); $('fingerPhase').textContent=fp; $('statusFingerPhase').textContent=fp; const ff=!!val(t,['health','finger_worker','feedback_fault']); const fa=val(t,['health','finger_worker','feedback_age_s']); const inspire=ff?`FAULT ${n(fa,2,'s')}`:`OK ${n(fa,2,'s')}`; $('fingerFeedback').textContent=inspire; $('statusInspire').textContent=inspire;
  $('safetyFault').textContent=val(t,['mode','safety_fault_reason'],'none')||'none'; $('trackingHoldReason').textContent=val(t,['mode','tracking_hold_reason'],'none')||'none'; $('shutdownPending').textContent=boolWord(val(t,['mode','shutdown_pending']));
  const al=val(t,['tracking','alignment'],{}); $('alignmentFrames').textContent=`${al.stable_frames??0}/${al.required_frames??'—'}`;
  const hold=val(t,['tracking','hold'],{}); const holdPos=`${n(hold.position_error_m,3,' m')} / ${n(hold.position_limit_m,3,' m')}`,holdRot=`${n(hold.rotation_error_deg,1,'°')} / ${n(hold.rotation_limit_deg,1,'°')}`; $('holdPosition').textContent=holdPos; $('statusHoldPosition').textContent=holdPos; $('holdRotation').textContent=holdRot; $('statusHoldRotation').textContent=holdRot; $('resumeFrames').textContent=`${hold.stable_frames??0}/${hold.required_frames??'—'}`; $('resumeBar').style.width=`${pct(100*(hold.stable_frames||0)/(hold.required_frames||1))}%`; $('recoveryPanel').classList.toggle('hidden',state!=='XR_TRACKING_HOLD');
  const r3=val(t,['motion','r3'],{}), lb=val(t,['motion','lower_body'],{}); $('r3Max').textContent=n(r3.max_abs_axis,3); $('leftStick').textContent=`${n(r3.lx,2)}, ${n(r3.ly,2)}`; $('rightStick').textContent=`${n(r3.rx,2)}, ${n(r3.ry,2)}`; $('dqCombined').textContent=`${n(lb.dq_max_rps,3)} / ${n(lb.dq_rms_rps,3)} rad/s`; $('yawRate').textContent=n(lb.yaw_rate_rps,3,' rad/s');
  const armErr=val(t,['health','arm_publisher','error']); $('statusArmPublisher').textContent=armErr?`ERROR ${armErr}`:'OK';

  renderActionReadiness(t);
  renderRobot(t);
  const handFeedbackState=val(t,['hands','feedback_state'],[]);
  renderHandMatrix(val(t,['hands','current_left'],[]),val(t,['hands','current_right'],[]),handFeedbackState);
  setChip($('handTrackingSummary'),htrack?'TRACKING':'NO TRACK',htrack?'good':'warn');
  $('handModeDetail').textContent=hm;
  const handFbAge=val(t,['health','finger_worker','feedback_age_s']);
  const handFbFault=!!val(t,['health','finger_worker','feedback_fault']);
  const handFbSides=handFeedbackSides(handFeedbackState);
  const handFbValid=!!handFbSides.left&&!!handFbSides.right;
  $('handFeedbackDetail').textContent=handFbFault?`STALE ${n(handFbAge,2,'s')}`:handFbValid?`${n(handFbAge,2,'s')}`:'INVALID RANGE';
  setTone($('handFeedbackDetail'),handFbFault||!handFbValid?'warn':'good');
  $('handRetargets').textContent=val(t,['hands','retarget_count'],'—'); $('handReacquire').textContent=val(t,['hands','reacquire_ready'])?'READY':`${val(t,['hands','reacquire_count'],0)}/${val(t,['hands','reacquire_required_frames'],'—')}`; $('thumbStatus').textContent=val(t,['controller','symmetric_thumb_rotation'])===false?'NO':'YES';
  const c=t.camera||{}; const res=c.width&&c.height?`${c.width}×${c.height}`:'—'; $('cameraMeta').textContent=`${res} · ${n(c.display_fps,0,' fps')} · ${c.display_mode||'—'}`; $('cameraHudInfo').textContent=$('cameraMeta').textContent; $('statusCamera').textContent=c.webrtc_enabled?`WebRTC ${res}`:'disabled'; updateCameraButtons(t); if(!cameraPc&&!cameraConnecting)setChip($('cameraChip'),c.webrtc_enabled?'CAMERA OFFLINE':'CAMERA OFF',c.webrtc_enabled?'warn':null);
  updateFaultBanner(env,t); renderEvents(env);
}


/* ---------- PC2 / Unitree system health ---------- */
const bytes = (x) => {
  if(!finite(x)) return '—';
  let v=Number(x); const units=['B','KB','MB','GB','TB']; let i=0;
  while(Math.abs(v)>=1024 && i<units.length-1){v/=1024;i++;}
  return `${v.toFixed(i>=3?1:0)} ${units[i]}`;
};
const duration = (sec) => {
  if(!finite(sec)) return '—';
  let s=Math.max(0,Math.floor(Number(sec))); const d=Math.floor(s/86400); s%=86400; const h=Math.floor(s/3600); s%=3600; const m=Math.floor(s/60);
  return d?`${d}d ${h}h`:h?`${h}h ${m}m`:`${m}m`;
};
function endpointState(id,on){ const el=$(id); if(!el)return; el.textContent=on?'LISTENING':'OFFLINE'; setTone(el,on?'good':'warn'); }
function processState(id,count){ const el=$(id); if(!el)return; const n=Number(count||0); el.textContent=n>0?`${n} running`:'not found'; setTone(el,n>0?'good':'warn'); }
let lastRobotServices=[];
function renderRobotServices(api){
  const available=!!api?.available;
  setChip($('robotStateChip'),available?'API ONLINE':api?.enabled===false?'API OFF':'API UNAVAILABLE',available?'good':api?.enabled===false?null:'warn');
  $('robotStateClientVersion').textContent=api?.client_api_version??'—';
  $('robotStateServerVersion').textContent=api?.server_api_version??'—';
  const match=api?.api_version_match;
  $('robotStateVersionMatch').textContent=match===true?'YES':match===false?'NO':'—';
  setTone($('robotStateVersionMatch'),match===true?'good':match===false?'warn':null);
  $('robotStateServiceCount').textContent=finite(api?.service_count)?String(api.service_count):'—';
  const err=$('robotStateError');
  if(api?.error){err.textContent=api.error;}else{err.textContent=api?.module?`read-only via ${api.module}`:'read-only service inventory';}
  err.classList.remove('hidden');
  lastRobotServices=Array.isArray(api?.services)?api.services:[];
  renderServiceList();
}
function renderServiceList(){
  const list=$('robotServiceList'); if(!list)return;
  list.innerHTML='';
  const filter=String($('serviceFilter')?.value||'').trim().toLowerCase();
  const all=Array.isArray(lastRobotServices)?lastRobotServices:[];
  const services=filter?all.filter(s=>String(s?.name||'').toLowerCase().includes(filter)):all;
  // Unitree RobotState service status polarity is 0 = ON, 1 = OFF.
  // New monitor packets publish `enabled`; fall back to the raw Unitree status
  // for compatibility with a monitor that was started before this UI update.
  const serviceEnabled=(s)=>typeof s?.enabled==='boolean'?s.enabled:Number(s?.status)===0?true:Number(s?.status)===1?false:null;
  const on=all.filter(s=>serviceEnabled(s)===true).length;
  const off=all.filter(s=>serviceEnabled(s)===false).length;
  const unknown=all.length-on-off;
  const protectedCount=all.filter(s=>!!s?.protect).length;
  const counts=$('robotServiceCounts');
  if(counts) counts.textContent=`${on} on · ${off} off${unknown?` · ${unknown} unknown`:''} · ${protectedCount} protected${filter?` · ${services.length} shown`:''}`;
  if(!services.length){list.innerHTML=`<div class="empty">${all.length?'No matching services':'No service inventory yet'}</div>`;return;}
  for(const svc of services){
    const row=document.createElement('div'); row.className='service-row';
    const name=document.createElement('span'); name.className='service-name'; name.textContent=String(svc?.name||'?'); name.title=name.textContent;
    const protect=document.createElement('span'); protect.className='service-protect'; protect.textContent=svc?.protect?'PROTECTED':'—';
    const state=document.createElement('strong');
    const enabled=serviceEnabled(svc), rawStatus=Number(svc?.status);
    state.textContent=enabled===true?'ON':enabled===false?'OFF':`STATE ${Number.isFinite(rawStatus)?rawStatus:'—'}`;
    state.title=Number.isFinite(rawStatus)?`Unitree raw service status: ${rawStatus} (0=ON, 1=OFF)`:'';
    if(enabled===true) state.className='good-text'; else if(enabled===false) state.className='dim'; else state.className='warn-text';
    row.append(name,protect,state); list.appendChild(row);
  }
}
function renderBaseSensing(base){
  const imu=base?.imu||{}, odom=base?.odometry||{};
  const imuFresh=!!imu.available && finite(imu.age_s) && Number(imu.age_s)<1.0;
  const odomFresh=!!odom.available && finite(odom.age_s) && Number(odom.age_s)<1.5;
  setChip($('baseSensorChip'),imuFresh?(odomFresh?'IMU + ODOM LIVE':'IMU LIVE'):'SENSORS OFFLINE',imuFresh?'good':'warn');

  const rpy=Array.isArray(imu.rpy_rad)?imu.rpy_rad:[];
  const gyro=Array.isArray(imu.gyroscope_rps)?imu.gyroscope_rps:[];
  const acc=Array.isArray(imu.accelerometer_mps2)?imu.accelerometer_mps2:[];
  const quat=Array.isArray(imu.quaternion_wxyz)?imu.quaternion_wxyz:[];
  const rad2deg=x=>finite(x)?Number(x)*180/Math.PI:null;
  $('imuTopic').textContent=imu.topic||'rt/secondary_imu';
  $('imuRoll').textContent=n(rad2deg(rpy[0]),1,'°');
  $('imuPitch').textContent=n(rad2deg(rpy[1]),1,'°');
  $('imuYaw').textContent=n(rad2deg(rpy[2]),1,'°');
  $('imuGyroX').textContent=n(gyro[0],3);
  $('imuGyroY').textContent=n(gyro[1],3);
  $('imuGyroZ').textContent=n(gyro[2],3);
  $('imuAccelX').textContent=n(acc[0],2);
  $('imuAccelY').textContent=n(acc[1],2);
  $('imuAccelZ').textContent=n(acc[2],2);
  $('imuQuaternion').textContent=quat.length>=4?`q [${quat.slice(0,4).map(v=>finite(v)?Number(v).toFixed(3):'—').join(', ')}]`:'q —';
  const imuBits=[];
  if(finite(imu.temperature_c)) imuBits.push(`${Number(imu.temperature_c).toFixed(0)} °C`);
  if(finite(imu.age_s)) imuBits.push(`${(Number(imu.age_s)*1000).toFixed(0)} ms`);
  if(finite(imu.sample_count)) imuBits.push(`#${imu.sample_count}`);
  $('imuMeta').textContent=imuBits.join(' · ') || (imu.error||'waiting');

  const pos=Array.isArray(odom.position_m)?odom.position_m:[];
  const vel=Array.isArray(odom.velocity_mps)?odom.velocity_mps:[];
  $('odomTopic').textContent=odom.topic||'waiting';
  $('odomPosX').textContent=n(pos[0],3,' m'); $('odomPosY').textContent=n(pos[1],3,' m'); $('odomPosZ').textContent=n(pos[2],3,' m');
  $('odomVelX').textContent=n(vel[0],3,' m/s'); $('odomVelY').textContent=n(vel[1],3,' m/s'); $('odomVelZ').textContent=n(vel[2],3,' m/s');
  $('odomYawRate').textContent=`yaw rate ${n(odom.yaw_speed_rps,3,' rad/s')}`;
  const odomBits=[];
  if(finite(odom.age_s)) odomBits.push(`${(Number(odom.age_s)*1000).toFixed(0)} ms`);
  if(finite(odom.sample_count)) odomBits.push(`#${odom.sample_count}`);
  $('odomMeta').textContent=odomBits.join(' · ') || '—';
  const note=$('odomNote');
  if(odomFresh){note.textContent='Read-only odometry stream is live.'; setTone(note,'good');}
  else {note.textContent=odom.error||'Waiting for rt/odommodestate; the odometer service may be inactive.'; setTone(note,odom.error?'warn':null);}
}
function renderSystem(env){
  const mon=env?.monitor||{}; const sys=env?.system;
  const online=!!mon.online;
  setChip($('systemMonitorChip'),online?'MONITOR LIVE':'MONITOR OFFLINE',online?'good':'warn');
  $('systemAge').textContent=finite(mon.packet_age_s)?`${Number(mon.packet_age_s).toFixed(1)} s`:'—';
  if(!sys){
    $('systemHostname').textContent='PC2 monitor not running';
    renderRobotServices({enabled:false,available:false,error:'Run g1_dashboard_system_monitor.py on PC2.'});
    renderBaseSensing(null);
    return;
  }
  const host=sys.host||{}, cpu=host.cpu||{}, mem=host.memory||{}, disk=host.disk_root||{}, therm=host.thermal||{}, net=host.network||{};
  $('systemHostname').textContent=host.hostname||'PC2';
  $('systemCpu').textContent=finite(cpu.used_pct)?`${Number(cpu.used_pct).toFixed(0)}%`:'—';
  setTone($('systemCpu'),finite(cpu.used_pct)&&Number(cpu.used_pct)>=90?'bad':finite(cpu.used_pct)&&Number(cpu.used_pct)>=75?'warn':'good');
  $('systemLoad').textContent=`load ${n(cpu.load_1m,2)} / ${n(cpu.load_5m,2)} / ${n(cpu.load_15m,2)}`;
  $('systemRam').textContent=finite(mem.used_pct)?`${Number(mem.used_pct).toFixed(0)}%`:'—';
  $('systemRamDetail').textContent=`${bytes(mem.available_bytes)} free / ${bytes(mem.total_bytes)}`;
  setTone($('systemRam'),finite(mem.used_pct)&&Number(mem.used_pct)>=90?'bad':finite(mem.used_pct)&&Number(mem.used_pct)>=80?'warn':'good');
  $('systemDisk').textContent=finite(disk.used_pct)?`${Number(disk.used_pct).toFixed(0)}%`:'—';
  $('systemDiskDetail').textContent=`${bytes(disk.free_bytes)} free / ${bytes(disk.total_bytes)}`;
  setTone($('systemDisk'),finite(disk.used_pct)&&Number(disk.used_pct)>=95?'bad':finite(disk.used_pct)&&Number(disk.used_pct)>=85?'warn':'good');
  $('systemTemp').textContent=finite(therm.max_c)?`${Number(therm.max_c).toFixed(0)} °C`:'—';
  const zones=Array.isArray(therm.zones)?therm.zones:[]; $('systemTempZone').textContent=zones.length?(zones[0].name||'thermal zone'):'no thermal data';
  setTone($('systemTemp'),finite(therm.max_c)&&Number(therm.max_c)>=90?'bad':finite(therm.max_c)&&Number(therm.max_c)>=80?'warn':'good');
  $('systemUptime').textContent=duration(host.uptime_s);
  $('systemInterface').textContent=`${net.interface||'—'} · ${net.operstate||'unknown'}${net.carrier===1?' · carrier':''}`;
  setTone($('systemInterface'),net.operstate==='up'?'good':'warn');
  $('systemIpv4').textContent=net.ipv4||'—';
  $('systemTraffic').textContent=`RX ${bytes(net.rx_bytes)} · TX ${bytes(net.tx_bytes)}`;
  const ep=sys.endpoints||{}; endpointState('endpointTelevuer',!!ep.televuer_8012); endpointState('endpointCamera',!!ep.camera_60001); endpointState('endpointDashboard',!!ep.dashboard_8080);
  const pr=sys.processes||{}; processState('processController',pr.controller); processState('processTeleimager',pr.teleimager); processState('processInspire',pr.inspire);
  renderRobotServices(sys.robot_state_api||{});
  renderBaseSensing(sys.base_sensing||{});
}
$('serviceFilter')?.addEventListener('input',renderServiceList);

let systemBusy=false;
async function pollSystem(){
  if(systemBusy)return; systemBusy=true;
  try{const r=await fetch('/api/system',{cache:'no-store'}); if(!r.ok)throw new Error(`HTTP ${r.status}`); renderSystem(await r.json());}
  catch(err){setChip($('systemMonitorChip'),'MONITOR OFFLINE','warn'); console.debug('system monitor unavailable',err);}
  finally{systemBusy=false;}
}

let poseBusy=false;
let lastPoseSeq=-1;
let lastPoseSource=0;
function updatePoseHud(){
  const st=G1Twin.getStats();
  $('poseAgeStat').textContent=finite(st.sourceAgeMs)?`${Math.round(st.sourceAgeMs)} ms`:'—';
  $('poseRxStat').textContent=finite(st.rxHz)?`${st.rxHz.toFixed(1)} Hz`:'—';
  $('poseFpsStat').textContent=finite(st.fps)?`${Math.round(st.fps)} FPS`:'—';
  $('poseSeqStat').textContent=st.sequence==null?'—':String(st.sequence);
}
async function pollPose(){
  if(poseBusy)return; poseBusy=true; const started=performance.now();
  try{
    const r=await fetch('/api/pose',{cache:'no-store'}); if(!r.ok)throw new Error(`HTTP ${r.status}`); const pose=await r.json();
    const seq=Number(pose.sequence), src=Number(pose.source_unix_time_s);
    const controllerRestart = Number.isFinite(seq) && seq < lastPoseSeq && Number.isFinite(src) && src > lastPoseSource + 0.5;
    if(Number.isFinite(seq)&&(seq>lastPoseSeq||controllerRestart)){ lastPoseSeq=seq; if(Number.isFinite(src))lastPoseSource=src; G1Twin.updatePose(pose); }
  }catch(err){ console.debug('pose fast path unavailable',err); }
  finally{ poseBusy=false; const wait=Math.max(0,(1000/30)-(performance.now()-started)); setTimeout(pollPose,wait); }
}
async function poll(){
  if(pollBusy)return; pollBusy=true;
  try{ const r=await fetch('/api/latest',{cache:'no-store'}); if(!r.ok)throw new Error(`HTTP ${r.status}`); render(await r.json()); updatePoseHud(); }
  catch(err){ setChip($('bridgeChip'),'BRIDGE OFFLINE','bad'); console.error('dashboard status poll failed',err); }
  finally{pollBusy=false;}
}

pollPose(); poll(); pollSystem(); setInterval(poll,250); setInterval(updatePoseHud,250); setInterval(pollSystem,250);

})();
