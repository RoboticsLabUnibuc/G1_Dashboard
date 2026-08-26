import { G1Twin } from './g1_model.js';
import { PointCloud3D } from './pointcloud_view.js';

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
let cameraProcessStatus=null;
let cameraProcessPollBusy=false;
let cameraProcessActionBusy=false;
let cameraModeActionBusy=false;
let cameraYoloActionBusy=false;
let pointViewSendBusy=false;
let pointViewLocal=null;
let pointViewInitialized=false;
let pointCloudFetchBusy=false;
let pointCloudPollTimer=null;
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

/* ---------- Step 5.1 controller process + XR action manager ---------- */
let controllerConfig=null;
let controllerStatus=null;
let controllerPollBusy=false;
let controllerActionBusy=false;
let xrActionBusy=false;
let latestActionReadiness=null;
let managementAuthBusy=false;
let managementAfterAuth=null;

function currentManagementKey(){
  return sessionStorage.getItem('g1ManagementKey')||'';
}
function storeManagementKey(key){
  const value=String(key||'').trim();
  if(value)sessionStorage.setItem('g1ManagementKey',value);
  else sessionStorage.removeItem('g1ManagementKey');
  if($('managementKeyInput'))$('managementKeyInput').value=value;
  if($('controllerManagementKey'))$('controllerManagementKey').value=value;
}
function setManagementKeyError(message){
  const el=$('managementKeyError'); if(!el)return;
  if(!message){el.textContent='';el.classList.add('hidden');return;}
  el.textContent=message;el.classList.remove('hidden');
}
function showManagementKeyPrompt(message='',afterAuth=null){
  managementAfterAuth=typeof afterAuth==='function'?afterAuth:null;
  setManagementKeyError(message);
  $('managementKeyInput').value=currentManagementKey();
  $('managementKeyModal').classList.remove('hidden');
  document.body.classList.add('modal-open');
  setTimeout(()=>$('managementKeyInput').focus(),0);
}
function closeManagementKeyPrompt(){
  $('managementKeyModal').classList.add('hidden');
  if($('controllerModal').classList.contains('hidden'))document.body.classList.remove('modal-open');
  managementAfterAuth=null;
}
async function verifyManagementKey(key){
  const value=String(key||'').trim();
  if(!value)throw new Error('Paste the management key printed by the current ./start_dashboard.sh.');
  const r=await fetch('/api/controller/auth',{
    method:'POST',
    headers:{'Content-Type':'application/json','X-G1-Management-Key':value},
    body:'{}',
    cache:'no-store'
  });
  let body={};try{body=await r.json();}catch{}
  if(!r.ok)throw new Error(body.error||`HTTP ${r.status}`);
  return body;
}
async function bootstrapManagementKey(){
  try{
    const cfg=await loadControllerConfig();
    if(!cfg?.enabled||!cfg?.management_key_required)return;
    const saved=currentManagementKey();
    if(saved){
      try{await verifyManagementKey(saved);return;}
      catch{storeManagementKey('');showManagementKeyPrompt('The saved management key is no longer valid. The dashboard may have restarted; enter the new key printed by ./start_dashboard.sh.');return;}
    }
    showManagementKeyPrompt('Enter the current management key to enable listener, camera, ENTER/EXIT TELEOP, and allowlisted service controls.');
  }catch(err){
    console.debug('management-key bootstrap unavailable',err);
  }
}

function controllerStateTone(state){
  if(state==='RUNNING') return 'good';
  if(state==='STOPPING'||state==='RUNNING_EXTERNAL') return 'warn';
  if(state==='UNAVAILABLE') return 'bad';
  return null;
}
function controllerStateLabel(state){
  if(state==='RUNNING_EXTERNAL') return 'EXTERNAL';
  return state||'—';
}
function renderControllerProcess(st){
  controllerStatus=st||{};
  const state=controllerStatus.state||'UNAVAILABLE';
  const tone=controllerStateTone(state);
  setChip($('controllerProcessChip'),`CTRL ${controllerStateLabel(state)}`,tone);
  setChip($('controllerProcessState'),controllerStateLabel(state),tone);
  const pid=controllerStatus.pid;
  $('controllerProcessPid').textContent=pid?`pid ${pid}`:'pid —';
  $('controllerProcessUptime').textContent=finite(controllerStatus.uptime_s)?`uptime ${duration(controllerStatus.uptime_s)}`:'uptime —';
  const inspire=controllerStatus?.dependencies?.inspire||{};
  const inspireState=inspire.state||'—';
  const inspireLabel=inspireState==='RUNNING_MANAGED'?'Inspire MANAGED':inspireState==='RUNNING_EXTERNAL'?'Inspire EXTERNAL':inspireState==='STOPPED'?'Inspire STOPPED':inspireState==='CONFLICT'?'Inspire CONFLICT':inspireState==='UNAVAILABLE'?'Inspire SETUP':'Inspire —';
  $('controllerInspireState').textContent=inspireLabel;
  setTone($('controllerInspireState'),inspireState==='RUNNING_MANAGED'||inspireState==='RUNNING_EXTERNAL'?'good':inspireState==='CONFLICT'?'bad':inspireState==='UNAVAILABLE'?'warn':null);
  let detail='Process manager unavailable.';
  if(state==='STOPPED') detail=inspireState==='RUNNING_EXTERNAL'?'Ready; external Inspire service will be reused and left running on stop.':'Ready; Start launches Inspire first, then the whitelisted teleop listener.';
  else if(state==='RUNNING') detail=inspireState==='RUNNING_MANAGED'?'Listener + dashboard-managed Inspire service are running.':'Dashboard-managed listener is running; Inspire is externally owned.';
  else if(state==='STOPPING') detail='Controlled listener stop requested; Inspire stops only after controller handback/exit.';
  else if(state==='RUNNING_EXTERNAL') detail='Teleop listener is already running outside this dashboard. Lifecycle and XR actions are locked here.';
  else if(controllerStatus.last_error) detail=controllerStatus.last_error;
  else if(inspire.error) detail=`Inspire dependency: ${inspire.error}`;
  else if(controllerStatus.enabled===false) detail='Process actions disabled by dashboard startup configuration.';
  $('controllerProcessDetail').textContent=detail;
  $('controllerConfigureBtn').disabled=controllerActionBusy || !controllerStatus.can_start;
  $('controllerConfigureBtn').textContent=state==='STOPPED'?'Configure & start':'Configure & start';
  $('controllerStopBtn').disabled=controllerActionBusy || state!=='RUNNING' || !controllerStatus.can_stop;
  if(state==='STOPPING') $('controllerStopBtn').textContent='Stopping…'; else $('controllerStopBtn').textContent='Stop listener';
}
async function pollControllerProcess(){
  if(controllerPollBusy)return; controllerPollBusy=true;
  try{
    const r=await fetch('/api/controller',{cache:'no-store'});
    if(!r.ok) throw new Error(`HTTP ${r.status}`);
    renderControllerProcess(await r.json());
  }catch(err){
    setChip($('controllerProcessChip'),'CTRL OFFLINE','warn');
    setChip($('controllerProcessState'),'OFFLINE','warn');
    $('controllerProcessDetail').textContent='Controller process manager endpoint unavailable.';
    $('controllerConfigureBtn').disabled=true; $('controllerStopBtn').disabled=true;
    console.debug('controller process manager unavailable',err);
  }finally{controllerPollBusy=false;}
}
async function loadControllerConfig(){
  if(controllerConfig)return controllerConfig;
  const r=await fetch('/api/controller/config',{cache:'no-store'});
  if(!r.ok)throw new Error(`controller config HTTP ${r.status}`);
  controllerConfig=await r.json();
  return controllerConfig;
}
function setControllerModalError(message){
  const el=$('controllerModalError');
  if(!message){el.textContent='';el.classList.add('hidden');return;}
  el.textContent=message; el.classList.remove('hidden');
}
function shellPreviewArg(arg){
  const s=String(arg); return /^[A-Za-z0-9_./:=+-]+$/.test(s)?s:`'${s.replaceAll("'","'\\''")}'`;
}
function controllerParamValue(spec){
  const input=$(`controller-param-${spec.name}`);
  if(!input)return spec.default;
  if(spec.type==='bool')return !!input.checked;
  const x=Number(input.value);
  return spec.type==='int'?Math.round(x):x;
}
function controllerModalParameters(){
  const out={};
  for(const spec of controllerConfig?.parameter_specs||[])out[spec.name]=controllerParamValue(spec);
  return out;
}
function updateControllerCommandPreview(){
  if(!controllerConfig)return;
  const cmd=[controllerConfig.controller_python,controllerConfig.controller_script,...(controllerConfig.fixed_args||[])];
  for(const spec of controllerConfig.parameter_specs||[]){
    const value=controllerParamValue(spec);
    if(spec.type==='bool'){if(value)cmd.push(spec.flag);}
    else cmd.push(`${spec.flag}=${value}`);
  }
  $('controllerCommandPreview').textContent=cmd.map(shellPreviewArg).join(' \\\n  ');
}
function setControllerParam(spec,value){
  const input=$(`controller-param-${spec.name}`); if(!input)return;
  if(spec.type==='bool'){input.checked=!!value;return;}
  input.value=String(value);
  const range=$(`controller-range-${spec.name}`); if(range)range.value=String(value);
}
function resetControllerDefaults(){
  if(!controllerConfig)return;
  for(const spec of controllerConfig.parameter_specs||[])setControllerParam(spec,controllerConfig.known_good?.[spec.name]??spec.default);
  updateControllerCommandPreview(); setControllerModalError('');
}
function makeControllerParamRow(spec){
  const row=document.createElement('div');
  row.className=`controller-param-row${spec.type==='bool'?' controller-bool-row':''}`;
  const label=document.createElement('label'); label.textContent=spec.label; label.title=`${spec.flag}${spec.unit?` · ${spec.unit}`:''}`;
  if(spec.type==='bool'){
    const wrap=document.createElement('label'); wrap.className='toggle';
    const input=document.createElement('input'); input.type='checkbox'; input.id=`controller-param-${spec.name}`; input.checked=!!spec.default;
    const text=document.createElement('span'); text.textContent=spec.default?'Enabled':'Disabled';
    input.addEventListener('change',()=>{text.textContent=input.checked?'Enabled':'Disabled';updateControllerCommandPreview();});
    wrap.append(input,text); row.append(label,wrap); return row;
  }
  const range=document.createElement('input'); range.type='range'; range.id=`controller-range-${spec.name}`; range.min=spec.min; range.max=spec.max; range.step=spec.step; range.value=spec.default;
  const numWrap=document.createElement('div'); numWrap.className='controller-param-number';
  const number=document.createElement('input'); number.type='number'; number.id=`controller-param-${spec.name}`; number.min=spec.min; number.max=spec.max; number.step=spec.step; number.value=spec.default;
  const unit=document.createElement('span'); unit.textContent=spec.unit||'';
  const sync=(from,to)=>{to.value=from.value;updateControllerCommandPreview();};
  range.addEventListener('input',()=>sync(range,number)); number.addEventListener('input',()=>sync(number,range));
  numWrap.append(number,unit); row.append(label,range,numWrap); return row;
}
function buildControllerModal(cfg){
  const locked=$('controllerLockedSettings'); locked.innerHTML='';
  for(const item of cfg.locked_settings||[]){const d=document.createElement('div');const a=document.createElement('span');a.textContent=item.label;const b=document.createElement('strong');b.textContent=item.value;b.title=item.value;d.append(a,b);locked.appendChild(d);}
  const sections=$('controllerParameterSections'); sections.innerHTML='';
  const groups=new Map();
  for(const spec of cfg.parameter_specs||[]){if(!groups.has(spec.section))groups.set(spec.section,[]);groups.get(spec.section).push(spec);}
  for(const [name,specs] of groups){
    const section=document.createElement('section');section.className='controller-param-section';
    const head=document.createElement('header');head.textContent=name.toUpperCase();
    const list=document.createElement('div');list.className='controller-param-list';
    for(const spec of specs)list.appendChild(makeControllerParamRow(spec));
    section.append(head,list);sections.appendChild(section);
  }
  $('controllerModalRuntime').textContent=`${cfg.controller_python} · ${cfg.controller_script}`;
  resetControllerDefaults();
}
function updateControllerStartEnabled(){
  const cfg=controllerConfig;
  const ready=!!cfg?.enabled&&!!cfg?.controller_script_exists&&!!cfg?.controller_hash_match&&!!cfg?.controller_python_exists&&controllerStatus?.state==='STOPPED'&&$('controllerSafetyAck').checked&&!controllerActionBusy;
  $('controllerStartBtn').disabled=!ready;
}
async function openControllerModal(){
  setControllerModalError('');
  $('controllerModal').classList.remove('hidden');
  document.body.classList.add('modal-open');
  $('controllerManagementKey').value=currentManagementKey();
  $('controllerSafetyAck').checked=false;
  try{
    const cfg=await loadControllerConfig();
    buildControllerModal(cfg);
    if(!cfg.enabled)setControllerModalError('Process actions are disabled. Restart the dashboard with G1_DASHBOARD_PROCESS_ACTIONS=1.');
    else if(!cfg.controller_script_exists)setControllerModalError(`Controller script not found: ${cfg.controller_script}`);
    else if(!cfg.controller_hash_match)setControllerModalError(`Controller hash mismatch. Expected ${cfg.controller_expected_sha256}; got ${cfg.controller_actual_sha256||'unreadable'}. Launch is locked.`);
    else if(!cfg.controller_python_exists)setControllerModalError(`Controller Python not executable: ${cfg.controller_python}`);
    else if(cfg?.inspire_dependency?.state==='CONFLICT')setControllerModalError('Multiple/conflicting inspire_g1 processes detected. Resolve them before launching the managed listener.');
    else if(cfg?.inspire_dependency?.state==='UNAVAILABLE'&&!cfg?.inspire_dependency?.helper_installed)setControllerModalError('Inspire lifecycle setup required on PC2: run sudo ./install_inspire_helper.sh once, then reopen this window.');
    updateControllerStartEnabled();
  }catch(err){setControllerModalError(String(err));$('controllerStartBtn').disabled=true;}
}
function closeControllerModal(){ $('controllerModal').classList.add('hidden'); document.body.classList.remove('modal-open'); }
async function controllerPost(path,payload={}){
  const key=String($('controllerManagementKey')?.value||currentManagementKey()||'').trim();
  if(!key)throw new Error('Enter the management key first.');
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-G1-Management-Key':key},body:JSON.stringify(payload)});
  let body={}; try{body=await r.json();}catch{}
  if(r.status===401){storeManagementKey('');showManagementKeyPrompt('Management key rejected. Enter the key printed by the currently running ./start_dashboard.sh.');}
  if(!r.ok)throw new Error(body.error||`HTTP ${r.status}`);
  storeManagementKey(key);
  if(body.controller)renderControllerProcess(body.controller);
  return body;
}

function xrActionButtonLabel(operation){
  if(operation==='REQUEST_XR')return 'ENTER TELEOP';
  if(operation==='CANCEL_XR_REQUEST')return 'CANCEL ENTRY';
  if(operation==='HAND_BACK_ARMS')return 'EXIT TELEOP';
  if(operation==='TRANSITION_IN_PROGRESS')return 'TRANSITIONING…';
  return 'TELEOP ACTION';
}
function setXrActionResult(text,tone=null){
  const el=$('xrActionResult'); if(!el)return;
  el.textContent=text||'';
  el.classList.remove('good','warn');
  if(tone)el.classList.add(tone);
}
async function requestXrAction(operation){
  const key=currentManagementKey();
  if(!key)throw new Error('Enter the management key first.');
  const r=await fetch('/api/controller/action',{
    method:'POST',
    headers:{'Content-Type':'application/json','X-G1-Management-Key':key},
    body:JSON.stringify({operation})
  });
  let body={}; try{body=await r.json();}catch{}
  if(body.controller)renderControllerProcess(body.controller);
  if(r.status===401){storeManagementKey('');showManagementKeyPrompt('Management key rejected. Enter the key printed by the currently running ./start_dashboard.sh.');}
  if(!r.ok){
    const reason=body?.action?.reason||body?.error||`HTTP ${r.status}`;
    throw new Error(reason);
  }
  return body.action||{};
}
$('managementKeySubmitBtn').addEventListener('click',async()=>{
  if(managementAuthBusy)return;
  managementAuthBusy=true;$('managementKeySubmitBtn').disabled=true;setManagementKeyError('');
  try{
    const key=$('managementKeyInput').value.trim();
    await verifyManagementKey(key);
    storeManagementKey(key);
    const next=managementAfterAuth;
    closeManagementKeyPrompt();
    if(next)setTimeout(next,0);
  }catch(err){setManagementKeyError(err.message||String(err));}
  finally{managementAuthBusy=false;$('managementKeySubmitBtn').disabled=false;}
});
$('managementKeyInput').addEventListener('keydown',(e)=>{if(e.key==='Enter')$('managementKeySubmitBtn').click();});
$('managementKeyReadOnlyBtn').addEventListener('click',()=>closeManagementKeyPrompt());

$('controllerConfigureBtn').addEventListener('click',()=>{
  if(!currentManagementKey()){showManagementKeyPrompt('Enter the management key before configuring a managed listener.',()=>openControllerModal());return;}
  openControllerModal();
});
$('controllerModalCloseBtn').addEventListener('click',closeControllerModal);
$('controllerModalCancelBtn').addEventListener('click',closeControllerModal);
$('controllerModal').addEventListener('click',(e)=>{if(e.target===$('controllerModal'))closeControllerModal();});
$('controllerManagementKey').addEventListener('input',()=>storeManagementKey($('controllerManagementKey').value));
$('controllerSafetyAck').addEventListener('change',updateControllerStartEnabled);
$('controllerResetDefaultsBtn').addEventListener('click',resetControllerDefaults);
$('controllerStartBtn').addEventListener('click',async()=>{
  if(controllerActionBusy)return; controllerActionBusy=true; setControllerModalError(''); $('controllerStartBtn').disabled=true;
  try{
    await controllerPost('/api/controller/start',{parameters:controllerModalParameters()});
    closeControllerModal(); await pollControllerProcess();
  }catch(err){setControllerModalError(err.message||String(err));}
  finally{controllerActionBusy=false; renderControllerProcess(controllerStatus||{}); if(!$('controllerModal').classList.contains('hidden'))updateControllerStartEnabled();}
});
$('controllerStopBtn').addEventListener('click',async()=>{
  if(controllerActionBusy)return;
  if(!currentManagementKey()){showManagementKeyPrompt('Enter the management key to request a controlled listener stop.',$('controllerStopBtn').click.bind($('controllerStopBtn')));return;}
  if(!window.confirm('Request a controlled stop? The controller performs its normal handback first; a dashboard-managed Inspire server is stopped only after the controller exits.'))return;
  controllerActionBusy=true;renderControllerProcess(controllerStatus||{});
  try{await controllerPost('/api/controller/stop',{});await pollControllerProcess();}
  catch(err){window.alert(`Stop request failed: ${err.message||err}`);}
  finally{controllerActionBusy=false;renderControllerProcess(controllerStatus||{});}
});

$('xrActionBtn').addEventListener('click',async()=>{
  if(xrActionBusy)return;
  if(!currentManagementKey()){showManagementKeyPrompt('Enter the management key to use ENTER/EXIT TELEOP.',$('xrActionBtn').click.bind($('xrActionBtn')));return;}
  const handover=latestActionReadiness?.xr_handover||{};
  const operation=handover.operation||'NONE';
  if(!['REQUEST_XR','CANCEL_XR_REQUEST','HAND_BACK_ARMS'].includes(operation))return;
  xrActionBusy=true;
  $('xrActionBtn').disabled=true;
  setXrActionResult(`Sending ${xrActionButtonLabel(operation).toLowerCase()} request…`);
  try{
    const response=await requestXrAction(operation);
    setXrActionResult(`${response.status||'ACCEPTED'} · ${response.reason||operation}`,'good');
  }catch(err){
    setXrActionResult(err.message||String(err),'warn');
  }finally{
    xrActionBusy=false;
    if(latestEnv?.telemetry)renderActionReadiness(latestEnv.telemetry);
    await pollControllerProcess();
  }
});
document.addEventListener('keydown',(e)=>{if(e.key==='Escape'&&!$('controllerModal').classList.contains('hidden'))closeControllerModal();});

/* ---------- Camera / teleimager process + WebRTC ---------- */
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
function cameraProcessTone(state){
  if(state==='RUNNING'||state==='RUNNING_EXTERNAL')return 'good';
  if(state==='STOPPING')return 'warn';
  if(state==='CONFLICT'||state==='UNAVAILABLE')return 'bad';
  return null;
}
function renderCameraProcess(st){
  cameraProcessStatus=st||{};
  const state=cameraProcessStatus.state||'UNAVAILABLE';
  const badge=$('cameraProcessState');
  const text=state==='RUNNING'?'SERVER ON':state==='RUNNING_EXTERNAL'?'SERVER EXT':state==='STOPPED'?'SERVER OFF':state==='STOPPING'?'SERVER STOPPING':state==='CONFLICT'?'SERVER CONFLICT':'SERVER SETUP';
  badge.textContent=text;badge.className=`camera-process-state${cameraProcessTone(state)?` ${cameraProcessTone(state)}`:''}`;
  const connected=!!cameraPc && ['connected','connecting','new'].includes(cameraPc.connectionState||'new');
  if(state==='RUNNING'){
    $('cameraStopBtn').textContent='Stop camera';
    if(!connected)$('cameraStopBtn').classList.remove('hidden');
  }else if(state==='RUNNING_EXTERNAL'){
    $('cameraStopBtn').textContent='Disconnect';
    if(!connected)$('cameraStopBtn').classList.add('hidden');
  }else if(!connected){
    $('cameraStopBtn').classList.add('hidden');
  }
  if(state==='STOPPED')$('cameraConnectBtn').textContent='Start & connect';
  else $('cameraConnectBtn').textContent='Connect camera';
  renderCameraModes();
  updateCameraButtons(latestEnv?.telemetry||{});
}
async function pollCameraProcess(){
  if(cameraProcessPollBusy)return cameraProcessStatus;
  cameraProcessPollBusy=true;
  try{
    const r=await fetch('/api/camera',{cache:'no-store'});
    if(!r.ok)throw new Error(`HTTP ${r.status}`);
    const st=await r.json();renderCameraProcess(st);return st;
  }catch(err){
    cameraProcessStatus={state:'UNAVAILABLE',can_start:false,can_stop:false,last_error:String(err)};
    renderCameraProcess(cameraProcessStatus);
    console.debug('camera process endpoint unavailable',err);
    return cameraProcessStatus;
  }finally{cameraProcessPollBusy=false;}
}
async function cameraProcessPost(path,payload={}){
  const key=currentManagementKey();
  if(!key)throw new Error('Enter the management key first.');
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-G1-Management-Key':key},body:JSON.stringify(payload||{})});
  let body={};try{body=await r.json();}catch{}
  if(r.status===401){storeManagementKey('');showManagementKeyPrompt('Management key rejected. Enter the key printed by the currently running ./start_dashboard.sh.');}
  if(!r.ok)throw new Error(body.error||`HTTP ${r.status}`);
  if(body.camera)renderCameraProcess(body.camera);
  return body.camera||cameraProcessStatus;
}
function cameraModeLabel(mode){
  return ({rgb:'RGB',depth:'DEPTH',overlay:'OVERLAY',near:'NEAR',disparity:'DISPARITY',pointcloud:'POINT CLOUD',topdown:'TOP-DOWN'})[mode]||String(mode||'—').toUpperCase();
}
function renderCameraModes(){
  const st=cameraProcessStatus||{};
  const requested=st.mode_requested||'rgb';
  const actual=st.mode_actual||null;
  const controllable=!!st.mode_control && st.state==='RUNNING';
  document.querySelectorAll('[data-camera-mode]').forEach(btn=>{
    const mode=btn.dataset.cameraMode;
    btn.classList.toggle('active',mode===requested);
    btn.disabled=!controllable || cameraModeActionBusy;
    btn.title=controllable?'Switch the shared Teleimager WebRTC output without reconnecting.':'Mode switching requires the dashboard-managed RealSense camera server.';
  });

  const badge=$('cameraModeState');
  if(badge){
    const waiting=controllable && actual!==requested;
    badge.textContent=waiting?`MODE ${cameraModeLabel(requested)}…`:`MODE ${cameraModeLabel(actual||requested)}`;
    badge.className=`camera-mode-state${waiting?' warn':actual?' good':''}`;
  }

  const yoloToggle=$('cameraYoloToggle');
  const yoloControl=$('cameraYoloControl');
  const yoloBadge=$('cameraYoloState');
  const yoloRequested=!!st.yolo_requested;
  const yoloControllable=!!st.yolo_control && st.state==='RUNNING';
  const yoloWaiting=yoloControllable && (
    !st.yolo_ack_online ||
    st.yolo_actual!==yoloRequested
  );

  if(yoloToggle){
    yoloToggle.checked=yoloRequested;
    yoloToggle.indeterminate=yoloWaiting;
    yoloToggle.disabled=!yoloControllable || cameraYoloActionBusy;
  }

  if(yoloControl){
    yoloControl.classList.toggle('active',yoloRequested);
    yoloControl.classList.toggle(
      'busy',
      cameraYoloActionBusy || yoloWaiting
    );
    yoloControl.title=yoloControllable
      ?'Run one YOLO inference pipeline on RGB and reuse detections across aligned camera views.'
      :'YOLO control requires the dashboard-managed RealSense camera server.';
  }

  if(yoloBadge){
    yoloBadge.textContent=yoloWaiting
      ?`YOLO ${yoloRequested?'ON':'OFF'}…`
      :`YOLO ${yoloRequested?'ON':'OFF'}`;
    yoloBadge.className=`camera-yolo-state${yoloWaiting?' warn':yoloRequested?' good':''}`;
  }

  renderPointViewControls();
}

const POINT_VIEW_DEFAULT={yaw_deg:22,pitch_deg:14,distance_m:3.16,target_z_m:2.0};
function pointClamp(x,lo,hi){return Math.max(lo,Math.min(hi,Number(x)));}
function normalizePointView(raw={}){
  return {
    yaw_deg:pointClamp(finite(raw.yaw_deg)?raw.yaw_deg:POINT_VIEW_DEFAULT.yaw_deg,-180,180),
    pitch_deg:pointClamp(finite(raw.pitch_deg)?raw.pitch_deg:POINT_VIEW_DEFAULT.pitch_deg,-82,82),
    distance_m:pointClamp(finite(raw.distance_m)?raw.distance_m:POINT_VIEW_DEFAULT.distance_m,1,8),
    target_z_m:pointClamp(finite(raw.target_z_m)?raw.target_z_m:POINT_VIEW_DEFAULT.target_z_m,.5,5),
  };
}
function pointCloudActive(){return (cameraProcessStatus?.mode_requested||'rgb')==='pointcloud'&&cameraProcessStatus?.state==='RUNNING';}
function currentPointView(){return normalizePointView(PointCloud3D.getView?.()||pointViewLocal||POINT_VIEW_DEFAULT);}

async function syncPointViewToCamera(view){
  // Local WebGL orbit never requires authentication. If management access is
  // unlocked, sync only the final viewpoint to the server-rendered WebRTC
  // point cloud so the headset roughly follows without making drag latency
  // depend on HTTP/H.264 round trips.
  if(pointViewSendBusy||!currentManagementKey()||cameraProcessStatus?.state!=='RUNNING'||!cameraProcessStatus?.point_view_control)return;
  pointViewSendBusy=true;
  try{
    const st=await cameraProcessPost('/api/camera/view',{view:normalizePointView(view)});
    if(st)cameraProcessStatus=st;
  }catch(err){console.warn('headset point-view sync failed',err);}
  finally{pointViewSendBusy=false;renderPointViewControls();}
}

const pointCloudViewer=PointCloud3D.init($('pointCloudCanvas'),(view,final)=>{
  pointViewLocal=normalizePointView(view);
  renderPointViewControls();
  if(final)syncPointViewToCamera(pointViewLocal);
});
PointCloud3D.setView(POINT_VIEW_DEFAULT,false);

function updatePointCloudReadout(){
  const readout=$('pointViewReadout');if(!readout)return;
  const v=currentPointView(),st=PointCloud3D.getStats();
  const points=st.points?`${(st.points/1000).toFixed(st.points>=10000?0:1)}k pts`:'waiting';
  const hz=st.dataHz>0?`${st.dataHz.toFixed(0)} Hz`:'— Hz';
  readout.textContent=`Y ${v.yaw_deg>=0?'+':''}${v.yaw_deg.toFixed(0)}° · P ${v.pitch_deg>=0?'+':''}${v.pitch_deg.toFixed(0)}° · ${v.distance_m.toFixed(1)}m · ${points} · ${hz}`;
}
function renderPointViewControls(){
  const strip=$('pointViewStrip');if(!strip)return;
  const show=(cameraProcessStatus?.mode_requested||'rgb')==='pointcloud'&&cameraProcessStatus?.state==='RUNNING';
  strip.classList.toggle('hidden',!show);
  strip.querySelectorAll('[data-point-view-preset]').forEach(b=>b.disabled=!show);
  const stage=$('cameraStage');
  if(stage)stage.classList.toggle('pointcloud-webgl-active',show);
  PointCloud3D.setVisible(show);
  if(show&&!pointViewInitialized){
    const initial=normalizePointView(cameraProcessStatus?.point_view_actual||cameraProcessStatus?.point_view_requested||POINT_VIEW_DEFAULT);
    PointCloud3D.setView(initial,false);pointViewInitialized=true;
  }
  updatePointCloudReadout();
  if(show)ensurePointCloudPoll();
}
async function pollPointCloud(){
  pointCloudPollTimer=null;
  if(!pointCloudActive())return;
  if(pointCloudFetchBusy){ensurePointCloudPoll();return;}
  pointCloudFetchBusy=true;
  try{
    const r=await fetch('/api/camera/pointcloud',{cache:'no-store'});
    if(r.ok&&r.status!==204){
      const b=await r.arrayBuffer();
      PointCloud3D.setSnapshot(b);
      updatePointCloudReadout();
    }
  }catch(err){console.debug('point-cloud snapshot unavailable',err);}
  finally{pointCloudFetchBusy=false;if(pointCloudActive())pointCloudPollTimer=setTimeout(pollPointCloud,65);}
}
function ensurePointCloudPoll(){
  if(!pointCloudActive()||pointCloudPollTimer||pointCloudFetchBusy)return;
  pointCloudPollTimer=setTimeout(pollPointCloud,0);
}
const POINT_VIEW_PRESETS={
  front:{yaw_deg:0,pitch_deg:0,distance_m:3.0,target_z_m:2.0},
  left:{yaw_deg:-70,pitch_deg:12,distance_m:3.35,target_z_m:2.0},
  right:{yaw_deg:70,pitch_deg:12,distance_m:3.35,target_z_m:2.0},
  above:{yaw_deg:0,pitch_deg:78,distance_m:3.8,target_z_m:2.0},
  reset:POINT_VIEW_DEFAULT,
};
function applyPointViewPreset(name){
  const preset=POINT_VIEW_PRESETS[name];if(!preset)return;
  pointViewLocal=normalizePointView(preset);
  PointCloud3D.setView(pointViewLocal,true);
  updatePointCloudReadout();
}

async function setCameraMode(mode){
  mode=String(mode||'').toLowerCase();
  if(!['rgb','depth','overlay','near','disparity','pointcloud','topdown'].includes(mode)||cameraModeActionBusy)return;
  if(!currentManagementKey()){
    showManagementKeyPrompt('Enter the management key to switch the shared camera view.',()=>setCameraMode(mode));
    return;
  }
  if(cameraProcessStatus?.state!=='RUNNING'||!cameraProcessStatus?.mode_control){
    window.alert('Camera modes are available only while the dashboard-managed RealSense camera server is running.');
    return;
  }
  cameraModeActionBusy=true;
  renderCameraModes();
  try{
    const st=await cameraProcessPost('/api/camera/mode',{mode});
    if(st)renderCameraProcess(st);
  }catch(err){
    window.alert(`Camera mode switch failed: ${err.message||err}`);
  }finally{
    cameraModeActionBusy=false;
    await pollCameraProcess();
    renderCameraModes();
  }
}
async function setCameraYolo(enabled){
  enabled=!!enabled;
  if(cameraYoloActionBusy)return;

  if(!currentManagementKey()){
    renderCameraModes();
    showManagementKeyPrompt(
      'Enter the management key to enable or disable YOLO.',
      ()=>setCameraYolo(enabled)
    );
    return;
  }

  if(
    cameraProcessStatus?.state!=='RUNNING' ||
    !cameraProcessStatus?.yolo_control
  ){
    renderCameraModes();
    window.alert(
      'YOLO control is available only while the dashboard-managed RealSense camera server is running.'
    );
    return;
  }

  cameraYoloActionBusy=true;
  renderCameraModes();

  try{
    const st=await cameraProcessPost(
      '/api/camera/yolo',
      {enabled}
    );
    if(st)renderCameraProcess(st);
  }catch(err){
    window.alert(`YOLO switch failed: ${err.message||err}`);
  }finally{
    cameraYoloActionBusy=false;
    await pollCameraProcess();
    renderCameraModes();
  }
}

document.querySelectorAll('[data-camera-mode]').forEach(btn=>btn.addEventListener('click',()=>setCameraMode(btn.dataset.cameraMode)));
$('cameraYoloToggle')?.addEventListener('change',event=>setCameraYolo(event.target.checked));
document.querySelectorAll('[data-point-view-preset]').forEach(btn=>btn.addEventListener('click',()=>applyPointViewPreset(btn.dataset.pointViewPreset)));

function updateCameraButtons(t){
  const base=cameraBaseFromTelemetry(t);
  cameraUrl=cameraOfferFromTelemetry(t);
  const link=$('cameraTrustLink');
  if(base){ link.href=base; link.classList.remove('hidden'); } else { link.removeAttribute('href'); link.classList.add('hidden'); }
  const configured=val(t,['camera','webrtc_enabled'], cameraUrl?true:false);
  const state=cameraProcessStatus?.state||'UNAVAILABLE';
  const serverActionable=state==='RUNNING'||state==='RUNNING_EXTERNAL'||(state==='STOPPED'&&cameraProcessStatus?.can_start);
  $('cameraConnectBtn').disabled=!configured || !cameraUrl || cameraConnecting || cameraProcessActionBusy || !serverActionable;
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
  $('cameraOverlayText').textContent=cameraProcessStatus?.state==='RUNNING'?'teleimager is running; connect when ready.':'Click Start & connect to launch teleimager.';
  $('cameraConnectBtn').classList.remove('hidden');
  if(cameraProcessStatus?.state==='RUNNING'){$('cameraStopBtn').textContent='Stop camera';$('cameraStopBtn').classList.remove('hidden');}
  else $('cameraStopBtn').classList.add('hidden');
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
      $('cameraStopBtn').textContent=cameraProcessStatus?.state==='RUNNING'?'Stop camera':'Disconnect';
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
    $('cameraOverlayText').textContent='teleimager may still be starting, the camera may be occupied, or its certificate may need trust.';
    cameraState('ERROR','bad');
  }finally{ cameraConnecting=false; updateCameraButtons(latestEnv?.telemetry||{}); }
}
async function startAndConnectCamera(){
  if(cameraProcessActionBusy||cameraConnecting)return;
  let st=await pollCameraProcess();
  if(st?.state==='STOPPED'){
    if(!currentManagementKey()){showManagementKeyPrompt('Enter the management key to start the camera server.',()=>startAndConnectCamera());return;}
    cameraProcessActionBusy=true;updateCameraButtons(latestEnv?.telemetry||{});
    $('cameraOverlayTitle').textContent='Starting camera server…';
    $('cameraOverlayText').textContent='Launching the dashboard-managed RealSense Teleimager stream on PC2.';
    cameraState('STARTING','warn');
    try{st=await cameraProcessPost('/api/camera/start');}
    catch(err){cameraState('ERROR','bad',err.message||String(err));window.alert(`Camera server start failed: ${err.message||err}`);return;}
    finally{cameraProcessActionBusy=false;}
  }
  if(st?.state==='CONFLICT'||st?.state==='UNAVAILABLE'||st?.state==='STOPPING'){
    cameraState('ERROR','bad',st?.last_error||`Camera process state: ${st?.state||'unknown'}`);return;
  }
  // teleimager process starts before its HTTPS/WebRTC socket. Give it a bounded
  // readiness window, then attempt the normal WebRTC negotiation.
  for(let i=0;i<24;i++){
    st=await pollCameraProcess();
    if(st?.port_ready)break;
    if(!['RUNNING','RUNNING_EXTERNAL'].includes(st?.state))break;
    await new Promise(resolve=>setTimeout(resolve,250));
  }
  await connectCamera();
}
async function stopCameraButton(){
  if(cameraProcessActionBusy)return;
  const st=await pollCameraProcess();
  if(st?.state==='RUNNING'){
    if(!currentManagementKey()){showManagementKeyPrompt('Enter the management key to stop the camera server.',()=>stopCameraButton());return;}
    if(!window.confirm('Stop the dashboard-managed camera server? The browser video connection will close first.'))return;
    stopCamera({silent:true});
    cameraProcessActionBusy=true;updateCameraButtons(latestEnv?.telemetry||{});
    try{await cameraProcessPost('/api/camera/stop');cameraState('OFFLINE',null,'teleimager stop requested.');}
    catch(err){window.alert(`Camera stop failed: ${err.message||err}`);}
    finally{cameraProcessActionBusy=false;await pollCameraProcess();}
    return;
  }
  // Never stop an externally-owned camera server from the dashboard.
  stopCamera();
}
$('cameraConnectBtn').addEventListener('click',()=>startAndConnectCamera());
$('cameraStopBtn').addEventListener('click',()=>stopCameraButton());

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
  latestActionReadiness=actions;
  const has=!!actions?.schema;
  const available=has && handover?.available===true;
  const label=has?(handover?.label||'No action'):'Controller action telemetry required';
  const operation=has?(handover?.operation||'NONE'):'—';
  const next=has?(handover?.would_enter_state||'—'):'—';
  const reason=has?(handover?.reason||'—'):'Controller action-readiness telemetry required.';
  const channel=actions?.request_channel_enabled===true;
  const managerReady=controllerStatus?.can_request_action===true;

  $('actionLabel').textContent=label;
  $('actionOperation').textContent=operation;
  $('actionNextState').textContent=next;
  $('actionReason').textContent=reason;
  const readinessText=!has?'READ ONLY':!channel?'READ ONLY':available?'READY':'BLOCKED';
  const readinessTone=has&&channel?(available?'good':'warn'):null;
  setChip($('actionReadinessChip'),readinessText,readinessTone);

  const condChip=(id,text,ok,neutral=false)=>{const el=$(id); el.textContent=text; el.classList.toggle('cond-good',!!ok); el.classList.toggle('cond-warn',!ok&&!neutral); el.classList.toggle('cond-neutral',!!neutral);};
  condChip('actionCondLowstate',`LOW ${cond.lowstate_ok?'OK':'BAD'}`,cond.lowstate_ok===true,!has);
  condChip('actionCondXr',`XR ${cond.xr_ok?'OK':'WAIT'}`,cond.xr_ok===true,!has);
  condChip('actionCondStop',`STOP ${cond.stop_gate_ready?'READY':cond.stop_gate_instant?'TIMING':'WAIT'}`,cond.stop_gate_ready===true,!has);
  condChip('actionCondFault',`FAULT ${cond.safety_fault_clear?'CLEAR':'HOLD'}`,cond.safety_fault_clear===true,!has);

  const actionBtn=$('xrActionBtn');
  actionBtn.textContent=xrActionButtonLabel(operation);
  actionBtn.disabled=xrActionBusy || !channel || !available || !managerReady || !['REQUEST_XR','CANCEL_XR_REQUEST','HAND_BACK_ARMS'].includes(operation);
  actionBtn.classList.toggle('danger-btn',operation==='HAND_BACK_ARMS');
  actionBtn.classList.toggle('primary-btn',operation!=='HAND_BACK_ARMS');
  if(!xrActionBusy){
    if(!channel)setXrActionResult('Controller request channel disabled.');
    else if(!managerReady)setXrActionResult('Start the listener from this dashboard to enable XR requests.');
    else if(!available)setXrActionResult(reason,'warn');
    else setXrActionResult('Controller will re-check this action when clicked.');
  }

  setChip($('statusActionChip'),readinessText,readinessTone);
  $('statusActionLabel').textContent=label;
  $('statusActionOperation').textContent=operation;
  $('statusActionNextState').textContent=next;
  $('statusActionChannel').textContent=channel?'ENABLED':'DISABLED';
  setTone($('statusActionChannel'),channel?'good':null);
  $('statusActionReason').textContent=reason+(channel?'':' Browser requests are unavailable until the controller action channel is enabled.');
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
  const c=t.camera||{}; const res=c.width&&c.height?`${c.width}×${c.height}`:'—'; const sharedMode=cameraProcessStatus?.mode_actual||cameraProcessStatus?.mode_requested||'rgb'; $('cameraMeta').textContent=`${res} · ${n(c.display_fps,0,' fps')} · ${cameraModeLabel(sharedMode)}`; $('cameraHudInfo').textContent=`${res} · ${n(c.display_fps,0,' fps')} · ${cameraModeLabel(sharedMode)} · shared with headset`; $('statusCamera').textContent=c.webrtc_enabled?`WebRTC ${res}`:'disabled'; updateCameraButtons(t); if(!cameraPc&&!cameraConnecting)setChip($('cameraChip'),c.webrtc_enabled?'CAMERA OFFLINE':'CAMERA OFF',c.webrtc_enabled?'warn':null);
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
let serviceControlConfig=null;
let serviceControlPollBusy=false;
let serviceActionBusyName=null;

const serviceEnabled=(s)=>typeof s?.enabled==='boolean'?s.enabled:Number(s?.status)===0?true:Number(s?.status)===1?false:null;
function servicePolicyFor(svc){
  const name=String(svc?.name||'');
  if(svc?.protect)return 'PROTECTED';
  const p=serviceControlConfig?.policy||{};
  if(Array.isArray(p.hard_deny_services)&&p.hard_deny_services.includes(name))return 'PROTECTED';
  if(Array.isArray(p.read_only_services)&&p.read_only_services.includes(name))return 'READ_ONLY';
  if(Array.isArray(p.allowed_services)&&p.allowed_services.includes(name))return 'ALLOWED';
  return 'UNKNOWN';
}
function renderServiceControlConfig(cfg){
  serviceControlConfig=cfg||{};
  const enabled=!!serviceControlConfig.enabled;
  const worker=serviceControlConfig.worker||{};
  const ready=enabled&&worker.status==='READY';
  setChip($('serviceControlChip'),ready?'CONTROL READY':enabled?'CONTROL OFFLINE':'CONTROL OFF',ready?'good':enabled?'warn':null);
  if($('serviceControlChip'))$('serviceControlChip').title=worker.reason||'';
  renderServiceList();
}
async function pollServiceControl(){
  if(serviceControlPollBusy)return;
  serviceControlPollBusy=true;
  try{
    const r=await fetch('/api/services/control',{cache:'no-store'});
    if(!r.ok)throw new Error(`HTTP ${r.status}`);
    renderServiceControlConfig(await r.json());
  }catch(err){
    serviceControlConfig={enabled:false,worker:{status:'OFFLINE',reason:String(err)},policy:{}};
    setChip($('serviceControlChip'),'CONTROL OFFLINE','warn');
    console.debug('service control endpoint unavailable',err);
    renderServiceList();
  }finally{serviceControlPollBusy=false;}
}
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
  if(api?.error){err.textContent=api.error;}else{err.textContent=api?.module?`inventory via ${api.module} · writes isolated in allowlisted worker`:'service inventory';}
  err.classList.remove('hidden');
  lastRobotServices=Array.isArray(api?.services)?api.services:[];
  renderServiceList();
}
function servicePolicyClassName(policy){return String(policy||'UNKNOWN').toLowerCase();}
function renderServiceList(){
  const list=$('robotServiceList'); if(!list)return;
  list.innerHTML='';
  const textFilter=String($('serviceFilter')?.value||'').trim().toLowerCase();
  const stateFilter=String($('serviceStateFilter')?.value||'ALL');
  const policyFilter=String($('servicePolicyFilter')?.value||'ALL');
  const all=Array.isArray(lastRobotServices)?lastRobotServices:[];
  const classified=all.map(s=>({svc:s,enabled:serviceEnabled(s),policy:servicePolicyFor(s)}));
  const services=classified.filter(({svc,enabled,policy})=>{
    if(textFilter&&!String(svc?.name||'').toLowerCase().includes(textFilter))return false;
    if(stateFilter==='ON'&&enabled!==true)return false;
    if(stateFilter==='OFF'&&enabled!==false)return false;
    if(policyFilter!=='ALL'&&policy!==policyFilter)return false;
    return true;
  });
  const on=classified.filter(x=>x.enabled===true).length;
  const off=classified.filter(x=>x.enabled===false).length;
  const unknownState=all.length-on-off;
  const allowedCount=classified.filter(x=>x.policy==='ALLOWED').length;
  const protectedCount=classified.filter(x=>x.policy==='PROTECTED').length;
  const counts=$('robotServiceCounts');
  const filtering=!!textFilter||stateFilter!=='ALL'||policyFilter!=='ALL';
  if(counts)counts.textContent=`${on} on · ${off} off${unknownState?` · ${unknownState} state?`:''} · ${allowedCount} allowed · ${protectedCount} protected${filtering?` · ${services.length} shown`:''}`;
  if(!services.length){list.innerHTML=`<div class="empty">${all.length?'No services match the active filters':'No service inventory yet'}</div>`;return;}
  const workerReady=!!serviceControlConfig?.enabled&&serviceControlConfig?.worker?.status==='READY';
  for(const {svc,enabled,policy} of services){
    const row=document.createElement('div'); row.className='service-row';
    const name=document.createElement('span'); name.className='service-name'; name.textContent=String(svc?.name||'?'); name.title=name.textContent;
    const policyEl=document.createElement('span'); policyEl.className=`service-policy ${servicePolicyClassName(policy)}`; policyEl.textContent=policy==='READ_ONLY'?'READ ONLY':policy;
    if(svc?.protect)policyEl.title='Unitree RobotState protect flag is set';
    const state=document.createElement('strong');
    const rawStatus=Number(svc?.status);
    state.textContent=enabled===true?'ON':enabled===false?'OFF':`STATE ${Number.isFinite(rawStatus)?rawStatus:'—'}`;
    state.title=Number.isFinite(rawStatus)?`Unitree raw service status: ${rawStatus} (0=ON, 1=OFF)`:'unknown service state';
    if(enabled===true)state.className='good-text';else if(enabled===false)state.className='dim';else state.className='warn-text';
    const control=document.createElement('span');control.className='service-control-cell';
    if(serviceActionBusyName===name.textContent){
      const busy=document.createElement('span');busy.className='service-switching';busy.textContent='SWITCHING…';control.appendChild(busy);
    }else if(policy==='ALLOWED'&&enabled!==null){
      const label=document.createElement('label');label.className='service-switch';label.title=workerReady?'Switch service through verified RobotState action worker':'Service action worker is not ready';
      const input=document.createElement('input');input.type='checkbox';input.checked=enabled===true;input.disabled=!workerReady;
      const track=document.createElement('span');track.className='service-switch-track';
      input.addEventListener('change',()=>{
        const desired=input.checked; input.checked=enabled===true;
        requestServiceState(name.textContent,desired);
      });
      label.append(input,track);control.appendChild(label);
    }else{
      const na=document.createElement('span');na.className='service-control-na';na.textContent=policy==='PROTECTED'?'LOCKED':'—';control.appendChild(na);
    }
    row.append(name,policyEl,state,control);list.appendChild(row);
  }
}
async function postServiceState(name,enabled){
  const key=currentManagementKey();
  if(!key)throw new Error('Enter the management key first.');
  const r=await fetch('/api/services/set',{
    method:'POST',headers:{'Content-Type':'application/json','X-G1-Management-Key':key},
    body:JSON.stringify({service:name,enabled})
  });
  let body={};try{body=await r.json();}catch{}
  if(r.status===401){storeManagementKey('');showManagementKeyPrompt('Management key rejected. Enter the key printed by the currently running ./start_dashboard.sh.');}
  if(!r.ok)throw new Error(body?.service_action?.reason||body.error||`HTTP ${r.status}`);
  return body.service_action||{};
}
async function requestServiceState(name,enabled){
  if(serviceActionBusyName)return;
  const desiredWord=enabled?'ON':'OFF';
  if(!currentManagementKey()){
    showManagementKeyPrompt(`Enter the management key to switch ${name} ${desiredWord}.`,()=>requestServiceState(name,enabled));
    return;
  }
  if(!window.confirm(`Switch Unitree service "${name}" ${desiredWord}?\n\nOnly explicitly ALLOWED services can reach ServiceSwitch. The worker will re-read ServiceList and report success only if the requested state is verified.`))return;
  serviceActionBusyName=name;renderServiceList();
  try{
    const action=await postServiceState(name,enabled);
    if(action?.after){
      const idx=lastRobotServices.findIndex(s=>String(s?.name||'')===name);
      if(idx>=0)lastRobotServices[idx]={...lastRobotServices[idx],...action.after};
    }
    renderServiceList();
    await pollSystem();
  }catch(err){
    window.alert(`Service switch failed: ${err.message||err}`);
    await pollServiceControl();
    await pollSystem();
  }finally{serviceActionBusyName=null;renderServiceList();}
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
$('serviceStateFilter')?.addEventListener('change',renderServiceList);
$('servicePolicyFilter')?.addEventListener('change',renderServiceList);

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

pollPose(); poll(); pollSystem(); pollControllerProcess(); pollCameraProcess(); pollServiceControl(); bootstrapManagementKey(); setInterval(poll,250); setInterval(updatePoseHud,250); setInterval(pollSystem,250); setInterval(pollControllerProcess,750); setInterval(pollCameraProcess,750); setInterval(pollServiceControl,2000);

})();
