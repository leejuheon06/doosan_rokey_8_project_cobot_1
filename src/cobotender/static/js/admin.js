function setText(id, text){
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function addLog(message){
  const box = document.getElementById('logBox');
  if (!box) return;
  const now = new Date().toLocaleTimeString('ko-KR', {hour12:false});
  box.insertAdjacentHTML('afterbegin', `<p>[${now}] ${message}</p>`);
}

function renderLogs(logs){
  const box = document.getElementById('logBox');
  if (!box || !Array.isArray(logs)) return;
  box.innerHTML = logs.map(log => `<p>${log}</p>`).join('') || '<p>[SYSTEM] 로그 대기 중</p>';
}

function updateMode(mode){
  const badge = document.getElementById('modeBadge');
  let cleanMode = String(mode || 'IDLE').toLowerCase();
  if (cleanMode === 'stopped') cleanMode = 'estop';
  if (badge) badge.className = 'mode-badge ' + cleanMode;
  setText('modeText', mode || 'IDLE');
  setText('currentMode', mode || 'IDLE');
}

const JOINT_RANGES = [
  {min: -360, max: 360},
  {min: -360, max: 360},
  {min: -360, max: 360},
  {min: -360, max: 360},
  {min: -360, max: 360},
  {min: -360, max: 360},
];

function clamp(value, min, max){
  return Math.max(min, Math.min(max, value));
}

function jointPercent(angle, range){
  const safeAngle = clamp(angle, range.min, range.max);
  return ((safeAngle - range.min) / (range.max - range.min)) * 100;
}

function updateJoints(joints){
  for (let index = 0; index < 6; index++){
    const jointNo = index + 1;
    const angle = Number((joints || [])[index] || 0);
    const range = JOINT_RANGES[index];
    const percent = jointPercent(angle, range);

    setText('joint' + jointNo, angle.toFixed(1) + '°');

    const bar = document.getElementById('jointBar' + jointNo);
    if (bar){
      bar.style.width = percent + '%';
      bar.title = `J${jointNo}: ${range.min}° ~ ${range.max}°`;
    }
  }
}

function updateJointVelocities(jointVelocities, averageVelocity){
  const velocities = Array.isArray(jointVelocities) ? jointVelocities : [];

  for (let index = 0; index < 6; index++){
    const jointNo = index + 1;
    const velocity = Number(velocities[index] || 0);
    setText('jointVel' + jointNo, velocity.toFixed(2));
  }

  const avg = Number(averageVelocity || 0);
  setText('avgJointVelocity', avg.toFixed(2));
}

function updateTaskSteps(taskIndex){
  document.querySelectorAll('#taskSteps li').forEach((item, index) => {
    item.classList.remove('done', 'active');
    if (index < taskIndex) item.classList.add('done');
    if (index === taskIndex) item.classList.add('active');
  });
}


function updateAdminBridgeStatus(bridge){
  bridge = bridge || {};

  const connected = Boolean(bridge.ui_bridge_connected || bridge.connected);
  const hwCode = bridge.hw_code ?? '-';
  const hwState = bridge.hw_state || 'STANDBY';
  const hwLabel = bridge.hw_label || '정상 대기';
  const recoveryRequired = Boolean(bridge.recovery_required || bridge.waiting_recovery);

  setText('hardwareState', `${hwState} (${hwCode})`);
  setText('recoveryState', bridge.waiting_recovery ? '복구 승인 대기' : (recoveryRequired ? '복구 필요' : '대기 없음'));
  setText('adminBridgeState', connected ? '연결됨' : '연결 대기');

  const recoverBtn = document.getElementById('recoverBtn');
  if (recoverBtn){
    recoverBtn.classList.toggle('armed', recoveryRequired);
    recoverBtn.title = recoveryRequired ? `${hwLabel} 상태입니다. 주변 안전 확인 후 Recovery를 누르세요.` : '수동 Recovery 명령을 전송합니다.';
  }

  if (recoveryRequired){
    updateMode('ESTOP');
  }
}

function updateDashboard(data){
  updateMode(data.mode);
  updateJoints(data.joints || []);
  setText('currentRecipe', data.recipe || '대기 중');
  setText('currentStep', data.step || 'Ready');
  const jointVelocities = data.jointVelocities || data.speed?.jointVelocities || [];
  const avgJointVelocity = data.jointVelocityAverage ?? data.speed?.jointAverage ?? 0;
  updateJointVelocities(jointVelocities, avgJointVelocity);
  updateTaskSteps(data.taskIndex || 0);
  updateAdminBridgeStatus(data.adminBridge || {});
  renderLogs(data.logs);
}

async function fetchRobotStatus(){
  try{
    const res = await fetch('/api/robot/status');
    if (!res.ok) throw new Error('status api error');
    const data = await res.json();
    updateDashboard(data);
  }catch(e){
    updateDashboard({
      mode:'ERROR',
      joints:[0,0,0,0,0,0],
      recipe:'상태 수신 실패',
      step:'API 연결 확인 필요',
      jointVelocities:[0,0,0,0,0,0],
      jointVelocityAverage:0,
      speed:{jointVelocities:[0,0,0,0,0,0],jointAverage:0},
      taskIndex:0,
      logs:['[SYSTEM] /api/robot/status 연결 실패'],
      adminBridge:{ui_bridge_connected:false}
    });
  }
}

const COMMAND_CONFIRM = {
  pause: {
    title: '로봇 일시정지',
    message: '현재 동작 단위가 끝난 뒤 로봇 작업을 일시정지합니다.',
    confirmText: '일시정지',
    icon: '⏸️',
    level: 'warning'
  },
  resume: {
    title: '작업 재개',
    message: '일시정지된 로봇 작업을 다시 진행합니다.',
    confirmText: '재개',
    icon: '▶️',
    level: 'normal'
  },
  estop: {
    title: '비상정지',
    message: '소프트 비상정지를 실행합니다. 로봇은 현재 모션을 즉시 정지하고 해제 명령을 기다립니다.',
    confirmText: '비상정지',
    icon: '🛑',
    level: 'danger'
  },
  estop_release: {
    title: '비상해제',
    message: '소프트 비상정지 상태만 해제합니다. 로봇을 이동시키지는 않습니다.',
    confirmText: '비상해제',
    icon: '🔓',
    level: 'warning'
  },
  home_return: {
    title: '홈복귀',
    message: '로봇을 홈 포지션으로 복귀시킵니다. 비상정지 상태라면 먼저 비상해제를 실행하세요.',
    confirmText: '홈복귀',
    icon: '🏠',
    level: 'warning'
  },
  recover: {
    title: '안전모드 Recovery',
    message: 'Protective Stop, Safe Off, 실제 E-Stop 해제 후 복구 승인을 전송합니다.',
    confirmText: 'Recovery 전송',
    icon: '🔧',
    level: 'recovery'
  }
};

let commandConfirmResolver = null;

function commandMeta(command){
  return COMMAND_CONFIRM[command] || {
    title: '로봇 명령 확인',
    message: `${command} 명령을 전송하시겠습니까?`,
    confirmText: '전송',
    icon: '⚙️',
    level: 'normal'
  };
}

function openCommandConfirmModal(command){
  const meta = commandMeta(command);
  const modal = document.getElementById('commandConfirmModal');
  const card = modal ? modal.querySelector('.command-modal-card') : null;
  const confirmButton = document.getElementById('commandConfirmButton');
  if (!modal || !card || !confirmButton){
    return Promise.resolve(true);
  }

  setText('commandConfirmTitle', meta.title);
  setText('commandConfirmMessage', meta.message);
  setText('commandConfirmIcon', meta.icon);
  confirmButton.textContent = meta.confirmText || '전송';

  card.classList.remove('danger', 'warning', 'recovery');
  if (meta.level && meta.level !== 'normal') card.classList.add(meta.level);

  modal.classList.remove('hidden');
  modal.setAttribute('aria-hidden', 'false');
  confirmButton.focus({preventScroll:true});

  return new Promise(resolve => {
    commandConfirmResolver = resolve;
  });
}

function closeCommandConfirmModal(confirmed){
  const modal = document.getElementById('commandConfirmModal');
  if (modal){
    modal.classList.add('hidden');
    modal.setAttribute('aria-hidden', 'true');
  }
  if (commandConfirmResolver){
    commandConfirmResolver(Boolean(confirmed));
    commandConfirmResolver = null;
  }
}

async function sendRobotCommand(command){
  const confirmed = await openCommandConfirmModal(command);
  if (!confirmed) return;

  try{
    const res = await fetch('/api/robot/command', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({command})
    });
    const data = await res.json();
    addLog(data.message || `${command} command sent.`);
    fetchRobotStatus();
  }catch(e){
    addLog(`${command} 명령 전송 실패`);
  }
}

fetchRobotStatus();
setInterval(fetchRobotStatus, 300);



let staffRequestPopupOpen = false;
let pendingStaffRequests = [];

function staffRequestIcon(type){
  if (type === '물') return '💧';
  if (type === '냅킨') return '🧻';
  if (type === '직원호출') return '🔔';
  if (String(type || '').startsWith('안주 주문')) return '🍽️';
  return '📌';
}

function playStaffRequestSound(){
  try{
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    const ctx = new AudioContext();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(880, ctx.currentTime);
    gain.gain.setValueAtTime(0.0001, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.18, ctx.currentTime + 0.02);
    gain.gain.exponentialRampToValueAtTime(0.0001, ctx.currentTime + 0.22);
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.24);
  }catch(e){
    /* 브라우저 정책상 소리가 차단될 수 있으므로 무시 */
  }
}

function showStaffRequestModal(requests){
  const modal = document.getElementById('staffRequestModal');
  const list = document.getElementById('staffRequestList');
  if (!modal || !list) return;

  pendingStaffRequests = requests;
  staffRequestPopupOpen = true;

  list.innerHTML = requests.map(req => `
    <div class="staff-request-item">
      <b>${staffRequestIcon(req.request_type)}</b>
      <span>${req.request_type}</span>
    </div>
  `).join('');

  modal.classList.remove('hidden');
  playStaffRequestSound();
}

async function confirmStaffRequests(){
  const modal = document.getElementById('staffRequestModal');
  if (modal) modal.classList.add('hidden');

  const requests = pendingStaffRequests.slice();
  pendingStaffRequests = [];

  for (const req of requests){
    try{
      await fetch(`/api/staff_requests/${req.id}/handle`, {
        method:'POST'
      });
    }catch(e){
      addLog(`${req.request_type} 요청 처리 실패`);
    }
  }

  staffRequestPopupOpen = false;
}

async function checkStaffRequests(){
  try{
    if (staffRequestPopupOpen) return;

    const res = await fetch('/api/staff_requests');
    if (!res.ok) throw new Error('staff requests api error');

    const requests = await res.json();

    if (!Array.isArray(requests) || requests.length === 0) return;

    showStaffRequestModal(requests);

  }catch(e){
    staffRequestPopupOpen = false;
  }
}

document.addEventListener('keydown', (event) => {
  if (event.key === 'Escape'){
    if (commandConfirmResolver){
      closeCommandConfirmModal(false);
      return;
    }
    if (staffRequestPopupOpen){
      confirmStaffRequests();
    }
  }
});

checkStaffRequests();
setInterval(checkStaffRequests, 1000);
