let menu = [];
let currentCategory = 'cocktail';
let selectedMenu = null;
let cart = [];
const slides = ['/static/images/slide_gatsby.jpg','/static/images/slide_gil_beer.jpg','/static/images/slide_public_warning.jpg'];
let slideIndex = 0;

setInterval(() => {
  const img = document.getElementById('slideImage');
  if (!img) return;
  slideIndex = (slideIndex + 1) % slides.length;
  img.src = slides[slideIndex];
}, 3000);

async function loadMenu(){
  const res = await fetch('/api/menu');
  menu = await res.json();
  renderMenu();
}
function won(n){ return Number(n).toLocaleString('ko-KR') + '원'; }
function showOrderScreen(){
  document.getElementById('standby').classList.add('hidden');
  document.getElementById('orderScreen').classList.remove('hidden');
  loadMenu();
}
document.querySelectorAll('.cat').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.cat').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentCategory = btn.dataset.category;
    renderMenu();
  });
});
function renderMenu(){
  const grid = document.getElementById('menuGrid');
  if (!grid) return;
  grid.innerHTML = '';
  menu.filter(m => m.category === currentCategory).forEach(m => {
    const disabled = m.sold_out === 1;
    const card = document.createElement('div');
    card.className = 'menu-card' + (disabled ? ' disabled' : '');
    card.innerHTML = `
      ${disabled ? '<div class="soldout">품절</div>' : ''}
      <img src="/static/images/${m.image}" alt="${m.name}" onerror="this.src='/static/images/placeholder.jpg'">
      <div class="info"><h3>${m.name}</h3><p>${won(m.price)}</p></div>`;
    card.onclick = () => openPopup(m);
    grid.appendChild(card);
  });
}
function openPopup(m){
  selectedMenu = m;
  document.getElementById('popupImage').src = '/static/images/' + m.image;
  document.getElementById('popupImage').onerror = e => e.target.src='/static/images/placeholder.jpg';
  document.getElementById('popupName').textContent = m.name;
  document.getElementById('popupDesc').textContent = m.description;
  document.getElementById('popupPrice').textContent = won(m.price);
  document.getElementById('qtyInput').value = 1;
  document.getElementById('popup').classList.remove('hidden');
}
function closePopup(){ document.getElementById('popup').classList.add('hidden'); }
function changeQty(delta){
  const input = document.getElementById('qtyInput');
  input.value = Math.max(1, Number(input.value || 1) + delta);
}
function addToCart(){
  const qty = Math.max(1, Number(document.getElementById('qtyInput').value || 1));
  const found = cart.find(x => x.id === selectedMenu.id);
  if (found) found.qty += qty;
  else cart.push({id:selectedMenu.id, name:selectedMenu.name, price:selectedMenu.price, qty});
  closePopup();
  renderCart();
}
function renderCart(){
  const box = document.getElementById('cartItems');
  if (cart.length === 0){ box.className='cart-items empty'; box.textContent='담긴 메뉴가 없습니다.'; }
  else {
    box.className='cart-items';
    box.innerHTML = cart.map((x,i) => `
      <div class="cart-line">
        <span class="cart-menu-name">${x.name}</span>
        <div class="cart-qty-control">
          <button onclick="changeCartQty(${i},-1)">-</button>
          <span>${x.qty}</span>
          <button onclick="changeCartQty(${i},1)">+</button>
        </div>
        <span class="cart-price">${won(x.price*x.qty)}</span>
        <button class="cart-delete" onclick="removeCart(${i})">삭제</button>
      </div>`).join('');
  }
  document.getElementById('cartTotal').textContent = won(cart.reduce((s,x)=>s+x.price*x.qty,0));
}
function changeCartQty(i, delta){
  cart[i].qty += delta;
  if (cart[i].qty <= 0) cart.splice(i,1);
  renderCart();
}
function removeCart(i){ cart.splice(i,1); renderCart(); }
function setLoadingText(text){
  const el = document.getElementById('loadingText');
  if (el) el.textContent = text;
}
function sleep(ms){ return new Promise(resolve => setTimeout(resolve, ms)); }
async function fetchRobotStatus(){
  const res = await fetch('/api/robot/status', {cache:'no-store'});
  if (!res.ok) throw new Error('robot status api error');
  return await res.json();
}
function robotStatusMessage(raw){
  const status = Number(raw);
  const messages = {
    0: '로봇 대기 상태 확인 중...',
    1: '음료 제조 중...',
    2: '제조 완료, 서빙 준비 중...',
    3: '서빙 위치로 이동 중...',
    4: '서빙 완료!',
    5: '초기 위치로 복귀 중...'
  };
  return messages[status] || '로봇 상태 확인 중...';
}
async function waitForRobotCompletion(){
  const pollMs = 500;
  const startTimeoutMs = 20000;
  const totalTimeoutMs = 10 * 60 * 1000;
  const startAt = Date.now();
  let seenRobotActive = false;

  while (true){
    let status = null;
    try{
      status = await fetchRobotStatus();
    }catch(e){
      setLoadingText('로봇 상태 확인 중...');
      if (Date.now() - startAt > totalTimeoutMs) throw e;
      await sleep(pollMs);
      continue;
    }

    const raw = Number(status.robot_status_raw ?? 0);
    if ([1, 2, 3, 4, 5].includes(raw)) seenRobotActive = true;
    setLoadingText(robotStatusMessage(raw));

    // Customer screen should stay in manufacturing/waiting screen while the robot is actually working.
    // Status 4 means delivered. Status 5 means the drink has already been delivered and the robot is returning home.
    // Status 0 after an active state means the task is completely finished.
    if (seenRobotActive && [4, 5, 0].includes(raw)) return true;

    // If the robot process state is missed or remains WAITING too long, do not lock the kiosk forever.
    if (!seenRobotActive && Date.now() - startAt > startTimeoutMs) return true;
    if (Date.now() - startAt > totalTimeoutMs) return true;

    await sleep(pollMs);
  }
}
function finishOrderFlow(){
  document.getElementById('loadingScreen').classList.add('hidden');
  document.getElementById('completeScreen').classList.remove('hidden');
  cart = [];
  renderCart();
  loadMenu();
  setTimeout(() => {
    document.getElementById('completeScreen').classList.add('hidden');
    document.getElementById('standby').classList.remove('hidden');
  }, 5000);
}
async function submitOrder(){
  if (cart.length === 0){ alert('장바구니가 비어있습니다.'); return; }

  const res = await fetch('/api/order', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({items:cart})
  });
  const data = await res.json();
  if (!data.ok){ alert(data.message || '주문 실패'); loadMenu(); return; }

  document.getElementById('orderScreen').classList.add('hidden');
  document.getElementById('loadingScreen').classList.remove('hidden');
  setLoadingText(data.requires_robot_work ? '주문을 확인 중...' : '재료를 섞는 중...');

  try{
    if (data.requires_robot_work){
      await waitForRobotCompletion();
    }else{
      await sleep(1500);
    }
  }catch(e){
    // 네트워크/ROS 상태 조회가 잠깐 실패해도 고객 화면이 멈추지 않도록 완료 화면으로 전환합니다.
    console.warn('robot status wait failed:', e);
  }

  finishOrderFlow();
}
