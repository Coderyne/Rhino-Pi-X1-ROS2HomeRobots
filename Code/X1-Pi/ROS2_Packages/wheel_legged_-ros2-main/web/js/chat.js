/**
 * 轮足机器人 — LLM 对话（roslibjs）
 * 主用 Service 同步调用，rosbridge 兼容性最好
 * 依赖: dashboard.js 暴露了 window.ros
 */
(function () {
  'use strict';

  const STORAGE_KEY = 'llm_chat_history';

  const $ = (id) => document.getElementById(id);
  const chatMessages = $('chatMessages');
  const chatInput = $('chatInput');
  const btnSend = $('btnSendChat');
  const btnClear = $('btnClearChat');
  const chatStatus = $('chatStatus');
  const chatEmpty = $('chatEmpty');

  let busy = false;

  /* ── History ─────────────────────────────── */

  function loadHistory() {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY)) || []; }
    catch (_) { return []; }
  }

  function saveHistory(history) {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(history)); }
    catch (_) { /* ignore */ }
  }

  function clearHistory() {
    localStorage.removeItem(STORAGE_KEY);
    chatMessages.querySelectorAll('.chat-bubble').forEach(b => b.remove());
    chatEmpty.style.display = 'flex';
  }

  function addToHistory(role, content, stats) {
    const h = loadHistory();
    h.push({ role, content, time: Date.now(), ...stats });
    saveHistory(h);
  }

  /* ── Rendering ──────────────────────────── */

  function createBubble(role) {
    const div = document.createElement('div');
    div.className = 'chat-bubble chat-bubble--' + role;
    div.innerHTML =
      '<div class="chat-bubble__role">' + (role === 'user' ? 'YOU' : 'Qwen3') + '</div>' +
      '<div class="chat-bubble__content"></div>';
    return div;
  }

  function getContent(bubble) {
    return bubble.querySelector('.chat-bubble__content');
  }

  function finalizeBubble(bubble, stats) {
    bubble.classList.remove('chat-bubble--pending');
    if (stats && (stats.tokens_generated || stats.inference_time_ms)) {
      const div = document.createElement('div');
      div.className = 'chat-bubble__stats';
      const parts = [];
      if (stats.tokens_generated) parts.push(stats.tokens_generated + ' tokens');
      if (stats.inference_time_ms) {
        const sec = (stats.inference_time_ms / 1000).toFixed(1);
        parts.push(sec + 's');
        if (stats.tokens_generated)
          parts.push(Math.round(stats.tokens_generated / (stats.inference_time_ms / 1000)) + ' tok/s');
      }
      div.textContent = parts.join(' · ');
      bubble.appendChild(div);
    }
  }

  function scrollToBottom() {
    chatMessages.scrollTop = chatMessages.scrollHeight;
  }

  function showStatus(text, generating) {
    chatStatus.textContent = text;
    chatStatus.className = 'chat-status' + (generating ? ' generating' : '');
  }

  function resetUI() {
    busy = false;
    chatInput.disabled = false;
    btnSend.disabled = false;
    chatInput.focus();
  }

  function showError(msg) {
    console.error('[LLM]', msg);
    // 移除 pending 气泡
    const pending = chatMessages.querySelector('.chat-bubble--pending');
    if (pending) pending.remove();
    showStatus('出错: ' + msg, false);
    resetUI();
  }

  /* ── 核心: 发送消息 ────────────────────── */

  function sendMessage() {
    const text = chatInput.value.trim();
    if (!text) return;
    if (busy) { console.warn('[LLM] 忙碌中'); return; }

    const ros = window.ros;
    if (!ros) { showError('ROS 未加载'); return; }

    // 检查 rosbridge 连接状态
    // roslibjs v1.4.0 Ros 对象有 isConnected 属性
    // 如果没有该属性，假定已连接（通过 dashboard.js 已验证）
    const connected = (typeof ros.isConnected === 'boolean') ? ros.isConnected : true;
    if (!connected) {
      showError('rosbridge 未连接');
      return;
    }

    busy = true;
    chatInput.value = '';
    autoResize();
    chatInput.disabled = true;
    btnSend.disabled = true;

    // 用户气泡
    const userBubble = createBubble('user');
    getContent(userBubble).textContent = text;
    chatMessages.appendChild(userBubble);
    addToHistory('user', text, {});
    chatEmpty.style.display = 'none';

    // 助手气泡（pending）
    const assistantBubble = createBubble('assistant');
    assistantBubble.classList.add('chat-bubble--pending');
    getContent(assistantBubble).textContent = '';
    chatMessages.appendChild(assistantBubble);
    scrollToBottom();

    showStatus('推理中...', true);
    console.log('[LLM] 发送请求:', text.substring(0, 50));

    // 调用 Service
    const client = new ROSLIB.Service({
      ros: ros,
      name: '/llm_bridge/chat',
      serviceType: 'llm_interfaces/srv/Chat',
    });

    const req = new ROSLIB.ServiceRequest({ prompt: text });
    const startMs = Date.now();

    client.callService(req, function (result) {
      console.log('[LLM] 收到响应:', result);
      const elapsed = Date.now() - startMs;

      if (!result || !result.response) {
        showError('Service 返回空响应');
        return;
      }

      if (result.response.indexOf('[ERROR]') !== -1 ||
          result.response.indexOf('[BUSY]') !== -1) {
        getContent(assistantBubble).textContent = result.response;
        assistantBubble.classList.remove('chat-bubble--pending');
        assistantBubble.classList.add('chat-bubble--error');
        addToHistory('assistant', result.response, {});
        showStatus('待命中', false);
        resetUI();
        return;
      }

      // 成功
      const stats = {
        tokens_generated: result.tokens_generated || 0,
        inference_time_ms: result.inference_time_ms || elapsed,
      };
      getContent(assistantBubble).textContent = result.response;
      finalizeBubble(assistantBubble, stats);
      addToHistory('assistant', result.response, stats);
      showStatus('待命中', false);
      resetUI();
      scrollToBottom();

    }, function (error) {
      console.error('[LLM] Service 调用失败:', error);
      showError('Service 调用失败: ' + (error || '检查 rosbridge 和节点'));
    });
  }

  /* ── Auto-resize textarea ─────────────── */

  function autoResize() {
    chatInput.style.height = 'auto';
    chatInput.style.height = Math.min(chatInput.scrollHeight, 100) + 'px';
  }

  /* ── Restore history ─────────────────── */

  function restoreHistory() {
    const history = loadHistory();
    if (!history.length) return;

    chatEmpty.style.display = 'none';
    history.forEach(function (msg) {
      const bubble = createBubble(msg.role);
      getContent(bubble).textContent = msg.content;
      if (msg.role === 'assistant' && (msg.tokens_generated || msg.inference_time_ms)) {
        finalizeBubble(bubble, {
          tokens_generated: msg.tokens_generated,
          inference_time_ms: msg.inference_time_ms,
        });
      }
      chatMessages.appendChild(bubble);
    });
    scrollToBottom();
  }

  /* ── Events ──────────────────────────────── */

  btnSend.addEventListener('click', function () {
    console.log('[LLM] 发送按钮点击');
    sendMessage();
  });

  chatInput.addEventListener('keydown', function (e) {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  });

  chatInput.addEventListener('input', autoResize);

  btnClear.addEventListener('click', function () {
    clearHistory();
    showStatus('待命中', false);
  });

  /* ── Init ────────────────────────────────── */

  function init() {
    if (!window.ros) {
      setTimeout(init, 200);
      return;
    }

    window.ros.on('connection', function () {
      console.log('[LLM] rosbridge 已连接');
      showStatus('待命中', false);
    });
    window.ros.on('close', function () {
      console.warn('[LLM] rosbridge 断开');
      showStatus('rosbridge 断开', false);
    });
    window.ros.on('error', function (err) {
      console.error('[LLM] rosbridge 错误:', err);
      showStatus('rosbridge 错误', false);
    });

    restoreHistory();
    console.log('[LLM] chat.js 初始化完成');
  }

  init();

})();
