     
// This file contains the HTML data for the ESP32.

const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, minimum-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
  <title>MINI-FORK Controller</title>
  <style>
    :root {
      --bg: #101519;
      --panel: #1a2329;
      --line: #3d4b53;
      --text: #f3f6f7;
      --muted: #aebbc1;
      --accent: #f0a51a;
      --button: #303c44;
      --danger: #e24b3b;
    }
    * { box-sizing: border-box; }
    html, body {
      width: 100%;
      min-height: 100%;
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, Helvetica, sans-serif;
      overscroll-behavior: none;
    }
    body {
      min-height: var(--app-height, 100dvh);
      padding: max(10px, env(safe-area-inset-top)) max(10px, env(safe-area-inset-right)) max(10px, env(safe-area-inset-bottom)) max(10px, env(safe-area-inset-left));
      touch-action: none;
    }
    button { font: inherit; color: var(--text); }
    .controller { width: min(1100px, 100%); min-height: calc(var(--app-height, 100dvh) - 20px); margin: 0 auto; display: flex; flex-direction: column; }
    .topbar { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 2px 2px 10px; }
    h1 { margin: 0; font-size: clamp(1.35rem, 5vw, 2rem); letter-spacing: .12em; }
    .topbar-right { display: flex; align-items: center; gap: 10px; }
    .connection { display: flex; align-items: center; gap: 7px; color: var(--muted); font-size: .75rem; text-transform: uppercase; letter-spacing: .06em; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 8px var(--danger); }
    .connected .dot { background: #39c66b; box-shadow: 0 0 8px #39c66b; }
    .fullscreen-button { padding: 7px 9px; border: 1px solid var(--line); border-radius: 8px; background: var(--button); color: var(--text); font-size: .72rem; cursor: pointer; touch-action: manipulation; }
    .control-area { flex: 1; min-height: 0; display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1.25fr); gap: 10px; }
    .panel { min-width: 0; background: var(--panel); border: 1px solid var(--line); border-radius: 18px; padding: clamp(10px, 2vw, 16px); }
    .panel-title { margin: 0 0 10px; color: var(--muted); font-size: .72rem; letter-spacing: .1em; text-transform: uppercase; }
    .left-controls { display: flex; min-height: 100%; flex-direction: column; justify-content: center; gap: 12px; }
    .mast-buttons { display: grid; grid-template-columns: 1fr; gap: 10px; }
    .tilt-rocker { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .control-button {
      min-height: 68px;
      border: 1px solid #586770;
      border-radius: 15px;
      background: var(--button);
      box-shadow: 0 4px 0 #080b0d;
      font-size: .88rem;
      font-weight: 700;
      cursor: pointer;
      touch-action: none;
      user-select: none;
    }
    .control-button:active, .control-button.active {
      transform: translateY(3px);
      background: var(--accent);
      color: #101519;
      box-shadow: 0 1px 0 #080b0d;
    }
    .symbol { display: block; margin-bottom: 4px; font-size: 2rem; line-height: 1; }
    .tilt-rocker .control-button { min-height: 82px; }
    .light-button { min-height: 54px; }
    .joystick-panel { display: flex; min-height: 100%; flex-direction: column; align-items: center; justify-content: center; }
    .joystick-label { align-self: stretch; margin-bottom: 10px; }
    .joystick-label .panel-title { margin-bottom: 0; }
    .joystick {
      position: relative;
      width: min(70vw, 380px);
      aspect-ratio: 1;
      border: 2px solid #56666e;
      border-radius: 50%;
      background:
        linear-gradient(90deg, transparent 49.5%, #34434b 49.5%, #34434b 50.5%, transparent 50.5%),
        linear-gradient(0deg, transparent 49.5%, #34434b 49.5%, #34434b 50.5%, transparent 50.5%),
        radial-gradient(circle, #26333a 0 42%, #202c32 43% 70%, #182126 71%);
      box-shadow: inset 0 0 0 12px #101519, 0 5px 0 #080b0d;
      touch-action: none;
    }
    .joystick::before, .joystick::after { position: absolute; color: #73828a; font-size: 1.4rem; opacity: .8; }
    .joystick::before { content: "▲"; top: 7%; left: calc(50% - .5em); }
    .joystick::after { content: "▼"; bottom: 7%; left: calc(50% - .5em); }
    .joystick-knob {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 29%;
      aspect-ratio: 1;
      transform: translate(-50%, -50%);
      border: 4px solid #fff4d3;
      border-radius: 50%;
      background: var(--accent);
      box-shadow: 0 5px 0 #8b5d06, 0 0 0 7px rgba(240,165,26,.15);
      pointer-events: none;
    }
    .joystick-values { display: flex; justify-content: space-between; width: min(70vw, 380px); margin-top: 12px; color: var(--muted); font-size: .75rem; }
    .joystick-values span { color: var(--text); font-variant-numeric: tabular-nums; }
    .pc-info { margin-top: 10px; padding: 9px 12px; border: 1px solid var(--line); border-radius: 12px; color: var(--muted); font-size: .76rem; line-height: 1.6; text-align: center; }
    kbd { display: inline-block; min-width: 22px; padding: 0 5px; border: 1px solid #687881; border-radius: 4px; color: var(--text); text-align: center; }
    .noselect { -webkit-user-select: none; user-select: none; }

    @media (max-width: 520px) and (orientation: portrait) {
      .controller { min-height: calc(var(--app-height, 100dvh) - 20px); }
      .control-area { min-height: 0; grid-template-columns: minmax(92px, .78fr) minmax(0, 1.22fr); }
      .left-controls { gap: 9px; }
      .control-button { min-height: 62px; padding: 5px 3px; font-size: .72rem; }
      .tilt-rocker .control-button { min-height: 70px; }
      .symbol { font-size: 1.65rem; }
      .joystick { width: min(58vw, 280px); }
      .joystick-values { width: min(58vw, 280px); font-size: .68rem; }
      .pc-info { font-size: .67rem; line-height: 1.45; }
    }
    @media (min-width: 800px) {
      .control-area { grid-template-columns: minmax(260px, .75fr) minmax(420px, 1.25fr); }
      .control-button { min-height: 82px; }
      .mast-buttons { grid-template-columns: 1fr 1fr; }
      .mast-buttons .up-button { grid-column: 1; }
      .mast-buttons .down-button { grid-column: 2; }
      .joystick { width: min(42vw, 430px); }
      .joystick-values { width: min(42vw, 430px); }
      .pc-info { font-size: .82rem; }
    }
    @media (orientation: landscape) and (max-height: 600px) {
      body { padding-top: 6px; padding-bottom: 6px; }
      .topbar { padding-bottom: 5px; }
      .control-area { min-height: 0; }
      .panel { padding: 8px; }
      .left-controls { gap: 6px; }
      .control-button { min-height: 52px; }
      .tilt-rocker .control-button { min-height: 58px; }
      .symbol { font-size: 1.45rem; }
      .joystick { width: min(48vh, 310px); }
      .joystick-values { width: min(48vh, 310px); margin-top: 5px; }
      .pc-info { margin-top: 5px; padding: 4px 8px; }
    }
  </style>
</head>
<body class="noselect">
  <main class="controller">
    <header class="topbar">
      <h1>MINI-FORK</h1>
      <div class="topbar-right">
        <button id="fullscreenButton" class="fullscreen-button" type="button">FULLSCREEN</button>
        <div id="connection" class="connection"><span class="dot"></span><span id="connectionText">Offline</span></div>
      </div>
    </header>

    <section class="control-area">
      <section class="panel left-controls" aria-label="Forklift attachment controls">
        <div>
          <h2 class="panel-title">Mast</h2>
          <div class="mast-buttons">
            <button class="control-button up-button" data-action="mast" data-value="6" data-release="0" aria-label="Raise mast"><span class="symbol">&#8593;</span>RAISE</button>
            <button class="control-button down-button" data-action="mast" data-value="5" data-release="0" aria-label="Lower mast"><span class="symbol">&#8595;</span>LOWER</button>
          </div>
        </div>
        <div>
          <h2 class="panel-title">Tilt</h2>
          <div class="tilt-rocker">
            <button class="control-button" data-action="mTilt" data-value="1" data-repeat="true" aria-label="Tilt forward"><span class="symbol">&#8634;</span>TILT</button>
            <button class="control-button" data-action="mTilt" data-value="2" data-repeat="true" aria-label="Tilt back"><span class="symbol">&#8635;</span>TILT</button>
          </div>
        </div>
        <button class="control-button light-button" data-action="light" data-value="6" data-toggle="true" aria-label="Toggle lights"><span class="symbol">&#9788;</span>LIGHTS</button>
      </section>

      <section class="panel joystick-panel" aria-label="Driving joystick">
        <div class="joystick-label"><h2 class="panel-title">Drive</h2></div>
        <div id="joystick" class="joystick">
          <div id="joystickKnob" class="joystick-knob"></div>
        </div>
        <div class="joystick-values">
          <span>Throttle: <span id="throttleValue">0</span></span>
          <span>Steering: <span id="steeringValue">86</span></span>
        </div>
      </section>
    </section>

    <footer class="pc-info">
      PC keys:
      <kbd>W</kbd>/<kbd>S</kbd> drive &nbsp;
      <kbd>A</kbd>/<kbd>D</kbd> steer &nbsp;
      <kbd>I</kbd>/<kbd>K</kbd> mast &nbsp;
      <kbd>J</kbd>/<kbd>L</kbd> tilt &nbsp;
      <kbd>X</kbd> lights
    </footer>
  </main>

  <script>
    var webSocketCarInputUrl = "ws:\/\/" + window.location.hostname + "/CarInput";
    var websocketCarInput;
    var repeatTimers = new Map();
    var pressedKeys = new Set();
    var activeMastButton = null;
    var activeTiltButton = null;
    var joystick = document.getElementById("joystick");
    var joystickKnob = document.getElementById("joystickKnob");
    var joystickTimer = null;
    var joystickActive = false;
    var joystickState = { throttle: 0, steering: 86 };
    var lastJoystickPointerId = null;

    function updateViewportHeight() {
      document.documentElement.style.setProperty("--app-height", window.innerHeight + "px");
    }

    function updateFullscreenButton() {
      var button = document.getElementById("fullscreenButton");
      button.textContent = document.fullscreenElement ? "EXIT FULLSCREEN" : "FULLSCREEN";
    }

    function toggleFullscreen() {
      if (document.fullscreenElement) {
        if (document.exitFullscreen) document.exitFullscreen();
        return;
      }
      if (document.documentElement.requestFullscreen) {
        document.documentElement.requestFullscreen().catch(function() {});
      }
    }

    function setConnection(online) {
      document.getElementById("connection").classList.toggle("connected", online);
      document.getElementById("connectionText").textContent = online ? "Connected" : "Offline";
    }

    function sendButtonInput(key, value) {
      if (!websocketCarInput || websocketCarInput.readyState !== WebSocket.OPEN) return;
      websocketCarInput.send(key + "," + value);
    }

    function stopRepeating(action) {
      if (!repeatTimers.has(action)) return;
      clearInterval(repeatTimers.get(action));
      repeatTimers.delete(action);
    }

    function startRepeating(action, value) {
      stopRepeating(action);
      sendButtonInput(action, value);
      repeatTimers.set(action, setInterval(function() { sendButtonInput(action, value); }, 50));
    }

    function releaseButton(button) {
      if (!button.classList.contains("active")) return;
      button.classList.remove("active");
      var action = button.dataset.action;
      if (action === "mast" && activeMastButton === button) activeMastButton = null;
      if (action === "mTilt" && activeTiltButton === button) activeTiltButton = null;
      stopRepeating(action);
      if (button.dataset.release !== undefined) sendButtonInput(action, button.dataset.release);
    }

    function pressButton(button) {
      if (button.classList.contains("active")) return;
      var action = button.dataset.action;
      if (action === "mast" && activeMastButton && activeMastButton !== button) releaseButton(activeMastButton);
      if (action === "mTilt" && activeTiltButton && activeTiltButton !== button) releaseButton(activeTiltButton);
      button.classList.add("active");
      if (action === "mast") activeMastButton = button;
      if (action === "mTilt") activeTiltButton = button;
      var value = button.dataset.value;
      if (button.dataset.toggle === "true") {
        sendButtonInput(action, value);
      } else if (button.dataset.repeat === "true") {
        startRepeating(action, value);
      } else {
        sendButtonInput(action, value);
      }
    }

    document.querySelectorAll(".control-button").forEach(function(button) {
      button.addEventListener("pointerdown", function(event) {
        event.preventDefault();
        if (button.setPointerCapture) button.setPointerCapture(event.pointerId);
        pressButton(button);
      });
      button.addEventListener("pointerup", function(event) {
        event.preventDefault();
        releaseButton(button);
      });
      button.addEventListener("pointercancel", function() { releaseButton(button); });
      button.addEventListener("lostpointercapture", function() { releaseButton(button); });
    });

    function updateJoystickPosition(event) {
      var rect = joystick.getBoundingClientRect();
      var centerX = rect.left + rect.width / 2;
      var centerY = rect.top + rect.height / 2;
      var radius = rect.width * .5;
      var maxTravel = radius * .58;
      var x = event.clientX - centerX;
      var y = event.clientY - centerY;
      var distance = Math.sqrt(x * x + y * y);
      if (distance > maxTravel) {
        x = x * maxTravel / distance;
        y = y * maxTravel / distance;
      }
      var normalizedX = x / maxTravel;
      var normalizedY = y / maxTravel;
      var deadZone = .08;
      if (Math.abs(normalizedX) < deadZone) normalizedX = 0;
      if (Math.abs(normalizedY) < deadZone) normalizedY = 0;
      joystickState.throttle = Math.round(-normalizedY * 255);
      joystickState.steering = Math.round(86 + normalizedX * 46);
      joystickState.throttle = Math.max(-255, Math.min(255, joystickState.throttle));
      joystickState.steering = Math.max(40, Math.min(132, joystickState.steering));
      joystickKnob.style.left = (50 + normalizedX * 29) + "%";
      joystickKnob.style.top = (50 + normalizedY * 29) + "%";
      document.getElementById("throttleValue").textContent = joystickState.throttle;
      document.getElementById("steeringValue").textContent = joystickState.steering;
    }

    function sendJoystickState() {
      sendButtonInput("throttle", joystickState.throttle);
      sendButtonInput("steering", joystickState.steering);
    }

    function resetJoystick() {
      joystickActive = false;
      if (joystickTimer !== null) {
        clearInterval(joystickTimer);
        joystickTimer = null;
      }
      joystickState.throttle = 0;
      joystickState.steering = 86;
      joystickKnob.style.left = "50%";
      joystickKnob.style.top = "50%";
      document.getElementById("throttleValue").textContent = "0";
      document.getElementById("steeringValue").textContent = "86";
      sendButtonInput("throttle", "0");
      sendButtonInput("steering", "86");
    }

    joystick.addEventListener("pointerdown", function(event) {
      event.preventDefault();
      lastJoystickPointerId = event.pointerId;
      if (joystick.setPointerCapture) joystick.setPointerCapture(event.pointerId);
      joystickActive = true;
      updateJoystickPosition(event);
      sendJoystickState();
      joystickTimer = setInterval(sendJoystickState, 50);
    });
    joystick.addEventListener("pointermove", function(event) {
      if (!joystickActive || event.pointerId !== lastJoystickPointerId) return;
      event.preventDefault();
      updateJoystickPosition(event);
    });
    joystick.addEventListener("pointerup", function(event) {
      if (event.pointerId === lastJoystickPointerId) resetJoystick();
    });
    joystick.addEventListener("pointercancel", resetJoystick);
    joystick.addEventListener("lostpointercapture", function() { if (joystickActive) resetJoystick(); });

    function stopAllControls() {
      stopRepeating("mTilt");
      if (activeMastButton) releaseButton(activeMastButton);
      if (activeTiltButton) releaseButton(activeTiltButton);
      resetJoystick();
      pressedKeys.clear();
    }

    function keyboardCommand(key, down) {
      if (pressedKeys.has(key) === down) return;
      if (down) pressedKeys.add(key); else pressedKeys.delete(key);
      if (key === "w" || key === "s") {
        joystickState.throttle = down ? (key === "w" ? 255 : -255) : 0;
        joystickState.steering = 86;
        sendButtonInput("throttle", joystickState.throttle);
        if (!down) sendButtonInput("steering", "86");
      } else if (key === "a" || key === "d") {
        joystickState.steering = down ? (key === "a" ? 40 : 132) : 86;
        sendButtonInput("steering", joystickState.steering);
      } else if (key === "i" || key === "k") {
        sendButtonInput("mast", down ? (key === "i" ? "6" : "5") : "0");
      } else if (key === "j" || key === "l") {
        if (down) startRepeating("mTilt", key === "j" ? "1" : "2"); else stopRepeating("mTilt");
      } else if (key === "x" && down) {
        sendButtonInput("light", "1");
      }
    }

    function handleKey(event, down) {
      var key = event.key.toLowerCase();
      if (["w", "a", "s", "d", "i", "k", "j", "l", "x"].indexOf(key) === -1) return;
      event.preventDefault();
      keyboardCommand(key, down);
    }
    document.addEventListener("keydown", function(event) { handleKey(event, true); });
    document.addEventListener("keyup", function(event) { handleKey(event, false); });

    function initCarInputWebSocket() {
      websocketCarInput = new WebSocket(webSocketCarInputUrl);
      websocketCarInput.onopen = function() { setConnection(true); };
      websocketCarInput.onclose = function() { setConnection(false); stopAllControls(); setTimeout(initCarInputWebSocket, 2000); };
      websocketCarInput.onerror = function() { setConnection(false); };
      websocketCarInput.onmessage = function() {};
    }

    window.onload = initCarInputWebSocket;
    updateViewportHeight();
    window.addEventListener("resize", updateViewportHeight);
    window.addEventListener("orientationchange", function() { setTimeout(updateViewportHeight, 250); });
    document.getElementById("fullscreenButton").addEventListener("click", toggleFullscreen);
    document.addEventListener("fullscreenchange", updateFullscreenButton);
    window.addEventListener("blur", stopAllControls);
    document.addEventListener("visibilitychange", function() { if (document.hidden) stopAllControls(); });
  </script>
</body>
</html>
)HTMLHOMEPAGE";
