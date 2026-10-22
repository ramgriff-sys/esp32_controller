#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
 

// TB6612 motor driver pins
constexpr int PWMA = 5;
constexpr int AIN2 = 18;
constexpr int AIN1 = 19;
constexpr int STBY = 21;
constexpr int BIN1 = 33;
constexpr int BIN2 = 32;
constexpr int PWMB = 27;
 

// Sensor pins
constexpr int LEFT_IR = 22;
constexpr int RIGHT_IR = 23;
constexpr int TABLE_LEVEL = LOW;
constexpr int SDA_PIN = 25;
constexpr int SCL_PIN = 26;
 

// Network settings
constexpr char AP_NAME[] = "Simon the Sumo";
constexpr char AP_PASS[] = "sumobot123";
constexpr unsigned long MOTOR_TIMEOUT_MS = 300;


WebServer server(80);
VL53L0X tof;
enum AutoMode { AUTO_OFF, AUTO_FOLLOW, AUTO_ATTACK };
 

bool tofOnline = false;
bool tofTimeout = false;
bool tofAddressFound = false;
bool leftTable = false;
bool rightTable = false;
bool fastSensors = false;
volatile bool edgeInterrupt = false;
 

AutoMode autoMode = AUTO_OFF;
uint8_t edgeRecovery = 0;
unsigned long recoveryUntil = 0;
unsigned long searchChanged = 0;
int autoPwm = 128;
int detectMm = 700;
int followMm = 300;
int searchDirection = 1;
const char* autoStateText = "Stopped";
 

uint16_t tofMm = 0;
unsigned long tofSamples = 0;
unsigned long lastTofRead = 0;
unsigned long lastTofRetry = 0;
unsigned long lastMotorCommand = 0;
int motorA=0, motorB=0;
unsigned long lastBusScan=0;
 

// Older diagnostics UI kept as a fallback.
const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover"><meta name="apple-mobile-web-app-capable" content="yes"><meta name="theme-color" content="#18130f"><title>Sumo Diagnostics</title><style>
:root{ui-monospace,Consolas,monospace;color-scheme:dark}*{box-sizing:border-box}body{0;background:#18130f;color:#f4e8d5}main{min-height:100dvh;padding:max(10px,env(safe-area-inset-top)) max(10px,env(safe-area-inset-right)) max(10px,env(safe-area-inset-bottom)) max(10px,env(safe-area-inset-left))}.top{display:flex;align-items:center;justify-content:space-between;gap:10px;10px}.title{font:900 clamp(1rem,4vw,1.5rem) sans-serif;color:#e96745;letter-spacing:.08em}.link{font-size:.72rem;color:#e7aa59}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.card{background:#251d17;border:1px solid #604b3b;border-radius:12px;padding:12px;min-width:0}.card h2{font:800 .78rem sans-serif;letter-spacing:.08em;color:#bca992;0 0 10px}.big{font:900 clamp(1.6rem,7vw,3.4rem)/1 sans-serif;color:#f1b35e}.sub{color:#ad9b88;font-size:.7rem;5px}.row{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:7px 0;border-bottom:1px solid #45362b}.row:last-child{border:0}.ok{color:#a9c491}.bad{color:#ff715d}.warn{color:#e7aa59}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;6px;background:currentColor}.motors{grid-column:1/-1}.motor-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}button{border:1px solid #765b46;border-radius:9px;background:#30241c;color:#f4e8d5;padding:13px 7px;font:bold .75rem inherit;touch-action:none}.stop{background:#9d3f2e}.power{display:flex;gap:10px;align-items:center;10px;font-size:.75rem}.power input{flex:1;accent-color:#e96745}.note{grid-column:1/-1;color:#a89582;font-size:.67rem;text-align:center}.bar{height:9px;background:#100d0a;border-radius:8px;overflow:hidden;9px}.fill{height:100%;width:0;background:#e96745;transition:width .08s}.future{opacity:.55}@media(orientation:landscape) and (max-height:450px){main{padding:5px max(8px,env(safe-area-inset-right)) 5px max(8px,env(safe-area-inset-left))}.top{5px}.grid{grid-template-columns:1.1fr 1fr 1.4fr;gap:5px}.card{padding:7px;border-radius:8px}.card h2{4px}.motors{grid-column:auto;grid-row:1/3;grid-column:3}.motor-grid{gap:4px}button{padding:9px 4px}.note{display:none}.row{padding:4px 0}.big{font-size:1.8rem}.power{5px}}
</style></head><body><main><div class="top"><div class="title">SUMO // FULL DIAGNOSTICS</div><div id="link" class="link">CONNECTING</div></div><div class="grid">
<section class="card"><h2>FLOOR SENSORS</h2><div class="row">LEFT • D22 <b id="left">--</b></div><div class="row">RIGHT • D23 <b id="right">--</b></div><div class="sub">Either edge immediately disables both motors.</div></section>
<section class="card"><h2>TOF 1 • VL53L0X V2</h2><div><span id="tofDot" class="dot warn"></span><b id="tofState">CHECKING</b></div><div class="big"><span id="mm">--</span><small> mm</small></div><div class="bar"><div id="rangeBar" class="fill"></div></div><div id="i2cState" class="sub">SDA D25 • SCL D26 • checking 0x29</div></section>
<section class="card motors"><h2>MOTOR TESTS • HOLD TO RUN</h2><div class="motor-grid"><button data-cmd="lf">LEFT FWD</button><button data-cmd="f">BOTH FWD</button><button data-cmd="rf">RIGHT FWD</button><button data-cmd="lr">LEFT REV</button><button id="stop" class="stop">BRAKE</button><button data-cmd="rr">RIGHT REV</button></div><label class="power">POWER <b id="powerText">30%</b><input id="power" type="range" min="20" max="100" value="30"></label><div class="sub">Output: A <b id="ma">0</b>% • B <b id="mb">0</b>%</div></section>
<section class="card future"><h2>FUTURE TOF ARRAY</h2><div class="row">TOF 2 <b>NOT INSTALLED</b></div><div class="row">TOF 3 <b>NOT INSTALLED</b></div><div class="sub">Each additional identical sensor needs a dedicated XSHUT GPIO for address assignment.</div></section>
<div class="note">Wi-Fi: Simon the Sumo • Motors brake on release, edge, or lost browser commands.</div></div></main><script>
const $=x=>document.getElementById(x);let holdTimer=null,cmd='stop';const power=()=>Math.round(+$('power').value*2.55);function send(c=cmd){cmd=c;fetch(`/motor?cmd=${c}&pwm=${power()}`,{cache:'no-store'}).catch(()=>{})}function stop(){clearInterval(holdTimer);holdTimer=null;send('stop')}document.querySelectorAll('[data-cmd]').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);send(b.dataset.cmd);clearInterval(holdTimer);holdTimer=setInterval(()=>send(),100)});['pointerup','pointercancel','lostpointercapture'].forEach(x=>b.addEventListener(x,stop))});$('stop').onclick=stop;$('power').oninput=()=>{$('powerText').textContent=$('power').value+'%'};
async function poll(){try{const d=await(await fetch('/data',{cache:'no-store'})).json();$('link').textContent='LIVE • '+d.ip;for(const k of ['left','right']){const el=$(k);el.textContent=d[k+'_table']?'TABLE':'EDGE';el.className=d[k+'_table']?'ok':'bad'}$('tofState').textContent=d.tof_online?(d.tof_timeout?'TIMEOUT':'ONLINE'):(d.tof_address?'0x29 FOUND • INIT FAILED':'NOT FOUND');$('tofState').className=d.tof_online&&!d.tof_timeout?'ok':'bad';$('tofDot').className='dot '+(d.tof_online&&!d.tof_timeout?'ok':'bad');$('i2cState').textContent=d.tof_address?'I²C 0x29 RESPONDING • SDA D4 • SCL D2':'NO RESPONSE AT 0x29 • CHECK POWER/SDA/SCL';$('i2cState').className='sub '+(d.tof_address?'ok':'bad');$('mm').textContent=d.tof_valid?d.tof_mm:'--';$('rangeBar').style.width=(d.tof_valid?Math.max(2,100-Math.min(d.tof_mm,2000)/20):0)+'%';$('ma').textContent=Math.round(d.motor_a/2.55);$('mb').textContent=Math.round(d.motor_b/2.55)}catch(e){$('link').textContent='LINK LOST';$('link').className='link bad'}setTimeout(poll,80)}addEventListener('pagehide',stop);addEventListener('blur',stop);document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});poll();
</script></body></html>)HTML";
 

// Main controller served by the ESP32.
const char DASHBOARD[] PROGMEM=R"HTML(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>NES Sumo Bot Controller</title><style>* { box-sizing: border-box; } :root { --body: #e4e4e4; --shell: #c7c7c7; --dark: #292929; --red: #b83240; --green: #9bbc72; --screen-text: #263b20; } body { margin: 0; min-height: 100vh; padding: 20px; display: flex; align-items: center; justify-content: center; background: var(--body); font-family: Arial, sans-serif; } .controller { width: 100%; max-width: 440px; padding: 22px; background: var(--shell); border: 3px solid #999; border-radius: 12px; box-shadow: 0 10px 20px #0003; } .brand { text-align: center; font-size: 21px; font-weight: 900; letter-spacing: 4px; color: var(--dark); margin-bottom: 14px; } nav { display: flex; gap: 8px; margin-bottom: 18px; } .nav-button { flex: 1; padding: 10px 5px; background: #aaa; color: #333; border: 2px solid #888; border-radius: 5px; font-weight: bold; font-size: 12px; cursor: pointer; } .nav-button.active { background: #333; color: white; border-color: #222; } .screen { background: var(--green); color: var(--screen-text); border: 6px solid #777; border-radius: 5px; padding: 12px; margin-bottom: 22px; font-family: monospace; box-shadow: inset 0 2px 6px #0004; } .screen-header { display: flex; justify-content: space-between; gap: 8px; font-size: 10px; font-weight: bold; } #connection { text-align: right; } #direction { font-size: 25px; font-weight: bold; margin: 12px 0 6px; overflow-wrap: anywhere; } .status-line { font-size: 11px; margin-top: 5px; } .sensor-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-top: 12px; padding-top: 10px; border-top: 1px solid #526d3d; } .sensor { padding: 7px 3px; text-align: center; background: #88a861; border: 1px solid #698648; } .sensor-name { display: block; font-size: 10px; margin-bottom: 5px; } .sensor-value { font-size: 15px; font-weight: bold; } .edge-status { display: flex; justify-content: space-between; gap: 10px; margin-top: 9px; font-size: 10px; } .view[hidden] { display: none; } .section-label { text-align: center; font-size: 10px; font-weight: bold; color: #555; margin-bottom: 12px; letter-spacing: 1px; } .controls { display: grid; grid-template-columns: 1fr 1fr; align-items: center; gap: 12px; } .control-section { min-width: 0; } .joystick { width: min(100%, 155px); aspect-ratio: 1; margin: auto; position: relative; border-radius: 50%; background: linear-gradient(45deg, transparent 48%, #555 49%, #555 51%, transparent 52%), linear-gradient(-45deg, transparent 48%, #555 49%, #555 51%, transparent 52%), #333; border: 5px solid #999; box-shadow: 0 4px 0 #777; touch-action: none; user-select: none; } .stick { width: 43%; height: 43%; position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%); border-radius: 50%; background: linear-gradient(145deg, #555, #171717); border: 3px solid #111; box-shadow: 0 3px 4px #0008; pointer-events: none; } .buttons { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; justify-items: center; align-items: center; } .face-button { width: clamp(42px, 11vw, 53px); aspect-ratio: 1; border: 3px solid #7d2630; border-radius: 50%; background: var(--red); color: white; font-size: 18px; font-weight: bold; box-shadow: 0 4px 0 #777; cursor: pointer; touch-action: none; user-select: none; } .face-button:active, .face-button.selected { transform: translateY(3px); box-shadow: none; background: #8d202d; } .button-caption { text-align: center; font-size: 9px; color: #555; margin-top: 14px; } .panel { margin-top: 23px; padding: 13px; background: #b7b7b7; border-radius: 5px; border: 1px solid #aaa; } .panel-title { font-size: 11px; font-weight: bold; letter-spacing: 1px; margin-bottom: 11px; color: #333; } .preset-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; } .preset { padding: 11px 5px; border: 2px solid #888; border-radius: 5px; background: #d0d0d0; color: #292929; font-weight: bold; font-size: 11px; cursor: pointer; } .preset:active, .preset.selected { background: #999; transform: translateY(1px); } .autonomous { width: 100%; padding: 13px; border: 2px solid #276a39; border-radius: 5px; background: #39854d; color: white; font-size: 12px; font-weight: bold; letter-spacing: 1px; cursor: pointer; } .autonomous.enabled { background: #b83240; border-color: #7d2630; } .stop { display: block; width: 100%; margin-top: 18px; padding: 14px; background: #333; color: white; border: none; border-radius: 5px; font-size: 14px; font-weight: bold; letter-spacing: 3px; box-shadow: 0 4px 0 #777; cursor: pointer; } .stop:active { transform: translateY(3px); box-shadow: none; } .hardware-row { display: flex; justify-content: space-between; align-items: center; gap: 10px; padding: 12px 0; border-bottom: 1px solid #999; font-size: 12px; } .hardware-row:last-child { border-bottom: none; } .hardware-name { font-weight: bold; } .hardware-state { font-size: 10px; font-weight: bold; color: #555; text-align: right; } .hardware-note { font-size: 11px; line-height: 1.5; color: #444; margin-top: 14px; } .check-button { width: 100%; padding: 12px; background: #333; color: white; border: none; border-radius: 5px; margin-top: 12px; font-weight: bold; cursor: pointer; } .note { text-align: center; color: #555; font-size: 10px; margin: 18px 0 0; } @media (max-width: 360px) { body { padding: 8px; } .controller { padding: 14px; } .controls { gap: 8px; } .buttons { gap: 8px; } .joystick { width: 125px; max-width: 100%; } } .setting { display:flex; flex-wrap:wrap; align-items:center; gap:8px; margin:12px 0; font-size:12px; } .setting input[type="range"] {width:100%} .setting select {max-width:100%} button, #joystick {touch-action:none} [hidden] {display:none!important}</style>
</head><body><main class="controller"><div class="brand">SUMO BOT</div><nav><button class="nav-button active" data-view="controllerView"> CONTROLLER </button><button class="nav-button" data-view="hardwareView"> HARDWARE CHECK </button></nav>
<section class="screen"><div class="screen-header"><span>ROBOT SYSTEM</span><span id="connection">CONNECTING</span></div><div id="direction">READY</div><div class="status-line"> MODE: <span id="modeText">MANUAL</span></div><div class="status-line"> SPEED: <span id="speedText">0%</span> &nbsp; | &nbsp; INPUT: <span id="inputText">NONE</span></div><div class="sensor-grid"><div class="sensor"><span class="sensor-name">LEFT</span><span class="sensor-value" id="leftDistance">NOT INSTALLED</span></div><div class="sensor"><span class="sensor-name">FRONT</span><span class="sensor-value" id="frontDistance">-- cm</span></div><div class="sensor"><span class="sensor-name">RIGHT</span><span class="sensor-value" id="rightDistance">NOT INSTALLED</span></div></div><div class="edge-status"><span>EDGE L: <strong id="edgeLeft">--</strong></span><span>EDGE R: <strong id="edgeRight">--</strong></span></div></section>
<section class="view" id="controllerView"><div class="controls"><section class="control-section"><div class="section-label">JOYSTICK</div><div class="joystick" id="joystick"><div class="stick" id="stick"></div></div><div class="button-caption"> PUSH FARTHER = FASTER </div></section>
<section class="control-section"><div class="section-label">MOVEMENT</div><div class="buttons"><button class="face-button" data-command="forward" aria-label="Forward">B</button><button class="face-button" data-command="right" aria-label="Right">A</button><button class="face-button" data-command="left" aria-label="Left">Y</button><button class="face-button" data-command="backward" aria-label="Backward">X</button></div><div class="button-caption"> HOLD TO MOVE </div></section>
</div><section class="panel"><div class="panel-title">PRESET MOTIONS</div><div class="preset-grid"><button class="preset" data-preset="charge"> CHARGE </button><button class="preset" data-preset="spin"> SPIN </button><button class="preset" data-preset="sweep"> SWEEP </button><button class="preset" data-preset="retreat"> RETREAT </button></div><p class="hardware-note"> Hold a preset to run it. Release to stop. Charge: forward 65%; spin: right 30%; sweep: alternate turns 30%; retreat: reverse 40%. </p></section>
<section class="panel"><div class="panel-title">AUTONOMOUS SUMO</div><label class="setting">Mode <select id="autoMode"><option value="follow">Object Following</option><option value="attack">Attack Demo</option></select></label><label class="setting">Speed <span id="autoSpeedText">50%</span><input id="autoSpeed" type="range" min="20" max="100" value="50" step="1"></label><label class="setting">Detection range <span id="detectText">700 mm</span><input id="detect" type="range" min="150" max="1500" value="700" step="50"></label><label class="setting">Following distance <span id="followText">300 mm</span><input id="followDistance" type="range" min="120" max="800" value="300" step="20"></label><label class="setting"><input id="fastSensors" type="checkbox"> Fast sensor mode</label><button class="autonomous" id="autonomousButton"> ENABLE AUTONOMOUS </button><p class="hardware-note" id="autonomousInfo"> Manual control selected. Autonomous behavior is not active. </p></section>
<section class="panel"><label class="setting">Button / demo speed <span id="manualSpeedText">30%</span><input id="manualSpeed" type="range" min="20" max="100" value="30" step="1"></label><div class="panel-title">TIMED DEMOS</div><div class="preset-grid"><button class="preset" data-demo="spin">Slow Spin</button><button class="preset" data-demo="wiggle">Wiggle</button><button class="preset" data-demo="motors">Motor Check</button></div></section>
<button class="stop" id="stop">STOP ROBOT</button></section>
<section class="view" id="hardwareView" hidden><section class="panel" style="margin-top:0"><div class="panel-title">HARDWARE STATUS</div><div class="hardware-row"><span class="hardware-name">ESP32 Wi-Fi</span><span class="hardware-state" id="wifiStatus">CONNECTING</span></div><div class="hardware-row"><span class="hardware-name">Motor driver</span><span class="hardware-state" id="motorStatus">NOT VERIFIED</span></div><div class="hardware-row"><span class="hardware-name">Left ToF sensor</span><span class="hardware-state" id="tofLeftStatus">NOT INSTALLED</span></div><div class="hardware-row"><span class="hardware-name">Front ToF sensor</span><span class="hardware-state" id="tofFrontStatus">NO DATA</span></div><div class="hardware-row"><span class="hardware-name">Right ToF sensor</span><span class="hardware-state" id="tofRightStatus">NOT INSTALLED</span></div><div class="hardware-row"><span class="hardware-name">Left edge sensor</span><span class="hardware-state" id="irLeftStatus">NO DATA</span></div><div class="hardware-row"><span class="hardware-name">Right edge sensor</span><span class="hardware-state" id="irRightStatus">NO DATA</span></div><div class="hardware-row"><span class="hardware-name">I2C devices</span><span class="hardware-state" id="bus">--</span></div><button class="check-button" id="checkHardware"> REFRESH STATUS </button><p class="hardware-note" id="hardwareMessage">Waiting for live ESP32 status.</p></section>
<button class="stop" id="hardwareStop">STOP ROBOT</button></section>
<p class="note">RETRO CONTROLLER • ESP32 SUMO BOT</p></main><script>
const directionText = document.getElementById("direction"); const joystick = document.getElementById("joystick"); const stick = document.getElementById("stick"); const speedText = document.getElementById("speedText"); const inputText = document.getElementById("inputText"); const modeText = document.getElementById("modeText"); let dragging = false; let autonomous = false; let activePointer = null; let currentSpeed = 0; let activeCommand = null; let activePreset = null; function setDirection(command) { directionText.textContent = command.toUpperCase(); } function updateSpeed(value) { currentSpeed = Math.round( Math.max(0, Math.min(100, value)) ); speedText.textContent = currentSpeed + "%"; } function clearSelections() { document.querySelectorAll(".face-button, .preset").forEach(button => { button.classList.remove("selected"); }); } function resetJoystick() { stick.style.left = "50%"; stick.style.top = "50%"; } function stopRobot() { dragging = false; activePointer = null; activeCommand = null; activePreset = null; resetJoystick(); clearSelections(); updateSpeed(0); if (!autonomous) { setDirection("READY"); inputText.textContent = "NONE"; } sendCommand("stop", 0); } function getJoystickCommand(dx, dy) { const x = dx; const y = dy; if (Math.abs(x) < 0.12 && Math.abs(y) < 0.12) { return "ready"; } if (Math.abs(x) > Math.abs(y) * 1.5) { return x > 0 ? "right" : "left"; } if (Math.abs(y) > Math.abs(x) * 1.5) { return y > 0 ? "backward" : "forward"; } if (y < 0) { return x > 0 ? "forward-right" : "forward-left"; } return x > 0 ? "backward-right" : "backward-left"; } function moveJoystick(event) { const rect = joystick.getBoundingClientRect(); let dx = event.clientX - (rect.left + rect.width / 2); let dy = event.clientY - (rect.top + rect.height / 2); const max = rect.width * 0.34; const distance = Math.hypot(dx, dy); if (distance > max) { dx = dx / distance * max; dy = dy / distance * max; } stick.style.left = `calc(50% + ${dx}px)`; stick.style.top = `calc(50% + ${dy}px)`; const normalizedX = dx / max; const normalizedY = dy / max; const magnitude = Math.min( 1, Math.hypot(normalizedX, normalizedY) ); const deadZone = 0.12; if (magnitude < deadZone) { updateSpeed(0); setDirection("READY"); inputText.textContent = "JOYSTICK"; sendCommand("stop", 0); return; } const speed = Math.round( 100 * Math.pow( (magnitude - deadZone) / (1 - deadZone), 1.25 ) ); const command = getJoystickCommand( normalizedX, normalizedY ); updateSpeed(speed); setDirection(command); inputText.textContent = "JOYSTICK"; sendCommand(command, speed); } joystick.addEventListener("pointerdown", event => { if (!connected || modePending || autonomous || dragging || activeCommand || activePreset) return; event.preventDefault(); dragging = true; activePointer = event.pointerId; joystick.setPointerCapture(event.pointerId); moveJoystick(event); }); joystick.addEventListener("pointermove", event => { if (dragging && event.pointerId === activePointer) { moveJoystick(event); } }); function releaseJoystick(event) { if (!dragging) return; if ( event && activePointer !== null && event.pointerId !== activePointer ) return; stopRobot(); } joystick.addEventListener("pointerup", releaseJoystick); joystick.addEventListener("pointercancel", releaseJoystick); joystick.addEventListener("lostpointercapture", () => { if (dragging) stopRobot(); }); document.querySelectorAll("[data-command]").forEach(button => { const command = button.dataset.command; button.addEventListener("pointerdown", event => { if (!connected || modePending || autonomous || dragging || activeCommand || activePreset) return; event.preventDefault(); activeCommand = command; activePointer=event.pointerId; button.classList.add("selected"); const buttonSpeed = +document.getElementById('manualSpeed').value; updateSpeed(buttonSpeed); setDirection(command); inputText.textContent = "BUTTON"; sendCommand(command, buttonSpeed); try { button.setPointerCapture(event.pointerId); } catch (_) {} }); button.addEventListener("pointerup", event => { if (activeCommand !== command || activePointer!==event.pointerId) return; try { if (button.hasPointerCapture(event.pointerId)) { button.releasePointerCapture(event.pointerId); } } catch (_) {} stopRobot(); }); button.addEventListener("pointercancel", event=>{if(activeCommand===command && activePointer===event.pointerId)stopRobot();}); button.addEventListener("lostpointercapture", () => { if (activeCommand === command) stopRobot(); }); }); document.querySelectorAll("[data-preset]").forEach(button => { const preset = button.dataset.preset; button.addEventListener("pointerdown", event => { if (!connected || modePending || autonomous || dragging || activeCommand || activePreset) { return; } event.preventDefault(); activePreset = preset; activePointer=event.pointerId; button.classList.add("selected"); updateSpeed(100); setDirection(preset); inputText.textContent = "PRESET"; sendCommand("preset", 100, { preset }); try { button.setPointerCapture(event.pointerId); } catch (_) {} }); const releasePreset=event=>{if(activePreset===preset && activePointer===event.pointerId)stopRobot();}; button.addEventListener('pointerup',releasePreset); button.addEventListener('pointercancel',releasePreset); button.addEventListener("lostpointercapture", () => { if (activePreset === preset) stopRobot(); }); }); const autonomousButton = document.getElementById("autonomousButton"); const autonomousInfo = document.getElementById("autonomousInfo"); autonomousButton.addEventListener('click',()=>{ if(!connected||modePending) return; const enable=!autonomous; stopRobot(); sendCommand('autonomous',0,{enabled:enable}); modeText.textContent='REQUEST PENDING'; autonomousInfo.textContent='Waiting for robot confirmation.'; }); document.getElementById("stop").addEventListener("click", () => { if (autonomous) { autonomous = false; autonomousButton.textContent = "ENABLE AUTONOMOUS"; autonomousButton.classList.remove("enabled"); autonomousInfo.textContent = "Stop requested. Manual mode selected."; modeText.textContent = "MANUAL"; sendCommand("autonomous", 0, { enabled: false }); } stopRobot(); }); document.getElementById("hardwareStop").addEventListener("click", () => { if (autonomous) { autonomous = false; autonomousButton.textContent = "ENABLE AUTONOMOUS"; autonomousButton.classList.remove("enabled"); modeText.textContent = "MANUAL"; sendCommand("autonomous", 0, { enabled: false }); } stopRobot(); }); document.querySelectorAll("[data-view]").forEach(button => { button.addEventListener("click", () => { if (dragging || activeCommand || activePreset) stopRobot(); const target = button.dataset.view; document.querySelectorAll(".view").forEach(view => { view.hidden = view.id !== target; }); document.querySelectorAll(".nav-button").forEach(navButton => { navButton.classList.toggle("active", navButton === button); }); }); }); function updateSensorDisplay(data) { const distanceText = value => { return Number.isFinite(value) ? value + " cm" : "-- cm"; }; document.getElementById("leftDistance").textContent = distanceText(data.left); document.getElementById("frontDistance").textContent = distanceText(data.front); document.getElementById("rightDistance").textContent = distanceText(data.right); document.getElementById("edgeLeft").textContent = typeof data.edgeLeft === "boolean" ? (data.edgeLeft ? "EDGE" : "CLEAR") : "--"; document.getElementById("edgeRight").textContent = typeof data.edgeRight === "boolean" ? (data.edgeRight ? "EDGE" : "CLEAR") : "--"; document.getElementById("tofLeftStatus").textContent = Number.isFinite(data.left) ? "DATA RECEIVED" : "NO DATA"; document.getElementById("tofFrontStatus").textContent = Number.isFinite(data.front) ? "DATA RECEIVED" : "NO DATA"; document.getElementById("tofRightStatus").textContent = Number.isFinite(data.right) ? "DATA RECEIVED" : "NO DATA"; document.getElementById("irLeftStatus").textContent = typeof data.edgeLeft === "boolean" ? "DATA RECEIVED" : "NO DATA"; document.getElementById("irRightStatus").textContent = typeof data.edgeRight === "boolean" ? "DATA RECEIVED" : "NO DATA"; } updateSpeed(0); let wantedMotor = null, controlQueue = Promise.resolve(), motorBusy = false; let connected = false, pollBusy = false, demoToken = 0; let commandEpoch=0, modePending=false; function communicationFailed() { connected=false; wantedMotor=null; ++demoToken; ++commandEpoch; modePending=false; dragging=false; activePointer=null; activeCommand=null; activePreset=null; resetJoystick(); clearSelections(); updateSpeed(0); inputText.textContent='NONE'; directionText.textContent='LINK LOST'; modeText.textContent='UNKNOWN'; document.getElementById('connection').textContent='DISCONNECTED'; document.getElementById('wifiStatus').textContent='DISCONNECTED'; autonomousInfo.textContent='Connection lost; robot mode is unknown.'; for(const id of ['frontDistance','edgeLeft','edgeRight','tofFrontStatus','irLeftStatus','irRightStatus','motorStatus','bus']) document.getElementById(id).textContent='--'; } function request(url) { const abort = new AbortController(); const timeout = setTimeout(() => abort.abort(), 1500); return fetch(url, {cache:'no-store', signal:abort.signal}) .then(r => { if (!r.ok) throw new Error('HTTP '+r.status); return r.text(); }) .finally(() => clearTimeout(timeout)); } function ordered(url, changesMode=false) { const epoch=commandEpoch, isStop=url.startsWith('/motor?cmd=stop'); if(changesMode) modePending=true; controlQueue=controlQueue.then(async()=>{ if(!isStop && epoch!==commandEpoch) return; await request(url); }).catch(communicationFailed).finally(()=>{ if(changesMode && epoch===commandEpoch) modePending=false; }); return controlQueue; } function autoUrl(mode) { const value = id => document.getElementById(id).value; return `/auto?mode=${mode}&speed=${value('autoSpeed')}&range=${value('detect')}&follow=${value('followDistance')}&fast=${+document.getElementById('fastSensors').checked}`; } function transmitMotor() { if (!wantedMotor || motorBusy || !connected) return; motorBusy = true; const intent = wantedMotor; controlQueue = controlQueue.then(() => { if (wantedMotor !== intent) return; return request(`/motor?cmd=${intent.cmd}&pwm=${intent.pwm}`).then(reply=>{ if(reply==='EDGE') {stopRobot(); setDirection('EDGE DETECTED');} else if(reply!=='OK') throw new Error('Unexpected motor response'); }); }).catch(communicationFailed) .finally(() => { motorBusy=false; }); } function sendCommand(command, speed = 0, extra = {}) { if (command === 'status') { pollHardware(); return; } if (command === 'stop') { wantedMotor=null; ++demoToken; ++commandEpoch; ordered('/motor?cmd=stop&pwm=0',true); return; } if (!connected) return; if (command === 'autonomous') { wantedMotor=null; ++commandEpoch; ordered(autoUrl(extra.enabled ? document.getElementById('autoMode').value : 'off'),true); return; } const commands = {forward:'f', backward:'r', left:'l', right:'rt', 'forward-left':'rf', 'forward-right':'lf', 'backward-left':'rr', 'backward-right':'lr', lf:'lf',rf:'rf',f:'f',r:'r',l:'l',rt:'rt'}; if (command === 'preset') { const presets = {charge:['f',65],spin:['rt',30],sweep:['l',30],retreat:['r',40]}; const preset = presets[extra.preset]; if (!preset) return; wantedMotor={cmd:preset[0],pwm:Math.round(preset[1]*2.55), sweep:extra.preset==='sweep', started:Date.now()}; updateSpeed(preset[1]); } else { if (!commands[command]) return; wantedMotor={cmd:commands[command],pwm:Math.round(Math.max(0,Math.min(100,speed))*255/100)}; } transmitMotor(); } setInterval(() => { if (wantedMotor && wantedMotor.sweep) { wantedMotor.cmd = Math.floor((Date.now()-wantedMotor.started)/350)%2 ? 'rt':'l'; } transmitMotor(); },90); function globalStop() { autonomous=false; stopRobot(); modeText.textContent='STOP REQUESTED'; } addEventListener('blur',globalStop); addEventListener('pagehide',globalStop); document.addEventListener('visibilitychange',()=>{if(document.hidden)globalStop();}); async function pollHardware() { if(pollBusy) return; pollBusy=true; const epoch=commandEpoch, pendingAtStart=modePending; const put=(id,value)=>{document.getElementById(id).textContent=value;}; try { const d=JSON.parse(await request('/data')); if(!['OFF','FOLLOW','ATTACK'].includes(d.auto_mode) || typeof d.left_table!=='boolean' || typeof d.right_table!=='boolean') throw new Error('Invalid telemetry'); if(epoch!==commandEpoch || pendingAtStart || modePending) return; connected=true; autonomous=d.auto_mode!=='OFF'; put('connection','CONNECTED'); put('wifiStatus',d.ip); put('modeText',autonomous?d.auto_mode:'MANUAL'); put('autonomousButton',autonomous?'DISABLE AUTONOMOUS':'ENABLE AUTONOMOUS'); autonomousButton.classList.toggle('enabled',autonomous); put('autonomousInfo',d.auto_state); document.getElementById('fastSensors').checked=d.fast_mode; updateSensorDisplay({front:d.tof_valid?Math.round(d.tof_mm)/10:undefined, edgeLeft:!d.left_table,edgeRight:!d.right_table}); for(const id of ['leftDistance','rightDistance','tofLeftStatus','tofRightStatus']) put(id,'NOT INSTALLED'); put('tofFrontStatus',!d.tof_online?(d.tof_address?'FOUND / INIT FAILED':'NOT FOUND'): d.tof_timeout?'TIMEOUT':d.tof_valid?'ONLINE':'NO VALID RANGE'); put('irLeftStatus',d.left_table?'TABLE':'EDGE'); put('irRightStatus',d.right_table?'TABLE':'EDGE'); put('motorStatus',`OUTPUT A ${Math.round(d.motor_a/2.55)}% / B ${Math.round(d.motor_b/2.55)}%`); put('bus',d.i2c_devices); put('hardwareMessage','Motor values are commanded outputs, not physical motor feedback.'); const edge=!d.left_table||!d.right_table; if(edge && !autonomous && wantedMotor) stopRobot(); if(edge) setDirection(autonomous?d.auto_state:'EDGE DETECTED'); else if(autonomous) setDirection(d.auto_state); else if(!wantedMotor) setDirection('READY'); } catch (_) { const wasConnected=connected; communicationFailed(); if(wasConnected) sendCommand('stop'); put('connection','DISCONNECTED'); put('wifiStatus','DISCONNECTED'); put('modeText','UNKNOWN'); put('autonomousInfo','Connection lost; robot mode is unknown.'); put('hardwareMessage','Connection lost; hardware readings are unavailable.'); for(const id of ['frontDistance','edgeLeft','edgeRight','tofFrontStatus','irLeftStatus','irRightStatus','motorStatus','bus']) put(id,'--'); } finally {pollBusy=false;} } document.getElementById('checkHardware').onclick=pollHardware; document.getElementById('fastSensors').onchange=()=>{if(connected)ordered(autoUrl('settings'));}; for(const [id,out,suffix] of [['manualSpeed','manualSpeedText','%'],['autoSpeed','autoSpeedText','%'],['detect','detectText',' mm'],['followDistance','followText',' mm']]) { document.getElementById(id).oninput=()=>{document.getElementById(out).textContent=document.getElementById(id).value+suffix;}; } const demos={spin:[['rt',2500]],wiggle:Array.from({length:6},(_,i)=>[i%2?'rt':'l',350]), motors:[['lf',700],['rf',700],['f',700],['r',700]]}; document.querySelectorAll('[data-demo]').forEach(button=>button.onclick=async()=>{ if(!connected||modePending||autonomous||dragging||activeCommand||activePreset) return; stopRobot(); const token=++demoToken; activePreset='demo'; for(const [cmd,ms] of demos[button.dataset.demo]) { if(token!==demoToken) return; const speed=+document.getElementById('manualSpeed').value; updateSpeed(speed); setDirection(button.textContent.trim()); inputText.textContent='DEMO'; sendCommand(cmd,speed); await new Promise(resolve=>setTimeout(resolve,ms)); } if(token===demoToken)stopRobot(); }); setInterval(pollHardware,200); pollHardware();
</script></body></html>
)HTML";
 

void scanI2C() {
  // A full bus scan stalls the web server when a wire is loose.
  Wire.beginTransmission(0x29);
  tofAddressFound = Wire.endTransmission() == 0;
}
 

void setOneMotor(int in1, int in2, int pwmPin, int power) {
  power = -power;  // N20 motors are mounted opposite the driver labels.
 

  if (power == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, 255);
    return;
  }
 

  digitalWrite(in1, power > 0 ? HIGH : LOW);
  digitalWrite(in2, power > 0 ? LOW : HIGH);
  analogWrite(pwmPin, abs(power));
}
 

void setMotors(int leftPower, int rightPower) {
  motorA = leftPower;
  motorB = rightPower;
  digitalWrite(STBY, HIGH);
  setOneMotor(AIN1, AIN2, PWMA, leftPower);
  setOneMotor(BIN1, BIN2, PWMB, rightPower);
}
 

void brake() {
  setMotors(0, 0);
}
 

void IRAM_ATTR edgeISR() {
  // Disable the driver before the main loop handles recovery.
  digitalWrite(STBY, LOW);
  edgeInterrupt = true;
}
 

void readFloor() {
  leftTable = digitalRead(LEFT_IR) == TABLE_LEVEL;
  rightTable = digitalRead(RIGHT_IR) == TABLE_LEVEL;
 

  bool edgeDetected = !leftTable || !rightTable;
  bool manualDriving = autoMode == AUTO_OFF && (motorA || motorB);
  if (edgeDetected && manualDriving) brake();
}
 

void configureTofSpeed(bool fast) {
  if (!tofOnline) return;
 

  tof.stopContinuous();
  tof.setMeasurementTimingBudget(fast ? 20000 : 33000);
  tof.startContinuous(fast ? 20 : 40);
}
 

void stopAuto() {
  autoMode = AUTO_OFF;
  edgeRecovery = 0;
  autoStateText = "Stopped";
  brake();
}
 

void handleAuto() {
  String requestedMode = server.arg("mode");
  server.sendHeader("Cache-Control", "no-store");
  if (requestedMode == "off") {
    stopAuto();
    server.send(200, "text/plain", autoStateText);
    return;
  }
  if (requestedMode != "settings" && requestedMode != "follow" && requestedMode != "attack") {
    server.send(400, "text/plain", "Invalid autonomous mode");
    return;
  }
  // Partial settings requests retain values not supplied by the caller.
  if (server.hasArg("speed")) autoPwm = constrain(server.arg("speed").toInt(), 20, 100) * 255 / 100;
  if (server.hasArg("range")) detectMm = constrain(server.arg("range").toInt(), 150, 1500);
  if (server.hasArg("follow")) followMm = constrain(server.arg("follow").toInt(), 120, 800);
  if (server.hasArg("fast")) fastSensors = server.arg("fast") == "1";
  readFloor();
  if (requestedMode == "settings") {
    configureTofSpeed(fastSensors);
  } else if (!tofOnline || tofTimeout) {
    stopAuto();
    autoStateText = "ToF sensor unavailable";
  } else if (!leftTable || !rightTable) {
    stopAuto();
    autoStateText = "Place robot on table";
  } else {
    autoMode = requestedMode == "attack" ? AUTO_ATTACK : AUTO_FOLLOW;
    edgeRecovery = 0;
    edgeInterrupt = false;
    searchChanged = millis();
    autoStateText = "Starting";
    configureTofSpeed(fastSensors);
  }
  server.send(200, "text/plain", autoStateText);
}

bool handleEdgeRecovery() {
  bool edgeDetected = !leftTable || !rightTable || edgeInterrupt;
 

  if (edgeDetected && edgeRecovery == 0) {
    edgeInterrupt = false;
    edgeRecovery = 1;
    recoveryUntil = millis() + 300;
    autoStateText = "Backing away from edge";
    int recoveryPower = min(autoPwm, 110);
    setMotors(-recoveryPower, -recoveryPower);
    return true;
  }
 

  if (edgeRecovery == 1) {
    if (millis() < recoveryUntil) {
      int recoveryPower = min(autoPwm, 110);
      setMotors(-recoveryPower, -recoveryPower);
      return true;
    }
 

    brake();
    edgeRecovery = 2;
    autoStateText = "Waiting for table";
    return true;
  }
 

  if (edgeRecovery == 2) {
    brake();
    if (!leftTable || !rightTable) return true;
 

    edgeRecovery = 0;
    searchDirection = -searchDirection;
    searchChanged = millis();
  }
 

  return false;
}
 

void updateAutonomous() {
  if (autoMode == AUTO_OFF || handleEdgeRecovery()) return;
 

  bool targetSeen = tofOnline && !tofTimeout && tofMm > 0 &&
                    tofMm < 8190 && tofMm <= detectMm;
  int searchPwm = max(55, min(105, autoPwm / 2));
 

  if (autoMode == AUTO_ATTACK) {
    if (targetSeen) {
      autoStateText = "Attacking";
      setMotors(autoPwm, autoPwm);
    } else {
      autoStateText = "Searching";
      setMotors(searchDirection * searchPwm, -searchDirection * searchPwm);
    }
    return;
  }
 

  if (!targetSeen) {
    if (millis() - searchChanged > 1400) {
      searchDirection = -searchDirection;
      searchChanged = millis();
    }
    autoStateText = "Looking for object";
    setMotors(searchDirection * searchPwm, -searchDirection * searchPwm);
  } else if (tofMm > followMm + 60) {
    autoStateText = "Following";
    setMotors(autoPwm, autoPwm);
  } else if (tofMm + 60 < followMm) {
    autoStateText = "Making space";
    setMotors(-autoPwm / 2, -autoPwm / 2);
  } else {
    autoStateText = "Target reached";
    brake();
  }
}






 

void handleMotor() {
  server.sendHeader("Cache-Control", "no-store");
  if (autoMode != AUTO_OFF) stopAuto();
 

  readFloor();
  int power = constrain(server.arg("pwm").toInt(), 0, 255);
  String command = server.arg("cmd");
  bool edgeDetected = !leftTable || !rightTable;
 

  if (command == "stop" || edgeDetected) brake();
  else if (command == "lf") setMotors(power, 0);
  else if (command == "lr") setMotors(-power, 0);
  else if (command == "rf") setMotors(0, power);
  else if (command == "rr") setMotors(0, -power);
  else if (command == "f") setMotors(power, power);
  else if (command == "r") setMotors(-power, -power);
  else if (command == "l") setMotors(-power, power);
  else if (command == "rt") setMotors(power, -power);
  else brake();
 

  lastMotorCommand = millis();
  server.send(200, "text/plain", edgeDetected ? "EDGE" : "OK");
}





 

void handleData() {
  readFloor();
  char json[650];
  const char* devices = tofAddressFound ? "0x29" : "None";
  bool tofValid = tofOnline && !tofTimeout && tofMm > 0 && tofMm < 8190;
  const char* mode = autoMode == AUTO_FOLLOW ? "FOLLOW" :
                     autoMode == AUTO_ATTACK ? "ATTACK" : "OFF";
 

  snprintf(json, sizeof(json),
           "{\"ip\":\"%s\",\"left_table\":%s,\"right_table\":%s,"
           "\"tof_online\":%s,\"tof_address\":%s,\"tof_timeout\":%s,"
           "\"tof_valid\":%s,\"tof_mm\":%u,\"tof_samples\":%lu,"
           "\"motor_a\":%d,\"motor_b\":%d,\"i2c_devices\":\"%s\","
           "\"auto_mode\":\"%s\",\"auto_state\":\"%s\",\"fast_mode\":%s}",
           WiFi.softAPIP().toString().c_str(),
           leftTable ? "true" : "false", rightTable ? "true" : "false",
           tofOnline ? "true" : "false", tofAddressFound ? "true" : "false",
           tofTimeout ? "true" : "false", tofValid ? "true" : "false",
           tofMm, tofSamples, abs(motorA), abs(motorB), devices, mode,
           autoStateText, fastSensors ? "true" : "false");
 

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}
 

void setupPins() {
  const int outputs[] = {PWMA, AIN2, AIN1, STBY, BIN1, BIN2, PWMB};
  for (int pin : outputs) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
 

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  digitalWrite(STBY, LOW);
  attachInterrupt(digitalPinToInterrupt(LEFT_IR), edgeISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_IR), edgeISR, RISING);
}
 

void setupTof() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(12);
  delay(150);
 

  scanI2C();
  tof.setTimeout(60);
  tofOnline = tofAddressFound && tof.init();
  if (tofOnline) {
    tof.setMeasurementTimingBudget(33000);
    tof.startContinuous(40);
  }
}
 

void setupWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASS);
 

  server.on("/", []() { server.sendHeader("Cache-Control", "no-store"); server.send_P(200, "text/html; charset=utf-8", DASHBOARD); });
  server.on("/data", handleData);
  server.on("/motor", handleMotor);
  server.on("/auto", handleAuto);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302);
  });
  server.begin();
}
 

void setup() {
  Serial.begin(115200);
  setupPins();
  setupTof();
  setupWebServer();
 

  Serial.printf("Ready at http://%s | ToF: %s\n",
                WiFi.softAPIP().toString().c_str(),
                tofOnline ? "online" : "not found");
}
 

void loop() {
  server.handleClient();
  readFloor();
  updateAutonomous();
 

  bool manualTimedOut = autoMode == AUTO_OFF && (motorA || motorB) &&
                        millis() - lastMotorCommand > MOTOR_TIMEOUT_MS;
  if (manualTimedOut) brake();
 

  if (!tofOnline && millis() - lastBusScan >= 3000) {
    lastBusScan = millis();
    scanI2C();
  }
 

  if (!tofOnline && tofAddressFound && millis() - lastTofRetry >= 3000) {
    lastTofRetry = millis();
    tof.setTimeout(60);
    tofOnline = tof.init();
    if (tofOnline) configureTofSpeed(fastSensors);
  }
 

  unsigned long tofInterval = fastSensors ? 20 : 40;
  if (tofOnline && millis() - lastTofRead >= tofInterval) {
    lastTofRead = millis();
    tofMm = tof.readRangeContinuousMillimeters();
    tofTimeout = tof.timeoutOccurred();
    ++tofSamples;
  }
 

  if (!fastSensors) delay(1);
}