#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>

#define CAN_TX 10
#define CAN_RX 20
const char* AP_SSID = "VCD-Tool";
const char* AP_PASS = "123456789";

struct Telemetry {
  uint16_t soc;
  uint16_t soh;
  uint16_t deltaSoc;
  uint16_t voltage;
  uint16_t imbalance;
  int      balancing;
  int      rpm;
  int      driveMode;
  bool     sideStand;
  bool     frontBrake, rearBrake, highBeam, startSwitch;
  bool     killSwitch, storageSwitch, horn;
  bool     indicatorLeftSide, indicatorRightSide, indicatorCenterSide, keyOn;
  int      rangeKm;
  uint16_t cellV[14];
  uint8_t  cellSoh[14];
  uint32_t chargerOV;
  uint32_t chargerOI;
  uint32_t chargerIV;
  int16_t  motorI;
  int16_t  motorTemp;
  bool     displayFan;
  uint16_t chargerFan1, chargerFan2;
  bool     forcedEco, smartEco;
  bool     seatLock, chargerLock1, chargerLock2;
  bool     chargerConnected;
  int16_t  cellTemp[5];
  int16_t  afeDieTemp, packCurrent;
  int16_t  humidity, bmsAmbient;
  int16_t  eftTemp, eciTemp, senseTemp;
  uint16_t deratedDsgI, deratedChgI, deratedRegenI, dsgLimTemp;
  uint16_t chgCellRule, chgCmdRateLtd;
  int16_t  chgPostQuant;
  uint16_t soe, socEkf, maxSoc, minSocEst;
  int16_t  minV1Est, minV2Est;
  uint16_t cellSoc[14];
  uint16_t boardTempRefV, vref2, vref3, mosfetPwrV, afeVreg;
  int16_t  imuAccX, imuAccY, imuAccZ;
  int16_t  eulerRoll, eulerPitch, eulerHeading;
  uint32_t odometer;
  uint64_t bmsUptimeMs;
  uint8_t  bmsHwVer, bmsFw[3], bcmFw[3], gtwFw[3], motorFw[3];
  char     binId[16];
  uint16_t pduVoltage;              
  float    chgSocLimit;             
  int16_t  controllerTemp;          
  int16_t  motorMcuTemp;            
  int16_t  throttleOut, throttle1, throttle3;  
  int32_t  tcRpm;                   
  int32_t  mcRpm;                   
  char     vin[20];                 
  char     scooterId[10];           
  unsigned long lastDiagMs;        
  uint8_t  navJoystick;             
  uint8_t  parkAssistA, parkAssistB;             
  unsigned long lastParkAMs, lastParkBMs;
  unsigned long lastVinMs, lastSidMs, lastGtwMs;  
};

Telemetry snap;
SemaphoreHandle_t mtx;
WebServer server(80);
unsigned long lastDiag = 0;
uint32_t maxQueueBacklog = 0;
volatile uint32_t lastCanRxMs = 0;
const uint32_t CAN_OFFLINE_TIMEOUT_MS = 3000;
const uint32_t CAN_DEGRADED_TIMEOUT_MS = 1200;

inline const char* driveModeName(int v){
  switch(v){
    case 1: return "SPORTS";
    case 2: return "DRIVE";
    case 3: return "ECO";
    case 6: return "WARP";
    case 8: return "S. ECO";
    default: return "UNKNOWN";
  }
}

const char htmlPage[] PROGMEM =
"<!doctype html>\n"
"<html lang=\"en\">\n"
"<head>\n"
"<meta charset=\"utf-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"<title>Vehicle CAN Diagnostic Tool </title>\n"
"<style>\n"
"  :root {\n"
"    --blue: #2f52ad; --blue-dark: #244493; --green: #18c96b;\n"
"    --amber: #e7a526; --red: #dc3545; --ink: #20242b;\n"
"    --muted: #737b88; --line: #dce2ec; --page: #f4f6fa; --card: #fff;\n"
"  }\n"
"  * { box-sizing: border-box; }\n"
"  body { margin: 0; font-family: Inter, ui-sans-serif, system-ui, -apple-system, \"Segoe UI\", \"Segoe UI Emoji\", \"Apple Color Emoji\", \"Noto Color Emoji\", Arial, sans-serif; color: var(--ink); background: var(--page); }\n"
"  button { font: inherit; }\n"
"  .topbar { min-height: 76px; background: var(--blue); color: white; display:flex; align-items:center; justify-content:space-between; gap:16px; padding: 15px clamp(16px, 4vw, 48px); }\n"
"  .brand { font-weight: 800; letter-spacing:.055em; font-size: clamp(16px, 2vw, 22px); line-height:1.15; text-transform:uppercase; }\n"
"  .brand small { display:block; font-size:.67em; letter-spacing:.04em; margin-top:4px; opacity:.96; }\n"
"  .hamburger { width:42px; height:42px; display:grid; place-content:center; gap:5px; border:1px solid #ffffff55; background:#ffffff10; border-radius:10px; cursor:pointer; }\n"
"  .hamburger span { width:21px; height:2px; display:block; background:white; border-radius:2px; }\n"
"  .status { display:flex; justify-content:center; align-items:center; min-height:36px; color:white; font-size:13px; font-weight:800; letter-spacing:.11em; transition:background .2s ease; }\n"
"  .status.online { background:var(--green); }\n"
"  .status.amber { background:var(--amber); color:#2b2516; }\n"
"  .status.offline { background:var(--red); }\n"
"  .pulse { display:inline-flex; align-items:center; gap:5px; margin-right:7px; }\n"
"  .pulse .paren { animation:pulse 1.1s ease-in-out infinite; }\n"
"  .pulse .dot { width:9px; height:9px; border-radius:50%; background:currentColor; display:inline-block; box-shadow:0 0 0 0 currentColor; animation:dotpulse 1.1s infinite; }\n"
"  .status.amber .dot { animation:none; box-shadow:none; }\n"
"  .status.offline .pulse { display:none; }\n"
"  .status.submenu-mode #statusText, .status.submenu-mode .pulse { display:none; }\n"
"  @keyframes pulse { 0%,100% { opacity:.25; transform:scale(.94); } 50% { opacity:1; transform:scale(1.08); } }\n"
"  @keyframes dotpulse { 0%,100% { box-shadow:0 0 0 0 #ffffff66; } 50% { box-shadow:0 0 0 5px #ffffff20; } }\n"
"  .wrap { width:100%; max-width:none; margin:0; padding:clamp(16px,3vw,34px) clamp(18px,4vw,56px); }\n"
"  .wrap:has(.detail[style*=\"display: block\"]) { padding-top:0; }\n"
"  .home-card { background:transparent; border:0; box-shadow:none; padding:clamp(18px,3vw,34px) 0; display:grid; grid-template-columns:minmax(180px,.85fr) minmax(0,1.6fr); align-items:center; gap:clamp(18px,4vw,44px); min-height:360px; }\n"
"  .scooter-area { display:flex; flex-direction:column; align-items:center; justify-content:center; gap:14px; min-width:0; }\n"
"  .scooter { width:min(100%, 310px); height:auto; }\n"
"  .scooter-reference { display:block; width:min(100%, 330px); height:auto; max-height:250px; object-fit:contain; overflow:visible; transition:filter .35s ease, opacity .35s ease, transform .35s ease; transform-origin:50% 70%; }\n"
"  .scooter-placeholder { width:min(100%, 330px); height:auto; aspect-ratio:786/736; display:block; object-fit:contain; border-radius:24px; background:linear-gradient(145deg, rgba(120,140,160,.08), rgba(120,140,160,.02)); }\n"
"  .tile:disabled, .menu button:disabled { cursor:not-allowed !important; opacity:.32 !important; filter:grayscale(1) !important; pointer-events:none !important; }\n"
"  .control:disabled, #menuToggle:disabled, .back:disabled { cursor:not-allowed !important; opacity:.35 !important; filter:grayscale(1) !important; pointer-events:none !important; }\n"
"  .scooter-area.online .scooter-reference { animation:scooterFloat 2.8s ease-in-out infinite; filter:none; opacity:1; }\n"
"  .scooter-area.amber .scooter-reference { animation:scooterFloat 3.8s ease-in-out infinite; filter:saturate(.78); opacity:.9; }\n"
"  .scooter-area.offline .scooter-reference { animation:none; filter:grayscale(1); opacity:.48; transform:none; }\n"
"  @keyframes scooterFloat { 0%,100% { transform:translateY(0); } 50% { transform:translateY(-4px); } }\n"
"  .scooter-caption { color:var(--muted); font-size:12px; text-align:center; }\n"
"  .tiles { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:18px 12px; }\n"
"  .tile { border:0; background:transparent; border-radius:14px; padding:9px 4px 8px; min-width:0; color:var(--ink); cursor:pointer; transition:transform .16s ease, background .16s ease, opacity .16s ease; }\n"
"  .tile:hover:not(:disabled), .tile:focus-visible:not(:disabled) { background:#f5f7fc; transform:translateY(-2px); outline:2px solid #cbd7f6; outline-offset:1px; }\n"
"  .tile:disabled { cursor:not-allowed; opacity:.35; filter:grayscale(1); }\n"
"  .tile-icon { width:50px; height:50px; border-radius:14px; display:grid; place-items:center; margin:0 auto 9px; }\n"
"  .tile-icon svg { width:25px; height:25px; fill:none; stroke:currentColor; stroke-width:1.9; stroke-linecap:round; stroke-linejoin:round; }\n"
"  .tile-label { font-size:13px; line-height:1.25; font-weight:650; display:block; min-height:33px; }\n"
"  .vehicle { background:#e1eaff; color:#2355b6; } .cells { background:#ddf7e8; color:#167746; }\n"
"  .bms { background:#f0e3ff; color:#7944a7; } .motor { background:#ffeadb; color:#b94c14; }\n"
"  .charger { background:#fff2c9; color:#966a00; } .firmware { background:#dff1ff; color:#17628f; }\n"
"  .thermals { background:#ffe0e7; color:#b42b4c; } .others { background:#e8eaee; color:#4a5565; }\n"
"  .info { margin:18px 0 0; color:var(--muted); font-size:13px; line-height:1.5; display:flex; justify-content:space-between; gap:14px; flex-wrap:wrap; }\n"
"  .demo-controls { display:none; background:#fff; border:1px solid var(--line); border-radius:14px; padding:16px; margin:18px 0 0; }\n"
"  .demo-controls h2 { font-size:14px; margin:0 0 10px; }\n"
"  .control-row { display:flex; gap:9px; flex-wrap:wrap; align-items:center; }\n"
"  .control { border:1px solid var(--line); background:#fff; border-radius:9px; padding:9px 13px; cursor:pointer; font-size:13px; font-weight:650; }\n"
"  .control.active { box-shadow:inset 0 0 0 2px currentColor; }\n"
"  .control[data-mode=\"online\"] { color:#137842; } .control[data-mode=\"amber\"] { color:#8c6500; } .control[data-mode=\"offline\"] { color:#bd2434; }\n"
"  .demo-note { margin:10px 0 0; color:var(--muted); font-size:12px; line-height:1.45; }\n"
"  .detail { display:none; background:transparent; border:0; min-height:calc(100vh - 120px); margin-top:0; }\n"
"  .detail-head { display:flex; align-items:center; gap:12px; background:#272727; color:white; padding:14px clamp(16px,3vw,30px); margin:calc(-1 * clamp(16px,3vw,34px)) calc(-1 * clamp(18px,4vw,56px)) 0; }\n"
"  .back { border:0; color:white; background:transparent; cursor:pointer; font-size:22px; padding:0 8px 0 0; }\n"
"  .detail-head .tile-icon { width:38px; height:38px; margin:0; border-radius:9px; flex-shrink:0; }\n"
"  .detail-head h2 { margin:0; font-size:16px; }\n"
"  .detail-head #detailIcon { display:grid; }\n"
"  .detail-head { min-height:0; padding-top:14px; padding-bottom:14px; }\n"
"  .detail-body { padding:clamp(18px,3vw,32px) 0; }\n"
"  .detail-body p { color:var(--muted); font-size:14px; line-height:1.5; }\n"
"  .placeholder-grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px; margin-top:18px; }\n"
"  .metric { border:1px solid var(--line); border-radius:12px; padding:16px; }\n"
"  .metric small { color:var(--muted); display:block; margin-bottom:7px; }\n"
"  .metric strong { font-size:21px; }\n"
"\n"
"\n"
"  .data-connection { margin:0 0 18px; padding:12px 14px; border:1px solid var(--line); border-radius:12px; display:flex; align-items:center; gap:10px; background:#fff; font-size:12px; color:var(--muted); }\n"
"  .connection-dot { width:9px; height:9px; border-radius:50%; background:#9aa3af; flex-shrink:0; }\n"
"  .data-connection.connected .connection-dot { background:#18c96b; box-shadow:0 0 0 4px #18c96b20; }\n"
"  .data-connection.disconnected .connection-dot { background:#dc3545; }\n"
"  .telemetry-intro { margin:0 0 18px; color:var(--muted); font-size:13px; line-height:1.5; }\n"
"  .telemetry-group { margin:24px 0 0; }\n"
"  .group-title { display:flex; align-items:center; gap:9px; margin:0 0 12px; font-size:14px; font-weight:800; letter-spacing:.015em; }\n"
"  .group-title .group-symbol { width:30px; height:30px; display:grid; place-items:center; border-radius:9px; background:#e8eefb; font-size:16px; }\n"
"  .telemetry-grid { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:10px; }\n"
"  .telemetry-item { position:relative; min-width:0; padding:14px 14px 13px; border:1px solid #e3e8f1; border-radius:13px; background:linear-gradient(145deg,#fff,#fafbfe); box-shadow:0 3px 12px #233b6b08; }\n"
"  .telemetry-item:before { content:\"\"; position:absolute; left:0; top:12px; bottom:12px; width:3px; border-radius:0 4px 4px 0; background:var(--metric-accent,#8ba4d9); }\n"
"  .telemetry-label { display:flex; align-items:flex-start; gap:7px; color:#596477; font-size:12px; line-height:1.35; min-height:31px; }\n"
"  .telemetry-label .mini-icon { font-size:14px; line-height:1; }\n"
"  .telemetry-value { display:block; margin-top:8px; font-size:19px; font-weight:800; letter-spacing:-.025em; color:#202938; overflow-wrap:anywhere; }\n"
"  .telemetry-unit { font-size:11px; font-weight:650; color:#7b8492; margin-left:4px; letter-spacing:0; }\n"
"  .field-tag { display:inline-block; margin-top:8px; padding:3px 6px; border-radius:5px; background:#eef2f8; color:#687487; font-size:9px; font-weight:800; letter-spacing:.06em; text-transform:uppercase; }\n"
"  .field-tag.estimated { background:#fff3d5; color:#946700; }\n"
"  .field-tag.hypothesis { background:#f3e8ff; color:#7b42a5; }\n"
"\n"
"  .cell-table-wrap { margin-top:14px; overflow-x:auto; border:1px solid #dfe6ef; border-radius:12px; background:#fff; }\n"
"  .cell-table { width:100%; border-collapse:collapse; font-size:13px; min-width:420px; }\n"
"  .cell-table th { padding:12px 14px; text-align:left; background:#eef3fb; color:#42516a; font-size:11px; letter-spacing:.04em; text-transform:uppercase; }\n"
"  .cell-table td { padding:10px 14px; border-top:1px solid #edf0f5; }\n"
"  .cell-table tbody tr:nth-child(even) { background:#fafbfd; }\n"
"  .cell-name { font-weight:750; color:#2f52ad; }\n"
"  .about-content { max-width:900px; margin:0 auto; }\n"
"  .about-hero { padding:26px; border-radius:18px; background:linear-gradient(135deg,#eaf0ff,#f6f8ff 55%,#e5f7f0); }\n"
"  .about-hero .about-mark { font-size:34px; }\n"
"  .about-hero h3 { margin:8px 0; font-size:22px; }\n"
"  .about-hero p { margin:0; max-width:700px; }\n"
"  .about-section { margin-top:24px; }\n"
"  .about-section h3 { font-size:15px; margin:0 0 12px; }\n"
"  .about-link-row { display:flex; align-items:center; gap:12px; padding:14px; border:1px solid var(--line); border-radius:12px; margin-top:9px; background:#fff; }\n"
"  .about-link-icon { width:38px; height:38px; display:grid; place-items:center; border-radius:10px; background:#edf1f8; font-size:18px; }\n"
"  .about-link-row strong { display:block; font-size:13px; }\n"
"  .about-link-row small { display:block; color:var(--muted); margin-top:3px; }\n"
"  .about-link-placeholder { margin-left:auto; font-size:11px; font-weight:800; color:#8a94a3; background:#f0f2f6; padding:6px 8px; border-radius:7px; }\n"
"\n"
"  .empty-state { margin-top:18px; padding:28px 20px; border:1px dashed #cbd3df; border-radius:14px; text-align:center; color:var(--muted); background:#ffffff80; }\n"
"  .empty-state .empty-icon { font-size:30px; margin-bottom:8px; }\n"
"  .detail-body > .prototype-note { margin-top:24px; padding:12px 14px; border-left:3px solid #e7a526; background:#fff9e9; border-radius:0 9px 9px 0; color:#76612d; font-size:12px; }\n"
"  @media (max-width:900px) { .telemetry-grid { grid-template-columns:repeat(2,minmax(0,1fr)); } }\n"
"  @media (max-width:480px) {\n"
"    .telemetry-grid { grid-template-columns:repeat(2,minmax(0,1fr)); gap:8px; }\n"
"    .telemetry-item { padding:12px 10px 11px; }\n"
"    .telemetry-label { font-size:11px; min-height:32px; }\n"
"    .telemetry-value { font-size:17px; }\n"
"    .group-title { font-size:13px; }\n"
"    .detail-body { padding:14px 0; }\n"
"  }\n"
"\n"
"  .menu { display:none; background:var(--blue-dark); padding:10px clamp(16px,4vw,48px); gap:8px; flex-wrap:wrap; }\n"
"  .menu.open { display:flex; }\n"
"  .menu button { border:1px solid #ffffff44; background:#ffffff10; color:white; border-radius:8px; padding:8px 11px; cursor:pointer; }\n"
"  .menu button:disabled { opacity:.35; cursor:not-allowed; }\n"
"  @media (max-width:760px) {\n"
"    .topbar { min-height:72px; }\n"
"    .wrap { padding:14px; }\n"
"    .home-card { grid-template-columns:1fr; gap:15px; padding:18px 12px 16px; min-height:0; }\n"
"    .scooter { width:min(65%,240px); }\n"
"    .scooter-reference { width:min(72%,260px); max-height:210px; }\n"
"    .tiles { grid-template-columns:repeat(4,minmax(0,1fr)); gap:10px 2px; }\n"
"    .tile-icon { width:44px; height:44px; border-radius:12px; }\n"
"    .tile-label { font-size:11px; min-height:29px; }\n"
"    .detail-body { padding:16px; }\n"
"  }\n"
"  @media (max-width:390px) {\n"
"    .tiles { grid-template-columns:repeat(2,minmax(0,1fr)); gap:10px; }\n"
"    .tile { display:flex; align-items:center; gap:10px; text-align:left; padding:8px; }\n"
"    .tile-icon { margin:0; flex-shrink:0; }\n"
"    .tile-label { min-height:0; font-size:12px; }\n"
"  }\n"
"  @media (min-width:1100px) {\n"
"    .wrap { padding-top:34px; }\n"
"    .home-card { min-height:410px; }\n"
"    .tile-icon { width:56px; height:56px; }\n"
"    .tile-label { font-size:14px; }\n"
"  }\n"
"  @media (prefers-reduced-motion: reduce) { *,*::before,*::after { animation-duration:.01ms !important; animation-iteration-count:1 !important; transition-duration:.01ms !important; } }\n"
".about-placeholder-link{color:inherit;text-decoration:underline;opacity:.75;cursor:default}\n"
".cell-table-wrap{width:100%;max-width:100%;overflow-x:auto;-webkit-overflow-scrolling:touch}\n.cell-table{width:100%;min-width:0;table-layout:fixed;border-collapse:collapse}\n.cell-table th,.cell-table td{padding:7px 4px;overflow-wrap:anywhere;word-break:break-word;font-size:clamp(10px,2.5vw,13px)}\n.cell-table th:first-child,.cell-table td:first-child{width:24%}\n.cell-table th:not(:first-child),.cell-table td:not(:first-child){width:25.33%}\n</style>\n"
"</head>\n"
"<body>\n"
"<header class=\"topbar\">\n"
"  <div class=\"brand\">Vehicle CAN <small>Diagnostic Tool</small></div>\n"
"  <button class=\"hamburger\" id=\"menuToggle\" aria-label=\"Open navigation menu\" aria-expanded=\"false\"><span></span><span></span><span></span></button>\n"
"</header>\n"
"<div id=\"statusBar\" class=\"status offline\" role=\"status\" aria-live=\"polite\">\n"
"  <span class=\"pulse\" aria-hidden=\"true\"><span class=\"paren\">((</span><span class=\"dot\"></span><span class=\"paren\">))</span></span>\n"
"  <span id=\"statusText\">ONLINE</span>\n"
"</div>\n"
"<nav id=\"menu\" class=\"menu\" aria-label=\"Category navigation\"></nav>\n"
"<main class=\"wrap\">\n"
"  <section id=\"homeView\" class=\"home-card\" aria-label=\"Diagnostic categories\">\n"
"    <div class=\"scooter-area offline\">\n"
"      <!-- Background-removed scooter image embedded as SVG for a portable single-file prototype -->\n"
"      <img class=\"scooter-reference scooter-placeholder\" src=\"data:image/webp;base64,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\" alt=\"Ather electric scooter\" width=\"786\" height=\"736\">\n"
"      <div class=\"scooter-caption\">Vehicle diagnostic overview</div>\n"
"    </div>\n"
"    <div id=\"tiles\" class=\"tiles\"></div>\n"
"  </section>\n"
"\n"
"  <section id=\"detailView\" class=\"detail\" aria-live=\"polite\">\n"
"    <div class=\"detail-head\">\n"
"      <button class=\"back\" id=\"backButton\" aria-label=\"Back to categories\">❮❮</button>\n"
"      <div id=\"detailIcon\" class=\"tile-icon vehicle\"></div>\n"
"      <h2 id=\"detailTitle\">Vehicle</h2>\n"
"      <span style=\"margin-left:auto;color:#18c96b;font-size:12px;font-weight:800\" id=\"detailLive\">LIVE</span>\n"
"    </div>\n"
"    <div class=\"detail-body\">\n"
"      <div class=\"data-connection\" id=\"dataConnection\"><span class=\"connection-dot\"></span><span><strong id=\"connectionTitle\">ESP32 data source not checked</strong><br><span id=\"connectionMessage\">For live data, serve this updated page from the ESP32 firmware so it can call the existing /data, /cells, /charger, and /extra endpoints.</span></span></div>\n"
"      <p id=\"detailDescription\">Category telemetry will appear here when connected to the vehicle.</p>\n"
"      <div id=\"telemetryContent\"></div>\n"
"      <p class=\"prototype-note\" id=\"dataSourceNote\">Waiting for the ESP32 telemetry API. This page uses only fields exposed by the supplied firmware; unsupported fields remain unavailable. Estimated or hypothesized signals are marked.</p>\n"
"    </div>\n"
"  </section>\n"
"\n"
"  <section class=\"demo-controls\" aria-label=\"Prototype controls\">\n"
"    <h2>Local prototype controls</h2>\n"
"    <div class=\"control-row\">\n"
"      <button class=\"control active\" data-mode=\"online\">Simulate Online</button>\n"
"      <button class=\"control\" data-mode=\"amber\">Simulate Intermittent CAN</button>\n"
"      <button class=\"control\" data-mode=\"offline\">Simulate Offline</button>\n"
"    </div>\n"
"    <p class=\"demo-note\">These buttons are for testing the UI only. Online = fresh CAN frames and animated (( ● )). Amber = CAN frames still arriving, but intermittently. Offline = no CAN frames; animation disappears and all eight category options become greyed out and unclickable. The prototype starts online.</p>\n"
"  </section>\n"
"  <div class=\"info\"><span id=\"availabilityText\"> </span><span>Vehicle CAN Diagnostic tool</span></div>\n"
"</main>\n"
"<script>\n"
"const categories = [\n"
"  {id:'vehicle', label:'Vehicle', cls:'vehicle', desc:'Vehicle identity, ride mode, switches, locks, odometer and range.', icon:'<circle cx=\"6.5\" cy=\"17.5\" r=\"3\"/><circle cx=\"18.5\" cy=\"17.5\" r=\"3\"/><path d=\"M6.5 17.5h4l3-6h3l2 6M11 11.5l-2-4H6M13.5 11.5l-1.5-3M15.5 7.5h2.5M8.5 13.5l-2 4\"/> '},\n"
"  {id:'cells', label:'Cell Details', cls:'cells', desc:'Individual cell voltage, state of charge, state of health and voltage imbalance.', icon:'<rect x=\"5\" y=\"5\" width=\"14\" height=\"16\" rx=\"2\"/><path d=\"M9 3v2m6-2v2m-4 6l-2 3h3l-1 3 4-5h-3l1-2\"/> '},\n"
"  {id:'bms', label:'BMS', cls:'bms', desc:'Pack-level SoC, SoH, current, balancing, protection limits and BMS diagnostics.', icon:'<path d=\"M12 3l8 3v5c0 5-3.5 8-8 10-4.5-2-8-5-8-10V6z\"/><path d=\"M9 12l2 2 4-4\"/> '},\n"
"  {id:'motor', label:'Motor & Controller', cls:'motor', desc:'Motor current and RPM, throttle signals, controller telemetry and temperatures.', icon:'<path d=\"M12 3v3m0 12v3M3 12h3m12 0h3\"/><circle cx=\"12\" cy=\"12\" r=\"8\"/><path d=\"M12 12l4-4\"/> '},\n"
"  {id:'charger', label:'Charger', cls:'charger', desc:'Charger voltage, current, calculated power, fans, connection and charge limit.', icon:'<path d=\"M13 2L5 13h6l-1 9 9-12h-6z\"/> '},\n"
"  {id:'firmware', label:'Firmware & Software', cls:'firmware', desc:'BMS, BCM, gateway and motor firmware versions, diagnostics and scanner health.', icon:'<rect x=\"6\" y=\"6\" width=\"12\" height=\"12\" rx=\"2\"/><path d=\"M9 2v4m6-4v4M9 18v4m6-4v4M2 9h4m-4 6h4m12-6h4m-4 6h4M10 10h4v4h-4z\"/> '},\n"
"  {id:'thermals', label:'Thermals', cls:'thermals', desc:'Cell, BMS, motor and controller temperatures, humidity and thermal limits.', icon:'<path d=\"M10 14.5V5a2 2 0 0 1 4 0v9.5a5 5 0 1 1-4 0z\"/><path d=\"M12 11v7\"/> '},\n"
"  {id:'about', label:'About', cls:'others', desc:'Project information, contributors, repository, suggestions, and ways to contribute.', icon:'<circle cx=\"12\" cy=\"12\" r=\"9\"/><path d=\"M12 11v5\"/><circle cx=\"12\" cy=\"7.5\" r=\".7\" fill=\"currentColor\" stroke=\"none\"/> '}\n"
"];\n"
"\n"
"const telemetryGroups = {\n"
"  vehicle: [\n"
"    {title:'Vehicle identification & ride data', symbol:'🛵', accent:'#4778d0', fields:[\n"
"      ['Vehicle identification number','🪪','—',''],['Scooter ID','🔖','—',''],['Odometer','🛣️','—','km'],['Estimated range','🧭','—','km','estimated'],['Drive mode','⚙️','—',''],['Peak power','⚡','—','kW']\n"
"    ]},\n"
"    {title:'Switches & rider controls', symbol:'🎛️', accent:'#4778d0', fields:[\n"
"      ['Key switch','🔑','—',''],['Start switch','▶️','—',''],['Kill switch','⏹️','—',''],['Storage switch','🧰','—',''],['Parking assistant','🅿️','—',''],['Nav joystick','🕹️','—','','hypothesis'],['Front brake','🖐️','—',''],['Rear brake','🖐️','—',''],['High beam','💡','—',''],['Horn','📣','—',''],['Indicators','↔️','—',''],['Blinkers','🚦','—',''],['Side stand','🦶','—',''],['Seat lock','🔒','—','']\n"
"    ]},\n"
"    {title:'Vehicle IMU Details', symbol:'📐', accent:'#4778d0', fields:["
"      ['IMU acceleration X','📐','—','m/s²'],['IMU acceleration Y','📐','—','m/s²'],['IMU acceleration Z','📐','—','m/s²']"
"    ]},"
"    {title:'Vehicle Orientation', symbol:'🧭', accent:'#4778d0', fields:["
"      ['Pitch','↕️','—','°'],['Roll','↪️','—','°'],['Heading','🧭','—','°']"
"    ]},"
"  ],"
"  cells: [\n"
"    {title:'Pack & cell summary', symbol:'🔋', accent:'#23945d', fields:[\n"
"      ['Pack state of charge','🔋','—','%'],['Minimum-voltage estimate','📉','—','V','estimated'],['Cell voltage imbalance','⚖️','—','mV','estimated'],['Cell temperature Probe 1','🌡️','—','°C'],['Cell temperature Probe 2','🌡️','—','°C'],['Cell temperature Probe 3','🌡️','—','°C'],['Cell temperature Probe 4','🌡️','—','°C'],['Cell temperature Probe 5','🌡️','—','°C','Unused in NMC'],['Delta SoC','↔️','—','%']"
"    ]}\n"
"  ],\n"
"  bms: ["
"    {title:'Battery pack health', symbol:'🔋', accent:'#8b5cc7', fields:["
"      ['Pack state of charge','🔋','—','%'],['Pack state of health','🩺','—','%'],['Delta SoC','↔️','—','%'],['Pack voltage','⚡','—','V'],['Pack current','〰️','—','A'],['Balancing state','⚖️','—',''],['State of Energy','🔌','—','%'],['EKF SoC estimate','🧮','—','%'],['Minimum SoC','📉','—','%'],['Maximum SoC','📈','—','%']"
"    ]},"
"    {title:'Current limits & charging', symbol:'🔌', accent:'#8b5cc7', fields:["
"      ['Derated discharge current','⬇️','—','A'],['Derated charge current','⬆️','—','A'],['Derated regenerative current','♻️','—','A'],['Temperature-based discharge limit','🌡️','—','A']"
"    ]},"
"    {title:'BMS diagnostics & identity', symbol:'🛡️', accent:'#8b5cc7', fields:["
"      ['AFE die temperature','🌡️','—','°C'],['BMS uptime','⏱️','—',''],['BMS hardware version','🧩','—',''],['Battery identification','🔖','—','']"
"    ]},"
"    {title:'Charging Algorithm Limit', symbol:'⚖️', accent:'#8b5cc7', fields:["
"      ['Cell rule charge limit','⚖️','—','A'],['Rate-limited charge current','⚖️','—','A'],['Post-quantizer charge value','⚖️','—','A']"
"    ]},"
"    {title:'Internal Reference Voltages', symbol:'🔌', accent:'#8b5cc7', fields:["
"      ['Board temperature reference voltage','🔌','—','V'],['VREF2','🔌','—','V'],['VREF3','🔌','—','V'],['MOSFET power voltage','🔌','—','V'],['AFE regulator voltage','🔌','—','V']"
"    ]},"
"  ],"
"  motor: [\n"
"    {title:'Motor & controller telemetry', symbol:'⚙️', accent:'#c56a30', fields:[\n"
"      ['Motor current','〰️','—','A'],['Motor temperature','🌡️','—','°C'],['Controller temperature','🌡️','—','°C'],['Motor MCU temperature','🧠','—','°C'],['Throttle','🎚️','—','',''],['Traction-control RPM','🔄','—','RPM','hypothesis'],['Motor-control RPM','🔄','—','RPM','estimated'],['Motor firmware','💾','—','']\n"
"    ]}\n"
"  ],\n"
"  charger: [\n"
"    {title:'Charging & power delivery', symbol:'⚡', accent:'#b38a16', fields:[\n"
"      ['Charger input voltage (AC)','🔌','—','V'],['Charger output voltage (DC)','⚡','—','V'],['Charger output current','〰️','—','A'],['Calculated power','📊','—','W','estimated'],['Charger connected','🔗','—',''],['Charger fan speeds','🌀','—','RPM'],['Charge SoC limit','🔋','—','%'],['PDU voltage','⚡','—','V']\n"
"    ]},\n"
"    {title:'6 kW grid support', symbol:'⛽', accent:'#b38a16', fields:[\n"
"      ['Grid support capability','⚡','—',''],['Support level','📊','—','']\n"
"    ], note:'Determine By BMS'}\n"
"  ],\n"
"  firmware: [\n"
"    {title:'Vehicle firmware & hardware', symbol:'💾', accent:'#2383ad', fields:[\n"
"      ['BMS firmware','🔋','—',''],['BCM firmware','🧠','—',''],['Gateway firmware','🌐','—',''],['Motor firmware','⚙️','—',''],['BMS hardware version','🧩','—','']\n"
"    ]},\n"
"    {title:'Diagnostic tool & CAN health', symbol:'🩺', accent:'#2383ad', fields:[\n"
"      ['Diagnostic tool activity','🩺','—',''],['CAN RX missed frames','📥','—','count'],['CAN RX overrun count','⚠️','—','count'],['Arbitration lost count','🔀','—','count'],['Maximum queue backlog','📚','—','count'],['Free heap','🧠','—','bytes'],['Wi-Fi AP status','📶','—','']\n"
"    ]}\n"
"  ],\n"
"  thermals: [\n"
"    {title:'Battery & BMS temperatures', symbol:'🌡️', accent:'#c64b68', fields:[\n"
"      ['Cell temperature Probe 1','🌡️','—','°C'],['Cell temperature Probe 2','🌡️','—','°C'],['Cell temperature Probe 3','🌡️','—','°C'],['Cell temperature Probe 4','🌡️','—','°C'],['Cell temperature Probe 5','🌡️','—','°C','Unused in NMC'],['AFE die temperature','🧠','—','°C'],['BMS ambient temperature','🌡️','—','°C'],['Humidity','💧','—','%'],['FET temperature','🔥','—','°C','hypothesis'],['ECI temperature','🔥','—','°C','hypothesis'],['Sense-resistor temperature','🌡️','—','°C','hypothesis']\n"
"    ]},\n"
"    {title:'Drive unit & thermal limits', symbol:'🛞', accent:'#c64b68', fields:[\n"
"      ['Motor temperature','⚙️','—','°C'],['Controller temperature','🧩','—','°C'],['Motor MCU temperature','🧠','—','°C'],['Temperature-related discharge restriction','⚠️','—','']\n"
"    ]}\n"
"  ],\n"
"  about: []\n"
"};\n"
"\n"
"function renderTelemetry(cat) {\n"
"  const root = document.getElementById('telemetryContent');\n"
"  root.innerHTML = '';\n"
"  const groups = telemetryGroups[cat.id] || [];\n"
"  if (!groups.length) {\n"
"    root.innerHTML = `<div class=\"empty-state\"><div class=\"empty-icon\">ℹ️</div><strong>Vehicle CAN Diagnostic Tool</strong><p>VCD Tool is an ESP32-C3-based Ather vehicle CAN diagnostic dashboard. It presents available vehicle, battery, motor/controller, charger, firmware, and thermal telemetry from the connected vehicle. Displayed values depend on the signals decoded by the firmware.</p><strong>Contributors</strong><p> SAM0-0 — A-OBD Project.</p><p> Kukun7610 — VCD-Tool Project.</p><strong>GitHub Project Repository</strong><p><a href=\"https://github.com/Kukun7610/Vehicle_CAN_Diagnostic_tool\" class=\"about-placeholder-link\">Repository link to be added</a></p><strong>Suggestion Link</strong><p><a href=\"https://github.com/Kukun7610/Vehicle_CAN_Diagnostic_tool/issues/new/choose\" class=\"about-placeholder-link\">Suggestion form link to be added</a></p><strong>Contribution Link</strong><p><a href=\"https://github.com/Kukun7610/Vehicle_CAN_Diagnostic_tool/blob/main/.github/PULL_REQUEST_TEMPLATE.md\" class=\"about-placeholder-link\">Contribution guide link to be added</a></p></div>`;\n"
"    return;\n"
"  }\n"
"  groups.forEach(group => {\n"
"    const section = document.createElement('section');\n"
"    section.className = 'telemetry-group';\n"
"    const heading = document.createElement('h3');\n"
"    heading.className = 'group-title';\n"
"    heading.innerHTML = `<span class=\"group-symbol\" style=\"background:${group.accent}18\">${group.symbol}</span>${group.title}`;\n"
"    const grid = document.createElement('div');\n"
"    grid.className = 'telemetry-grid';\n"
"    group.fields.forEach(field => {\n"
"      const [label, ico, value, unit, tag] = field;\n"
"      const item = document.createElement('div');\n"
"      item.className = 'telemetry-item';\n"
"      item.style.setProperty('--metric-accent', group.accent);\n"
"      const tagMarkup = tag ? `<span class=\"field-tag ${tag}\">${tag === 'estimated' ? 'Estimated' : 'Hypothesis / verify'}</span>` : '';\n"
"      item.innerHTML = `<div class=\"telemetry-label\"><span class=\"mini-icon\">${ico}</span><span>${label}</span></div><strong class=\"telemetry-value\" data-label=\"${label.replace(/\"/g, \"&quot;\")}\">${value}${unit ? `<span class=\"telemetry-unit\">${unit}</span>` : ''}</strong>${tagMarkup}`;\n"
"      grid.appendChild(item);\n"
"    });\n"
"    section.append(heading, grid);\n"
"    if (group.note) {\n"
"      const note = document.createElement('p');\n"
"      note.className = 'telemetry-intro';\n"
"      note.textContent = group.note;\n"
"      section.appendChild(note);\n"
"    }\n"
"    if (cat.id === 'cells' && group.title === 'Pack & cell summary') {\n"
"      const tableSection = document.createElement('section');\n"
"      tableSection.className = 'telemetry-group';\n"
"      tableSection.innerHTML = `<h3 class=\"group-title\"><span class=\"group-symbol\" style=\"background:#23945d18\">🧪</span>Individual cell details From FC1–C14</h3>\n"
"        <div class=\"cell-table-wrap\"><table class=\"cell-table\"><thead><tr><th>Cell</th><th>Voltage</th><th>SoC</th><th>SoH</th></tr></thead>\n"
"        <tbody>${Array.from({length:14},(_,i)=>`<tr><td class=\"cell-name\">Cell ${i+1}</td><td>— V</td><td>— %</td><td>— %</td></tr>`).join('')}</tbody></table></div>`;\n"
"      root.appendChild(section);\n"
"      root.appendChild(tableSection);\n"
"      return;\n"
"    }\n"
"    root.appendChild(section);\n"
"  });\n"
"  if (cat.id === 'about') {\n"
"    root.innerHTML = `<div class=\"about-content\">\n"
"      <div class=\"about-hero\"><div class=\"about-mark\">🛵</div><h3>About this project</h3>\n"
"        <p>An ESP32-C3-based Ather vehicle CAN diagnostic dashboard that brings decoded vehicle, battery, motor and controller, charger, firmware, and thermal telemetry together in one responsive interface. It is intended to make live data easier to inspect during testing and diagnostics. Displayed values depend on the CAN frames and signals supported by the connected firmware.</p></div>\n"
"      <div class=\"about-section\"><h3>Contributors</h3>\n"
"        <div class=\"about-link-row\"><span class=\"about-link-icon\">👥</span><div><strong>Project contributors</strong><small>Credits for development, CAN research, testing, documentation, and community feedback.</small></div><span class=\"about-link-placeholder\">Names pending</span></div></div>\n"
"      <div class=\"about-section\"><h3>Project links</h3>\n"
"        <div class=\"about-link-row\"><span class=\"about-link-icon\">🐙</span><div><strong>GitHub Project Repository</strong><small>Source code, documentation, and issue tracking.</small></div><span class=\"about-link-placeholder\">Link pending</span></div>\n"
"        <div class=\"about-link-row\"><span class=\"about-link-icon\">💡</span><div><strong>Suggestion Link</strong><small>Share ideas and request new features.</small></div><span class=\"about-link-placeholder\">Link pending</span></div>\n"
"        <div class=\"about-link-row\"><span class=\"about-link-icon\">🤝</span><div><strong>Contribution Link</strong><small>Find out how to contribute to the project.</small></div><span class=\"about-link-placeholder\">Link pending</span></div>\n"
"      </div></div>`;\n"
"  }\n"
"}\n"
"\n"
"\n"
"// Live integration reads only JSON endpoints implemented in the supplied .ino.\n"
"const liveData={data:null,cells:null,charger:null,extra:null};\n"
"const CAN_OFFLINE_TIMEOUT_MS=3000;\n"
"const CAN_DEGRADED_TIMEOUT_MS=1200;\n"
"function metric(label,value){document.querySelectorAll('.telemetry-value[data-label]').forEach(el=>{if(el.dataset.label===label)el.textContent=value===undefined||value===null||value===''?'—':String(value);});}\n"
"const f=(v,n=0)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'—';\n"
"const V=(v,n=2)=>Number.isFinite(Number(v))?f(v,n)+' V':'—';\n"
"const P=v=>Number.isFinite(Number(v))?f(Number(v)/100,2)+' %':'—';\n"
"const T=v=>(v!==undefined&&v!==null&&v!==''&&Number.isFinite(Number(v))&&Number(v)!==0&&Number(v)!==-32768)?f(Number(v)/100,2)+' °C':'—';\n"
"const A=v=>Number.isFinite(Number(v))?f(Number(v)/100,2)+' A':'—';\n"
"const sw=(v,on='Active',off='Inactive')=>v===true||v===1?on:off;\n"
"const ver=v=>Array.isArray(v)?v.join('.'):'—';\n"
"function up(v){v=Number(v)||0;return Math.floor(v/86400)+'d '+Math.floor(v%86400/3600)+'h '+Math.floor(v%3600/60)+'m';}\n"
"// ===== fields missing from this build, added back in =====\n"
"function ageTxt(ms){ ms=Number(ms); if(!Number.isFinite(ms)||ms<0) return 'never seen';\n"
"  return ms<60000 ? Math.floor(ms/1000)+'s ago' : Math.floor(ms/60000)+'m ago'; }\n"
"function parkAssistTxt(e){\n"
"  const aSeen=Number.isFinite(Number(e.pkaage))&&Number(e.pkaage)>=0;\n"
"  const bSeen=Number.isFinite(Number(e.pkbage))&&Number(e.pkbage)>=0;\n"
"  if(!aSeen&&!bSeen) return 'Not triggered';\n"
"  const parts=[];\n"
"  if(aSeen) parts.push('A triggered '+ageTxt(e.pkaage));\n"
"  if(bSeen) parts.push('B triggered '+ageTxt(e.pkbage));\n"
"  return parts.join(' \\u00b7 ');\n"
"}\n"
"function gridSupportTxt(d,e){\n"
"  const soc=Number(d.soc), dc=Number(e.dc);\n"
"  if(!Number.isFinite(soc)||!Number.isFinite(dc)) return undefined;\n"
"  if(soc>80) return 'Discharge below 80% to calculate';\n"
"  if(dc>6000) return 'Yes, Full Support';\n"
"  return 'Partial';\n"
"}\n"

"function updateLiveMetrics(){\n"
" const d=liveData.data||{},c=liveData.cells||{},ch=liveData.charger||{},e=liveData.extra||{};\n"
" const values={\n"
" 'Vehicle identification number':e.vin?e.vin+' ('+ageTxt(e.vinage)+')':undefined,'Scooter ID':e.sid?e.sid+' ('+ageTxt(e.sidage)+')':undefined,'Odometer':e.odo!==undefined?f(e.odo/1000,1)+' km':undefined,\n"
" 'Estimated range':d.range!==undefined?f(d.range)+' km':undefined,'Drive mode':d.mode,'Peak power':'Not exposed by this firmware',\n"
" 'Key switch':sw(d.key,'Enabled','Disabled'),'Start switch':sw(d.start,'Pressed','Not pressed'),\n"
" 'Kill switch':sw(d.kill),'Storage switch':sw(d.storage),'Parking assistant':parkAssistTxt(e),'Nav joystick':e.nav!==undefined?('Code '+e.nav+' (direction unconfirmed)'):undefined,\n"
" 'Front brake':sw(d.fbrake,'Pressed','Not pressed'),'Rear brake':sw(d.rbrake,'Pressed','Not pressed'),\n"
" 'High beam':sw(d.hbeam,'On','Off'),'Horn':sw(d.horn),\n"
" 'Indicators':(d.indl||d.indr||d.indc)?'Active':'Inactive','Blinkers':(d.indl||d.indr||d.indc)?'Active':'Inactive',\n"
" 'Side stand':sw(d.stand,'Down','Up'),'Seat lock':sw(ch.seat,'Locked','Unlocked'),\n"
" 'IMU acceleration X':e.ax,'IMU acceleration Y':e.ay,'IMU acceleration Z':e.az,\n"
" 'Roll':e.er!==undefined?f(e.er/10000,4)+' °':undefined,'Pitch':e.ep!==undefined?f(e.ep/10000,4)+' °':undefined,\n"
" 'Heading':e.eh!==undefined?f(e.eh/10000,4)+' °':undefined,\n"
" 'Pack state of charge':d.soc!==undefined?f(d.soc,2)+' %':undefined,\n"
" 'Pack state of health':d.soh!==undefined?f(d.soh,2)+' %':undefined,\n"
" 'Delta SoC':d.bdelta!==undefined?f(d.bdelta,2)+' %':undefined,\n"
" 'Minimum-voltage estimate':e.v1!==undefined?V(e.v1/10000,4):undefined,\n"
" 'Cell voltage imbalance':d.imb!==undefined?V(d.imb,4):undefined,\n"
" 'Cell temperature readings':Array.isArray(e.ct)?e.ct.slice(0,4).map(T).join(' · '):undefined,\n"
" 'Cell temperatures':Array.isArray(e.ct)?e.ct.slice(0,4).map(T).join(' · '):undefined,\n"
" 'Cell temperature Probe 1':Array.isArray(e.ct)?T(e.ct[0]):undefined,\n"
" 'Cell temperature Probe 2':Array.isArray(e.ct)?T(e.ct[1]):undefined,\n"
" 'Cell temperature Probe 3':Array.isArray(e.ct)?T(e.ct[2]):undefined,\n"
" 'Cell temperature Probe 4':Array.isArray(e.ct)?T(e.ct[3]):undefined,\n"
" 'Cell temperature Probe 5':Array.isArray(e.ct)?T(e.ct[4]):undefined,\n"
" 'Pack voltage':d.volt!==undefined?V(d.volt,2):undefined,'Pack current':Number.isFinite(Number(e.ci))?f(Number(e.ci)/100,2)+' A':undefined,\n"
" 'Balancing state':ch.bal!==undefined?(Number(ch.bal)?'Balancing':'Not balancing'):undefined,\n"
" 'State of Energy':P(e.soe),'EKF SoC estimate':P(e.sekf),'Maximum SoC estimate':P(e.msoc),'Minimum SoC estimate':P(e.minsoc),\n"
" 'Derated discharge current':Number.isFinite(Number(e.dd))?f(Number(e.dd)/100,2)+' A':undefined,'Derated charge current':Number.isFinite(Number(e.dc))?f(Number(e.dc)/100,2)+' A':undefined,'Derated regenerative current':Number.isFinite(Number(e.dr))?f(Number(e.dr)/100,2)+' A':undefined,\n"
" 'Temperature-based discharge limit':Number.isFinite(Number(e.dl))?f(Number(e.dl)/100,2)+' A':undefined,\n"
" 'Charge algorithm limits':e.cr!==undefined?`Cell rule ${f(e.cr/100,2)} Â· rate limited ${f(e.crl/100,2)} Â· post-quantizer ${f(e.cpq/100,2)}`:undefined,\n"
" 'AFE die temperature':T(e.die),'BMS uptime':e.up!==undefined?up(e.up):undefined,\n"
" 'BMS hardware version':e.bhw!==undefined?'v'+e.bhw:undefined,'Battery identification':e.bin,\n"
" 'Board temperature reference voltage':e.btr!==undefined?V(e.btr/1000,3):undefined,\n"
" 'VREF2':e.vr2!==undefined?V(e.vr2/10000,4):undefined,\n"
" 'VREF3':e.vr3!==undefined?V(e.vr3/10000,4):undefined,\n"
" 'MOSFET power voltage':e.mos!==undefined?V(e.mos/1000,3):undefined,\n"
" 'AFE regulator voltage':e.vreg!==undefined?V(e.vreg/1000,3):undefined,\n"
" 'Internal reference voltages':e.btr!==undefined?`Board ${V(e.btr/1000,3)} · VREF2 ${V(e.vr2/10000,4)} · VREF3 ${V(e.vr3/10000,4)} · MOS ${V(e.mos/1000,3)} · AFE ${V(e.vreg/1000,3)}`:undefined,\n"
" 'Motor current':ch.mbi!==undefined?f(ch.mbi,1)+' A':undefined,'Motor temperature':ch.mt!==undefined?f(ch.mt,1)+' °C':undefined,\n"
" 'Controller temperature':e.ctemp!==undefined?f(e.ctemp)+' °C':undefined,'Motor MCU temperature':e.mcutemp!==undefined?f(e.mcutemp)+' °C':undefined,\n"
" 'Throttle':e.tho!==undefined?f(e.tho)+' %':undefined,\n"
" 'Traction-control RPM':e.tcrpm,'Motor-control RPM':e.mcrpm,'Motor firmware':ver(e.mfw),\n"
" 'Charger input voltage (AC)':ch.cinv!==undefined?V(ch.cinv,3):undefined,\n"
" 'Charger output voltage (DC)':ch.coutv!==undefined?V(ch.coutv,3):undefined,\n"
" 'Charger output current':ch.couti!==undefined?f(ch.couti,3)+' A':undefined,\n"
" 'Calculated power':ch.watts!==undefined?(Number(ch.watts)>=1000?f(Number(ch.watts)/1000,2)+' kW':f(ch.watts,1)+' W'):undefined,\n"
" 'Charger connected':ch.cconnected!==undefined?sw(ch.cconnected,'Connected','Disconnected'):undefined,\n"
" 'Charger fan speeds':ch.fan1!==undefined?`${ch.fan1} / ${ch.fan2} RPM`:undefined,\n"
" 'Charge SoC limit':e.chglim!==undefined?f(e.chglim/100,1)+' %':undefined,\n"
" 'PDU voltage':e.pdu!==undefined?V(e.pdu/1000,2):undefined,\n"
" 'Grid support capability':gridSupportTxt(d,e),\n"
" 'Support level':e.dc!==undefined?(Number(e.dc)>6000?'Full support · 6 kW':'Partial support · 3.3 kW'):'—',\n"
" 'BMS firmware':ver(e.bfw),'BCM firmware':ver(e.cfw),'Gateway firmware':e.gfw?ver(e.gfw)+' ('+ageTxt(e.gtwage)+')':'0.0.0 (never seen - this is why it still reads 0.0.0)',\n"
" 'Diagnostic tool activity':e.diagage!==undefined?(e.diagage<0?'Not seen':e.diagage<120000?'Active '+Math.floor(e.diagage/1000)+' s ago':'Not seen recently'):undefined,\n"
" 'Wi-Fi AP status':'Configured as VCD-Tool; runtime status not exposed by API',\n"
" 'FET temperature':T(e.eft),'ECI temperature':T(e.eci),'Sense-resistor temperature':T(e.sr),\n"
" 'BMS ambient temperature':T(e.amb),'Humidity':P(e.hum)\n"
" };\n"
" Object.entries(values).forEach(([k,v])=>metric(k,v));\n"
" for(let i=0;i<14;i++){\n"
"  const row=document.querySelectorAll('.cell-table tbody tr')[i]; if(!row)continue;\n"
"  if(c.v&&c.v[i]!==undefined)row.cells[1].textContent=f(c.v[i],4)+' V';\n"
"  if(e.cs&&e.cs[i]!==undefined)row.cells[2].textContent=P(e.cs[i]);\n"
"  if(c.h&&c.h[i]!==undefined)row.cells[3].textContent=f(c.h[i])+' %';\n"
" }\n"
" document.getElementById('dataSourceNote').textContent='Live values are pulled directly from the vehicle CAN bus using the provided firmware endpoints.';\n"
"}\n"
"async function pollFirmwareApi(){\n"
" if(location.protocol==='file:'){\n"
"  document.getElementById('dataConnection').className='data-connection disconnected';\n"
"  document.getElementById('connectionTitle').textContent='This local file is not connected to the ESP32';\n"
"  document.getElementById('connectionMessage').textContent='Open the dashboard from the VCD-Tool Wi-Fi access point. The interface remains offline until live CAN traffic is confirmed.';\n"
"  updateStatus('offline');\n"
"  return;\n"
" }\n"
" try{\n"
"  const paths=[['data','/data'],['cells','/cells'],['charger','/charger'],['extra','/extra']];\n"
"  const result=await Promise.all(paths.map(async([key,url])=>{const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error(url+' HTTP '+r.status);return[key,await r.json()];}));\n"
"  result.forEach(([key,value])=>liveData[key]=value);\n"
"  const d=liveData.data||{};\n"
"  const age=Number(d.canAgeMs);\n"
"  const isFresh=d.canOnline===true && Number.isFinite(age) && age<CAN_OFFLINE_TIMEOUT_MS;\n"
"  updateStatus(!isFresh?'offline':age>=CAN_DEGRADED_TIMEOUT_MS?'amber':'online');\n"
"  if(isFresh){\n"
"   updateLiveMetrics();\n"
"   document.getElementById('dataConnection').className='data-connection connected';\n"
"   document.getElementById('connectionTitle').textContent=age>=CAN_DEGRADED_TIMEOUT_MS?'CAN data intermittent':'CAN data connected';\n"
"   document.getElementById('connectionMessage').textContent='Last CAN frame '+(age<1000?age+' ms ago':(age/1000).toFixed(1)+' s ago')+'.';\n"
"  } else {\n"
"   document.getElementById('dataConnection').className='data-connection disconnected';\n"
"   document.getElementById('connectionTitle').textContent='CAN OFFLINE';\n"
"   document.getElementById('connectionMessage').textContent='No CAN frame has been received recently. Values may be stale; all diagnostic categories are disabled until CAN traffic resumes.';\n"
"  }\n"
" }catch(err){\n"
"  updateStatus('offline');\n"
"  document.getElementById('dataConnection').className='data-connection disconnected';\n"
"  document.getElementById('connectionTitle').textContent='ESP32 telemetry API unavailable';\n"
"  document.getElementById('connectionMessage').textContent='Cannot read telemetry endpoints. All diagnostic categories remain disabled until the ESP32 API and CAN traffic are available.';\n"
" }\n"
"}\n"
"\n"
"let mode = 'offline';\n"
"let selected = null;\n"
"const tiles = document.getElementById('tiles');\n"
"const menu = document.getElementById('menu');\n"
"const statusBar = document.getElementById('statusBar');\n"
"const statusText = document.getElementById('statusText');\n"
"const availabilityText = document.getElementById('availabilityText');\n"
"\n"
"function iconMarkup(cat) { return `<svg viewBox=\"0 0 24 24\" aria-hidden=\"true\">${cat.icon}</svg>`; }\n"
"function renderCategories() {\n"
"  tiles.innerHTML = '';\n"
"  menu.innerHTML = '';\n"
"  categories.forEach(cat => {\n"
"    const b = document.createElement('button');\n"
"    b.className = 'tile'; b.dataset.category = cat.id;\n"
"    b.innerHTML = `<span class=\"tile-icon ${cat.cls}\">${iconMarkup(cat)}</span><span class=\"tile-label\">${cat.label}</span>`;\n"
"    b.addEventListener('click', () => openCategory(cat.id));\n"
"    tiles.appendChild(b);\n"
"    const mb = document.createElement('button');\n"
"    mb.textContent = cat.label; mb.dataset.category = cat.id;\n"
"    mb.addEventListener('click', () => { openCategory(cat.id); menu.classList.remove('open'); document.getElementById('menuToggle').setAttribute('aria-expanded','false'); });\n"
"    menu.appendChild(mb);\n"
"  });\n"
"  updateStatus(mode);\n"
"}\n"
"function updateStatus(nextMode) {\n"
"  mode = nextMode;\n"
"  statusBar.className = 'status ' + mode + (selected ? ' submenu-mode' : '');\n"
"  const scooterArea = document.querySelector('.scooter-area');\n"
"  if (scooterArea) scooterArea.className = 'scooter-area ' + mode;\n"
"  statusText.textContent = mode === 'online' ? 'ONLINE' : mode === 'amber' ? 'CAN ACTIVITY INTERMITTENT' : 'OFFLINE';\n"
"  const enabled = mode !== 'offline';\n"
"  document.querySelectorAll('.tile, .menu button').forEach(btn => btn.disabled = !enabled);\n"
"  document.getElementById('menuToggle').disabled = !enabled;\n"
"  document.getElementById('backButton').disabled = !enabled;\n"
"  document.querySelectorAll('.control').forEach(btn => btn.classList.toggle('active', btn.dataset.mode === mode));\n"
"  availabilityText.textContent = mode === 'online'\n"
"    ? 'CAN data is being received frequently. Categories are enabled.'\n"
"    : mode === 'amber'\n"
"      ? 'CAN frames are still arriving, but intermittently. Categories remain available.'\n"
"      : 'No CAN data is being received. Categories are disabled until data resumes.';\n"
"  document.getElementById('detailLive').textContent = mode === 'offline' ? 'OFFLINE' : mode === 'amber' ? 'DEGRADED' : 'LIVE';\n"
"  document.getElementById('detailLive').style.color = mode === 'offline' ? '#ff6572' : mode === 'amber' ? '#f0bd45' : '#18c96b';\n"
"  if (mode === 'offline' && selected) closeCategory();\n"
"}\n"
"function openCategory(id) {\n"
"  if (mode === 'offline') return;\n"
"  const cat = categories.find(c => c.id === id);\n"
"  if (!cat) return;\n"
"  selected = id;\n"
"  statusBar.classList.add('submenu-mode');\n"
"  document.getElementById('homeView').style.display = 'none';\n"
"  document.getElementById('detailView').style.display = 'block';\n"
"  document.getElementById('detailTitle').textContent = cat.label;\n"
"  document.getElementById('detailIcon').className = 'tile-icon ' + cat.cls;\n"
"  document.getElementById('detailIcon').innerHTML = iconMarkup(cat);\n"
"  document.getElementById('detailDescription').textContent = cat.desc;\n"
"  renderTelemetry(cat);\n"
"  window.scrollTo({top:0,behavior:'smooth'});\n"
"}\n"
"function closeCategory() {\n"
"  selected = null;\n"
"  statusBar.classList.remove('submenu-mode');\n"
"  document.getElementById('homeView').style.display = 'grid';\n"
"  document.getElementById('detailView').style.display = 'none';\n"
"}\n"
"document.getElementById('backButton').addEventListener('click', closeCategory);\n"
"document.getElementById('menuToggle').addEventListener('click', () => {\n"
"  if (mode === 'offline') return;\n"
"  const isOpen = menu.classList.toggle('open');\n"
"  document.getElementById('menuToggle').setAttribute('aria-expanded', String(isOpen));\n"
"});\n"
"document.querySelectorAll('.control').forEach(btn => { btn.disabled = true; });\n"
"renderCategories();\n"
"pollFirmwareApi();\n"
"setInterval(pollFirmwareApi,1500);\n"
"</script>\n"
"</body>\n"
"</html>\n"
"\n"
;


void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", htmlPage); }
char jsonBuf[512];
void handleData(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  int n=snprintf(jsonBuf,sizeof(jsonBuf),
    "{\"soc\":%d.%02d,\"soh\":%d.%02d,\"bdelta\":%d.%02d,\"volt\":%d.%02d,\"imb\":%d.%04d,\"rpm\":%d,\"mode\":\"%s\",\"range\":%d,\"fbrake\":%s,\"rbrake\":%s,\"hbeam\":%s,\"start\":%s,\"kill\":%s,\"storage\":%s,\"horn\":%s,\"indl\":%s,\"indr\":%s,\"indc\":%s,\"key\":%s,\"stand\":%s,\"dfan\":%s,\"feco\":%s,\"seco\":%s,\"canAgeMs\":%lu,\"canOnline\":%s}",
    t.soc/100,abs(t.soc%100), t.soh/100,abs(t.soh%100), t.deltaSoc/100,abs(t.deltaSoc%100),
    t.voltage/100,abs(t.voltage%100), t.imbalance/10000,abs(t.imbalance%10000),
    t.rpm,driveModeName(t.driveMode),t.rangeKm,
    t.frontBrake?"true":"false",t.rearBrake?"true":"false",t.highBeam?"true":"false",t.startSwitch?"true":"false",
    t.killSwitch?"true":"false",t.storageSwitch?"true":"false",t.horn?"true":"false",
    t.indicatorLeftSide?"true":"false",t.indicatorRightSide?"true":"false",t.indicatorCenterSide?"true":"false",t.keyOn?"true":"false",t.sideStand?"true":"false",
    t.displayFan?"true":"false",t.forcedEco?"true":"false",t.smartEco?"true":"false",
    (unsigned long)(lastCanRxMs ? millis()-lastCanRxMs : 0xFFFFFFFFUL),
    (lastCanRxMs && (uint32_t)(millis()-lastCanRxMs)<CAN_OFFLINE_TIMEOUT_MS)?"true":"false");
  if(n<0||n>=(int)sizeof(jsonBuf)){server.send(500,"application/json","{}");return;}
  server.send(200,"application/json",jsonBuf);
}

char cellBuf[512];
void handleCells(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  char *p=cellBuf; int rem=sizeof(cellBuf);
  int n=snprintf(p,rem,"{\"v\":["); if(n<0)return; p+=n; rem-=n;
  for(int i=0;i<14;i++){ n=snprintf(p,rem,"%d.%04d%s",t.cellV[i]/10000,abs(t.cellV[i]%10000),i<13?",":""); if(n<0)return; p+=n; rem-=n; }
  n=snprintf(p,rem,"],\"h\":["); if(n<0)return; p+=n; rem-=n;
  for(int i=0;i<14;i++){ n=snprintf(p,rem,"%d%s",t.cellSoh[i],i<13?",":""); if(n<0)return; p+=n; rem-=n; }
  snprintf(p,rem,"]}");
  server.send(200,"application/json",cellBuf);
}

char chargerBuf[384];
void handleCharger(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  const int32_t motorI=t.motorI;
  const int32_t motorMag=motorI<0?-motorI:motorI;
  const char *motorSign=motorI<0?"-":"";
  const uint64_t powerProduct=(uint64_t)t.chargerOV*(uint64_t)t.chargerOI;
  const uint32_t wattsWhole=(uint32_t)(powerProduct/1000000ULL);
  const unsigned wattsFrac=(unsigned)((powerProduct%1000000ULL)/10000ULL);
  snprintf(chargerBuf,sizeof(chargerBuf),
    "{\"coutv\":%d.%03d,\"couti\":%d.%03d,\"cinv\":%d.%03d,\"mbi\":%s%d.%01d,\"mt\":%d,\"bal\":%d,\"watts\":%u.%02u,\"fan1\":%d,\"fan2\":%d,\"cconnected\":%s,\"seat\":%s,\"clock1\":%s,\"clock2\":%s}",
    (int)(t.chargerOV/1000),abs((int)(t.chargerOV%1000)),
    (int)(t.chargerOI/1000),abs((int)(t.chargerOI%1000)),
    (int)(t.chargerIV/1000),abs((int)(t.chargerIV%1000)),
    motorSign,(int)(motorMag/10),abs((int)(motorMag%10)),
    (int)t.motorTemp, t.balancing, wattsWhole, wattsFrac,
    t.chargerFan1, t.chargerFan2,
    t.chargerConnected?"true":"false",t.seatLock?"true":"false",t.chargerLock1?"true":"false",t.chargerLock2?"true":"false");
  server.send(200,"application/json",chargerBuf);
}

#define EXTRA_APPEND(...) do{ n=snprintf(p,rem,__VA_ARGS__); if(n<0||n>=rem){server.send(500,"application/json","{}");return;} p+=n; rem-=n; }while(0)
char extraBuf[1792];
void handleExtra(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  char bin[16]; int bi=0;
  for(int i=0;i<14;i++){ char c=t.binId[i]; if(!c)break; bin[bi++]=((c>='0'&&c<='9')||(c>='A'&&c<='Z')||(c>='a'&&c<='z'))?c:'?'; }
  bin[bi]=0;
  char *p=extraBuf; int rem=sizeof(extraBuf); int n;
  EXTRA_APPEND("{\"ct\":[%d,%d,%d,%d,%d],\"die\":%d,\"ci\":%d,\"hum\":%d,\"amb\":%d,\"eft\":%d,\"eci\":%d,\"sr\":%d,",
    t.cellTemp[0],t.cellTemp[1],t.cellTemp[2],t.cellTemp[3],t.cellTemp[4],
    t.afeDieTemp,t.packCurrent,t.humidity,t.bmsAmbient,t.eftTemp,t.eciTemp,t.senseTemp);
  EXTRA_APPEND("\"dd\":%u,\"dc\":%u,\"dr\":%u,\"dl\":%u,\"cr\":%u,\"crl\":%u,\"cpq\":%d,",
    t.deratedDsgI,t.deratedChgI,t.deratedRegenI,t.dsgLimTemp,t.chgCellRule,t.chgCmdRateLtd,t.chgPostQuant);
  EXTRA_APPEND("\"soe\":%u,\"sekf\":%u,\"msoc\":%u,\"minsoc\":%u,\"v1\":%d,\"v2\":%d,\"cs\":[",
    t.soe,t.socEkf,t.maxSoc,t.minSocEst,t.minV1Est,t.minV2Est);
  for(int i=0;i<14;i++){ EXTRA_APPEND("%u%s",t.cellSoc[i],i<13?",":""); }
  EXTRA_APPEND("],\"btr\":%u,\"vr2\":%u,\"vr3\":%u,\"mos\":%u,\"vreg\":%u,",
    t.boardTempRefV,t.vref2,t.vref3,t.mosfetPwrV,t.afeVreg);
  EXTRA_APPEND("\"ax\":%d,\"ay\":%d,\"az\":%d,\"er\":%d,\"ep\":%d,\"eh\":%d,",
    t.imuAccX,t.imuAccY,t.imuAccZ,t.eulerRoll,t.eulerPitch,t.eulerHeading);
  EXTRA_APPEND("\"odo\":%lu,\"up\":%lu,\"bhw\":%u,\"bfw\":[%u,%u,%u],\"cfw\":[%u,%u,%u],\"gfw\":[%u,%u,%u],\"mfw\":[%u,%u,%u],\"bin\":\"%s\",",
    (unsigned long)t.odometer,(unsigned long)(t.bmsUptimeMs/1000ULL),t.bmsHwVer,
    t.bmsFw[0],t.bmsFw[1],t.bmsFw[2],t.bcmFw[0],t.bcmFw[1],t.bcmFw[2],
    t.gtwFw[0],t.gtwFw[1],t.gtwFw[2],t.motorFw[0],t.motorFw[1],t.motorFw[2],bin);
  // ===== v4.9 additions =====
  { int32_t f=(int32_t)(t.chgSocLimit*100.0f);
    EXTRA_APPEND("\"pdu\":%u,\"chglim\":%d,\"ctemp\":%d,\"mcutemp\":%d,\"tho\":%d,\"th1\":%d,\"th3\":%d,\"tcrpm\":%d,\"mcrpm\":%d,\"diagage\":%ld,",
      t.pduVoltage,f,t.controllerTemp,t.motorMcuTemp,t.throttleOut,t.throttle1,t.throttle3,
      (int)t.tcRpm,(int)t.mcRpm, t.lastDiagMs? (long)(millis()-t.lastDiagMs) : -1L); }
  { char vin[21]; int vi=0;
    for(int i=0;i<17;i++){ char c=t.vin[i]; if(c<32||c>126) break; vin[vi++]=c; } vin[vi]=0;
    char sid[9]; int si=0;
    for(int i=0;i<8;i++){ char c=t.scooterId[i]; if(c<32||c>126) break; sid[si++]=c; } sid[si]=0;
    EXTRA_APPEND("\"vin\":\"%s\",\"vinage\":%ld,\"sid\":\"%s\",\"sidage\":%ld,\"gtwage\":%ld,",
      vin, t.lastVinMs?(long)(millis()-t.lastVinMs):-1L, sid, t.lastSidMs?(long)(millis()-t.lastSidMs):-1L,
      t.lastGtwMs?(long)(millis()-t.lastGtwMs):-1L); }
  // ===== fields missing from this build, added back in =====
  EXTRA_APPEND("\"nav\":%u,\"pka\":%u,\"pkaage\":%ld,\"pkb\":%u,\"pkbage\":%ld}",
    t.navJoystick, t.parkAssistA, t.lastParkAMs?(long)(millis()-t.lastParkAMs):-1L,
    t.parkAssistB, t.lastParkBMs?(long)(millis()-t.lastParkBMs):-1L);
  server.send(200,"application/json",extraBuf);
}

void setupCAN(){
  pinMode(CAN_RX, INPUT_PULLUP);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX,(gpio_num_t)CAN_RX,TWAI_MODE_NORMAL);
  g.rx_queue_len=128; g.tx_queue_len=0;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g,&t,&f));
  ESP_ERROR_CHECK(twai_start());
}

void IRAM_ATTR applyFrame(Telemetry &t, const twai_message_t &m){
  if(m.extd||m.rtr) return;
  const uint8_t *d=m.data;
  switch(m.identifier){
    case 0x131: if(m.data_length_code<6)return; t.soc=d[4]+(d[5]<<8); t.balancing=((d[2]>>4)|((d[3]&0x0F)<<4))&1; t.chargerConnected=((d[2]>>6)&1); break;
    case 0x14D: if(m.data_length_code<6)return; t.soh=d[4]+(d[5]<<8); break;
    case 0x13D: if(m.data_length_code<6)return; t.deltaSoc=d[4]+(d[5]<<8); break;
    case 0x141: if(m.data_length_code<8)return; t.voltage=d[0]+(d[1]<<8); t.imbalance=d[2]+(d[3]<<8); break;
    case 0x200: if(m.data_length_code<8)return; t.rangeKm=d[6]; t.displayFan=((d[7]>>4)&1); break;
    case 0x510: if(m.data_length_code<4)return; { int16_t raw=(int16_t)(d[2]+(d[3]<<8)); t.rpm=raw*5/6; t.tcRpm=t.rpm; } break;  // working hypothesis: Traction Control RPM
    case 0x601: if(m.data_length_code<4)return; t.mcRpm=(int32_t)(d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24)); break;  // probable: Motor Control RPM (real ride only)
    case 0x101: if(m.data_length_code<5)return; t.driveMode=d[1]; t.smartEco=((d[3]>>6)&1); t.forcedEco=((d[4]>>2)&1); t.navJoystick=d[1]&0x07; break;  // D1 bits 0-2: nav joystick, raw code
    case 0x102:
      if(m.data_length_code<5)return;
      if(m.data_length_code>=8) t.pduVoltage=d[6]+(d[7]<<8);   // confirmed ~12.0-12.1 V
      t.sideStand=(d[4]&1);
      t.frontBrake=((d[0]>>0)&1); t.rearBrake=((d[0]>>1)&1); t.highBeam=((d[0]>>2)&1);
      t.startSwitch=((d[0]>>4)&1); t.killSwitch=((d[0]>>5)&1); t.storageSwitch=((d[0]>>6)&1);
      t.horn=((d[1]>>1)&1); t.indicatorLeftSide=((d[2]>>0)&1); t.indicatorRightSide=((d[2]>>1)&1); t.indicatorCenterSide=((d[2]>>2)&1); t.keyOn=((d[4]>>3)&1);
      break;
    case 0x103:
      if(m.data_length_code<2)return;
      t.seatLock=((d[1]>>1)&1); t.chargerLock2=((d[1]>>2)&1); t.chargerLock1=((d[1]>>3)&1);
      break;
    case 0x208:
      if(m.data_length_code<4)return;
      t.chargerFan1=d[0]+(d[1]<<8); t.chargerFan2=d[2]+(d[3]<<8);
      break;
    case 0x132: if(m.data_length_code<8)return; t.cellV[0]=d[0]+(d[1]<<8); t.cellV[1]=d[2]+(d[3]<<8); t.cellV[2]=d[4]+(d[5]<<8); t.cellV[3]=d[6]+(d[7]<<8); break;
    case 0x133: if(m.data_length_code<8)return; t.cellV[4]=d[0]+(d[1]<<8); t.cellV[5]=d[2]+(d[3]<<8); t.cellV[6]=d[4]+(d[5]<<8); t.cellV[7]=d[6]+(d[7]<<8); break;
    case 0x134: if(m.data_length_code<8)return; t.cellV[8]=d[0]+(d[1]<<8); t.cellV[9]=d[2]+(d[3]<<8); t.cellV[10]=d[4]+(d[5]<<8); t.cellV[11]=d[6]+(d[7]<<8); break;
    case 0x135: if(m.data_length_code<4)return; t.cellV[12]=d[0]+(d[1]<<8); t.cellV[13]=d[2]+(d[3]<<8); if(m.data_length_code>=8){ t.afeDieTemp=(int16_t)(d[4]+(d[5]<<8)); t.packCurrent=(int16_t)(d[6]+(d[7]<<8)); } break;
    case 0x14e: if(m.data_length_code<8)return; t.cellSoh[0]=d[0]; t.cellSoh[1]=d[1]; t.cellSoh[2]=d[2]; t.cellSoh[3]=d[3]; t.cellSoh[4]=d[4]; t.cellSoh[5]=d[5]; t.cellSoh[6]=d[6]; t.cellSoh[7]=d[7]; break;
    case 0x14f: if(m.data_length_code<6)return; t.cellSoh[8]=d[0]; t.cellSoh[9]=d[1]; t.cellSoh[10]=d[2]; t.cellSoh[11]=d[3]; t.cellSoh[12]=d[4]; t.cellSoh[13]=d[5]; break;
    case 0x201:
      if(m.data_length_code<8)return;
      t.chargerOV=d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24);
      t.chargerOI=d[4]|(d[5]<<8)|(d[6]<<16)|((uint32_t)d[7]<<24);
      break;
    case 0x203:
      if(m.data_length_code<4)return;
      t.chargerIV=d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24);
      break;
  
    case 0x130: if(m.data_length_code<5)return; t.bmsUptimeMs=(uint64_t)d[0]|((uint64_t)d[1]<<8)|((uint64_t)d[2]<<16)|((uint64_t)d[3]<<24)|((uint64_t)d[4]<<32); break;
    case 0x136: if(m.data_length_code<8)return; for(int i=0;i<4;i++) t.cellTemp[i]=(int16_t)(d[i*2]+(d[i*2+1]<<8)); break;
    case 0x137: if(m.data_length_code<8)return; t.cellTemp[4]=(int16_t)(d[0]+(d[1]<<8)); t.eftTemp=(int16_t)(d[2]+(d[3]<<8)); t.eciTemp=(int16_t)(d[4]+(d[5]<<8)); t.senseTemp=(int16_t)(d[6]+(d[7]<<8)); break;
    case 0x139: if(m.data_length_code<6)return; t.deratedDsgI=d[0]+(d[1]<<8); t.deratedChgI=d[2]+(d[3]<<8); t.deratedRegenI=d[4]+(d[5]<<8); break;
    case 0x140: if(m.data_length_code<6)return; t.humidity=(int16_t)(d[0]+(d[1]<<8)); t.bmsAmbient=(int16_t)(d[2]+(d[3]<<8)); t.socEkf=d[4]+(d[5]<<8); break;
    case 0x142: if(m.data_length_code<8)return; t.minV1Est=(int16_t)(d[0]+(d[1]<<8)); t.minV2Est=(int16_t)(d[2]+(d[3]<<8)); t.minSocEst=d[4]+(d[5]<<8); t.dsgLimTemp=d[6]+(d[7]<<8); break;
    case 0x143: if(m.data_length_code<8)return; for(int i=0;i<4;i++) t.cellSoc[i]=d[i*2]+(d[i*2+1]<<8); break;
    case 0x144: if(m.data_length_code<8)return; for(int i=0;i<4;i++) t.cellSoc[4+i]=d[i*2]+(d[i*2+1]<<8); break;
    case 0x145: if(m.data_length_code<8)return; for(int i=0;i<4;i++) t.cellSoc[8+i]=d[i*2]+(d[i*2+1]<<8); break;
    case 0x146: if(m.data_length_code<6)return; t.cellSoc[12]=d[0]+(d[1]<<8); t.cellSoc[13]=d[2]+(d[3]<<8); t.maxSoc=d[4]+(d[5]<<8); break;
    case 0x148: if(m.data_length_code<6)return; t.chgCellRule=d[0]+(d[1]<<8); t.chgCmdRateLtd=d[2]+(d[3]<<8); t.chgPostQuant=(int16_t)(d[4]+(d[5]<<8)); break;
    case 0x151: if(m.data_length_code<6)return; t.soe=d[4]+(d[5]<<8); break;
    case 0x154: if(m.data_length_code<8)return; t.boardTempRefV=d[0]+(d[1]<<8); t.vref2=d[2]+(d[3]<<8); t.vref3=d[4]+(d[5]<<8); t.mosfetPwrV=d[6]+(d[7]<<8); break;
    case 0x155: if(m.data_length_code<2)return; t.afeVreg=d[0]+(d[1]<<8); break;
    case 0x110: if(m.data_length_code<6)return; t.imuAccX=(int16_t)(d[0]+(d[1]<<8)); t.imuAccY=(int16_t)(d[2]+(d[3]<<8)); t.imuAccZ=(int16_t)(d[4]+(d[5]<<8)); break;
    case 0x112: if(m.data_length_code<6)return; t.eulerRoll=(int16_t)(d[0]+(d[1]<<8)); t.eulerPitch=(int16_t)(d[2]+(d[3]<<8)); t.eulerHeading=(int16_t)(d[4]+(d[5]<<8)); break;
    case 0x212: if(m.data_length_code<4)return; t.odometer=d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24); break;
    case 0x162: if(m.data_length_code<7)return; t.bmsHwVer=d[1]; t.bmsFw[0]=d[4]; t.bmsFw[1]=d[5]; t.bmsFw[2]=d[6]; break;
    case 0x107: if(m.data_length_code<7)return; t.bcmFw[0]=d[4]; t.bcmFw[1]=d[5]; t.bcmFw[2]=d[6]; break;
    case 0x216: if(m.data_length_code<7)return;
      if (d[4] || d[5] || d[6]) { t.gtwFw[0]=d[4]; t.gtwFw[1]=d[5]; t.gtwFw[2]=d[6]; t.lastGtwMs=millis(); }
      break;
    case 0x70B: if(m.data_length_code<4)return; t.motorFw[0]=d[1]; t.motorFw[1]=d[2]; t.motorFw[2]=d[3]; break;
    case 0x163: if(m.data_length_code<8)return; for(int i=0;i<6;i++) t.binId[i]=(char)d[2+i]; break;
    case 0x164: if(m.data_length_code<8)return; for(int i=0;i<8;i++) t.binId[6+i]=(char)d[i]; t.binId[14]=0; break;
    case 0x38c:
      if(m.data_length_code<8)return;
      t.throttleOut=((d[3]>>2)|((d[4]&3)<<6))-(((d[4]>>1)&1)*256);
      t.throttle3=((d[5]>>4)|((d[6]&15)<<4))-(((d[6]>>3)&1)*256);
      t.throttle1=((d[6]>>4)|((d[7]&15)<<4))-(((d[7]>>3)&1)*256);
      break;
    case 0x206: if(m.data_length_code<8)return; { float f; memcpy(&f,&d[4],4); t.chgSocLimit=f; } break;  
    case 0x91: if(m.data_length_code<8)return; memcpy(t.vin,d,8); t.lastVinMs=millis(); break;  
    case 0x92: if(m.data_length_code<8)return; memcpy(t.vin+8,d,8); t.vin[16]=0; t.lastVinMs=millis(); break;  
    case 0x93: if(m.data_length_code<1)return; t.vin[16]=(char)d[0]; t.vin[17]=0; t.lastVinMs=millis(); break;  
    case 0x7A: if(m.data_length_code<8)return; memcpy(t.scooterId,d,8); t.scooterId[8]=0; t.lastSidMs=millis(); break; 
    case 0x7B1: case 0x7B9: t.lastDiagMs=millis(); break;                                      
    case 0x120: if(m.data_length_code<3)return; t.parkAssistA=d[2]; t.lastParkAMs=millis(); break;  
    case 0x121: if(m.data_length_code<3)return; t.parkAssistB=d[2]; t.lastParkBMs=millis(); break;  
    case 0x18c:
      if(m.data_length_code<5)return;
      t.motorI=((d[2]>>5)|((d[3]&31)<<3))+((((d[3]>>5)|((d[4]&31)<<3))&1)*256)-(((d[3]>>5)&1)*512);
      t.motorTemp=(((d[4]&63)<<2)|(d[3]>>6))-40;                                   
      if(m.data_length_code>=6) t.controllerTemp=(((d[5]&63)<<2)|(d[4]>>6))-40;    
      break;
    case 0x209: if(m.data_length_code<3)return; t.motorMcuTemp=(int16_t)d[2]; break;  
  }
}

void canTask(void *){
  Telemetry local;
  xSemaphoreTake(mtx, portMAX_DELAY);
  local = snap;
  xSemaphoreGive(mtx);
  uint32_t backlog = 0;
  for(;;){
    twai_message_t msg;
    if(twai_receive(&msg,0)==ESP_OK){
      lastCanRxMs = millis();
      applyFrame(local, msg);
      backlog++;
    } else {
      if(backlog>0){
        xSemaphoreTake(mtx, portMAX_DELAY);
        snap = local;
        xSemaphoreGive(mtx);
        if(backlog>maxQueueBacklog) maxQueueBacklog=backlog;
        backlog=0;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.persistent(false); WiFi.disconnect(true,true); delay(200);
  WiFi.mode(WIFI_AP); delay(200);
  WiFi.softAP(AP_SSID,AP_PASS,6,0,4); WiFi.setTxPower(WIFI_POWER_8_5dBm);
  server.on("/",handleRoot); server.on("/data",handleData); server.on("/cells",handleCells); server.on("/charger",handleCharger); server.on("/extra",handleExtra); server.begin();
  setupCAN();
  mtx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(canTask,"can",4096,NULL,2,NULL,0);
}

void loop(){
  server.handleClient();
  if(millis()-lastDiag>=5000){
    lastDiag=millis();
    twai_status_info_t st;
    if(twai_get_status_info(&st)==ESP_OK){
      Serial.printf("[DIAG] rx_missed:%lu rx_overrun:%lu arb_lost:%lu max_backlog:%lu heap:%lu\n",
        (unsigned long)st.rx_missed_count,(unsigned long)st.rx_overrun_count,
        (unsigned long)st.arb_lost_count,(unsigned long)maxQueueBacklog,
        (unsigned long)ESP.getFreeHeap());
    }
    maxQueueBacklog=0;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}
