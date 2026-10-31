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
  // ===== EXTRA (newly discovered) fields =====
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
  // ===== v4.9 additions =====
  uint16_t pduVoltage;              // 0x102 D6-D7, confirmed ~12.0-12.1 V
  float    chgSocLimit;             // 0x206 D4-D7, float32 - confirmed by real charging test
  int16_t  controllerTemp;          // 0x18C D4-D5, bit-packed - 2nd of 3 distinct temperatures
  int16_t  motorMcuTemp;            // 0x209 D2 - 3rd, independent temperature reading
  int16_t  throttleOut, throttle1, throttle3;  // 0x38C, bit-packed
  int32_t  tcRpm;                   // 0x510 - original sketch's RPM frame (working hypothesis: Traction Control RPM)
  int32_t  mcRpm;                   // 0x601 - probable real motor/wheel-speed RPM (seen only during a real ride)
  char     vin[20];                 // 0x91+0x92+0x93 ASCII (second-scooter / service-session only)
  char     scooterId[10];           // 0x7A ASCII (second-scooter / service-session only)
  unsigned long lastDiagMs;         // last time a 0x7B1/0x7B9 diagnostic frame was seen
  // ===== fields missing from this build, added back in =====
  uint8_t  navJoystick;             // 0x101 D1 bits 0-2, raw code (which code = which direction unconfirmed)
  uint8_t  parkAssistA, parkAssistB;             // 0x120 / 0x121, one-shot triggers (probable Parking Assist)
  unsigned long lastParkAMs, lastParkBMs;
  unsigned long lastVinMs, lastSidMs, lastGtwMs;  // freshness: these only update when their rare
                                                   // broadcast frame is actually on the bus - nothing in
                                                   // listen-only code can make that happen sooner, so this
                                                   // just shows how old the value is instead of a silent "--"
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
"    --amber: #e7a526; --red: #dc3545; --ink: #20242b; --border:#dce2ec;\n"
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
"  .thermals { background:#ffe0e7; color:#b42b4c; } .diagnostic { background:#dff1ff; color:#17628f; } .others { background:#e8eaee; color:#4a5565; }\n"
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
"  .field-tag.rarely-seen { background:#e5efff; color:#2856a6; }\n"
"  .field-tag.connect-charge { background:#2856a6; color:#ffffff; }\n"
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
".cell-table-wrap{width:100%;max-width:100%;overflow-x:auto;-webkit-overflow-scrolling:touch}\n.cell-table{width:100%;min-width:0;table-layout:fixed;border-collapse:collapse}\n.cell-table th,.cell-table td{padding:7px 4px;overflow-wrap:anywhere;word-break:break-word;font-size:clamp(10px,2.5vw,13px)}\n.cell-table th:first-child,.cell-table td:first-child{width:24%}\n.cell-table th:not(:first-child),.cell-table td:not(:first-child){width:25.33%}\n@media (prefers-color-scheme: dark) {\n    :root {\n      color-scheme: dark;\n      --ink: #edf1f7; --muted: #aab4c3; --line: #3a4352;\n      --page: #141922; --card: #202734;\n    }\n    body { background:var(--page); color:var(--ink); }\n    .tile:hover:not(:disabled), .tile:focus-visible:not(:disabled) { background:#252e3c; outline-color:#536b9e; }\n    .demo-controls, .control, .data-connection, .cell-table-wrap, .about-link-row { background:#202734; color:var(--ink); }\n    .detail-head { background:#202734; color:var(--ink); border-bottom:1px solid var(--line); }\n    .metric, .telemetry-item { background:#202734; border-color:#3a4352; box-shadow:none; }\n    .telemetry-item { background:linear-gradient(145deg,#222b39,#1c232f); }\n    .telemetry-label { color:#b7c1d0; }\n    .telemetry-value { color:#f1f4f9; }\n    .telemetry-unit { color:#aab4c3; }\n    .field-tag { background:#303a4a; color:#c4cedc; }\n    .field-tag.estimated { background:#493b1d; color:#f2cf75; }\n    .field-tag.hypothesis { background:#3b2b50; color:#d7b7ff; }\n    .field-tag.rarely-seen { background:#263c5d; color:#b8d2ff; }\n    .cell-table th { background:#2b3545; color:#d5deeb; }\n    .cell-table td { border-top-color:#394252; }\n    .cell-table tbody tr:nth-child(even) { background:#1b222d; }\n    .cell-name { color:#9dbaff; }\n    .group-title .group-symbol, .about-link-icon { background:#303a4a; color:#dce5f3; }\n    .about-hero { background:linear-gradient(135deg,#27334b,#252c3b 55%,#1d3934); }\n    .about-link-placeholder { background:#303846; color:#c1cad7; }\n    .empty-state { background:#20273480; border-color:#465165; }\n    .detail-body > .prototype-note { background:#382f1d; color:#e8d29b; }\n    .scooter-placeholder { background:linear-gradient(145deg,rgba(160,180,205,.12),rgba(160,180,205,.04)); }\n  }\n"
"  .diagnostic-tabs{display:flex;flex-wrap:wrap;gap:8px;margin:4px 0 18px}\n"
"  .diagnostic-tab{border:1px solid var(--border);background:var(--card);color:var(--ink);border-radius:12px;padding:10px 12px;font-weight:650;cursor:pointer}\n"
"  .diagnostic-tab.active{background:var(--blue);border-color:var(--blue);color:#fff}\n"
"  .diag-panel{border:1px solid var(--border);border-radius:16px;padding:clamp(16px,2.5vw,24px);margin:0 0 20px;background:var(--card);box-shadow:0 3px 14px rgba(30,45,70,.045)}\n"
"  .diag-panel h3{margin:0 0 8px;font-size:17px}\n"
"  .diag-muted{color:var(--muted);font-size:13px;line-height:1.5}\n"
"  .diag-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:14px;margin-top:16px}\n"
"  .diag-metric{border:1px solid var(--border);border-radius:12px;padding:11px;min-width:0}\n"
"  .diag-metric span{display:block;color:var(--muted);font-size:12px;margin-bottom:5px}\n"
"  .diag-metric strong{font-size:17px;overflow-wrap:anywhere}\n"
"  .diag-actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}\n"
"  .diag-actions button{border:0;border-radius:10px;padding:10px 14px;font-weight:700;cursor:pointer;background:var(--blue);color:white}\n"
"  .diag-actions button.secondary{background:var(--blue-dark)}\n"
"  .diag-actions button.danger{background:var(--red)}\n"
"  .diag-actions button:disabled{opacity:.45;cursor:not-allowed}\n"
"  .diag-profile{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:10px}\n"
"  .diag-profile label{border:1px solid var(--border);border-radius:14px;padding:14px;display:flex;gap:10px;align-items:flex-start;cursor:pointer;min-width:0;line-height:1.45;transition:border-color .18s,box-shadow .18s,transform .18s}\n"
"  .diag-profile label:has(input:checked){border-color:var(--blue);box-shadow:0 0 0 2px color-mix(in srgb,var(--blue) 16%,transparent);background:color-mix(in srgb,var(--blue) 5%,var(--card))}\n"
"  .diag-profile input{margin-top:4px;accent-color:var(--blue);flex:0 0 auto}\n"
"  .diag-clock{display:flex;align-items:center;gap:20px;margin-top:20px;padding:16px;border-radius:16px;background:color-mix(in srgb,var(--blue) 5%,var(--card));border:1px solid var(--border)}\n"
"  .diag-clock-face{width:92px;height:92px;flex:0 0 92px;border-radius:50%;display:grid;place-items:center;background:conic-gradient(var(--green) var(--progress,0%),var(--border) 0);position:relative}\n"
"  .diag-clock-face:before{content:\"\";position:absolute;inset:7px;border-radius:50%;background:var(--card)}\n"
"  .diag-clock-face strong{position:relative;font-size:15px;font-variant-numeric:tabular-nums}\n"
"  .diag-clock-face.running{animation:diagRingPulse 2s ease-in-out infinite}\n"
"  .diag-clock-face.running strong{animation:diagPulse 1.3s ease-in-out infinite}\n"
"  @keyframes diagRingPulse{0%,100%{filter:drop-shadow(0 0 0 rgba(24,201,107,0))}50%{filter:drop-shadow(0 0 8px rgba(24,201,107,.28))}}\n"
"  .diag-list{margin:8px 0 0;padding-left:20px;line-height:1.55}\n"
"  @keyframes diagPulse{50%{opacity:.55;transform:scale(.96)}}\n"
"  @media(max-width:680px){.diag-grid{grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.diag-profile{grid-template-columns:1fr}.diag-clock{align-items:center;gap:14px;padding:12px}.diag-clock-face{width:78px;height:78px;flex-basis:78px}.diag-actions button{flex:1 1 42%}}\n"
"  @media(max-width:390px){.diag-grid{grid-template-columns:1fr}.diag-clock{align-items:flex-start;flex-direction:column}}\n"
"</style>\n"
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
"      <img class=\"scooter-reference scooter-placeholder\" src=\"data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCA4MDAgNjAwIiB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIj4KICA8ZGVmcz4KICAgIDwhLS0gRHVhbC1Nb2RlIFRpdGFuaXVtIFNpbHZlciAtLT4KICAgIDxsaW5lYXJHcmFkaWVudCBpZD0iYm9keU1haW4iIHgxPSIwJSIgeTE9IjAlIiB4Mj0iMTAwJSIgeTI9IjEwMCUiPgogICAgICA8c3RvcCBvZmZzZXQ9IjAlIiBzdG9wLWNvbG9yPSIjZWNlZmYxIiAvPgogICAgICA8c3RvcCBvZmZzZXQ9IjQ1JSIgc3RvcC1jb2xvcj0iIzkwYTRhZSIgLz4KICAgICAgPHN0b3Agb2Zmc2V0PSIxMDAlIiBzdG9wLWNvbG9yPSIjNTQ2ZTdhIiAvPgogICAgPC9saW5lYXJHcmFkaWVudD4KICAgIAogICAgPGxpbmVhckdyYWRpZW50IGlkPSJib2R5RGFyayIgeDE9IjAlIiB5MT0iMCUiIHgyPSIwJSIgeTI9IjEwMCUiPgogICAgICA8c3RvcCBvZmZzZXQ9IjAlIiBzdG9wLWNvbG9yPSIjNDU1YTY0IiAvPgogICAgICA8c3RvcCBvZmZzZXQ9IjEwMCUiIHN0b3AtY29sb3I9IiMxMTExMTEiIC8+CiAgICA8L2xpbmVhckdyYWRpZW50PgogICAgCiAgICA8bGluZWFyR3JhZGllbnQgaWQ9Im5lb25HcmVlbiIgeDE9IjAlIiB5MT0iMCUiIHgyPSIxMDAlIiB5Mj0iMCUiPgogICAgICA8c3RvcCBvZmZzZXQ9IjAlIiBzdG9wLWNvbG9yPSIjMDBlNjc2IiAvPgogICAgICA8c3RvcCBvZmZzZXQ9IjEwMCUiIHN0b3AtY29sb3I9IiMxZGU5YjYiIC8+CiAgICA8L2xpbmVhckdyYWRpZW50PgogICAgCiAgICA8bGluZWFyR3JhZGllbnQgaWQ9ImJlYW0iIHgxPSIwJSIgeTE9IjUwJSIgeDI9IjEwMCUiIHkyPSI1MCUiPgogICAgICA8c3RvcCBvZmZzZXQ9IjAlIiBzdG9wLWNvbG9yPSIjZTBmN2ZhIiBzdG9wLW9wYWNpdHk9IjAuODUiIC8+CiAgICAgIDxzdG9wIG9mZnNldD0iMTAwJSIgc3RvcC1jb2xvcj0iI2UwZjdmYSIgc3RvcC1vcGFjaXR5PSIwIiAvPgogICAgPC9saW5lYXJHcmFkaWVudD4KCiAgICA8IS0tIEhvbG9ncmFwaGljIFByb2plY3Rpb24gQmVhbSAtLT4KICAgIDxsaW5lYXJHcmFkaWVudCBpZD0iaG9sb0JlYW0iIHgxPSIwJSIgeTE9IjEwMCUiIHgyPSIwJSIgeTI9IjAlIj4KICAgICAgPHN0b3Agb2Zmc2V0PSIwJSIgc3RvcC1jb2xvcj0iIzAwZTY3NiIgc3RvcC1vcGFjaXR5PSIwLjQiIC8+CiAgICAgIDxzdG9wIG9mZnNldD0iMTAwJSIgc3RvcC1jb2xvcj0iIzAwZTY3NiIgc3RvcC1vcGFjaXR5PSIwIiAvPgogICAgPC9saW5lYXJHcmFkaWVudD4KICAgIAogICAgPGxpbmVhckdyYWRpZW50IGlkPSJkYXRhU3RyZWFtIiB4MT0iMCUiIHkxPSIxMDAlIiB4Mj0iMTAwJSIgeTI9IjAlIj4KICAgICAgPHN0b3Agb2Zmc2V0PSIwJSIgc3RvcC1jb2xvcj0iIzAwZTY3NiIgc3RvcC1vcGFjaXR5PSIwLjE1IiAvPgogICAgICA8c3RvcCBvZmZzZXQ9IjEwMCUiIHN0b3AtY29sb3I9IiMwMGU2NzYiIHN0b3Atb3BhY2l0eT0iMC45IiAvPgogICAgPC9saW5lYXJHcmFkaWVudD4KICAgIAogICAgPHJhZGlhbEdyYWRpZW50IGlkPSJ0aXJlR3JhZGllbnQiIGN4PSI1MCUiIGN5PSI1MCUiIHI9IjUwJSI+CiAgICAgIDxzdG9wIG9mZnNldD0iNzAlIiBzdG9wLWNvbG9yPSIjMjYzMjM4IiAvPgogICAgICA8c3RvcCBvZmZzZXQ9IjEwMCUiIHN0b3AtY29sb3I9IiMwNTA1MDUiIC8+CiAgICA8L3JhZGlhbEdyYWRpZW50PgogICAgCiAgICA8IS0tIEFkYXB0aXZlIEJsdWVwcmludCBHcmlkIC0tPgogICAgPHBhdHRlcm4gaWQ9ImdyaWQiIHdpZHRoPSI0MCIgaGVpZ2h0PSI0MCIgcGF0dGVyblVuaXRzPSJ1c2VyU3BhY2VPblVzZSI+CiAgICAgIDxwYXRoIGQ9Ik0gNDAgMCBMIDAgMCAwIDQwIiBmaWxsPSJub25lIiBzdHJva2U9IiM3ODkwOWMiIHN0cm9rZS13aWR0aD0iMC43IiBzdHJva2Utb3BhY2l0eT0iMC4zIiAvPgogICAgICA8Y2lyY2xlIGN4PSI0MCIgY3k9IjQwIiByPSIxLjUiIGZpbGw9IiM3ODkwOWMiIG9wYWNpdHk9IjAuNCIgLz4KICAgIDwvcGF0dGVybj4KCiAgICA8IS0tIEN1c3RvbSBDYXJib24gRmliZXIgVGV4dHVyZSAtLT4KICAgIDxwYXR0ZXJuIGlkPSJjYXJib24iIHdpZHRoPSI2IiBoZWlnaHQ9IjYiIHBhdHRlcm5Vbml0cz0idXNlclNwYWNlT25Vc2UiPgogICAgICA8cmVjdCB3aWR0aD0iNiIgaGVpZ2h0PSI2IiBmaWxsPSIjMTExMTExIiAvPgogICAgICA8cGF0aCBkPSJNMCw2IGw2LC02IG0tMiw4IGw0LC00IiBzdHJva2U9IiMyNjMyMzgiIHN0cm9rZS13aWR0aD0iMiIgc3Ryb2tlLWxpbmVjYXA9InNxdWFyZSIgLz4KICAgIDwvcGF0dGVybj4KCiAgICA8ZmlsdGVyIGlkPSJnbG93IiB4PSItMzAlIiB5PSItMzAlIiB3aWR0aD0iMTYwJSIgaGVpZ2h0PSIxNjAlIj4KICAgICAgPGZlR2F1c3NpYW5CbHVyIHN0ZERldmlhdGlvbj0iMy41IiByZXN1bHQ9ImJsdXIiIC8+CiAgICAgIDxmZU1lcmdlPgogICAgICAgIDxmZU1lcmdlTm9kZSBpbj0iYmx1ciIgLz4KICAgICAgICA8ZmVNZXJnZU5vZGUgaW49IlNvdXJjZUdyYXBoaWMiIC8+CiAgICAgIDwvZmVNZXJnZT4KICAgIDwvZmlsdGVyPgogIDwvZGVmcz4KCiAgPHN0eWxlPgogICAgLndoZWVsIHsgdHJhbnNmb3JtLWJveDogZmlsbC1ib3g7IHRyYW5zZm9ybS1vcmlnaW46IGNlbnRlcjsgfQogICAgLnNwaW4geyBhbmltYXRpb246IHJvdGF0ZVdoZWVsIDAuNHMgbGluZWFyIGluZmluaXRlOyB9CiAgICAuYm91bmNlIHsgYW5pbWF0aW9uOiBjaGFzc2lzQm91bmNlIDIuMnMgZWFzZS1pbi1vdXQgaW5maW5pdGU7IH0KICAgIC5zaGFkb3ctcHVsc2UgeyB0cmFuc2Zvcm0tYm94OiBmaWxsLWJveDsgdHJhbnNmb3JtLW9yaWdpbjogY2VudGVyOyBhbmltYXRpb246IHNoYWRvd1NjYWxlIDIuMnMgZWFzZS1pbi1vdXQgaW5maW5pdGU7IH0KICAgIC5nbG93LXB1bHNlIHsgYW5pbWF0aW9uOiBnbG93aW5nUHVsc2UgMXMgaW5maW5pdGUgYWx0ZXJuYXRlOyB9CiAgICAuaG9sby1mbGlja2VyIHsgYW5pbWF0aW9uOiBob2xvZ3JhbUZsaWNrZXIgMC4xNXMgaW5maW5pdGUgYWx0ZXJuYXRlOyB9CiAgICAuZGF0YS1mbG9hdCB7IGFuaW1hdGlvbjogZGF0YVVwIDIuNXMgY3ViaWMtYmV6aWVyKDAuNCwgMCwgMC4yLCAxKSBpbmZpbml0ZTsgZm9udC1mYW1pbHk6IG1vbm9zcGFjZTsgZm9udC1zaXplOiAxNHB4OyBmb250LXdlaWdodDogYm9sZDsgfQogICAgLmRhdGEtZmxvYXQtMiB7IGFuaW1hdGlvbjogZGF0YVVwIDIuNXMgY3ViaWMtYmV6aWVyKDAuNCwgMCwgMC4yLCAxKSBpbmZpbml0ZTsgYW5pbWF0aW9uLWRlbGF5OiAwLjhzOyBmb250LWZhbWlseTogbW9ub3NwYWNlOyBmb250LXNpemU6IDEycHg7IGZvbnQtd2VpZ2h0OiBib2xkOyB9CiAgICAuZGF0YS1mbG9hdC0zIHsgYW5pbWF0aW9uOiBkYXRhVXAgMi41cyBjdWJpYy1iZXppZXIoMC40LCAwLCAwLjIsIDEpIGluZmluaXRlOyBhbmltYXRpb24tZGVsYXk6IDEuNnM7IGZvbnQtZmFtaWx5OiBtb25vc3BhY2U7IGZvbnQtc2l6ZTogMTZweDsgZm9udC13ZWlnaHQ6IGJvbGQ7IH0KICAgIC5ibGluayB7IGFuaW1hdGlvbjogYmxpbmtlciAwLjRzIHN0ZXBzKDIsIHN0YXJ0KSBpbmZpbml0ZTsgfQoKICAgIEBrZXlmcmFtZXMgcm90YXRlV2hlZWwgeyBmcm9tIHsgdHJhbnNmb3JtOiByb3RhdGUoMGRlZyk7IH0gdG8geyB0cmFuc2Zvcm06IHJvdGF0ZSgzNjBkZWcpOyB9IH0KICAgIEBrZXlmcmFtZXMgY2hhc3Npc0JvdW5jZSB7IDAlLCAxMDAlIHsgdHJhbnNmb3JtOiB0cmFuc2xhdGVZKDApOyB9IDUwJSB7IHRyYW5zZm9ybTogdHJhbnNsYXRlWSgtNHB4KTsgfSB9CiAgICBAa2V5ZnJhbWVzIHNoYWRvd1NjYWxlIHsgMCUsIDEwMCUgeyB0cmFuc2Zvcm06IHNjYWxlKDEpOyBvcGFjaXR5OiAwLjY7IH0gNTAlIHsgdHJhbnNmb3JtOiBzY2FsZSgwLjk3KTsgb3BhY2l0eTogMC40OyB9IH0KICAgIEBrZXlmcmFtZXMgZ2xvd2luZ1B1bHNlIHsgZnJvbSB7IG9wYWNpdHk6IDAuNTsgfSB0byB7IG9wYWNpdHk6IDE7IH0gfQogICAgQGtleWZyYW1lcyBob2xvZ3JhbUZsaWNrZXIgeyBmcm9tIHsgb3BhY2l0eTogMC43OyB9IHRvIHsgb3BhY2l0eTogMC45NTsgfSB9CiAgICBAa2V5ZnJhbWVzIGJsaW5rZXIgeyB0byB7IHZpc2liaWxpdHk6IGhpZGRlbjsgfSB9CiAgICBAa2V5ZnJhbWVzIGRhdGFVcCB7CiAgICAgIDAlIHsgdHJhbnNmb3JtOiB0cmFuc2xhdGUoMCwgMCkgc2NhbGUoMC41KTsgb3BhY2l0eTogMDsgfQogICAgICAxNSUgeyBvcGFjaXR5OiAxOyB9CiAgICAgIDg1JSB7IG9wYWNpdHk6IDE7IH0KICAgICAgMTAwJSB7IHRyYW5zZm9ybTogdHJhbnNsYXRlKDgwcHgsIC05MHB4KSBzY2FsZSgxLjEpOyBvcGFjaXR5OiAwOyB9CiAgICB9CiAgPC9zdHlsZT4KCiAgPCEtLSBCYWNrZ3JvdW5kIEdyaWQgLS0+CiAgPHJlY3Qgd2lkdGg9IjEwMCUiIGhlaWdodD0iMTAwJSIgZmlsbD0idXJsKCNncmlkKSIgLz4KCiAgPCEtLSBGbGlwcGVkIFdyYXBwZXIgR3JvdXAgKE1pcnJvcnMgdGhlIGVudGlyZSBzY29vdGVyIHRvIGZhY2UgbGVmdCkgLS0+CiAgPGcgaWQ9InJlcG8taGVyby1sZWZ0IiB0cmFuc2Zvcm09InRyYW5zbGF0ZSg4MDAsIDApIHNjYWxlKC0xLCAxKSI+CiAgICAKICAgIDwhLS0gRHluYW1pYyBHcm91bmQgU2hhZG93ICYgVW5kZXJnbG93IC0tPgogICAgPGVsbGlwc2UgY3g9IjM4MCIgY3k9IjUwNSIgcng9IjI2MCIgcnk9IjExIiBmaWxsPSIjMDAwMDAwIiBjbGFzcz0ic2hhZG93LXB1bHNlIiAvPgogICAgPGVsbGlwc2UgY3g9IjM4MCIgY3k9IjQ5NSIgcng9IjIyMCIgcnk9IjE1IiBmaWxsPSIjMDBlNjc2IiBvcGFjaXR5PSIwLjMiIGNsYXNzPSJnbG93LXB1bHNlIiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+CgogICAgPCEtLSBDSEFTU0lTIEdST1VQIC0tPgogICAgPGcgaWQ9ImNoYXNzaXMiIGNsYXNzPSJib3VuY2UiPgogICAgICAKICAgICAgPCEtLSBIZWFkbGlnaHQgQmVhbXMgLS0+CiAgICAgIDxwb2x5Z29uIHBvaW50cz0iNTEwLDI4OCA4MDAsMTY1IDgwMCw0OTUiIGZpbGw9InVybCgjYmVhbSkiIGNsYXNzPSJnbG93LXB1bHNlIiBzdHlsZT0icG9pbnRlci1ldmVudHM6IG5vbmU7IiAvPgogICAgICA8cG9seWdvbiBwb2ludHM9IjUyNSwyOTIgODAwLDIyNSA4MDAsNDM1IiBmaWxsPSJ1cmwoI2JlYW0pIiBvcGFjaXR5PSIwLjgiIHN0eWxlPSJwb2ludGVyLWV2ZW50czogbm9uZTsiIC8+IAoKICAgICAgPCEtLSBDQU4gRGlhZ25vc3RpYyBEYXRhIFN0cmVhbSAtLT4KICAgICAgPHBhdGggZD0iTSAyODAgMjM1IFEgMzUwIDEzMCA0MjUgMTYwIiBmaWxsPSJub25lIiBzdHJva2U9InVybCgjZGF0YVN0cmVhbSkiIHN0cm9rZS13aWR0aD0iMi41IiBzdHJva2UtZGFzaGFycmF5PSI0LDgiIGNsYXNzPSJnbG93LXB1bHNlIiAvPgogICAgICA8ZyB0cmFuc2Zvcm09InRyYW5zbGF0ZSgyOTAsIDE5MCkiPgogICAgICAgIDwhLS0gVGV4dCBlbGVtZW50cyBjb3VudGVyLWZsaXBwZWQgc28gdGhleSByZWFkIGNvcnJlY3RseSAtLT4KICAgICAgICA8ZyBjbGFzcz0iZGF0YS1mbG9hdCI+PHRleHQgeD0iLTE0IiB5PSIwIiBmaWxsPSIjMDBlNjc2IiB0cmFuc2Zvcm09InNjYWxlKC0xLCAxKSI+RkY8L3RleHQ+PC9nPgogICAgICAgIDxnIGNsYXNzPSJkYXRhLWZsb2F0LTIiPjx0ZXh0IHg9Ii0yOSIgeT0iLTE1IiBmaWxsPSIjMWRlOWI2IiB0cmFuc2Zvcm09InNjYWxlKC0xLCAxKSI+QTE8L3RleHQ+PC9nPgogICAgICAgIDxnIGNsYXNzPSJkYXRhLWZsb2F0LTMiPjx0ZXh0IHg9Ii00IiB5PSIyNSIgZmlsbD0iIzAwYjRkOCIgdHJhbnNmb3JtPSJzY2FsZSgtMSwgMSkiPjAwPC90ZXh0PjwvZz4KICAgICAgPC9nPgoKICAgICAgPCEtLSBSZWFyIE11ZGd1YXJkIChDYXJib24gRmliZXIpIC0tPgogICAgICA8cGF0aCBkPSJNIDE4NSAzMDAgTCA4NSAzNzUgTCA5NSAzOTAgTCAxOTUgMzIwIFoiIGZpbGw9InVybCgjY2FyYm9uKSIgc3Ryb2tlPSIjMjYzMjM4IiBzdHJva2Utd2lkdGg9IjEuNSIgLz4KICAgICAgPHJlY3QgeD0iNzUiIHk9IjM3NSIgd2lkdGg9IjI0IiBoZWlnaHQ9IjQ4IiBmaWxsPSIjMTExIiByeD0iNCIgdHJhbnNmb3JtPSJyb3RhdGUoLTM1IDg1IDM3NSkiIC8+CiAgICAgIDxyZWN0IHg9Ijc4IiB5PSIzODUiIHdpZHRoPSIxOCIgaGVpZ2h0PSIxMiIgZmlsbD0iI2ZmZmZmZiIgcng9IjEuNSIgdHJhbnNmb3JtPSJyb3RhdGUoLTM1IDg1IDM3NSkiIC8+CiAgICAgIDxjaXJjbGUgY3g9Ijg3IiBjeT0iNDE1IiByPSIzIiBmaWxsPSIjZmYxNzQ0IiB0cmFuc2Zvcm09InJvdGF0ZSgtMzUgODUgMzc1KSIgLz4KCiAgICAgIDwhLS0gU3dpbmdhcm0gJiBCZWx0IERyaXZlIC0tPgogICAgICA8cGF0aCBkPSJNIDIxMCA0MDAgTCAzMjUgMzc1IEwgMzM1IDMzMCBMIDIxMCAzODAgWiIgZmlsbD0idXJsKCNib2R5RGFyaykiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIxIiAvPiAKICAgICAgPHBhdGggZD0iTSAyMTAgMzk1IEwgMzE1IDM2NSIgc3Ryb2tlPSIjMzc0NzRmIiBzdHJva2Utd2lkdGg9IjUiIHN0cm9rZS1kYXNoYXJyYXk9IjEyLDYiIC8+CiAgICAgIAogICAgICA8IS0tIE1vbm9zaG9jayAtLT4KICAgICAgPGxpbmUgeDE9IjI4NSIgeTE9IjMwNSIgeDI9IjIzNSIgeTI9IjM5MCIgc3Ryb2tlPSIjMGEwYTBhIiBzdHJva2Utd2lkdGg9IjIyIiBzdHJva2UtbGluZWNhcD0icm91bmQiIC8+CiAgICAgIDxsaW5lIHgxPSIyODUiIHkxPSIzMDUiIHgyPSIyMzUiIHkyPSIzOTAiIHN0cm9rZT0idXJsKCNuZW9uR3JlZW4pIiBzdHJva2Utd2lkdGg9IjE0IiBzdHJva2UtZGFzaGFycmF5PSI2LDQiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KICAgICAgPGNpcmNsZSBjeD0iMjg1IiBjeT0iMzA1IiByPSI4IiBmaWxsPSIjOTBhNGFlIiAvPiAKCiAgICAgIDwhLS0gRXhwb3NlZCBBdGhlciBHcmVlbiBUcmVsbGlzIEZyYW1lIC0tPgogICAgICA8cGF0aCBkPSJNIDI5MCAyNzAgTCAzNDUgMzM1IEwgMjcwIDMzMCBaIiBmaWxsPSJub25lIiBzdHJva2U9InVybCgjbmVvbkdyZWVuKSIgc3Ryb2tlLXdpZHRoPSI4IiBzdHJva2UtbGluZWpvaW49InJvdW5kIiAvPgogICAgICA8bGluZSB4MT0iMjkwIiB5MT0iMjcwIiB4Mj0iMzEwIiB5Mj0iMzMyIiBzdHJva2U9InVybCgjbmVvbkdyZWVuKSIgc3Ryb2tlLXdpZHRoPSI4IiAvPgogICAgICA8Y2lyY2xlIGN4PSIzMTAiIGN5PSIzMzIiIHI9IjYiIGZpbGw9IiMxMTEiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIxIiAvPgoKICAgICAgPCEtLSBMb3dlciBCYXR0ZXJ5IFBhbiAtLT4KICAgICAgPHBhdGggZD0iTSAzMTAgMzgwIEwgNDk1IDM4MCBMIDQ2MCA0MjAgTCAzMjUgNDIwIFoiIGZpbGw9InVybCgjYm9keURhcmspIiBzdHJva2U9IiM0NTVhNjQiIHN0cm9rZS13aWR0aD0iMS41IiAvPgogICAgICA8cGF0aCBkPSJNIDM0MCA0MTAgTCA0NDUgNDEwIiBzdHJva2U9InVybCgjbmVvbkdyZWVuKSIgc3Ryb2tlLXdpZHRoPSIzLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KCiAgICAgIDwhLS0gTUFJTiBVTklCT0RZIEZBSVJJTkcgKFRpdGFuaXVtIFNpbHZlcikgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gNDUwIDE5MCBMIDUzMCAyODUgTCA1NDUgMzQ1IEwgNDk1IDM4NSBMIDMxMCAzODUgTCAyNTAgMzcwIEwgMTYwIDMxMCBMIDE2MCAyNjAgTCAyNzAgMjYwIEwgMzQ1IDMzNSBMIDQzMCAzMzUgTCA0MzUgMjQwIFoiIGZpbGw9InVybCgjYm9keU1haW4pIiBzdHJva2U9InJnYmEoMjU1LDI1NSwyNTUsMC40KSIgc3Ryb2tlLXdpZHRoPSIxLjUiIGZpbHRlcj0iZHJvcC1zaGFkb3coNHB4IDE0cHggMTZweCByZ2JhKDAsMCwwLDAuNDUpKSIgLz4KICAgICAgCiAgICAgIDwhLS0gU3BlY3VsYXIgSGlnaGxpZ2h0cyAtLT4KICAgICAgPHBhdGggZD0iTSA0NTAgMTkwIEwgNTEwIDI2NSBMIDQ2NSAyNzUgWiIgZmlsbD0iI2ZmZmZmZiIgb3BhY2l0eT0iMC42IiAvPiAKICAgICAgPHBhdGggZD0iTSAzNTAgMzM1IEwgNDI1IDMzNSBMIDQzNSAyNDAgWiIgZmlsbD0iI2ZmZmZmZiIgb3BhY2l0eT0iMC40IiAvPiAKICAgICAgPHBhdGggZD0iTSAxNjAgMjYwIEwgMjcwIDI2MCBMIDIxMCAzMDAgWiIgZmlsbD0iI2ZmZmZmZiIgb3BhY2l0eT0iMC4yIiAvPiAKCiAgICAgIDwhLS0gUGFuZWwgU2VhbXMgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gMzg1IDM0MCBMIDQ5NSAzODUiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIyLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4gCiAgICAgIDxwYXRoIGQ9Ik0gMjcwIDI2MCBMIDMxNSAzMDUiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIyLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KCiAgICAgIDwhLS0gQ2FyYm9uIEZpYmVyIFNpZGUgVmVudHMgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gNDg1IDM0NSBMIDUyNSAzNDUgTCA0OTUgMzc1IEwgNDUwIDM3NSBaIiBmaWxsPSJ1cmwoI2NhcmJvbikiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNDk1IDM1MCBMIDUxNSAzNTAgTCA0OTUgMzY1IEwgNDcwIDM2NSBaIiBmaWxsPSIjMWExYTFhIiAvPgogICAgICA8cGF0aCBkPSJNIDI2NSAzMTAgTCAzMzUgMzEwIEwgMzA1IDMzNSBMIDI0MCAzMzUgWiIgZmlsbD0idXJsKCNjYXJib24pIiAvPgoKICAgICAgPCEtLSBTdGVwcGVkIFNlYXQgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gMTUwIDI2MCBMIDIzMCAyNjAgTCAyNDUgMjQ1IEwgMzMwIDIzNSBMIDMxNSAyMjUgTCAyMjAgMjI1IFoiIGZpbGw9IiMwYTBhMGEiIHN0cm9rZT0iIzI2MzIzOCIgc3Ryb2tlLXdpZHRoPSIxIiAvPiAKICAgICAgPHBhdGggZD0iTSAyMzUgMjQ3IEwgMzIwIDIzNyIgc3Ryb2tlPSIjZmYxNzQ0IiBzdHJva2Utd2lkdGg9IjIuNSIgc3Ryb2tlLWRhc2hhcnJheT0iNSw0IiAvPiAKICAgICAgPHBhdGggZD0iTSAxNjAgMjYwIEwgMjI1IDI2MCBMIDIzNSAyNTAiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIC8+IAoKICAgICAgPCEtLSBXaXJlbGVzcyBEaWFnbm9zdGljIERvbmdsZSAtLT4KICAgICAgPGcgaWQ9InNlYXQtZG9uZ2xlIiB0cmFuc2Zvcm09InRyYW5zbGF0ZSgyNTUsIDIzOCkgcm90YXRlKC04KSI+CiAgICAgICAgPGVsbGlwc2UgY3g9IjE0IiBjeT0iMTIiIHJ4PSIxOCIgcnk9IjYiIGZpbGw9IiMwMDAwMDAiIG9wYWNpdHk9IjAuOCIgLz4KICAgICAgICA8cmVjdCB4PSIwIiB5PSIwIiB3aWR0aD0iMjgiIGhlaWdodD0iMTQiIGZpbGw9IiMxMTExMTEiIHJ4PSIzIiBzdHJva2U9IiM1NDZlN2EiIHN0cm9rZS13aWR0aD0iMS41IiAvPgogICAgICAgIDxwYXRoIGQ9Ik0gNCAxMCBMIDI0IDEwIiBzdHJva2U9IiMwNTA1MDUiIHN0cm9rZS13aWR0aD0iMi41IiAvPiAKICAgICAgICA8Y2lyY2xlIGN4PSI2IiBjeT0iNCIgcj0iMiIgZmlsbD0iIzAwZTY3NiIgY2xhc3M9ImJsaW5rIiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+CiAgICAgICAgPGNpcmNsZSBjeD0iMTQiIGN5PSI0IiByPSIyIiBmaWxsPSIjZmY5MTAwIiBjbGFzcz0iYmxpbmsiIHN0eWxlPSJhbmltYXRpb24tZGVsYXk6IDAuMnMiIGZpbHRlcj0idXJsKCNnbG93KSIgLz4KICAgICAgICA8Y2lyY2xlIGN4PSIyMiIgY3k9IjQiIHI9IjIiIGZpbGw9IiMwMGI0ZDgiIGNsYXNzPSJnbG93LXB1bHNlIiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+CiAgICAgIDwvZz4KCiAgICAgIDwhLS0gUGlsbGlvbiBHcmFiIFJhaWwgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gMTYwIDI1NSBMIDEwMCAyNTUgTCAxMTAgMjY1IEwgMTYwIDI2NSBaIiBmaWxsPSIjNDU1YTY0IiAvPgogICAgICA8cGF0aCBkPSJNIDE2MCAyNTcgTCAxMDUgMjU3IEwgMTE1IDI2MiBMIDE2MCAyNjIgWiIgZmlsbD0iI2IwYmVjNSIgLz4gCiAgICAgIAogICAgICA8IS0tIFRhaWwgTGlnaHQgQXNzZW1ibHkgLS0+CiAgICAgIDxwb2x5Z29uIHBvaW50cz0iMTYwLDI2MCAxNzAsMjc1IDE0NSwyNzUiIGZpbGw9IiNmZjE3NDQiIGNsYXNzPSJnbG93LXB1bHNlIiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+CiAgICAgIDxwb2x5Z29uIHBvaW50cz0iMTU4LDI2MiAxNjUsMjcwIDE0OCwyNzAiIGZpbGw9IiNmZmZmZmYiIG9wYWNpdHk9IjAuOSIgLz4gCgogICAgICA8IS0tIEZyb250IEZvcmsgLS0+CiAgICAgIDxsaW5lIHgxPSI0ODAiIHkxPSIyMzAiIHgyPSI1NDAiIHkyPSI0MDAiIHN0cm9rZT0iIzA1MDUwNSIgc3Ryb2tlLXdpZHRoPSIxOCIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiAvPgogICAgICA8bGluZSB4MT0iNDgwIiB5MT0iMjMwIiB4Mj0iNTI1IiB5Mj0iMzUwIiBzdHJva2U9IiMzNzQ3NGYiIHN0cm9rZS13aWR0aD0iMjQiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KICAgICAgPGxpbmUgeDE9IjUyNSIgeTE9IjM1MCIgeDI9IjU0MCIgeTI9IjQwMCIgc3Ryb2tlPSIjY2ZkOGRjIiBzdHJva2Utd2lkdGg9IjEyIiBzdHJva2UtbGluZWNhcD0icm91bmQiIC8+CgogICAgICA8IS0tIEZyb250IEZlbmRlciAoQ2FyYm9uIEZpYmVyKSAtLT4KICAgICAgPHBhdGggZD0iTSA0ODUgMzU1IEwgNTc1IDM0NSBMIDYxMCAzODAgTCA1NjAgMzk1IEwgNTEwIDM3NSBaIiBmaWxsPSJ1cmwoI2NhcmJvbikiIHN0cm9rZT0iIzQ1NWE2NCIgc3Ryb2tlLXdpZHRoPSIxLjUiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTAwIDM2NSBMIDU4NSAzNjAiIHN0cm9rZT0idXJsKCNuZW9uR3JlZW4pIiBzdHJva2Utd2lkdGg9IjMuNSIgLz4KCiAgICAgIDwhLS0gSGVhZGxpZ2h0IEFzc2VtYmx5IC0tPgogICAgICA8ZyBpZD0iaGVhZGxpZ2h0LWFzc2VtYmx5Ij4KICAgICAgICA8cGF0aCBkPSJNIDQ4MCAyNjUgTCA1NTUgMjg1IEwgNTQwIDMyMCBMIDQ3MCAyOTUgWiIgZmlsbD0iIzAwMDAwMCIgLz4KICAgICAgICA8cGF0aCBkPSJNIDQ4NSAyNzAgTCA1NDggMjg3IEwgNTM2IDMxNSBMIDQ3NiAyOTUgWiIgZmlsbD0iIzExMTExMSIgc3Ryb2tlPSIjNTQ2ZTdhIiBzdHJva2Utd2lkdGg9IjEuNSIgc3Ryb2tlLWxpbmVqb2luPSJyb3VuZCIgLz4KICAgICAgICA8cGF0aCBkPSJNIDQ4NSAyNzAgTCA1NDggMjg3IEwgNTM2IDMxNSIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjYjJlYmYyIiBzdHJva2Utd2lkdGg9IjQuNSIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBjbGFzcz0iZ2xvdy1wdWxzZSIgZmlsdGVyPSJ1cmwoI2dsb3cpIiAvPgogICAgICAgIAogICAgICAgIDxlbGxpcHNlIGN4PSI1MDgiIGN5PSIyODYiIHJ4PSI2LjUiIHJ5PSIxMyIgZmlsbD0iI2ZmZmZmZiIgdHJhbnNmb3JtPSJyb3RhdGUoMTUgNTA4IDI4NikiIGNsYXNzPSJnbG93LXB1bHNlIiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+CiAgICAgICAgPGVsbGlwc2UgY3g9IjUwOCIgY3k9IjI4NiIgcng9IjIuNSIgcnk9IjYiIGZpbGw9IiNlMGY3ZmEiIHRyYW5zZm9ybT0icm90YXRlKDE1IDUwOCAyODYpIiAvPiAKICAgICAgICAKICAgICAgICA8ZWxsaXBzZSBjeD0iNTI1IiBjeT0iMjkxIiByeD0iNSIgcnk9IjEwIiBmaWxsPSIjZmZmZmZmIiB0cmFuc2Zvcm09InJvdGF0ZSgxNSA1MjUgMjkxKSIgY2xhc3M9Imdsb3ctcHVsc2UiIGZpbHRlcj0idXJsKCNnbG93KSIgLz4KICAgICAgICA8ZWxsaXBzZSBjeD0iNTI1IiBjeT0iMjkxIiByeD0iMiIgcnk9IjQiIGZpbGw9IiNlMGY3ZmEiIHRyYW5zZm9ybT0icm90YXRlKDE1IDUyNSAyOTEpIiAvPiAKCiAgICAgICAgPHBvbHlnb24gcG9pbnRzPSI0NzYsMjg1IDQ4NiwyOTAgNDgwLDMwMCIgZmlsbD0iI2ZmOTEwMCIgY2xhc3M9Imdsb3ctcHVsc2UiIGZpbHRlcj0idXJsKCNnbG93KSIgLz4KICAgICAgPC9nPgoKICAgICAgPCEtLSBEYXNoYm9hcmQgJiBIYW5kbGViYXJzIC0tPgogICAgICA8bGluZSB4MT0iNDM1IiB5MT0iMTcwIiB4Mj0iNDYwIiB5Mj0iMjI1IiBzdHJva2U9IiMwYTBhMGEiIHN0cm9rZS13aWR0aD0iMjYiIC8+CiAgICAgIAogICAgICA8IS0tIEJyYWtlIExpbmVzICYgQ2FibGVzIC0tPgogICAgICA8cGF0aCBkPSJNIDQwMCAxNDUgUSA0NjAgMjEwIDQ1MCAyNTAiIGZpbGw9Im5vbmUiIHN0cm9rZT0iIzExMSIgc3Ryb2tlLXdpZHRoPSIzIiBzdHJva2UtbGluZWNhcD0icm91bmQiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNDUwIDEyNSBRIDQ4MCAxODAgNDcwIDI0MCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjMTExIiBzdHJva2Utd2lkdGg9IjIuNSIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiAvPgoKICAgICAgPCEtLSBIZWFkc2V0IENvd2wgLS0+CiAgICAgIDxwYXRoIGQ9Ik0gNDIwIDEzNSBMIDQ4NSAxNTUgTCA0NjUgMjAwIEwgNDAwIDE4MCBaIiBmaWxsPSJ1cmwoI2JvZHlNYWluKSIgc3Ryb2tlPSJyZ2JhKDI1NSwyNTUsMjU1LDAuNCkiIHN0cm9rZS13aWR0aD0iMS41IiAvPgogICAgICA8cGF0aCBkPSJNIDQyMCAxMzUgTCA0NjUgMTUwIEwgNDMwIDE2NSBaIiBmaWxsPSIjZmZmZmZmIiBvcGFjaXR5PSIwLjYiIC8+IAogICAgICA8bGluZSB4MT0iNDY1IiB5MT0iMTc1IiB4Mj0iNDgwIiB5Mj0iMTg1IiBzdHJva2U9IiNiMmViZjIiIHN0cm9rZS13aWR0aD0iNCIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBjbGFzcz0iZ2xvdy1wdWxzZSIgZmlsdGVyPSJ1cmwoI2dsb3cpIiAvPgogICAgICA8cG9seWdvbiBwb2ludHM9IjQ4NSwxNTUgNDcwLDEzMCA0MTUsMTM1IDQzMCwxNTAiIGZpbGw9IiMxMTEiIC8+CiAgICAgIAogICAgICA8IS0tIEhPTE9HUkFQSElDIERJQUdOT1NUSUMgSFVEIC0tPgogICAgICA8ZyBpZD0iaG9sb2dyYW0iIHRyYW5zZm9ybT0idHJhbnNsYXRlKDAsLTE1KSI+CiAgICAgICAgPHBvbHlnb24gcG9pbnRzPSI0MzAsMTQwIDQ2MCwxNDAgNDkwLDQwIDM3MCw1MCIgZmlsbD0idXJsKCNob2xvQmVhbSkiIGNsYXNzPSJob2xvLWZsaWNrZXIiIC8+CiAgICAgICAgPGcgY2xhc3M9ImhvbG8tZmxpY2tlciIgZmlsdGVyPSJ1cmwoI2dsb3cpIj4KICAgICAgICAgIDwhLS0gVGV4dCBpcyBjb3VudGVyLWZsaXBwZWQgbWF0aGVtYXRpY2FsbHkgLS0+CiAgICAgICAgICA8dGV4dCB4PSItNDY1IiB5PSI2NSIgZm9udC1mYW1pbHk9Im1vbm9zcGFjZSIgZm9udC1zaXplPSIxMiIgZm9udC13ZWlnaHQ9ImJvbGQiIGZpbGw9IiMwMGU2NzYiIHRyYW5zZm9ybT0ic2NhbGUoLTEsIDEpIj5DQU46IEFDVElWRTwvdGV4dD4KICAgICAgICAgIDx0ZXh0IHg9Ii00NjgiIHk9IjgwIiBmb250LWZhbWlseT0ibW9ub3NwYWNlIiBmb250LXNpemU9IjEwIiBmb250LXdlaWdodD0iYm9sZCIgZmlsbD0iIzFkZTliNiIgdHJhbnNmb3JtPSJzY2FsZSgtMSwgMSkiPjB4MUEyIFs0RiBBMV08L3RleHQ+CiAgICAgICAgICA8dGV4dCB4PSItNDczIiB5PSI5NSIgZm9udC1mYW1pbHk9Im1vbm9zcGFjZSIgZm9udC1zaXplPSIxMCIgZm9udC13ZWlnaHQ9ImJvbGQiIGZpbGw9IiMxZGU5YjYiIHRyYW5zZm9ybT0ic2NhbGUoLTEsIDEpIj4weDJCNCBbMDAgMDBdPC90ZXh0PgogICAgICAgICAgPHBvbHlsaW5lIHBvaW50cz0iMzcwLDEwNSA0ODAsMTAwIiBmaWxsPSJub25lIiBzdHJva2U9IiMwMGU2NzYiIHN0cm9rZS13aWR0aD0iMS41IiBzdHJva2UtZGFzaGFycmF5PSIyLDQiIC8+CiAgICAgICAgPC9nPgogICAgICA8L2c+CgogICAgICA8IS0tIEF0aGVyICJIYWxvIiBEYXNoYm9hcmQgVUkgLS0+CiAgICAgIDxnIHRyYW5zZm9ybT0ic2tld1goLTIwKSB0cmFuc2xhdGUoLTUsIC04KSI+CiAgICAgICAgPHJlY3QgeD0iNDIwIiB5PSIxMTUiIHdpZHRoPSI2MiIgaGVpZ2h0PSI2OCIgZmlsbD0iIzBhMGEwYSIgcng9IjYiIHN0cm9rZT0iIzQ1NWE2NCIgc3Ryb2tlLXdpZHRoPSIxLjUiIC8+CiAgICAgICAgPHJlY3QgeD0iNDIzIiB5PSIxMTgiIHdpZHRoPSI1NiIgaGVpZ2h0PSI2MiIgZmlsbD0iIzAwMTgyOSIgcng9IjQiIC8+IAogICAgICAgIDxwb2x5Z29uIHBvaW50cz0iNDIzLDExOCA0NTAsMTE4IDQzNSwxNzggNDIzLDE3OCIgZmlsbD0iI2ZmZmZmZiIgb3BhY2l0eT0iMC4wNSIgLz4KICAgICAgICAKICAgICAgICA8ZyBjbGFzcz0iZ2xvdy1wdWxzZSI+CiAgICAgICAgICA8Y2lyY2xlIGN4PSI0NTEiIGN5PSIxNDgiIHI9IjE2IiBmaWxsPSJub25lIiBzdHJva2U9IiMwMGU2NzYiIHN0cm9rZS13aWR0aD0iNCIgLz4KICAgICAgICAgIDxsaW5lIHgxPSI0NDUiIHkxPSIxNDgiIHgyPSI0NTciIHkyPSIxNDgiIHN0cm9rZT0iI2ZmZmZmZiIgc3Ryb2tlLXdpZHRoPSI0IiBzdHJva2UtbGluZWNhcD0icm91bmQiIC8+CiAgICAgICAgICA8cG9seWxpbmUgcG9pbnRzPSI0MjgsMTcwIDQzNSwxNjAgNDQyLDE3MCA0NTAsMTU1IiBmaWxsPSJub25lIiBzdHJva2U9IiMwMGI0ZDgiIHN0cm9rZS13aWR0aD0iMi41IiBzdHJva2UtbGluZWpvaW49InJvdW5kIiAvPgogICAgICAgICAgPGxpbmUgeDE9IjQyOCIgeTE9IjEyOCIgeDI9IjQ0MCIgeTI9IjEyOCIgc3Ryb2tlPSIjZmY5MTAwIiBzdHJva2Utd2lkdGg9IjMiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KICAgICAgICAgIDxyZWN0IHg9IjQ2OCIgeT0iMTI1IiB3aWR0aD0iNiIgaGVpZ2h0PSI2IiBmaWxsPSIjZmYxNzQ0IiByeD0iMS41IiBmaWx0ZXI9InVybCgjZ2xvdykiIC8+IAogICAgICAgIDwvZz4KICAgICAgPC9nPgoKICAgICAgPCEtLSBIYW5kbGViYXJzICYgR3JpcHMgLS0+CiAgICAgIDxyZWN0IHg9IjM4MCIgeT0iMTM1IiB3aWR0aD0iMTE1IiBoZWlnaHQ9IjE1IiBmaWxsPSIjMWExYTFhIiByeD0iNy41IiB0cmFuc2Zvcm09InJvdGF0ZSgtMTUgNDMwIDE0MikiIC8+CiAgICAgIDxyZWN0IHg9IjM2NSIgeT0iMTMzIiB3aWR0aD0iNDAiIGhlaWdodD0iMTkiIGZpbGw9IiMwYTBhMGEiIHJ4PSI0IiB0cmFuc2Zvcm09InJvdGF0ZSgtMTUgMzg1IDE0MikiIC8+CiAgICAgIDxsaW5lIHgxPSIzNjUiIHkxPSIxNDEiIHgyPSIzMjUiIHkyPSIxNTUiIHN0cm9rZT0iI2IwYmVjNSIgc3Ryb2tlLXdpZHRoPSI1LjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgLz4KICAgICAgCiAgICAgIDwhLS0gTWlycm9ycyAtLT4KICAgICAgPGxpbmUgeDE9IjQwMCIgeTE9IjEzMCIgeDI9IjM3NSIgeTI9IjY1IiBzdHJva2U9IiMxMTEiIHN0cm9rZS13aWR0aD0iNSIgLz4KICAgICAgPHBvbHlnb24gcG9pbnRzPSIzNjUsNjUgMzgwLDQ1IDQwNSw1NSAzOTAsNzUiIGZpbGw9IiMxYTFhMWEiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIxIiAvPgogICAgICA8cG9seWdvbiBwb2ludHM9IjM2OCw2MyAzNzgsNTAgMzk5LDU4IDM4OSw3MSIgZmlsbD0iI2UwZjdmYSIgLz4KICAgICAgCiAgICAgIDxsaW5lIHgxPSI0NjUiIHkxPSIxMTUiIHgyPSI0OTAiIHkyPSI1MCIgc3Ryb2tlPSIjMTExIiBzdHJva2Utd2lkdGg9IjUiIC8+CiAgICAgIDxwb2x5Z29uIHBvaW50cz0iNDgwLDUwIDQ5NSwzMCA1MjAsNDAgNTA1LDYwIiBmaWxsPSIjMWExYTFhIiBzdHJva2U9IiMzNzQ3NGYiIHN0cm9rZS13aWR0aD0iMSIgLz4KICAgICAgPHBvbHlnb24gcG9pbnRzPSI0ODMsNDggNDkzLDM1IDUxNCw0MyA1MDQsNTYiIGZpbGw9IiNlMGY3ZmEiIC8+CiAgICAgIAogICAgPC9nPgoKICAgIDwhLS0gUkVBUiBXSEVFTCAtLT4KICAgIDxnIGlkPSJyZWFyLXdoZWVsIiBjbGFzcz0id2hlZWwgc3BpbiI+CiAgICAgIDxjaXJjbGUgY3g9IjIxMCIgY3k9IjQwMCIgcj0iNzIiIGZpbGw9InVybCgjdGlyZUdyYWRpZW50KSIgLz4KICAgICAgPGNpcmNsZSBjeD0iMjEwIiBjeT0iNDAwIiByPSI2OSIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjMzc0NzRmIiBzdHJva2Utd2lkdGg9IjEuNSIgLz4gCiAgICAgIDxjaXJjbGUgY3g9IjIxMCIgY3k9IjQwMCIgcj0iNjQiIGZpbGw9Im5vbmUiIHN0cm9rZT0iIzExMSIgc3Ryb2tlLXdpZHRoPSI2IiBzdHJva2UtZGFzaGFycmF5PSIxMCw1IiAvPiAKICAgICAgCiAgICAgIDxjaXJjbGUgY3g9IjIxMCIgY3k9IjQwMCIgcj0iNDciIGZpbGw9IiNjZmQ4ZGMiIC8+CiAgICAgIDxjaXJjbGUgY3g9IjIxMCIgY3k9IjQwMCIgcj0iNDAiIGZpbGw9IiM3ODkwOWMiIC8+CiAgICAgIDxjaXJjbGUgY3g9IjIxMCIgY3k9IjQwMCIgcj0iMzIiIGZpbGw9IiMwYTBhMGEiIC8+CiAgICAgIAogICAgICA8cGF0aCBkPSJNIDIwMiAzNTMgTCAyMTggMzUzIEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICA8cGF0aCBkPSJNIDI0OSAzNzEgTCAyNTkgMzgwIEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICA8cGF0aCBkPSJNIDI0OSA0MjkgTCAyNTkgNDIwIEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICA8cGF0aCBkPSJNIDIwMiA0NDcgTCAyMTggNDQ3IEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICA8cGF0aCBkPSJNIDE3MSA0MjkgTCAxNjEgNDIwIEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICA8cGF0aCBkPSJNIDE3MSAzNzEgTCAxNjEgMzgwIEwgMjEwIDQwMCBaIiBmaWxsPSIjMjYzMjM4IiAvPgogICAgICAKICAgICAgPGNpcmNsZSBjeD0iMjEwIiBjeT0iNDAwIiByPSIyNCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjYjBiZWM1IiBzdHJva2Utd2lkdGg9IjUiIHN0cm9rZS1kYXNoYXJyYXk9IjYsMyIgLz4KICAgICAgPGNpcmNsZSBjeD0iMjEwIiBjeT0iNDAwIiByPSI5IiBmaWxsPSJ1cmwoI25lb25HcmVlbikiIC8+CiAgICA8L2c+CgogICAgPCEtLSBGUk9OVCBXSEVFTCAtLT4KICAgIDxnIGlkPSJmcm9udC13aGVlbCIgY2xhc3M9IndoZWVsIHNwaW4iPgogICAgICA8Y2lyY2xlIGN4PSI1NjAiIGN5PSI0MDAiIHI9IjcyIiBmaWxsPSJ1cmwoI3RpcmVHcmFkaWVudCkiIC8+CiAgICAgIDxjaXJjbGUgY3g9IjU2MCIgY3k9IjQwMCIgcj0iNjkiIGZpbGw9Im5vbmUiIHN0cm9rZT0iIzM3NDc0ZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIC8+CiAgICAgIDxjaXJjbGUgY3g9IjU2MCIgY3k9IjQwMCIgcj0iNjQiIGZpbGw9Im5vbmUiIHN0cm9rZT0iIzExMSIgc3Ryb2tlLXdpZHRoPSI2IiBzdHJva2UtZGFzaGFycmF5PSIxMCw1IiAvPgogICAgICAKICAgICAgPGNpcmNsZSBjeD0iNTYwIiBjeT0iNDAwIiByPSI0NyIgZmlsbD0iI2NmZDhkYyIgLz4KICAgICAgPGNpcmNsZSBjeD0iNTYwIiBjeT0iNDAwIiByPSI0MCIgZmlsbD0iIzc4OTA5YyIgLz4KICAgICAgPGNpcmNsZSBjeD0iNTYwIiBjeT0iNDAwIiByPSIxNiIgZmlsbD0iIzBhMGEwYSIgLz4KICAgICAgCiAgICAgIDxwYXRoIGQ9Ik0gNTUyIDM1MyBMIDU2OCAzNTMgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTk5IDM3MSBMIDYwOSAzODAgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTk5IDQyOSBMIDYwOSA0MjAgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTUyIDQ0NyBMIDU2OCA0NDcgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTIxIDQyOSBMIDUxMSA0MjAgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIDxwYXRoIGQ9Ik0gNTIxIDM3MSBMIDUxMSAzODAgTCA1NjAgNDAwIFoiIGZpbGw9IiMyNjMyMzgiIC8+CiAgICAgIAogICAgICA8Y2lyY2xlIGN4PSI1NjAiIGN5PSI0MDAiIHI9IjMyIiBmaWxsPSJub25lIiBzdHJva2U9IiNiMGJlYzUiIHN0cm9rZS13aWR0aD0iNyIgc3Ryb2tlLWRhc2hhcnJheT0iOCw0IiAvPgogICAgICA8cmVjdCB4PSI1MzIiIHk9IjM2NSIgd2lkdGg9IjIwIiBoZWlnaHQ9IjM4IiBmaWxsPSIjZmYxNzQ0IiByeD0iNCIgdHJhbnNmb3JtPSJyb3RhdGUoMzAgNTMyIDM2NSkiIC8+IAogICAgICA8Y2lyY2xlIGN4PSI1NjAiIGN5PSI0MDAiIHI9IjkiIGZpbGw9InVybCgjbmVvbkdyZWVuKSIgLz4KICAgIDwvZz4KCiAgPC9nPgo8L3N2Zz4=\" alt=\"Ather electric scooter\" width=\"786\" height=\"736\">\n"
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
"  {id:'diagnostic', label:'Diagnostic', cls:'diagnostic', desc:'Segment-specific telemetry, recording, analysis and JSON export.', icon:'<path d=\"M14.7 6.3a4 4 0 0 0-5.4 5.4l-5.6 5.6a2 2 0 0 0 2.8 2.8l5.6-5.6a4 4 0 0 0 5.4-5.4l-2.4 2.4-2.8-2.8z\"/> '},\n"
"  {id:'about', label:'About', cls:'others', desc:'Project information, contributors, repository, suggestions, and ways to contribute.', icon:'<circle cx=\"12\" cy=\"12\" r=\"9\"/><path d=\"M12 11v5\"/><circle cx=\"12\" cy=\"7.5\" r=\".7\" fill=\"currentColor\" stroke=\"none\"/> '}\n"
"];\n"
"\n"
"const telemetryGroups = {\n"
"  vehicle: [\n"
"    {title:'Vehicle identification & ride data', symbol:'🛵', accent:'#4778d0', fields:[\n"
"      ['Vehicle identification number','🪪','—','','connect-charge'],['Scooter ID','🔖','—','','connect-charge'],['Odometer','🛣️','—','km'],['Estimated range','🧭','—','km','estimated'],['Drive mode','⚙️','—',''],['Peak power','⚡','—','kW']\n"
"    ]},\n"
"    {title:'Switches & rider controls', symbol:'🎛️', accent:'#4778d0', fields:[\n"
"      ['Key switch','🔑','—',''],['Start switch','▶️','—',''],['Kill switch','⏹️','—',''],['Storage switch','🧰','—',''],['Parking assistant','🅿️','—',''],['Nav joystick','🕹️','—','','hypothesis'],['Front brake','🖐️','—',''],['Rear brake','🖐️','—',''],['High beam','💡','—',''],['Horn','📣','—',''],['Left indicator','⬅️','—',''],['Right indicator','➡️','—',''],['Center indicator','🔘','—',''],['Indicators','↔️','—',''],['Blinkers','🚦','—',''],['Side stand','🦶','—',''],['Seat lock','🔒','—','']\n"
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
"      ['Pack state of charge','🔋','—','%'],['Minimum-voltage estimate','📉','—','V','estimated'],['Cell voltage imbalance','⚖️','—','mV','estimated'],['Delta SoC','↔️','—','%'],['Cell temperature Probe 1','🌡️','—','°C'],['Cell temperature Probe 2','🌡️','—','°C'],['Cell temperature Probe 3','🌡️','—','°C'],['Cell temperature Probe 4','🌡️','—','°C'],['Cell temperature Probe 5','🌡️','—','°C','rarely-seen']"
"    ]}\n"
"  ],\n"
"  bms: ["
"    {title:'Battery pack health', symbol:'🔋', accent:'#8b5cc7', fields:["
"      ['Pack state of charge','🔋','—','%'],['Pack state of health','🩺','—','%'],['Delta SoC','↔️','—','%'],['Pack voltage','⚡','—','V'],['Pack current','〰️','—','A'],['Balancing state','⚖️','—',''],['State of Energy','🔌','—','%'],['EKF SoC estimate','🧮','—','%'],['Minimum SoC','📉','—','%'],['Maximum SoC','📈','—','%'],['Minimum cell voltage','📉','—','V','estimated'],['Maximum cell voltage','📈','—','V','estimated']"
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
"      ['BMS firmware','🔋','—',''],['BCM firmware','🧠','—','rarely-seen'],['Gateway firmware','🌐','—',''],['Motor firmware','⚙️','—',''],['BMS hardware version','🧩','—','']\n"
"    ]},\n"
"    {title:'Diagnostic tool & CAN health', symbol:'🩺', accent:'#2383ad', fields:[\n"
"      ['Diagnostic tool activity','🩺','—',''],['CAN RX missed frames','📥','—','count'],['CAN RX overrun count','⚠️','—','count'],['Arbitration lost count','🔀','—','count'],['Maximum queue backlog','📚','—','count'],['Free heap','🧠','—','bytes'],['Wi-Fi AP status','📶','—','']\n"
"    ]}\n"
"  ],\n"
"  thermals: [\n"
"    {title:'Battery & BMS temperatures', symbol:'🌡️', accent:'#c64b68', fields:[\n"
"      ['Cell temperature Probe 1','🌡️','—','°C'],['Cell temperature Probe 2','🌡️','—','°C'],['Cell temperature Probe 3','🌡️','—','°C'],['Cell temperature Probe 4','🌡️','—','°C'],['Cell temperature Probe 5','🌡️','—','°C','rarely-seen'],['AFE die temperature','🧠','—','°C'],['BMS ambient temperature','🌡️','—','°C'],['Humidity','💧','—','%'],['FET temperature','🔥','—','°C','hypothesis'],['ECI temperature','🔥','—','°C','hypothesis'],['Sense-resistor temperature','🌡️','—','°C','hypothesis']\n"
"    ]},\n"
"    {title:'Drive unit & thermal limits', symbol:'🛞', accent:'#c64b68', fields:[\n"
"      ['Motor temperature','⚙️','—','°C'],['Controller temperature','🧩','—','°C'],['Motor MCU temperature','🧠','—','°C'],['Temperature-related discharge restriction','⚠️','—','']\n"
"    ]}\n"
"  ],\n"
"  about: []\n"
"};\n"
"\n"
"function renderTelemetry(cat) {\n"
"  if (cat.id === 'diagnostic') { renderDiagnostic(); return; }\n"
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
"      const tagLabel = tag === 'estimated' ? 'Estimated' : tag === 'hypothesis' ? 'Hypothesis / Verify' : tag === 'rarely-seen' ? 'Rarely seen' : tag === 'connect-charge' ? 'Connect Charger' : '';\n"
"      const tagMarkup = tagLabel ? `<span class=\"field-tag ${tag}\">${tagLabel}</span>` : '';\n"
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
"      tableSection.innerHTML = `<h3 class=\"group-title\"><span class=\"group-symbol\" style=\"background:#23945d18\">🧪</span>Individual cell details From C1–C14</h3>\n"
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
" 'Left indicator':sw(d.indl,'Active','Inactive'),'Right indicator':sw(d.indr,'Active','Inactive'),'Center indicator':sw(d.indc,'Active','Inactive'),\n"
" 'Indicators':(d.indl||d.indr||d.indc)?'Active':'Inactive','Blinkers':(d.indl||d.indr||d.indc)?'Active':'Inactive',\n"
" 'Side stand':sw(d.stand,'Down','Up'),'Seat lock':sw(ch.seat,'Locked','Unlocked'),\n"
" 'IMU acceleration X':e.ax,'IMU acceleration Y':e.ay,'IMU acceleration Z':e.az,\n"
" 'Roll':e.er!==undefined?f(e.er/10000,4)+' °':undefined,'Pitch':e.ep!==undefined?f(e.ep/10000,4)+' °':undefined,\n"
" 'Heading':e.eh!==undefined?f(e.eh/10000,4)+' °':undefined,\n"
" 'Pack state of charge':d.soc!==undefined?f(d.soc,2)+' %':undefined,\n"
" 'Pack state of health':d.soh!==undefined?f(d.soh,2)+' %':undefined,\n"
" 'Delta SoC':d.bdelta!==undefined?f(d.bdelta,2)+' %':undefined,\n"
" 'Minimum-voltage estimate':e.v1!==undefined?V(Number(e.v1)/10000,4):undefined,\n"
" 'Minimum cell voltage':e.v1!==undefined?V(Number(e.v1)/10000,4):undefined,\n"
" 'Maximum cell voltage':e.v2!==undefined?V(Number(e.v2)/10000,4):undefined,\n"
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
" 'Minimum SoC':P(e.minsoc),'Maximum SoC':P(e.msoc),\n"
" 'Derated discharge current':Number.isFinite(Number(e.dd))?f(Number(e.dd)/100,2)+' A':undefined,'Derated charge current':Number.isFinite(Number(e.dc))?f(Number(e.dc)/100,2)+' A':undefined,'Derated regenerative current':Number.isFinite(Number(e.dr))?f(Number(e.dr)/100,2)+' A':undefined,\n"
" 'Temperature-based discharge limit':Number.isFinite(Number(e.dl))?f(Number(e.dl)/100,2)+' A':undefined,\n"
" 'Temperature-related discharge restriction':Number.isFinite(Number(e.dl))?f(Number(e.dl)/100,2)+' A':undefined,\n"
" 'Cell rule charge limit':e.cr!==undefined?f(Number(e.cr)/100,2)+' A':undefined,\n"
" 'Rate-limited charge current':e.crl!==undefined?f(Number(e.crl)/100,2)+' A':undefined,\n"
" 'Post-quantizer charge value':e.cpq!==undefined?f(Number(e.cpq)/100,2)+' A':undefined,\n"
" 'Charge algorithm limits':e.cr!==undefined?`Cell rule ${f(e.cr/100,2)} · rate limited ${f(e.crl/100,2)} · post-quantizer ${f(e.cpq/100,2)}`:undefined,\n"
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
"   if(selected==='diagnostic'){diagRenderMetrics();diagUpdateRecorderUI();}\n"

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
"let scooterAnimatedSrc = null;\n"
"let scooterStaticSrc = null;\n"
"function setScooterAnimationState(nextMode) {\n"
"  const img = document.querySelector('.scooter-reference');\n"
"  if (!img) return;\n"
"  if (!scooterAnimatedSrc) {\n"
"    scooterAnimatedSrc = img.getAttribute('src');\n"
"    try {\n"
"      const svgText = atob(scooterAnimatedSrc.split(',')[1]);\n"
"      const stopAnimations = ' *,*::before,*::after{animation:none!important;transition:none!important;caret-color:transparent!important} ';\n"
"      const staticSvg = svgText.replace('</style>', stopAnimations + '</style>');\n"
"      scooterStaticSrc = 'data:image/svg+xml;base64,' + btoa(staticSvg);\n"
"    } catch (e) {\n"
"      scooterStaticSrc = scooterAnimatedSrc;\n"
"    }\n"
"  }\n"
"  const wantedSrc = nextMode === 'offline' ? scooterStaticSrc : scooterAnimatedSrc;\n"
"  if (wantedSrc && img.getAttribute('src') !== wantedSrc) img.setAttribute('src', wantedSrc);\n"
"}\n"
"function updateStatus(nextMode) {\n"
"  mode = nextMode;\n"
"  statusBar.className = 'status ' + mode + (selected ? ' submenu-mode' : '');\n"
"  const scooterArea = document.querySelector('.scooter-area');\n"
"  if (scooterArea) scooterArea.className = 'scooter-area ' + mode;\n"
"  setScooterAnimationState(mode);\n"
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
"  if (mode === 'offline' && selected && !(diagSession && !diagSession.finalized)) closeCategory();\n"
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
"\n"
"\n"
"\n"
"\n"
"\n"
"const diagSegments = [\n"
" {id:'activity',name:'Vehicle & Button Activity',goal:'What the rider/system did, and how often.',profile:'activity'},\n"
" {id:'battery',name:'Battery Health & BMS',goal:'Pack condition, cell balance, temperature and electrical stress.',profile:'battery'},\n"
" {id:'motor',name:'Motor & Controller',goal:'Drivetrain load, throttle agreement and thermal behavior.',profile:'motor'},\n"
" {id:'charger',name:'Charger Performance',goal:'Charge behavior, power delivery and charge-limit response.',profile:'charger'},\n"
" {id:'ride',name:'RideCheck',goal:'Cross-section of ride telemetry, braking, motion and energy use.',profile:'ride'}\n"
"];\n"
"const diagProfiles = [\n"
" {id:'quick_check',name:'Quick Check',minutes:5,desc:'Short focused capture of core segment signals'},\n"
" {id:'snapshot',name:'Snapshot',minutes:15,desc:'Core metrics and compact summaries'},\n"
" {id:'drive_cycle',name:'Drive Cycle',minutes:30,desc:'Core metrics plus operating trends'}\n"
"];\n"
"let diagActiveSegment = 'activity';\n"
"let diagSession = null;\n"
"let diagSavedSessions = {};\n"
"let diagTimer = null;\n"
"let diagLastSampleMs = 0;\n"
"let diagTransitionState = {};\n"
"const diagTransitionStats = {};\n"
"const diagMetricNames = {\n"
" fbrake:'Front brake',rbrake:'Rear brake',hbeam:'High beam',start:'Start switch',kill:'Kill switch',\n"
" storage:'Storage switch',horn:'Horn',indl:'Left indicator',indr:'Right indicator',indc:'Center indicator',\n"
" key:'Key ON',stand:'Side stand',dfan:'Display fan',seat:'Seat lock',feco:'Forced ECO',seco:'Smart ECO'\n"
"};\n"
"function diagVal(v){return v===undefined||v===null||v===''?'Unavailable':v;}\n"
"function diagNum(v){const n=Number(v);return Number.isFinite(n)?n:null;}\n"
"function diagFmt(v,unit=''){const n=diagNum(v);return n===null?'Unavailable':(Number.isInteger(n)?String(n):n.toFixed(2))+unit;}\n"
"function diagGet(){\n"
" const d=liveData.data||{},c=liveData.cells||{},ch=liveData.charger||{},x=liveData.extra||{};\n"
" return {d,c,ch,x};\n"
"}\n"
"function diagMetric(label,value,unit=''){\n"
" const n=diagNum(value);\n"
" const display=(value===undefined||value===null||value==='')?'Unavailable':(typeof value==='boolean'?(value?'Active':'Inactive'):(n!==null?diagFmt(n,unit):String(value)));\n"
" return `<div class=\\\"diag-metric\\\"><span>${label}</span><strong>${display}</strong></div>`;\n"
"}\n"
"function diagCurrentValues(id){\n"
" const {d,c,ch,x}=diagGet();\n"
" const cells=Array.isArray(c.v)?c.v.map(Number).filter(Number.isFinite):[];\n"
" const cellSoh=Array.isArray(c.h)?c.h.map(Number).filter(Number.isFinite):[];\n"
" const cellSoc=Array.isArray(x.cs)?x.cs.map(Number).filter(Number.isFinite):[];\n"
" const temps=Array.isArray(x.ct)?x.ct.map(Number).filter(Number.isFinite):[];\n"
" const minCell=cells.length?Math.min(...cells):null, maxCell=cells.length?Math.max(...cells):null;\n"
" const spread=cells.length?maxCell-minCell:null;\n"
" const minTemp=temps.length?Math.min(...temps):null,maxTemp=temps.length?Math.max(...temps):null;\n"
" const values={\n"
"  fbrake:d.fbrake,rbrake:d.rbrake,hbeam:d.hbeam,start:d.start,kill:d.kill,storage:d.storage,horn:d.horn,\n"
"  indl:d.indl,indr:d.indr,indc:d.indc,key:d.key,stand:d.stand,dfan:d.dfan,feco:d.feco,seco:d.seco,\n"
"  mode:d.mode,rpm:d.rpm,odo:diagNum(x.odo)===null?null:Number(x.odo)/1000,bmsUptime:x.up,canAgeMs:d.canAgeMs,diagAgeMs:x.diagage,\n"
"  vinAgeMs:x.vinage,sidAgeMs:x.sidage,gatewayAgeMs:x.gtwage,vin:x.vin,scooterId:x.sid,navJoystick:x.nav,parkAssistA:x.pka,parkAssistB:x.pkb,firmware:{bms:x.bfw,bcm:x.cfw,gateway:x.gfw,motor:x.mfw},\n"
"  soc:d.soc,soh:d.soh,deltaSoc:d.bdelta,socEkf:diagNum(x.sekf)===null?null:Number(x.sekf)/100,soe:diagNum(x.soe)===null?null:Number(x.soe)/100,maxSoc:diagNum(x.msoc)===null?null:Number(x.msoc)/100,minSoc:diagNum(x.minsoc)===null?null:Number(x.minsoc)/100,\n"
"  packVoltage:d.volt,packCurrent:diagNum(x.ci)===null?null:Number(x.ci)/100,imbalance:d.imb,balancing:ch.bal,\n"
"  cellVoltageMin:minCell,cellVoltageMax:maxCell,cellVoltageSpread:spread,\n"
"  weakestCellIndex:cells.length?cells.indexOf(minCell)+1:null,strongestCellIndex:cells.length?cells.indexOf(maxCell)+1:null,\n"
"  cellSohMin:cellSoh.length?Math.min(...cellSoh):null,cellSohMax:cellSoh.length?Math.max(...cellSoh):null,\n"
"  cellSocMin:cellSoc.length?Math.min(...cellSoc)/100:null,cellSocMax:cellSoc.length?Math.max(...cellSoc)/100:null,\n"
"  cellTempMin:temps.length?minTemp/100:null,cellTempMax:temps.length?maxTemp/100:null,cellTempDelta:temps.length?(maxTemp-minTemp)/100:null,\n"
"  cellTemps:temps.map(n=>n/100),afeDieTemp:diagNum(x.die)===null?null:Number(x.die)/100,bmsAmbient:diagNum(x.amb)===null?null:Number(x.amb)/100,humidity:diagNum(x.h)===null?null:Number(x.h)/100,boardTempRefV:diagNum(x.btr)===null?null:Number(x.btr)/1000,vref2:diagNum(x.vr2)===null?null:Number(x.vr2)/10000,vref3:diagNum(x.vr3)===null?null:Number(x.vr3)/10000,mosfetPowerVoltage:diagNum(x.mos)===null?null:Number(x.mos)/1000,afeRegulator:diagNum(x.vreg)===null?null:Number(x.vreg)/1000,pduVoltage:diagNum(x.pdu)===null?null:Number(x.pdu)/1000,\n"
"  deratedDischarge:diagNum(x.dd)===null?null:Number(x.dd)/100,deratedCharge:diagNum(x.dc)===null?null:Number(x.dc)/100,deratedRegen:diagNum(x.dr)===null?null:Number(x.dr)/100,dischargeLimitTemp:diagNum(x.dl)===null?null:Number(x.dl)/100,\n"
"  motorCurrent:ch.mbi,motorTemp:ch.mt,controllerTemp:x.ctemp,motorMcuTemp:x.mcutemp,throttleOut:x.tho,throttle1:x.th1,throttle3:x.th3,mcRpm:x.mcrpm,tcRpm:x.tcrpm,\n"
"  chargerConnected:ch.cconnected,chargerInputVoltage:ch.cinv,chargerOutputVoltage:ch.coutv,chargerOutputCurrent:ch.couti,chargerPower:ch.watts,chargerFan1:ch.fan1,chargerFan2:ch.fan2,chargerLock1:ch.clock1,chargerLock2:ch.clock2,seatLock:ch.seat,chargeSocLimit:diagNum(x.chglim)===null?null:Number(x.chglim)/100,chargeCellRule:diagNum(x.cr)===null?null:Number(x.cr)/100,chargeCommandRateLimit:diagNum(x.crl)===null?null:Number(x.crl)/100,chargePostQuant:diagNum(x.cpq)===null?null:Number(x.cpq)/100,\n"
"  ax:x.ax,ay:x.ay,az:x.az,roll:x.er,pitch:x.ep,heading:x.eh\n"
" };\n"
" const discreteKeys=['fbrake','rbrake','hbeam','start','kill','storage','horn','indl','indr','indc','key','stand','dfan','feco','seco','chargerConnected','chargerLock1','chargerLock2','seatLock'];\n"
" discreteKeys.forEach(k=>{const raw=values[k];if(typeof raw==='string'){const t=raw.trim().toLowerCase();if(t==='true'||t==='1'||t==='on'||t==='active'||t==='pressed'||t==='connected'||t==='locked')values[k]=true;else if(t==='false'||t==='0'||t==='off'||t==='inactive'||t==='not pressed'||t==='disconnected'||t==='unlocked')values[k]=false;}else if(raw===0||raw===1)values[k]=raw===1;});\n"
" if(id==='activity') return values;\n"
" if(id==='battery') return values;\n"
" if(id==='motor') return values;\n"
" if(id==='charger') return values;\n"
" if(id==='ride') return values;\n"
" return values;\n"
"}\n"
"function diagSegmentMetricKeys(id){\n"
" if(id==='activity') return ['fbrake','rbrake','hbeam','start','kill','storage','horn','indl','indr','indc','key','stand','dfan','feco','seco','mode','rpm','odo','bmsUptime','canAgeMs','diagAgeMs','vinAgeMs','sidAgeMs','navJoystick','parkAssistA','parkAssistB','gatewayAgeMs'];\n"
" if(id==='battery') return ['soc','socEkf','soe','soh','deltaSoc','maxSoc','packVoltage','packCurrent','cellVoltageMin','cellVoltageMax','cellVoltageSpread','weakestCellIndex','strongestCellIndex','imbalance','balancing','cellSohMin','cellSohMax','cellSocMin','cellSocMax','cellTempMin','cellTempMax','cellTempDelta','afeDieTemp','bmsAmbient','humidity','deratedDischarge','deratedCharge','deratedRegen','dischargeLimitTemp','vref2','vref3','mosfetPowerVoltage','afeRegulator','pduVoltage'];\n"
" if(id==='motor') return ['rpm','mcRpm','tcRpm','motorCurrent','packCurrent','motorTemp','controllerTemp','motorMcuTemp','throttleOut','throttle1','throttle3','deratedDischarge','deratedRegen','soc','packVoltage','odo'];\n"
" if(id==='charger') return ['chargerConnected','chargerLock1','chargerLock2','seatLock','chargerInputVoltage','chargerOutputVoltage','chargerOutputCurrent','chargerPower','chargerFan1','chargerFan2','balancing','chargeSocLimit','deratedCharge','chargeCellRule','chargeCommandRateLimit','chargePostQuant','packVoltage','cellVoltageSpread','cellTempMin','cellTempMax','soc'];\n"
" return ['rpm','throttleOut','throttle1','throttle3','motorCurrent','packCurrent','packVoltage','soc','motorTemp','controllerTemp','motorMcuTemp','cellTempMin','cellTempMax','ax','ay','az','pitch','roll','odo','mode','fbrake','rbrake'];\n"
"}\n"
"function diagFieldLabel(k){\n"
" return ({soc:'SoC (%)',soh:'Reported SoH (%)',socEkf:'EKF SoC',soe:'SoE',deltaSoc:'Delta SoC',maxSoc:'Max SoC',packVoltage:'Pack voltage (V)',packCurrent:'Pack current',cellVoltageMin:'Cell voltage min (V)',cellVoltageMax:'Cell voltage max (V)',cellVoltageSpread:'Cell spread (V)',weakestCellIndex:'Weakest cell #',strongestCellIndex:'Strongest cell #',cellTempMin:'Cell temp min',cellTempMax:'Cell temp max',cellTempDelta:'Cell temp delta',afeDieTemp:'AFE die temp',bmsAmbient:'BMS ambient',humidity:'Humidity',deratedDischarge:'Derated discharge limit',deratedCharge:'Derated charge limit',deratedRegen:'Derated regen limit',dischargeLimitTemp:'Discharge limit temp',vref2:'VREF2',vref3:'VREF3',mosfetPowerVoltage:'MOSFET power voltage',afeRegulator:'AFE regulator',pduVoltage:'PDU voltage',rpm:'RPM',mcRpm:'MC RPM',tcRpm:'TC RPM',motorCurrent:'Motor current',motorTemp:'Motor temp',controllerTemp:'Controller temp',motorMcuTemp:'Motor MCU temp',throttleOut:'Throttle output',throttle1:'Throttle 1',throttle3:'Throttle 3',chargerConnected:'Charger connected',chargerInputVoltage:'Input voltage',chargerOutputVoltage:'Output voltage',chargerOutputCurrent:'Output current',chargerPower:'Charger power',chargerFan1:'Charger fan 1',chargerFan2:'Charger fan 2',chargerLock1:'Charger lock 1',chargerLock2:'Charger lock 2',seatLock:'Seat lock',chargeSocLimit:'Charge SoC limit',chargeCellRule:'Charge cell rule',chargeCommandRateLimit:'Charge command rate limit',chargePostQuant:'Charge post quant',cellVoltageSpread:'Cell voltage spread',ax:'IMU acceleration X',ay:'IMU acceleration Y',az:'IMU acceleration Z',pitch:'Pitch',roll:'Roll',odo:'Odometer',bmsUptime:'BMS uptime',canAgeMs:'CAN age (ms)',diagAgeMs:'Diagnostic age (ms)',vinAgeMs:'VIN age (ms)',sidAgeMs:'Scooter ID age (ms)',navJoystick:'Navigation joystick',parkAssistA:'Park assist A',parkAssistB:'Park assist B',gatewayAgeMs:'Gateway age (ms)',mode:'Drive mode',fbrake:'Front brake',rbrake:'Rear brake',hbeam:'High beam',start:'Start switch',kill:'Kill switch',storage:'Storage switch',horn:'Horn',indl:'Left indicator',indr:'Right indicator',indc:'Center indicator',key:'Key ON',stand:'Side stand',dfan:'Display fan',feco:'Forced ECO',seco:'Smart ECO',imbalance:'Cell imbalance',balancing:'Balancing state',cellSohMin:'Lowest cell SoH',cellSohMax:'Highest cell SoH',cellSocMin:'Lowest cell SoC',cellSocMax:'Highest cell SoC'})[k]||k;\n"
"}\n"
"function diagRenderMetrics(){\n"
" const root=document.getElementById('diagMetrics');\n"
" if(!root)return;\n"
" const v=diagCurrentValues(diagActiveSegment);\n"
" root.innerHTML=diagSegmentMetricKeys(diagActiveSegment).map(k=>{\n"
"  let value=v[k]; let unit='';\n"
"  if(['soc','soh','socEkf','soe','deltaSoc','maxSoc','humidity','chargeSocLimit','cellSohMin','cellSohMax','cellSocMin','cellSocMax'].includes(k))unit='%';\n"
"  if(['packVoltage','cellVoltageMin','cellVoltageMax','cellVoltageSpread','imbalance','chargerInputVoltage','chargerOutputVoltage','pduVoltage'].includes(k))unit=' V';\n"
"  if(['motorTemp','controllerTemp','motorMcuTemp','cellTempMin','cellTempMax','cellTempDelta','afeDieTemp','bmsAmbient','dischargeLimitTemp'].includes(k))unit=' °C';\n"
"  if(k==='canAgeMs'||k==='diagAgeMs'||k==='vinAgeMs'||k==='sidAgeMs')unit=' ms';\n"
"  if(k==='odo')unit=' km';\n"
"  if(typeof value==='boolean')value=value?'Active':'Inactive';\n"
"  if(value&&typeof value==='object')value=JSON.stringify(value);\n"
"  return diagMetric(diagFieldLabel(k),value,unit);\n"
" }).join('');\n"
" const quality=document.getElementById('diagQuality');\n"
" if(quality){\n"
"  const age=diagNum((liveData.data||{}).canAgeMs);\n"
"  quality.textContent=age===null?'CAN freshness unavailable':age<1200?'CAN data fresh':age<3000?'CAN data degraded':'CAN data stale';\n"
" }\n"
"}\n"
"function renderDiagnostic(){\n"
" const root=document.getElementById('telemetryContent');\n"
" root.innerHTML=`<div class=\"diagnostic-tabs\" role=\"tablist\" aria-label=\"Diagnostic segments\">\n"
" ${diagSegments.map(s=>`<button type=\"button\" class=\"diagnostic-tab ${s.id===diagActiveSegment?'active':''}\" role=\"tab\" aria-selected=\"${s.id===diagActiveSegment}\" data-segment=\"${s.id}\">${s.name}</button>`).join('')}\n"
" </div>\n"
" <section class=\"diag-panel\"><h3 id=\"diagSegmentTitle\"></h3><div class=\"diag-muted\" id=\"diagGoal\"></div><div id=\"diagMetrics\" class=\"diag-grid\"></div><p id=\"diagQuality\" class=\"diag-muted\"></p></section>\n"
" <section class=\"diag-panel\"><h3>Segment-specific recording</h3><p class=\"diag-muted\">Only metrics belonging to the selected segment are captured. Fast signals are sampled about once per second; slower metrics are included every 5 seconds, with identity and full cell snapshots saved at session boundaries and notable events.</p><p class=\"diag-muted\">The export separates raw readings, calculated summaries, data-quality warnings, and heuristic observations.</p>\n"
" <div class=\"diag-profile\">${diagProfiles.map((p,i)=>`<label><input type=\"radio\" name=\"diagProfile\" value=\"${p.id}\" ${i===0?'checked':''}><span><strong>${p.name}</strong><br><span class=\"diag-muted\">${p.minutes} minutes</span><br><span class=\"diag-muted\">${p.desc}</span></span></label>`).join('')}</div>\n"
" <div class=\"diag-clock\"><div id=\"diagClockFace\" class=\"diag-clock-face\"><strong id=\"diagClock\">15:00</strong></div><div><strong id=\"diagRecordState\">Ready to record</strong><p id=\"diagRecordDetail\" class=\"diag-muted\">Choose a profile, then start this segment's recording.</p><p id=\"diagSampleCount\" class=\"diag-muted\">0 samples</p></div></div>\n"
" <div class=\"diag-actions\"><button id=\"diagStart\">Start recording</button><button id=\"diagPause\" class=\"secondary\" disabled>Pause</button><button id=\"diagStop\" class=\"danger\" disabled>Stop & finalize</button><button id=\"diagExport\" class=\"secondary\" disabled>Download JSON</button></div>\n"
" </section>\n"
"`;\n"
" root.querySelectorAll('[data-segment]').forEach(b=>b.addEventListener('click',()=>{\n"
"   if(diagSession && !diagSession.finalized && diagSession.segment!==b.dataset.segment){alert('Stop or finalize the active recording before switching segments.');return;}\n"
"   diagActiveSegment=b.dataset.segment;renderDiagnostic();\n"
" }));\n"
" document.getElementById('diagSegmentTitle').textContent=diagSegments.find(s=>s.id===diagActiveSegment).name;\n"
" document.getElementById('diagGoal').textContent=diagSegments.find(s=>s.id===diagActiveSegment).goal;\n"
" document.getElementById('diagStart').addEventListener('click',diagStartRecording);\n"
" document.getElementById('diagPause').addEventListener('click',diagTogglePause);\n"
" document.getElementById('diagStop').addEventListener('click',()=>diagFinalize(false));\n"
" document.getElementById('diagExport').addEventListener('click',diagDownload);\n"
" diagRenderMetrics();diagUpdateRecorderUI();\n"
"}\n"
"function diagIdentity(){\n"
" const {x}=diagGet();\n"
" return {vin:x.vin||null,scooter_id:x.sid||null,firmware:{bms:x.bfw||null,bcm:x.cfw||null,gateway:x.gfw||null,motor:x.mfw||null},hardware_version:x.bhw??null,bin_id:x.bin||null};\n"
"}\n"
"function diagFullCellSnapshot(){\n"
" const {c,x}=diagGet();\n"
" return {cell_voltage_v:Array.isArray(c.v)?c.v.slice():[],cell_soh:Array.isArray(c.h)?c.h.slice():[],cell_soc:Array.isArray(x.cs)?x.cs.slice():[],cell_temperatures:Array.isArray(x.ct)?x.ct.slice():[]};\n"
"}\n"
"function diagBatteryFlags(v){\n"
" const flags=[];\n"
" if(diagNum(v.soh)!==null&&Number(v.soh)<70)flags.push({severity:'review',code:'LOW_REPORTED_SOH',message:'The BMS-reported SoH is below the configurable 70% screening threshold; verify the BMS definition and measurement conditions.'});\n"
" if(diagNum(v.cellVoltageSpread)!==null&&Number(v.cellVoltageSpread)>0.08)flags.push({severity:'review',code:'CELL_SPREAD_HIGH',message:'Cell-voltage spread exceeds the preliminary 0.08 V screening threshold. Compare at a similar SoC and load.'});\n"
" if(diagNum(v.cellTempDelta)!==null&&Number(v.cellTempDelta)>10)flags.push({severity:'review',code:'CELL_TEMP_DELTA_HIGH',message:'Cell temperature spread exceeds the preliminary 10 °C screening threshold.'});\n"
" if(diagNum(v.humidity)!==null&&Number(v.humidity)>80)flags.push({severity:'info',code:'HUMIDITY_HIGH',message:'Humidity reading is high; confirm sensor scaling and location. Humidity alone does not establish battery damage.'});\n"
" if(diagNum(v.canAgeMs)===null||Number(v.canAgeMs)>=3000)flags.push({severity:'data_quality',code:'CAN_STALE',message:'CAN data is stale or unavailable; health conclusions may be unreliable.'});\n"
" return flags;\n"
"}\n"
"function diagBuildSample(elapsedMs){\n"
" const v=diagCurrentValues(diagActiveSegment),keys=diagSegmentMetricKeys(diagActiveSegment);\n"
" const fastKeys={activity:['fbrake','rbrake','hbeam','start','kill','storage','horn','indl','indr','indc','key','stand','dfan','feco','seco','mode','rpm'],\n"
" battery:['soc','socEkf','soe','soh','packVoltage','packCurrent','cellVoltageSpread','weakestCellIndex','strongestCellIndex'],\n"
" motor:['rpm','mcRpm','tcRpm','motorCurrent','packCurrent','throttleOut','throttle1','throttle3','motorTemp','controllerTemp'],\n"
" charger:['chargerConnected','chargerInputVoltage','chargerOutputVoltage','chargerOutputCurrent','chargerPower','soc'],\n"
" ride:['rpm','throttleOut','throttle1','throttle3','motorCurrent','packCurrent','packVoltage','soc','fbrake','rbrake','odo','mode','ax','ay','az','pitch','roll']};\n"
" const fast=fastKeys[diagActiveSegment]||keys;\n"
" const snapshotExtras={\n"
"  activity:['odo','bmsUptime','canAgeMs','diagAgeMs','vinAgeMs','sidAgeMs','gatewayAgeMs'],\n"
"  battery:['soh','deltaSoc','cellVoltageMin','cellVoltageMax','cellVoltageSpread','cellTempMin','cellTempMax','humidity','deratedDischarge','deratedCharge','deratedRegen'],\n"
"  motor:['controllerTemp','motorMcuTemp','deratedDischarge','deratedRegen'],\n"
"  charger:['chargerFan1','chargerFan2','chargeSocLimit','deratedCharge','cellVoltageSpread'],\n"
"  ride:['motorTemp','controllerTemp','cellTempMin','cellTempMax','odo','pitch','roll']\n"
" };\n"
" const profileId=diagSession.profile;\n"
" const slowKeys=profileId==='snapshot'?(snapshotExtras[diagActiveSegment]||[]):keys;\n"
" const slowInterval=profileId==='snapshot'?10:5;\n"
" const metrics={};fast.forEach(k=>metrics[k]=v[k]??null);\n"
" if(Math.floor(elapsedMs/1000)%slowInterval===0) slowKeys.filter(k=>!fast.includes(k)).forEach(k=>metrics[k]=v[k]??null);\n"
" const sample={elapsed_ms:elapsedMs,metrics,data_quality:{can_age_ms:v.canAgeMs??null,can_online:!!(liveData.data||{}).canOnline}};\n"
" if(diagActiveSegment==='battery'){\n"
"  sample.cell_summary={min_v:v.cellVoltageMin,max_v:v.cellVoltageMax,spread_v:v.cellVoltageSpread,weakest_cell_index:v.weakestCellIndex,strongest_cell_index:v.strongestCellIndex,cell_temp_min:v.cellTempMin,cell_temp_max:v.cellTempMax};\n"
"  const cellAlert=diagNum(v.cellVoltageSpread)!==null&&v.cellVoltageSpread>0.08;\n"
"  if(!diagSession.lastCellSnapshot || elapsedMs===0 || (cellAlert&&!diagSession._cellAlertActive)){\n"
"   sample.cell_snapshot=diagFullCellSnapshot();diagSession.lastCellSnapshot=elapsedMs;\n"
"  }\n"
"  diagSession._cellAlertActive=cellAlert;\n"
" }\n"
" return sample;\n"
"}\n"
"function diagStartRecording(){\n"
" if(diagSession&&!diagSession.finalized){alert('A recording is already active. Stop it before starting another.');return;}\n"
" const profileId=document.querySelector('input[name=\"diagProfile\"]:checked')?.value||'snapshot';\n"
" const profile=diagProfiles.find(p=>p.id===profileId)||diagProfiles[0];\n"
" const v=diagCurrentValues(diagActiveSegment);\n"
" diagSession={schema_version:'1.0',project:'Vehicle CAN Diagnostic Tool',units:{pack_voltage:'V',cell_voltage:'V',temperature:'degC after /100 scaling where noted by firmware UI',pack_current:'A after /100 scaling',distance:'km',elapsed_time:'ms'},segment:diagActiveSegment,segment_name:diagSegments.find(s=>s.id===diagActiveSegment).name,profile:profile.id,planned_duration_seconds:profile.minutes*60,started_uptime_ms:Math.round(performance.now()),started_at:null,paused:false,finalized:false,elapsed_ms:0,samples:[],events:[],transition_stats:{},identity:diagIdentity(),initial_values:v,initial_cell_snapshot:diagFullCellSnapshot(),lastCellSnapshot:0,observations:[],summary:{}};\n"
" diagTransitionState={};Object.keys(diagMetricNames).forEach(k=>{diagTransitionState[k]=v[k];diagTransitionStats[k]={transitions:0,total_on_ms:0,on_since:(v[k]===true?0:null),last: v[k]};});\n"
" diagLastSampleMs=performance.now();diagSession.samples.push(diagBuildSample(0));\n"
" diagTimer=setInterval(diagTick,1000);diagUpdateRecorderUI();\n"
"}\n"
"function diagTick(){\n"
" if(!diagSession||diagSession.finalized||diagSession.paused)return;\n"
" const now=performance.now();diagSession.elapsed_ms=Math.max(0,Math.round(now-diagSession.started_uptime_ms));\n"
" const elapsed=diagSession.elapsed_ms;\n"
" const v=diagCurrentValues(diagSession.segment);\n"
" if(diagSession.segment==='activity'){\n"
"  const priorMode=diagSession._lastMode;\n"
"  if(priorMode!==undefined && v.mode!==priorMode)diagSession.events.push({elapsed_ms:elapsed,type:'drive_mode_change',from:priorMode,to:v.mode});\n"
"  diagSession._lastMode=v.mode;\n"
"  ['navJoystick','parkAssistA','parkAssistB'].forEach(k=>{\n"
"   const cur=v[k],prev=diagSession['_last_'+k];\n"
"   if(prev!==undefined && cur!==prev && cur!==null && cur!==undefined)diagSession.events.push({elapsed_ms:elapsed,type:'control_event',signal:k,from:prev,to:cur});\n"
"   diagSession['_last_'+k]=cur;\n"
"  });\n"
"  const priorUptime=diagSession._lastBmsUptime;\n"
"  if(priorUptime!==undefined && diagNum(v.bmsUptime)!==null && Number(v.bmsUptime)<Number(priorUptime)){\n"
"   diagSession.events.push({elapsed_ms:elapsed,type:'bms_reset_detected',previous_uptime:priorUptime,current_uptime:v.bmsUptime});\n"
"  }\n"
"  diagSession._lastBmsUptime=v.bmsUptime;\n"
"  Object.keys(diagMetricNames).forEach(k=>{\n"
"   const prev=diagTransitionState[k],cur=v[k];\n"
"   if(typeof cur==='boolean'&&typeof prev==='boolean'&&cur!==prev){\n"
"    const st=diagTransitionStats[k];st.transitions++;if(prev===true&&st.on_since!==null)st.total_on_ms+=elapsed-st.on_since;\n"
"    st.on_since=cur?elapsed:null;st.last=cur;\n"
"    diagSession.events.push({elapsed_ms:elapsed,type:'switch_transition',signal:k,from:prev,to:cur});\n"
"   }\n"
"   diagTransitionState[k]=cur;\n"
"  });\n"
"  if(v.kill===true&&Number(v.rpm)>0)diagSession.events.push({elapsed_ms:elapsed,type:'safety_flag',code:'KILL_SWITCH_WHILE_RPM_NONZERO'});\n"
"  if(v.stand===true&&Number(v.rpm)>0)diagSession.events.push({elapsed_ms:elapsed,type:'safety_flag',code:'SIDE_STAND_WHILE_RPM_NONZERO'});\n"
"  if((v.fbrake===true||v.rbrake===true)&&Number(v.throttleOut)>20)diagSession.events.push({elapsed_ms:elapsed,type:'safety_flag',code:'BRAKE_WITH_HIGH_THROTTLE'});\n"
" }\n"
" if(elapsed-diagLastSampleMs>=950){diagSession.samples.push(diagBuildSample(elapsed));diagLastSampleMs=elapsed;}\n"
" if(elapsed>=diagSession.planned_duration_seconds*1000){diagFinalize(true);return;}\n"
" diagUpdateRecorderUI();\n"
"}\n"
"function diagTogglePause(){\n"
" if(!diagSession||diagSession.finalized)return;\n"
" if(!diagSession.paused){diagSession.paused=true;diagSession.pauseStarted=performance.now();}\n"
" else {const now=performance.now(),delta=now-diagSession.pauseStarted;diagSession.started_uptime_ms+=delta;diagSession.paused=false;diagSession.events.push({elapsed_ms:diagSession.elapsed_ms,type:'recording_resumed'});}\n"
" diagUpdateRecorderUI();\n"
"}\n"
"function diagFinalize(auto){\n"
" if(!diagSession||diagSession.finalized)return;\n"
" if(diagTimer){clearInterval(diagTimer);diagTimer=null;}\n"
" const now=performance.now();\n"
" if(diagSession.paused&&diagSession.pauseStarted)diagSession.started_uptime_ms+=now-diagSession.pauseStarted;\n"
" diagSession.elapsed_ms=Math.max(0,Math.round(now-diagSession.started_uptime_ms));\n"
" const final=diagCurrentValues(diagSession.segment);\n"
" diagSession.samples.push(diagBuildSample(diagSession.elapsed_ms));\n"
" diagSession.final_values=final;diagSession.final_cell_snapshot=diagFullCellSnapshot();diagSession.finalized=true;\n"
" diagSavedSessions[diagSession.segment]=diagSession;\n"
" diagSession.ended_uptime_ms=Math.round(now);diagSession.end_reason=auto?'duration_completed':'manually_stopped';\n"
" if(diagSession.segment==='activity'){\n"
"  const elapsed=diagSession.elapsed_ms;\n"
"  Object.keys(diagTransitionStats).forEach(k=>{const st=diagTransitionStats[k];if(st.last===true&&st.on_since!==null)st.total_on_ms+=elapsed-st.on_since;});\n"
"  diagSession.transition_stats=JSON.parse(JSON.stringify(diagTransitionStats));\n"
"  const modeEvents=diagSession.events.filter(e=>e.type==='drive_mode_change');\n"
"  const modeDurations={};let currentMode=diagSession.initial_values.mode,prevElapsed=0;\n"
"  modeEvents.forEach(e=>{modeDurations[currentMode]=(modeDurations[currentMode]||0)+Math.max(0,e.elapsed_ms-prevElapsed);currentMode=e.to;prevElapsed=e.elapsed_ms;});\n"
"  modeDurations[currentMode]=(modeDurations[currentMode]||0)+Math.max(0,diagSession.elapsed_ms-prevElapsed);\n"
"  diagSession.mode_stats={changes:modeEvents.length,time_ms_by_mode:modeDurations};\n"
"  diagSession.event_counts={nav_joystick:diagSession.events.filter(e=>e.type==='control_event'&&e.signal==='navJoystick').length,park_assist_a:diagSession.events.filter(e=>e.type==='control_event'&&e.signal==='parkAssistA').length,park_assist_b:diagSession.events.filter(e=>e.type==='control_event'&&e.signal==='parkAssistB').length,bms_resets:diagSession.events.filter(e=>e.type==='bms_reset_detected').length};\n"
"  const startO=diagNum(diagSession.initial_values.odo),endO=diagNum(final.odo);\n"
"  diagSession.summary.distance_km=startO!==null&&endO!==null?Math.max(0,endO-startO):null;\n"
" }\n"
" if(diagSession.segment==='battery'){\n"
"  const a=diagNum(diagSession.initial_values.soc),b=diagNum(final.soc);\n"
"  diagSession.summary.soc_change_percent=a!==null&&b!==null?b-a:null;\n"
"  const currents=diagSession.samples.map(s=>diagNum(s.metrics.packCurrent)).filter(n=>n!==null);\n"
"  diagSession.summary.average_absolute_pack_current=currents.length?currents.reduce((a,b)=>a+Math.abs(b),0)/currents.length:null;\n"
"  diagSession.summary.peak_absolute_pack_current=currents.length?Math.max(...currents.map(Math.abs)):null;\n"
"  diagSession.summary.pack_voltage_stats=statsFor('packVoltage');\n"
"  diagSession.summary.cell_voltage_spread_stats=statsFor('cellVoltageSpread');\n"
"  diagSession.summary.cell_temperature_stats=statsFor('cellTempMax');\n"
"  const voltageCurrentPairs=diagSession.samples.map(s=>({v:diagNum(s.metrics.packVoltage),i:diagNum(s.metrics.packCurrent)})).filter(p=>p.v!==null&&p.i!==null);\n"
"  diagSession.summary.observed_voltage_span_v=voltageCurrentPairs.length?Math.max(...voltageCurrentPairs.map(p=>p.v))-Math.min(...voltageCurrentPairs.map(p=>p.v)):null;\n"
"  const resistanceCandidates=[];\n"
"  for(let i=1;i<voltageCurrentPairs.length;i++){const dv=voltageCurrentPairs[i-1].v-voltageCurrentPairs[i].v,di=voltageCurrentPairs[i].i-voltageCurrentPairs[i-1].i;if(Math.abs(di)>=5&&dv/di>0)resistanceCandidates.push(dv/di);}\n"
"  diagSession.summary.exploratory_internal_resistance_ohm=resistanceCandidates.length?resistanceCandidates.reduce((a,b)=>a+b,0)/resistanceCandidates.length:null;\n"
"  diagSession.summary.internal_resistance_note='Exploratory only; requires stable load steps, validated current polarity/scaling, and controlled conditions. Do not treat as a certified battery resistance measurement.';\n"
"  diagSession.observations=diagBatteryFlags(final);\n"
" }\n"
"\n"
" const numsFor=(key)=>diagSession.samples.map(s=>diagNum(s.metrics[key])).filter(n=>n!==null);\n"
" const statsFor=(key)=>{const a=numsFor(key);return a.length?{min:Math.min(...a),max:Math.max(...a),average:a.reduce((sum,n)=>sum+n,0)/a.length,count:a.length}:null;};\n"
" if(diagSession.segment==='motor'){\n"
"  const currents=numsFor('motorCurrent').map(Math.abs),temps=numsFor('motorTemp');\n"
"  const motorTemps=diagSession.samples.map(s=>({t:s.elapsed_ms,v:diagNum(s.metrics.motorTemp)})).filter(p=>p.v!==null);\n"
"  const tempRisePerMinute=motorTemps.length>1&&motorTemps[motorTemps.length-1].t>motorTemps[0].t?((motorTemps[motorTemps.length-1].v-motorTemps[0].v)/((motorTemps[motorTemps.length-1].t-motorTemps[0].t)/60000)):null;\n"
"  const motorTempAbove80=diagSession.samples.filter(s=>diagNum(s.metrics.motorTemp)!==null&&Number(s.metrics.motorTemp)>80).length;\n"
"  const controllerTempAbove80=diagSession.samples.filter(s=>diagNum(s.metrics.controllerTemp)!==null&&Number(s.metrics.controllerTemp)>80).length;\n"
"  diagSession.summary={...(diagSession.summary||{}),motor_current:{average_abs:currents.length?currents.reduce((a,b)=>a+b,0)/currents.length:null,peak_abs:currents.length?Math.max(...currents):null},motor_temperature:statsFor('motorTemp'),motor_temperature_rise_c_per_minute:tempRisePerMinute,controller_temperature:statsFor('controllerTemp'),temperature_screening_thresholds_c:{motor:80,controller:80},approx_time_above_80c_ms:{motor:motorTempAbove80*1000,controller:controllerTempAbove80*5000},rpm:statsFor('rpm'),rpm_channel_mismatch_samples:diagSession.samples.filter(s=>diagNum(s.metrics.mcRpm)!==null&&diagNum(s.metrics.tcRpm)!==null&&Math.abs(Number(s.metrics.mcRpm)-Number(s.metrics.tcRpm))>100).length,throttle_channel_mismatch_samples:diagSession.samples.filter(s=>diagNum(s.metrics.throttle1)!==null&&diagNum(s.metrics.throttle3)!==null&&Math.abs(Number(s.metrics.throttle1)-Number(s.metrics.throttle3))>50).length};\n"
" }\n"
" if(diagSession.segment==='charger'){\n"
"  const a=diagNum(diagSession.initial_values.soc),b=diagNum(final.soc);\n"
"  diagSession.summary={...(diagSession.summary||{}),soc_gained_percent:a!==null&&b!==null?b-a:null,output_power:statsFor('chargerPower'),output_current:statsFor('chargerOutputCurrent'),output_energy_wh:null,efficiency_percent:null,cc_to_cv_transition_time_seconds:null,output_current_following_derated_limit:null,limitations:['Input current is not exposed by the current firmware API, so input power and charger efficiency cannot be calculated reliably.','CC-to-CV transition time requires a validated charge-stage signal or an agreed inference rule.']};\n"
"  const samples=diagSession.samples;\n"
"  let wh=0,energyValid=samples.length>1;\n"
"  for(let i=1;i<samples.length;i++){\n"
"   const p0=diagNum(samples[i-1].metrics.chargerPower),p1=diagNum(samples[i].metrics.chargerPower);\n"
"   const dt=(samples[i].elapsed_ms-samples[i-1].elapsed_ms)/3600000;\n"
"   if(p0===null||p1===null||dt<0||dt>0.02){energyValid=false;break;}\n"
"   wh+=(p0+p1)/2*dt;\n"
"  }\n"
"  diagSession.summary.output_energy_wh=energyValid?wh:null;\n"
" }\n"
" if(diagSession.segment==='ride'){\n"
"  const startO=diagNum(diagSession.initial_values.odo),endO=diagNum(final.odo);\n"
"  const currents=numsFor('motorCurrent').map(Math.abs),temps=[...numsFor('motorTemp'),...numsFor('controllerTemp'),...numsFor('cellTempMin'),...numsFor('cellTempMax')];\n"
"  const volts=numsFor('packVoltage'),packI=numsFor('packCurrent');\n"
"  diagSession.summary={...(diagSession.summary||{}),distance_km:startO!==null&&endO!==null?Math.max(0,endO-startO):null,peak_motor_current:currents.length?Math.max(...currents):null,maximum_observed_temperature:temps.length?Math.max(...temps):null,voltage_min:volts.length?Math.min(...volts):null,voltage_max:volts.length?Math.max(...volts):null,brake_events:diagSession.events.filter(e=>e.type==='switch_transition'&&(e.signal==='fbrake'||e.signal==='rbrake')&&e.to===true).length,wh_per_km:null,average_speed_kmh:null,top_speed_kmh:null,ride_roughness_count:null,hard_braking_count:null,harsh_acceleration_count:null,tilt_or_fall_events:null,limitations:['Average/top speed require a validated RPM-to-road-speed conversion for the scooter configuration.','Wh/km requires validated current sign/scaling and distance calibration; it is not inferred here.','IMU thresholds and fall detection require vehicle-specific calibration and are not enabled as definitive alerts.']};\n"
" }\n"
" const ages=diagSession.samples.map(s=>s.data_quality.can_age_ms).filter(n=>Number.isFinite(n));\n"
" diagSession.data_quality={sample_count:diagSession.samples.length,missing_or_stale_can_samples:ages.filter(n=>n>=3000).length,can_age_ms_min:ages.length?Math.min(...ages):null,can_age_ms_max:ages.length?Math.max(...ages):null};\n"
" diagSession.ai_report_guidance={purpose:'Analyze only the selected segment data. Separate measured values, derived calculations, and hypotheses. Cite sample times for findings; mention missing/stale data; do not infer battery safety or validated SoH from heuristics alone.',requested_sections:['Executive summary','Observed trends','Anomalies with supporting samples','Data-quality limitations','Recommended follow-up checks']};\n"
" diagUpdateRecorderUI();\n"
"}\n"
"function diagUpdateRecorderUI(){\n"
" if(!document.getElementById('diagClock'))return;\n"
" const s=diagSession&&diagSession.segment===diagActiveSegment?diagSession:(diagSavedSessions[diagActiveSegment]||null);\n"
" const profile=diagProfiles.find(p=>p.id===(s?s.profile:(document.querySelector('input[name=\"diagProfile\"]:checked')?.value||'quick_check'))) ||diagProfiles[0];\n"
" const duration=s?s.planned_duration_seconds*1000:profile.minutes*60000;\n"
" const elapsed=s?s.elapsed_ms:0,remaining=Math.max(0,duration-elapsed);\n"
" const sec=Math.ceil(remaining/1000),m=String(Math.floor(sec/60)).padStart(2,'0'),ss=String(sec%60).padStart(2,'0');\n"
" document.getElementById('diagClock').textContent=m+':'+ss;\n"
" const face=document.getElementById('diagClockFace');face.style.setProperty('--progress',Math.min(100,duration?elapsed/duration*100:0)+'%');face.classList.toggle('running',!!s&&!s.finalized&&!s.paused);\n"
" document.getElementById('diagRecordState').textContent=!s?'Ready to record':s.finalized?'Recording finalized':s.paused?'Paused':'Recording '+s.segment_name;\n"
" document.getElementById('diagRecordDetail').textContent=!s?'Choose a profile, then start this segment only.':s.finalized?'Session finalized. Download the segment-specific JSON when ready.':s.paused?'Timer and sampling are paused.':'The timer animates while this segment is recording.';\n"
" document.getElementById('diagSampleCount').textContent=(s?s.samples.length:0)+' samples'+(s?' · '+Math.floor((s.elapsed_ms||0)/1000)+' seconds elapsed':'');\n"
" document.getElementById('diagStart').disabled=!!diagSession&&!diagSession.finalized;\n"
" document.getElementById('diagPause').disabled=!s||s.finalized;\n"
" document.getElementById('diagPause').textContent=s&&s.paused?'Resume':'Pause';\n"
" document.getElementById('diagStop').disabled=!s||s.finalized;\n"
" document.getElementById('diagExport').disabled=!s||!s.finalized;\n"
" const obs=document.getElementById('diagObservations');\n"
" if(obs&&s&&s.finalized){\n"
"  const items=s.observations||[];\n"
"  obs.innerHTML=items.length?items.map(o=>`<li><strong>${o.severity}</strong> — ${o.message}</li>`).join(''):'<li>No configured screening flags triggered; this does not certify the battery or vehicle as healthy.</li>';\n"
" }\n"
"}\n"
"function diagDownload(){\n"
" const session=diagSession&&diagSession.segment===diagActiveSegment&&diagSession.finalized?diagSession:diagSavedSessions[diagActiveSegment];\n"
" if(!session||!session.finalized)return;\n"
" const blob=new Blob([JSON.stringify(session,null,2)],{type:'application/json'});\n"
" const url=URL.createObjectURL(blob),a=document.createElement('a');\n"
" a.href=url;a.download='vcd-'+session.segment+'-'+session.profile+'.json';document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);\n"
"}\n"
"document.querySelectorAll('.control').forEach(btn => { btn.disabled = true; });\n"
"renderCategories();\n"
"pollFirmwareApi();\n"
"setInterval(pollFirmwareApi,1000);\n"
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
char extraBuf[2600];
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
    // ===== EXTRA (newly discovered) frames =====
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
    case 0x206: if(m.data_length_code<8)return; { float f; memcpy(&f,&d[4],4); t.chgSocLimit=f; } break;  // confirmed: charge SoC limit
    case 0x91: if(m.data_length_code<8)return; memcpy(t.vin,d,8); t.lastVinMs=millis(); break;  // VIN broadcast part 1
    case 0x92: if(m.data_length_code<8)return; memcpy(t.vin+8,d,8); t.vin[16]=0; t.lastVinMs=millis(); break;  // VIN broadcast part 2
    case 0x93: if(m.data_length_code<1)return; t.vin[16]=(char)d[0]; t.vin[17]=0; t.lastVinMs=millis(); break;  // VIN broadcast part 3
    case 0x7A: if(m.data_length_code<8)return; memcpy(t.scooterId,d,8); t.scooterId[8]=0; t.lastSidMs=millis(); break; // Scooter-ID broadcast
    case 0x7B1: case 0x7B9: t.lastDiagMs=millis(); break;                                      // diagnostic tool presence
    case 0x120: if(m.data_length_code<3)return; t.parkAssistA=d[2]; t.lastParkAMs=millis(); break;  // probable Parking Assist trigger A
    case 0x121: if(m.data_length_code<3)return; t.parkAssistB=d[2]; t.lastParkBMs=millis(); break;  
    case 0x18c:
      if(m.data_length_code<5)return;
      t.motorI=((d[2]>>5)|((d[3]&31)<<3))+((((d[3]>>5)|((d[4]&31)<<3))&1)*256)-(((d[3]>>5)&1)*512);
      t.motorTemp=(((d[4]&63)<<2)|(d[3]>>6))-40;                                   // Motor Temp
      if(m.data_length_code>=6) t.controllerTemp=(((d[5]&63)<<2)|(d[4]>>6))-40;    // Controller Temp (distinct from Motor Temp)
      break;
    case 0x209: if(m.data_length_code<3)return; t.motorMcuTemp=(int16_t)d[2]; break;  // Motor MCU Temp (3rd, independent reading)
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
