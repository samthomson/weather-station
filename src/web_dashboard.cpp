#include "web_dashboard.h"

#include "config_store.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

namespace {
  WebServer server(80);
  DNSServer dns;
  const byte DNS_PORT = 53;
  IPAddress AP_IP(192, 168, 4, 1);
  IPAddress AP_NETMASK(255, 255, 255, 0);

  DashboardCallbacks cb;
  DashboardStatus status;
  String g_ap_ssid;

  String macSuffix() {
    // efuse MAC is always populated (unlike WiFi.macAddress() which can read
    // back zeros if the radio hasn't been brought up yet).
    uint64_t chipId = ESP.getEfuseMac();
    char buf[7];
    snprintf(buf, sizeof(buf), "%02X%02X%02X",
             (unsigned)((chipId >> 16) & 0xFF),
             (unsigned)((chipId >>  8) & 0xFF),
             (unsigned)((chipId >>  0) & 0xFF));
    return String(buf);
  }

  // --- Embedded dashboard page ---
  const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Weather Station</title>
<style>
:root{
  --ink:#2a2521;--bg:#fbf7f0;--surface:#ffffff;--surface-hi:#f4ede0;
  --border:#e2dac8;--border-hi:#c8bca5;
  --text:#2a2521;--text-muted:#766c5f;--text-dim:#9a8f80;
  --orange:#d97e2e;--orange-hi:#c66c1f;--orange-soft:#fbe9d2;
  --purple:#7c5fb8;--purple-hi:#6a4ca8;--purple-soft:#ece5f6;
  --err:#c0392b;--err-soft:#fbe5e0;
}
*{box-sizing:border-box}
html,body{background:var(--bg)}
body{font-family:-apple-system,'Inter','Segoe UI',system-ui,sans-serif;margin:0 auto;padding:0 16px 32px;max-width:640px;background:var(--bg);color:var(--text);line-height:1.45}
header.brand{margin:0 -16px 8px;padding:0}
.brand-bar{height:4px;background:linear-gradient(90deg,var(--orange) 0%,var(--orange) 50%,var(--purple) 100%)}
header.brand .brand-inner{padding:18px 16px 8px}
h1{margin:0;font-size:22px;font-weight:700;letter-spacing:-.01em;color:var(--orange)}
h1 .dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--purple);margin:0 8px 2px 0;vertical-align:middle}
h2{margin:28px 0 10px;font-size:13px;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--purple);border-bottom:1px solid var(--border);padding-bottom:6px}
label{display:block;margin:10px 0 4px;font-size:13px;color:var(--text-muted);font-weight:500}
input,select,button,textarea{width:100%;padding:11px 12px;font-size:16px;background:var(--surface);color:var(--text);border:1px solid var(--border);border-radius:8px;font-family:inherit;transition:border-color .15s,box-shadow .15s,background .15s}
input:focus,select:focus,textarea:focus{outline:none;border-color:var(--orange);box-shadow:0 0 0 3px var(--orange-soft)}
input[readonly]{background:var(--surface-hi);color:var(--text-muted)}
button{background:var(--orange);color:#fff;border:none;font-weight:700;margin-top:10px;cursor:pointer;letter-spacing:.01em;box-shadow:0 1px 2px rgba(0,0,0,.06)}
button:hover{background:var(--orange-hi)}
button:active{transform:translateY(1px);box-shadow:none}
button.warn{background:var(--err);color:#fff}
button.secondary{background:var(--surface);color:var(--text);border:1px solid var(--border);box-shadow:0 1px 2px rgba(0,0,0,.04)}
button.secondary:hover{background:var(--surface-hi);border-color:var(--border-hi)}
.row{display:flex;gap:8px;align-items:flex-end}
.row>*{flex:1}
.row>button{flex:0 0 auto;width:auto;padding-left:16px;padding-right:16px}
.pill{display:inline-block;padding:3px 10px;border-radius:999px;background:var(--surface-hi);color:var(--text);font-size:12px;font-weight:600;margin-right:4px;border:1px solid var(--border)}
.pill.ok{background:var(--purple-soft);color:var(--purple-hi);border-color:var(--purple)}
.pill.warn{background:var(--orange-soft);color:var(--orange-hi);border-color:var(--orange)}
.pill.err{background:var(--err-soft);color:var(--err);border-color:var(--err)}
.muted{color:var(--text-muted);font-size:12px}
.wifilist{display:flex;flex-direction:column;gap:4px;max-height:220px;overflow-y:auto;margin:8px 0;padding:6px;background:var(--surface-hi);border:1px solid var(--border);border-radius:8px}
.wifilist .ap{padding:10px 12px;background:var(--surface);border:1px solid var(--border);border-radius:6px;display:flex;justify-content:space-between;cursor:pointer;font-size:14px}
.wifilist .ap:hover{background:var(--purple-soft);border-color:var(--purple)}
.wifilist .ap:active{background:var(--purple-soft)}
.toggle{display:flex;align-items:center;gap:12px;margin:8px 0;padding:10px 12px;background:var(--surface);border:1px solid var(--border);border-radius:8px}
.toggle input{width:20px;height:20px;flex:0 0 auto;accent-color:var(--orange)}
.toggle label{margin:0;color:var(--text);font-size:14px;flex:1;font-weight:500}
table{width:100%;border-collapse:collapse;font-size:14px;margin-top:6px}
td{padding:6px 6px;border-bottom:1px solid var(--border)}
td:first-child{color:var(--text-muted);width:45%}
details{background:var(--surface);padding:10px 14px;border:1px solid var(--border);border-radius:8px;margin-top:10px}
summary{cursor:pointer;color:var(--text-muted);padding:4px 0;font-weight:600}
.toast{position:fixed;bottom:24px;left:50%;transform:translate(-50%,20px);background:var(--surface);border:1px solid var(--border);padding:12px 22px;border-radius:10px;font-size:15px;font-weight:600;color:var(--text);opacity:0;transition:opacity .25s,transform .25s;pointer-events:none;box-shadow:0 12px 32px rgba(42,37,33,.18);z-index:50}
.toast.show{opacity:1;transform:translate(-50%,0)}
.toast.ok{background:var(--purple-soft);color:var(--purple-hi);border-color:var(--purple)}
.toast.err{background:var(--err-soft);color:var(--err);border-color:var(--err)}
button.saving{background:var(--surface-hi);color:var(--text-muted);cursor:wait;border:1px solid var(--border);box-shadow:none}
button.saved{background:var(--purple);color:#fff}
button.savefail{background:var(--err);color:#fff}
.section-help{font-size:12px;color:var(--text-muted);margin:0 0 8px;line-height:1.5}
#status{min-height:180px}
.status-line{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:8px}
.status-line .muted{flex-basis:100%;margin-top:2px}
#statusTable td:first-child{color:var(--text-muted);width:45%}
::placeholder{color:var(--text-dim)}
::selection{background:var(--orange);color:#fff}
</style>
</head>
<body>
<header class="brand">
  <div class="brand-bar"></div>
  <div class="brand-inner">
    <h1><span class="dot"></span>Weather Station <span id="deviceTag" class="muted" style="font-size:13px;font-weight:600;color:var(--purple);margin-left:6px"></span></h1>
    <div class="muted" id="topline">Loading...</div>
  </div>
</header>

<h2>Status</h2>
<div id="status">
  <div class="status-line" id="statusPills"></div>
  <div class="muted" id="statusLastPost">Last publish: -</div>
  <table id="statusTable"></table>
</div>

<h2>WiFi (home network)</h2>
<p class="section-help">The station also broadcasts its own setup WiFi (this page). Set your home WiFi here so it can post readings to the internet.</p>
<label>Network name (SSID)</label>
<input id="wifi_ssid" placeholder="e.g. MyHomeWiFi" autocomplete="off">
<button class="secondary" onclick="scanWifi()" id="scanBtn">Scan nearby networks</button>
<div class="wifilist" id="wifilist" style="display:none"></div>
<label>Password</label>
<div class="row">
  <input id="wifi_pass" type="password" autocomplete="off">
  <button class="secondary" onclick="toggleWifiPass()" id="wifiPassToggle">Show</button>
</div>
<button onclick="saveWifi(this)">Save WiFi and reconnect</button>

<h2>Station identity</h2>
<label>Name</label>
<input id="station_name" placeholder="e.g. My Garden Station">
<label>Description</label>
<input id="station_description" placeholder="Optional">
<label>Geohash <span class="muted">(find one at geohash.jorren.nl)</span></label>
<div class="row">
  <input id="station_geohash" placeholder="e.g. w5q6u">
  <button class="secondary" onclick="useGPS()">Use my GPS</button>
</div>
<label>Elevation (metres)</label>
<input id="station_elevation" type="text" placeholder="Optional">
<label>Power source</label>
<select id="station_power">
  <option>mains</option><option>solar</option><option>battery</option><option>solar_battery</option><option>usb</option>
</select>
<label>Connectivity</label>
<select id="station_connectivity">
  <option>wifi</option><option>cellular</option><option>ethernet</option><option>lora</option><option>satellite</option>
</select>
<button onclick="saveIdentity(this)">Save station identity</button>

<h2>Nostr</h2>
<label>Relay</label>
<select id="nostr_relay_preset" onchange="if(this.value)document.getElementById('nostr_relay').value=this.value">
  <option value="">-- pick a preset --</option>
  <option value="wss://wr.samt.st">wss://wr.samt.st</option>
  <option value="wss://relay.damus.io">wss://relay.damus.io</option>
  <option value="wss://nos.lol">wss://nos.lol</option>
  <option value="wss://relay.nostr.band">wss://relay.nostr.band</option>
</select>
<input id="nostr_relay" placeholder="wss://relay.example">
<label>Public key (station identity)</label>
<input id="pubkey_hex" readonly>
<label>Private key <span class="muted">(keep secret!)</span></label>
<div class="row">
  <input id="nostr_privkey" type="password" autocomplete="off">
  <button class="secondary" onclick="toggleKey()" id="keyToggle">Show</button>
</div>
<div class="row">
  <button class="secondary" onclick="copyKey()">Copy</button>
  <button class="secondary" onclick="regenKey()">Regenerate</button>
</div>
<button onclick="saveNostr(this)">Save Nostr settings</button>

<h2>Sensors (toggle what is actually wired)</h2>
<p class="section-help">Only enable a sensor if you have it physically connected.</p>
<div class="toggle"><input id="en_bme280" type="checkbox"><label for="en_bme280">BME280 - temperature, humidity, pressure</label></div>
<div class="toggle"><input id="en_bh1750" type="checkbox"><label for="en_bh1750">BH1750 - daylight (lux)</label></div>
<div class="toggle"><input id="en_rain" type="checkbox"><label for="en_rain">MH-RD - rain (analog)</label></div>
<div class="toggle"><input id="en_pms" type="checkbox"><label for="en_pms">PMS5003 / PMS7003 - particulates</label></div>
<label>PMS model</label>
<select id="pms_model"><option>PMS5003</option><option>PMS7003</option></select>
<button onclick="saveSensors(this)">Save sensor setup</button>

<h2>Advanced</h2>
<label>Post interval (seconds)</label>
<input id="post_interval_s" type="number" min="10" step="5">
<button onclick="saveAdvanced(this)">Save</button>
<details>
  <summary>Restart or factory reset</summary>
  <p class="section-help">Restart applies any pending hardware changes. Factory reset wipes WiFi credentials, station identity, and the Nostr key.</p>
  <button class="secondary" onclick="restart()">Restart now</button>
  <button class="warn" onclick="factoryReset()">Factory reset (wipes everything)</button>
</details>

<div class="toast" id="toast"></div>

<script>
let _cfg = {};
function $(id){return document.getElementById(id)}
function toast(msg, kind){
  const t = $('toast'); t.textContent = msg; t.className = 'toast show ' + (kind||'');
  setTimeout(()=>{ t.className = 'toast' }, 2500);
}
async function getJSON(url){ const r = await fetch(url); if(!r.ok) throw new Error(r.status); return r.json(); }
async function postJSON(url, body){
  const r = await fetch(url, { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body||{}) });
  if(!r.ok) throw new Error(r.status);
  return r.json();
}

function pill(text, kind){ return '<span class="pill ' + (kind||'') + '">' + text + '</span>' }
function fmt(v, n){ if(v===null||v===undefined||isNaN(v)) return '-'; return Number(v).toFixed(n==null?1:n) }

// Update status section in-place (one element at a time) so scroll position
// and the user's caret/focus never get yanked by the periodic refresh.
function setRow(rows, key, label, value){
  let tr = document.querySelector('#statusTable tr[data-k="' + key + '"]');
  if (value === null || value === undefined){
    if (tr) tr.remove();
    return;
  }
  if (!tr){
    tr = document.createElement('tr');
    tr.setAttribute('data-k', key);
    tr.innerHTML = '<td></td><td></td>';
    rows.appendChild(tr);
  }
  tr.children[0].textContent = label;
  tr.children[1].textContent = value;
}

async function refreshStatus(){
  try {
    const s = await getJSON('/api/status');
    const pills = $('statusPills');
    const sta = s.sta_connected
      ? pill(s.sta_ssid + ' ' + s.sta_rssi + 'dBm', 'ok') + pill(s.sta_ip)
      : pill('home WiFi: not connected', 'warn');
    const ws  = s.ws_connected ? pill('relay connected','ok') : pill('relay disconnected','warn');
    const next = sta + ws;
    if (pills.dataset.last !== next){
      pills.innerHTML = next;
      pills.dataset.last = next;
    }
    const lastPost = s.last_post_ms > 0
      ? Math.round((s.uptime_ms - s.last_post_ms)/1000) + 's ago'
      : 'never';
    const lp = 'Last publish: ' + lastPost;
    if ($('statusLastPost').textContent !== lp) $('statusLastPost').textContent = lp;

    const tbl = $('statusTable');
    setRow(tbl, 'temp',     s.has_temp     ? 'Temperature' : null, s.has_temp ? fmt(s.temp_c) + ' \u00b0C' : null);
    setRow(tbl, 'humidity', s.has_humidity ? 'Humidity'    : null, s.has_humidity ? fmt(s.humidity) + ' %' : null);
    setRow(tbl, 'pressure', s.has_pressure ? 'Pressure'    : null, s.has_pressure ? fmt(s.pressure_hpa) + ' hPa' : null);
    setRow(tbl, 'lux',      s.has_lux      ? 'Light'       : null, s.has_lux ? fmt(s.lux) + ' lux' : null);
    setRow(tbl, 'pm',       s.has_pm       ? 'PM 1 / 2.5 / 10' : null, s.has_pm ? (s.pm1 + ' / ' + s.pm25 + ' / ' + s.pm10) : null);
    setRow(tbl, 'rain',     (typeof s.rain_raw === 'number' && s.rain_raw > 0) ? 'Rain (raw)' : null, (typeof s.rain_raw === 'number' && s.rain_raw > 0) ? String(s.rain_raw) : null);
    if (!tbl.children.length){
      tbl.innerHTML = '<tr data-k="none"><td colspan="2" class="muted">No sensor data yet.</td></tr>';
    }
    const top = (_cfg.station_name || 'unnamed station') + ' \u00b7 free heap: ' + s.free_heap;
    if ($('topline').textContent !== top) $('topline').textContent = top;
    if (s.device_id){
      const tag = '#' + s.device_id;
      if ($('deviceTag').textContent !== tag) $('deviceTag').textContent = tag;
    }
  } catch(e){ /* leave previous */ }
}

async function loadConfig(reveal){
  const c = await getJSON('/api/config' + (reveal ? '?reveal=1' : ''));
  _cfg = c;
  $('wifi_ssid').value = c.wifi_ssid || '';
  $('wifi_pass').value = c.wifi_pass || '';
  $('station_name').value = c.station_name || '';
  $('station_description').value = c.station_description || '';
  $('station_geohash').value = c.station_geohash || '';
  $('station_elevation').value = c.station_elevation || '';
  $('station_power').value = c.station_power || 'mains';
  $('station_connectivity').value = c.station_connectivity || 'wifi';
  $('nostr_relay').value = c.nostr_relay || '';
  $('pubkey_hex').value = c.pubkey_hex || '';
  if (reveal) $('nostr_privkey').value = c.nostr_privkey || '';
  $('en_bme280').checked = !!c.en_bme280;
  $('en_bh1750').checked = !!c.en_bh1750;
  $('en_rain').checked   = !!c.en_rain;
  $('en_pms').checked    = !!c.en_pms;
  $('pms_model').value   = c.pms_model || 'PMS5003';
  $('post_interval_s').value = Math.round((c.post_interval_ms || 60000) / 1000);
}

async function scanWifi(){
  const btn = $('scanBtn'); btn.disabled = true; btn.textContent = 'Scanning...';
  try {
    const r = await postJSON('/api/wifi/scan');
    const list = $('wifilist'); list.innerHTML = ''; list.style.display = '';
    if (!r.networks || !r.networks.length) {
      list.innerHTML = '<div class="muted" style="padding:8px">No networks found.</div>';
    } else {
      r.networks.forEach(n => {
        const div = document.createElement('div');
        div.className = 'ap';
        const bars = '|'.repeat(Math.max(1, Math.min(4, Math.round((n.rssi + 90) / 10))));
        div.innerHTML = '<span>' + n.ssid + (n.secure ? ' (lock)' : '') + '</span><span class="muted">' + bars + ' ' + n.rssi + 'dBm</span>';
        div.onclick = () => { $('wifi_ssid').value = n.ssid; toast('Selected ' + n.ssid); };
        list.appendChild(div);
      });
    }
  } catch(e){ toast('Scan failed', 'err') }
  btn.disabled = false; btn.textContent = 'Scan nearby networks';
}

function toggleWifiPass(){
  const i = $('wifi_pass'); const b = $('wifiPassToggle');
  if (i.type === 'password'){ i.type = 'text'; b.textContent = 'Hide' } else { i.type = 'password'; b.textContent = 'Show' }
}
function toggleKey(){
  const i = $('nostr_privkey'); const b = $('keyToggle');
  if (i.value === '' || i.type === 'password'){
    loadConfig(true).then(()=>{ i.type = 'text'; b.textContent = 'Hide' });
  } else { i.type = 'password'; b.textContent = 'Show' }
}
async function copyKey(){
  if (!$('nostr_privkey').value) await loadConfig(true);
  try { await navigator.clipboard.writeText($('nostr_privkey').value); toast('Copied to clipboard', 'ok') }
  catch(e){ toast('Copy failed - long-press to select', 'err') }
}
async function regenKey(){
  if (!confirm('Generate a brand-new station identity? The old key will be lost.')) return;
  try { await postJSON('/api/nostr/regenerate-key'); await loadConfig(true); toast('New key generated', 'ok') }
  catch(e){ toast('Failed', 'err') }
}
function useGPS(){
  if (!navigator.geolocation) { toast('GPS not available', 'err'); return }
  toast('Getting GPS...');
  navigator.geolocation.getCurrentPosition(p => {
    const gh = geohashEncode(p.coords.latitude, p.coords.longitude, 5);
    $('station_geohash').value = gh;
    toast('Set to ' + gh, 'ok');
  }, e => toast('GPS denied or failed', 'err'), { timeout: 15000 });
}

// geohash encoder (precision = chars)
function geohashEncode(lat, lon, precision){
  const base32 = '0123456789bcdefghjkmnpqrstuvwxyz';
  let bits = 0, bit = 0, ch = 0, even = true, gh = '';
  let latMin = -90, latMax = 90, lonMin = -180, lonMax = 180;
  while (gh.length < precision){
    let mid;
    if (even){ mid = (lonMin + lonMax) / 2; if (lon >= mid){ ch = (ch << 1) | 1; lonMin = mid } else { ch = ch << 1; lonMax = mid } }
    else     { mid = (latMin + latMax) / 2; if (lat >= mid){ ch = (ch << 1) | 1; latMin = mid } else { ch = ch << 1; latMax = mid } }
    even = !even; bit++;
    if (bit === 5){ gh += base32[ch]; bit = 0; ch = 0 }
  }
  return gh;
}

async function saveCfg(btn, patch){
  const orig = btn ? btn.textContent : null;
  if (btn){
    btn.disabled = true;
    btn.classList.remove('saved','savefail');
    btn.classList.add('saving');
    btn.textContent = 'Saving...';
  }
  try {
    await postJSON('/api/config', patch);
    toast('Saved', 'ok');
    if (btn){
      btn.classList.remove('saving');
      btn.classList.add('saved');
      btn.textContent = 'Saved!';
      setTimeout(()=>{
        btn.classList.remove('saved');
        btn.textContent = orig;
        btn.disabled = false;
      }, 1800);
    }
    // Re-fetch so the displayed values reflect what's now stored (incl. any
    // server-side validation that may have clamped numbers etc.)
    await loadConfig();
    refreshStatus();
  } catch(e){
    toast('Save failed', 'err');
    if (btn){
      btn.classList.remove('saving');
      btn.classList.add('savefail');
      btn.textContent = 'Failed - retry?';
      setTimeout(()=>{
        btn.classList.remove('savefail');
        btn.textContent = orig;
        btn.disabled = false;
      }, 2500);
    }
  }
}
function saveWifi(btn){
  saveCfg(btn, { wifi_ssid: $('wifi_ssid').value.trim(), wifi_pass: $('wifi_pass').value });
}
function saveIdentity(btn){
  saveCfg(btn, {
    station_name: $('station_name').value.trim(),
    station_description: $('station_description').value.trim(),
    station_geohash: $('station_geohash').value.trim(),
    station_elevation: $('station_elevation').value.trim(),
    station_power: $('station_power').value,
    station_connectivity: $('station_connectivity').value,
  });
}
function saveNostr(btn){
  const k = $('nostr_privkey').value.trim();
  const patch = { nostr_relay: $('nostr_relay').value.trim() };
  if (k && k.length === 64) patch.nostr_privkey = k;
  else if (k && k.length > 0) { toast('Private key must be 64 hex chars', 'err'); return }
  saveCfg(btn, patch);
}
function saveSensors(btn){
  saveCfg(btn, {
    en_bme280: $('en_bme280').checked,
    en_bh1750: $('en_bh1750').checked,
    en_rain:   $('en_rain').checked,
    en_pms:    $('en_pms').checked,
    pms_model: $('pms_model').value,
  });
}
function saveAdvanced(btn){
  const s = parseInt($('post_interval_s').value, 10);
  if (!s || s < 10) { toast('Interval must be at least 10s', 'err'); return }
  saveCfg(btn, { post_interval_ms: s * 1000 });
}
async function restart(){
  if (!confirm('Restart the station now?')) return;
  try { await postJSON('/api/restart'); toast('Restarting...', 'ok') } catch(e){}
}
async function factoryReset(){
  if (!confirm('Wipe ALL settings and restart? You will need to set everything up again.')) return;
  try { await postJSON('/api/factory-reset'); toast('Resetting...', 'ok') } catch(e){}
}

loadConfig().then(refreshStatus);
setInterval(refreshStatus, 5000);
</script>
</body>
</html>
)HTML";

  // --- Helpers ---
  void sendJson(int code, const String& body) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(code, "application/json", body);
  }

  void sendOk() { sendJson(200, "{\"ok\":true}"); }

  void redirectToRoot() {
    server.sendHeader("Location", String("http://") + AP_IP.toString() + "/", true);
    server.send(302, "text/plain", "");
  }

  // --- Route handlers ---
  void handleRoot() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", DASHBOARD_HTML);
  }

  void handleStatus() {
    StaticJsonDocument<1024> doc;
    doc["sta_connected"] = status.sta_connected;
    doc["sta_ssid"]      = status.sta_ssid;
    doc["sta_ip"]        = status.sta_ip;
    doc["sta_rssi"]      = status.sta_rssi;
    doc["ws_connected"]  = status.ws_connected;
    doc["last_post_ms"]  = status.last_post_ms;
    doc["uptime_ms"]     = (uint32_t)millis();
    doc["free_heap"]     = (uint32_t)ESP.getFreeHeap();

    doc["has_temp"]     = status.has_temp;
    doc["has_humidity"] = status.has_humidity;
    doc["has_pressure"] = status.has_pressure;
    doc["has_lux"]      = status.has_lux;
    doc["has_pm"]       = status.has_pm;
    if (status.has_temp)     doc["temp_c"]       = status.temp_c;
    if (status.has_humidity) doc["humidity"]     = status.humidity;
    if (status.has_pressure) doc["pressure_hpa"] = status.pressure_hpa;
    if (status.has_lux)      doc["lux"]          = status.lux;
    doc["rain_raw"] = status.rain_raw;
    doc["pm1"]      = status.pm1;
    doc["pm25"]     = status.pm25;
    doc["pm10"]     = status.pm10;
    doc["pubkey_hex"] = status.pubkey_hex;
    doc["device_id"]  = status.device_id;

    String out; serializeJson(doc, out);
    sendJson(200, out);
  }

  void handleGetConfig() {
    bool reveal = server.hasArg("reveal") && server.arg("reveal") == "1";
    auto& c = config_store::current();
    StaticJsonDocument<1536> doc;
    doc["wifi_ssid"]            = c.wifi_ssid;
    doc["wifi_pass"]            = reveal ? c.wifi_pass : "";
    doc["nostr_relay"]          = c.nostr_relay;
    doc["nostr_privkey"]        = reveal ? c.nostr_privkey : "";
    doc["station_name"]         = c.station_name;
    doc["station_description"]  = c.station_description;
    doc["station_geohash"]      = c.station_geohash;
    doc["station_elevation"]    = c.station_elevation;
    doc["station_power"]        = c.station_power;
    doc["station_connectivity"] = c.station_connectivity;
    doc["post_interval_ms"]     = c.post_interval_ms;
    doc["en_bme280"]            = c.en_bme280;
    doc["en_bh1750"]            = c.en_bh1750;
    doc["en_rain"]              = c.en_rain;
    doc["en_pms"]               = c.en_pms;
    doc["pms_model"]            = c.pms_model;
    doc["pubkey_hex"]           = status.pubkey_hex;
    String out; serializeJson(doc, out);
    sendJson(200, out);
  }

  void handlePostConfig() {
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"missing body\"}"); return; }
    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) { sendJson(400, String("{\"error\":\"") + err.c_str() + "\"}"); return; }

    auto& c = config_store::current();
    bool wifiChanged = false, relayChanged = false, keyChanged = false;
    bool sensorsChanged = false, intervalChanged = false;

    auto setStr = [&](const char* k, String& dst, bool& changed) {
      if (doc.containsKey(k)) {
        String v = doc[k].as<String>();
        if (v != dst) { dst = v; changed = true; }
      }
    };

    {
      bool ssidCh = false, passCh = false;
      setStr("wifi_ssid", c.wifi_ssid, ssidCh);
      setStr("wifi_pass", c.wifi_pass, passCh);
      wifiChanged = ssidCh || passCh;
    }
    setStr("nostr_relay",          c.nostr_relay,          relayChanged);
    {
      bool dummy = false;
      setStr("station_name",        c.station_name,        dummy);
      setStr("station_description", c.station_description, dummy);
      setStr("station_geohash",     c.station_geohash,     dummy);
      setStr("station_elevation",   c.station_elevation,   dummy);
      setStr("station_power",       c.station_power,       dummy);
      setStr("station_connectivity",c.station_connectivity,dummy);
    }
    if (doc.containsKey("nostr_privkey")) {
      String k = doc["nostr_privkey"].as<String>();
      if (k.length() == 64 && config_store::isValidPrivKeyHex(k) && k != c.nostr_privkey) {
        c.nostr_privkey = k;
        keyChanged = true;
      } else if (k.length() != 64 && k.length() != 0) {
        sendJson(400, "{\"error\":\"private key must be 64 hex chars\"}");
        return;
      }
    }
    if (doc.containsKey("post_interval_ms")) {
      uint32_t v = doc["post_interval_ms"].as<uint32_t>();
      if (v < 10000) v = 10000;
      if (v != c.post_interval_ms) { c.post_interval_ms = v; intervalChanged = true; }
    }
    auto setBool = [&](const char* k, bool& dst) {
      if (doc.containsKey(k)) {
        bool v = doc[k].as<bool>();
        if (v != dst) { dst = v; sensorsChanged = true; }
      }
    };
    setBool("en_bme280", c.en_bme280);
    setBool("en_bh1750", c.en_bh1750);
    setBool("en_rain",   c.en_rain);
    setBool("en_pms",    c.en_pms);
    {
      bool dummy = false;
      String prev = c.pms_model;
      setStr("pms_model", c.pms_model, dummy);
      if (c.pms_model != prev) sensorsChanged = true;
    }

    config_store::save();
    if (cb.onApply) cb.onApply(wifiChanged, relayChanged, keyChanged, sensorsChanged, intervalChanged);
    sendOk();
  }

  void handleScan() {
    int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/false);
    StaticJsonDocument<2048> doc;
    JsonArray arr = doc.createNestedArray("networks");
    for (int i = 0; i < n && i < 20; i++) {
      JsonObject o = arr.createNestedObject();
      o["ssid"]   = WiFi.SSID(i);
      o["rssi"]   = WiFi.RSSI(i);
      o["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    sendJson(200, out);
  }

  void handleRegenKey() {
    auto& c = config_store::current();
    c.nostr_privkey = config_store::generateNostrPrivKeyHex();
    config_store::save();
    if (cb.onRegenerateKey) cb.onRegenerateKey();
    if (cb.onApply) cb.onApply(false, false, true, false, false);
    sendOk();
  }

  void handleRestart() {
    sendOk();
    delay(100);
    if (cb.onRestart) cb.onRestart();
    else ESP.restart();
  }

  void handleFactoryReset() {
    sendOk();
    delay(100);
    if (cb.onFactoryReset) cb.onFactoryReset();
    else {
      config_store::factoryReset();
      ESP.restart();
    }
  }

  void handleCaptiveProbe() {
    // Return a 302 to /. iOS / Android need this to recognise the network as
    // captive and pop the configuration sheet. (Returning a 200 with HTML
    // makes iOS treat the network as broken; returning "Success" makes it
    // dismiss the sheet entirely.)
    redirectToRoot();
  }

  void handleNotFound() {
    // Any other URL on the AP side (e.g. user typed something) -> dashboard.
    redirectToRoot();
  }
}

namespace web_dashboard {

void begin(const DashboardCallbacks& cbIn) {
  cb = cbIn;
  WiFi.mode(WIFI_AP_STA);
  g_ap_ssid = String("WeatherStation-") + macSuffix();
  WiFi.softAPConfig(AP_IP, AP_IP, AP_NETMASK);
  WiFi.softAP(g_ap_ssid.c_str());  // open network

  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(DNS_PORT, "*", AP_IP);

  // mDNS on STA side (best-effort)
  MDNS.begin("weather");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status",  HTTP_GET,  handleStatus);
  server.on("/api/config",  HTTP_GET,  handleGetConfig);
  server.on("/api/config",  HTTP_POST, handlePostConfig);
  server.on("/api/wifi/scan", HTTP_POST, handleScan);
  server.on("/api/nostr/regenerate-key", HTTP_POST, handleRegenKey);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);

  // Captive-portal probes
  server.on("/generate_204",              HTTP_GET, handleCaptiveProbe);  // Android
  server.on("/gen_204",                   HTTP_GET, handleCaptiveProbe);
  server.on("/hotspot-detect.html",       HTTP_GET, handleCaptiveProbe);  // iOS / macOS
  server.on("/library/test/success.html", HTTP_GET, handleCaptiveProbe);
  server.on("/connecttest.txt",           HTTP_GET, handleCaptiveProbe);  // Windows
  server.on("/redirect",                  HTTP_GET, handleCaptiveProbe);
  server.on("/ncsi.txt",                  HTTP_GET, handleCaptiveProbe);

  server.onNotFound(handleNotFound);
  server.begin();

  Serial.print("[dash] AP SSID: ");      Serial.println(g_ap_ssid);
  Serial.print("[dash] AP IP: ");        Serial.println(WiFi.softAPIP());
  Serial.println("[dash] HTTP server on port 80");
}

void handle() {
  dns.processNextRequest();
  server.handleClient();
}

void updateStatus(const DashboardStatus& s) { status = s; }
String apIp()   { return WiFi.softAPIP().toString(); }
String apSsid() { return g_ap_ssid; }

}  // namespace web_dashboard
