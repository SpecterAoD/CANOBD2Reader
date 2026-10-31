#include "WebAssets.h"

namespace WebAssets {

const char* simulationScenariosJsonArray() {
    return "['NormalSingleFrame','NormalMultiFrameVin','NormalMultiFrameDtc',"
           "'FlowControlRequired','TimeoutAfterFirstFrame','SequenceError',"
           "'BufferOverflow','MultipleEcusResponse','NegativeResponse',"
           "'DisplayNormalValues','DisplayWarningValues','DisplayCriticalValues',"
           "'DisplayTimeoutValues','DisplayMixedValues']";
}

const char* commonScript() {
    return R"JS(
const $=id=>document.getElementById(id);
// HTML-escape every value that comes from the device, the vehicle or the manifest.
function esc(v){return String(v??'--').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
async function api(url,opt){let r=await fetch(url,Object.assign({cache:'no-store',credentials:'same-origin'},opt||{}));if(!r.ok)throw new Error(url+': HTTP '+r.status+' '+(await r.text()));return r}
async function post(url,body){return api(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body||''})}
)JS";
}

const char* githubUpdatePage() {
    return R"HTML(<!doctype html><html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CANOBD2 %%DEVICE%% Updates</title><style>
body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;background:#07101d;color:#f8fafc;margin:0 auto;padding:14px;max-width:760px}
h1{font-size:1.45rem;margin:8px 0 12px}h2{font-size:1.05rem;margin:0 0 8px}.card{background:#182235;border:1px solid #263244;border-radius:16px;padding:14px;margin:10px 0}
button,input,select{width:100%;box-sizing:border-box;padding:12px;margin:6px 0;border-radius:12px;border:1px solid #334155;background:#0f172a;color:#f8fafc;font-size:15px}
button.primary{background:#0369a1}button.warn{background:#9a3412}a{color:#38bdf8}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:8px}.pill{padding:8px;border-radius:10px;background:#0f172a}.ok{color:#22c55e}.bad{color:#ef4444}.muted{color:#94a3b8}
table{width:100%;border-collapse:collapse;font-size:.9rem}th,td{border-bottom:1px solid #334155;padding:7px;text-align:left}.small{font-size:.84rem;color:#94a3b8}
</style></head><body>
<h1>Firmware-Updates &ndash; %%DEVICE%%</h1>
<p class="small">Es werden nur vom Projekt signierte Firmware-Dateien akzeptiert (Ed25519). Unsignierte oder veraenderte Dateien werden vor dem Umschalten verworfen. <a href="/">Zurueck</a></p>
<div class="card"><h2>Datei hochladen</h2><form id="uploadForm"><input type="file" id="file" accept=".bin"><button class="primary" type="submit">Signierte Firmware installieren</button></form><div id="uploadState" class="small">--</div></div>
<div class="card"><h2>WLAN / Hotspot fuer GitHub</h2><input id="ssid" placeholder="SSID" autocomplete="off"><input id="pass" placeholder="Passwort" type="password" autocomplete="new-password"><button class="primary" id="wifiSave">Speichern</button><button id="wifiConnect">Verbinden</button><button id="wifiForget">Vergessen</button><div id="wifi" class="small">--</div></div>
<div class="card"><h2>GitHub</h2><select id="channel"><option value="stable">Stable</option><option value="beta">Beta</option><option value="development">Development</option></select><button id="setChannel">Kanal setzen</button><button class="primary" id="check">Nach Updates suchen</button><button class="primary" id="installLatest">Neueste passende Version installieren</button><div id="status" class="kv"></div></div>
<div class="card"><h2>Versionen</h2><div class="small">Rollback ist nur manuell mit Bestaetigung moeglich.</div><table><thead><tr><th>Version</th><th>Kanal</th><th>Status</th><th></th></tr></thead><tbody id="versions"><tr><td colspan="4">Noch kein Manifest geladen.</td></tr></tbody></table></div>
<script>%%COMMON_JS%%
function pill(label,value,cls){return '<div class="pill">'+esc(label)+'<br><b class="'+(cls||'')+'">'+esc(value)+'</b></div>'}
async function act(fn){try{await fn()}catch(e){alert(e.message)}await refresh()}
$('wifiSave').onclick=()=>act(()=>post('/api/wifi/configure','ssid='+encodeURIComponent($('ssid').value)+'&password='+encodeURIComponent($('pass').value)));
$('wifiConnect').onclick=()=>act(()=>post('/api/wifi/connect'));
$('wifiForget').onclick=()=>act(()=>post('/api/wifi/forget'));
$('setChannel').onclick=()=>act(()=>post('/api/update/channel','channel='+encodeURIComponent($('channel').value)));
$('check').onclick=()=>act(()=>post('/api/update/check'));
$('installLatest').onclick=()=>{if(confirm('Neueste passende Firmware installieren? Das Geraet startet danach neu.'))act(()=>post('/api/update/install'))};
function install(version,status){let body='version='+encodeURIComponent(version);if(status==='rollback'){if(!confirm('Rollback auf '+version+' installieren?'))return;body+='&confirm=rollback'}else if(!confirm('Firmware '+version+' installieren?'))return;act(()=>post('/api/update/install',body))}
$('versions').onclick=e=>{let b=e.target.closest('button[data-version]');if(b)install(b.dataset.version,b.dataset.status)};
$('uploadForm').onsubmit=async e=>{e.preventDefault();let f=$('file').files[0];if(!f)return;let d=new FormData();d.append('firmware',f,f.name);$('uploadState').textContent='Upload laeuft ...';try{let r=await fetch('/update',{method:'POST',body:d,credentials:'same-origin'});$('uploadState').textContent=await r.text()}catch(err){$('uploadState').textContent='Fehler: '+err.message}};
function renderVersions(v){let rows=(v.versions||[]).map(x=>{let action='<span class="muted">'+esc(x.status)+'</span>';if(x.status==='newer'||x.status==='rollback'){action='<button class="'+(x.status==='newer'?'primary':'warn')+'" data-version="'+esc(x.version)+'" data-status="'+esc(x.status)+'">'+(x.status==='newer'?'Installieren':'Rollback')+'</button>'}return '<tr><td><b>'+esc(x.version)+'</b><br><span class="small">'+esc(x.createdAt||'')+'</span></td><td>'+esc(x.channel)+'</td><td>'+esc(x.status)+'</td><td>'+action+'</td></tr>'}).join('');$('versions').innerHTML=rows||'<tr><td colspan="4">Keine Versionen.</td></tr>'}
async function refresh(){try{let w=await (await api('/api/wifi/status')).json();let u=await (await api('/api/update/status')).json();let v=await (await api('/api/update/versions')).json();
$('wifi').textContent='WLAN: '+(w.connected?'verbunden '+(w.ip||''):'offline');$('channel').value=(u.channel||'stable').toLowerCase();
$('status').innerHTML=pill('Installiert',u.installed)+pill('Verfuegbar',u.availableVersion||'--',u.updateAvailable?'ok':'muted')+pill('Geraet',u.target)+pill('Fehler',u.lastError||'--',u.lastError?'bad':'ok');renderVersions(v)}catch(e){$('wifi').textContent='Fehler: '+e.message}}
refresh();setInterval(refresh,5000);
</script></body></html>)HTML";
}

} // namespace WebAssets
