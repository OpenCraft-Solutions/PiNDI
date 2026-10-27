'use strict';
// PiNDI Endpoint Manager front end: three pages over /api/state (polled), no libraries.

const $ = (sel, el = document) => el.querySelector(sel);
const $$ = (sel, el = document) => [...el.querySelectorAll(sel)];
const esc = s => String(s == null ? '' : s).replace(/[&<>"']/g,
  c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

const COLORS = ['#2bb3a3', '#3fb950', '#d29922', '#f778ba', '#a371f7', '#58a6ff', '#9bd36b', '#e5534b',
  '#79c0ff', '#f0883e', '#56d4dd', '#d2a8ff'];

let session = null;
let devices = [];
let history = {};
let discover = null;
const scanSelected = new Set();
let page = 'monitor';
let pollTimer = null;
const selected = new Set();
const busy = new Set(); // matrix crosspoints being switched

// ------------------------------------------------------------------ helpers

async function api(method, path, body, raw, headers) {
  const opt = { method, credentials: 'same-origin', headers: { 'X-PiNDI-Manager': '1', ...headers } };
  if (raw) {
    opt.body = raw;
    opt.headers['Content-Type'] = 'application/octet-stream';
  } else if (body !== undefined) {
    opt.body = JSON.stringify(body);
    opt.headers['Content-Type'] = 'application/json';
  }
  const r = await fetch(path, opt);
  if (r.status === 401 && path !== '/api/login') {
    showLogin();
    throw new Error('Signed out: sign in again.');
  }
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || 'HTTP ' + r.status);
  return data;
}

let toastTimer;
function toast(msg, bad) {
  const t = $('#toast');
  t.textContent = msg;
  t.className = 'toast' + (bad ? ' bad' : '');
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, bad ? 6000 : 3000);
}

const mbit = bps => bps == null ? '–' : bps >= 1e5 ? (bps / 1e6).toFixed(1) + ' Mbit/s' : (bps / 1e3).toFixed(0) + ' kbit/s';
const num = (v, d = 0, unit = '') => v == null ? '–' : Number(v).toFixed(d) + unit;
function duration(sec) {
  if (sec == null) return '–';
  const d = Math.floor(sec / 86400), h = Math.floor(sec / 3600) % 24, m = Math.floor(sec / 60) % 60;
  return d ? `${d} d ${h} h` : h ? `${h} h ${m} min` : `${m} min`;
}
function ago(t) {
  if (!t) return 'never';
  const s = Math.max(0, Date.now() / 1000 - t);
  return s < 60 ? Math.round(s) + ' s ago' : s < 3600 ? Math.round(s / 60) + ' min ago' : Math.round(s / 3600) + ' h ago';
}
const color = id => COLORS[Math.max(0, devices.findIndex(d => d.id === id)) % COLORS.length];
const playing = o => /^playing/i.test(o.state || '');
const devName = d => (d.status && d.status.device && d.status.device.name) || d.name;

function deviceBadge(d) {
  if (d.cert_changed) return '<span class="badge bad">certificate changed</span>';
  if (!d.online) return `<span class="badge bad" title="${esc(d.error)}">offline</span>`;
  const power = d.status.device.power;
  if (power && power !== 'OK') return `<span class="badge warn" title="${esc(power)}">warning</span>`;
  return '<span class="badge ok">online</span>';
}

// ------------------------------------------------------------------ monitoring

function renderMonitor() {
  const online = devices.filter(d => d.online);
  const outs = online.flatMap(d => d.status.outputs || []);
  const ins = online.flatMap(d => (d.status.inputs || []).filter(i => i.enabled));
  const alerts = devices.filter(d => !d.online || d.cert_changed ||
    (d.status && d.status.device.power && d.status.device.power !== 'OK')).length;
  const tile = (label, value, total, bad) =>
    `<div class="tile${bad ? ' bad' : ''}"><div class="label">${label}</div><div class="value">${value}` +
    `${total != null ? ` <small>/ ${total}</small>` : ''}</div></div>`;
  $('#tiles').innerHTML =
    tile('Devices online', online.length, devices.length, online.length < devices.length) +
    tile('Outputs showing a stream', outs.filter(playing).length, outs.length) +
    tile('Streams on air', ins.filter(i => i.on_air).length, ins.length) +
    tile('Alerts', alerts, null, alerts > 0);
  $('#empty-hint').hidden = devices.length > 0;

  $('#legend').innerHTML = devices.map(d =>
    `<span><span class="swatch" style="background:${color(d.id)}"></span>${esc(devName(d))}</span>`).join('');
  const series = key => devices.map(d => ({
    color: color(d.id), points: (history[d.id] || []).filter(p => p[key] != null).map(p => [p.t, p[key]]),
  }));
  const mbitSeries = key => series(key).map(s => ({ ...s, points: s.points.map(([t, v]) => [t, v / 1e6]) }));
  $('#charts').innerHTML =
    chart('CPU temperature (°C)', series('cpu_temp'), v => v.toFixed(0)) +
    chart('CPU load', series('load'), v => v.toFixed(1)) +
    chart('Network in (Mbit/s)', mbitSeries('rx_bps'), v => v.toFixed(0)) +
    chart('Network out (Mbit/s)', mbitSeries('tx_bps'), v => v.toFixed(0)) +
    chart('Memory used (%)', series('memory_pct'), v => v.toFixed(0), 100);

  $('#dev-table').innerHTML = '<thead><tr><th></th><th>Device</th><th>Address</th><th>Model</th><th>Version</th>' +
    '<th>Temp</th><th>Load</th><th>Memory</th><th>Storage</th><th>In</th><th>Out</th><th>Uptime</th>' +
    '<th>Outputs</th><th>Status</th></tr></thead><tbody>' + devices.map(d => {
      const s = d.status, m = (s && s.device.metrics) || {};
      const o = (s && s.outputs) || [];
      if (!d.online) {
        return `<tr><td><span class="swatch" style="background:${color(d.id)}"></span></td>` +
          `<td>${esc(devName(d))}</td><td>${esc(d.address)}</td><td colspan="10" class="muted wrap">` +
          `${esc(d.error)} · last seen ${ago(d.seen)}</td><td>${deviceBadge(d)}</td></tr>`;
      }
      return `<tr><td><span class="swatch" style="background:${color(d.id)}"></span></td><td>${esc(devName(d))}</td>` +
        `<td>${esc(d.address)}</td><td>${esc(s.device.model)}</td><td>${esc(s.device.version)}</td>` +
        `<td>${num(m.cpu_temp, 0, ' °C')}</td><td>${num(m.load, 2)}${m.cores ? ` <span class="muted">/ ${m.cores}</span>` : ''}</td>` +
        `<td>${num(m.memory_pct, 0, ' %')}</td><td>${num(m.storage_pct, 0, ' %')}</td>` +
        `<td>${mbit(m.rx_bps)}</td><td>${mbit(m.tx_bps)}</td><td>${duration(m.uptime)}</td>` +
        `<td>${o.filter(playing).length} / ${o.length}</td>` +
        `<td title="${esc(s.device.health + '\n' + s.device.power)}">${deviceBadge(d)}</td></tr>`;
    }).join('') + '</tbody>';

  const outRows = online.flatMap(d => (d.status.outputs || []).map(o =>
    `<tr><td>${esc(devName(d))}</td><td>${o.n} <span class="muted">${esc(o.connector)}</span></td>` +
    `<td><span class="dot ${!o.enabled ? '' : playing(o) ? 'ok' : 'warn'}"></span> ${esc(o.enabled ? o.state : 'disabled')}</td>` +
    `<td>${esc(o.source || o.target)}</td><td>${esc(o.video)}</td><td>${esc(o.latency)}</td></tr>`));
  $('#out-table').innerHTML = '<thead><tr><th>Device</th><th>Output</th><th>State</th><th>Source</th><th>Video</th>' +
    '<th>Latency</th></tr></thead><tbody>' + (outRows.join('') || '<tr><td colspan="6" class="muted">None</td></tr>') + '</tbody>';

  const inRows = online.flatMap(d => (d.status.inputs || []).map(i =>
    `<tr><td>${esc(devName(d))}</td><td>${esc(i.name)}</td>` +
    `<td><span class="dot ${i.on_air ? 'ok' : i.enabled ? 'warn' : ''}"></span> ${esc(i.state)}</td>` +
    `<td>${esc(i.mode)}</td><td>${esc(i.fps)}</td><td>${esc(i.audio)}</td></tr>`));
  $('#in-table').innerHTML = '<thead><tr><th>Device</th><th>Stream</th><th>State</th><th>Mode</th><th>FPS</th>' +
    '<th>Audio</th></tr></thead><tbody>' + (inRows.join('') || '<tr><td colspan="6" class="muted">None configured</td></tr>') + '</tbody>';
}

// A line chart as SVG: one line per device, shared time axis (what there is of the last hour, at least 5 min).
function chart(title, series, fmt, fixedMax) {
  const W = 560, H = 170, L = 38, R = 8, T = 8, B = 20;
  const now = Date.now() / 1000;
  const first = Math.min(now, ...series.flatMap(s => s.points.map(p => p[0])));
  const t0 = Math.max(now - 3600, Math.min(first, now - 300)), span = now - t0;
  const all = series.flatMap(s => s.points.filter(p => p[0] >= t0).map(p => p[1]));
  let svg = `<line class="grid" x1="${L}" x2="${W - R}" y1="${H - B}" y2="${H - B}"/>`;
  if (!all.length) {
    svg += `<text class="empty" x="${W / 2}" y="${H / 2}" text-anchor="middle">No data yet</text>`;
  } else {
    let max = fixedMax || Math.max(...all) * 1.15 || 1;
    const step = niceStep(max / 4);
    max = fixedMax || Math.ceil(max / step) * step;
    const x = t => L + (t - t0) / span * (W - L - R), y = v => H - B - v / max * (H - T - B);
    for (let v = 0; v <= max + 1e-9; v += fixedMax ? fixedMax / 4 : step) {
      svg += `<line class="grid" x1="${L}" x2="${W - R}" y1="${y(v)}" y2="${y(v)}"/>` +
        `<text class="axis" x="${L - 6}" y="${y(v) + 3}" text-anchor="end">${fmt(v)}</text>`;
    }
    for (const s of series) {
      const pts = s.points.filter(p => p[0] >= t0);
      if (pts.length) {
        svg += `<polyline class="series" stroke="${s.color}" points="${pts.map(p => `${x(p[0]).toFixed(1)},${y(p[1]).toFixed(1)}`).join(' ')}"/>`;
      }
    }
  }
  const hm = t => new Date(t * 1000).toTimeString().slice(0, 5);
  svg += `<text class="axis" x="${L}" y="${H - 5}">${hm(t0)}</text>` +
    `<text class="axis" x="${W - R}" y="${H - 5}" text-anchor="end">${hm(now)}</text>`;
  return `<div class="chart"><h3>${esc(title)}</h3><svg viewBox="0 0 ${W} ${H}" role="img" aria-label="${esc(title)}">${svg}</svg></div>`;
}

function niceStep(raw) {
  const p = Math.pow(10, Math.floor(Math.log10(raw || 1)));
  const f = raw / p;
  return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p;
}

// ------------------------------------------------------------------ video matrix

// A device sees its own outgoing streams at a loopback address: that is its network address for everyone else.
const loopback = ip => /^(127\.|::1$|localhost$)/i.test(ip || '');

// "None": the output waits for a stream called "none" on the device itself, which nothing sends, so it shows
// its splash screen. Always locked to 127.0.0.1, so no stream of that name elsewhere is picked up.
const NONE = { name: 'None', stream: 'none', ip: '127.0.0.1' };
const isNone = o => (o.stream || '').toLowerCase() === NONE.stream && o.source_ip === NONE.ip;

function sources() {
  // Every source any online device sees, once per name and sender address.
  const map = new Map();
  for (const d of devices.filter(d => d.online)) {
    for (const s of d.status.sources || []) {
      const ip = loopback(s.ip) && d.status.device.ip ? d.status.device.ip : s.ip;
      const key = s.name + '\t' + ip;
      if (!map.has(key)) map.set(key, { ...s, ip, pindi: false });
    }
  }
  for (const d of devices.filter(d => d.online)) {
    for (const i of d.status.inputs || []) {
      for (const s of map.values()) {
        if (s.ip === d.status.device.ip && s.stream.toLowerCase() === i.name.toLowerCase()) s.pindi = true;
      }
      // An input sent as H.264 RTP to a multicast group: any output can join it (rtp://group:port). Unicast
      // RTP goes to one fixed receiver, so it can't be routed from here.
      const ip = d.status.device.ip, name = `${d.status.device.ndi_name || devName(d)} (${i.name})`;
      if (i.enabled && i.codec === 'rtp' && isMulticast(i.rtp_dest)) {
        const stream = 'rtp://' + i.rtp_dest;
        map.set(stream + '\t' + ip, { name, stream, ip, pindi: true, kind: 'H.264 RTP' });
      }
      // An input sent as H.264 SRT that listens: outputs call it (srt://device:port, up to 8 of them). One that
      // calls its receiver, or is encrypted (the API doesn't give out the passphrase), can't be routed here.
      const port = i.enabled && i.codec === 'srt' && ip ? srtListenPort(i.srt_url) : null;
      if (port) {
        const latency = /[?&]latency=(\d+)/.exec(i.srt_url);
        const stream = `srt://${ip}:${port}` + (latency ? '?latency=' + latency[1] : '');
        map.set(stream + '\t' + ip, { name, stream, ip, pindi: true, kind: 'H.264 SRT' });
      }
    }
  }
  return [...map.values()].sort((a, b) => a.name.localeCompare(b.name));
}

const isMulticast = dest => { const a = Number(/^(\d+)\./.exec(dest || '')?.[1]); return a >= 224 && a <= 239; };
// The port of an input's SRT address if it listens unencrypted ("srt://:9000", "srt://0.0.0.0:9000?mode=listener"),
// else null. As srt_url() in pindi-web.py.
const srtListenPort = url => {
  const m = /^srt:\/\/@?([^:?\s]*):(\d+)(\?\S*)?$/i.exec(url || '');
  if (!m || / \(encrypted\)$/.test(url)) return null;
  const mode = /[?&]mode=(\w+)/i.exec(m[3] || '');
  const listen = mode ? mode[1].toLowerCase() === 'listener' : !m[1] || m[1] === '0.0.0.0';
  return listen ? Number(m[2]) : null;
};

function renderMatrix() {
  const outs = devices.filter(d => d.status).flatMap(d => (d.status.outputs || []).map(o => ({ d, o })));
  const filter = $('#matrix-filter').value.trim().toLowerCase();
  const srcs = sources().filter(s => !filter || (s.name + ' ' + s.ip).toLowerCase().includes(filter));
  const thumbs = $('#show-thumbs').checked;
  if (!outs.length) {
    $('#matrix').innerHTML = '<tbody><tr><td class="muted">No PiNDI outputs: add devices under Admin.</td></tr></tbody>';
    return;
  }
  const stamp = Math.floor(Date.now() / 5000);
  let html = '<thead><tr><th class="src corner">Sources ↓ &nbsp; Outputs →</th>' + outs.map(({ d, o }) =>
    `<th class="${d.online ? '' : 'offline'}">` +
    (thumbs && d.online ? (d.status.api_version >= 2 ?
      `<img alt="" src="/api/thumb?id=${encodeURIComponent(d.id)}&n=${o.n}&_=${stamp}">` :
      '<span class="nothumb">no preview<br>(agent before 1.3.0)</span>') : '') +
    `${esc(devName(d))}<span class="sub">Out ${o.n} · ${esc(o.connector)}</span>` +
    `<span class="sub"><span class="dot ${!d.online ? 'bad' : playing(o) ? 'ok' : 'warn'}"></span> ` +
    `${esc(d.online ? (o.enabled ? o.state : 'disabled') : 'offline')}</span></th>`).join('') + '</tr></thead><tbody>';

  const cell = (d, o, s, routed, live) => {
    const key = `${d.id}/${o.n}/${s ? s.name + '\t' + s.ip : ''}`;
    const cls = busy.has(key) ? 'busy' : live ? 'live' : routed ? 'routed' : '';
    // Agents before API version 4 would take an rtp:// or srt:// address for an NDI stream name.
    const old = !!(s && s.kind) && !(d.status.api_version >= 4);
    const label = old ? `${devName(d)} can't show ${s.kind} streams: update it to PiNDI 1.7.0 or newer` :
      s ? `${s.name} to ${devName(d)} output ${o.n}` : `${devName(d)} output ${o.n} back to its own stream`;
    return `<td><button class="${cls}" ${d.online && !old ? '' : 'disabled'} title="${esc(label)}" aria-label="${esc(label)}" ` +
      `aria-pressed="${routed}" data-dev="${esc(d.id)}" data-out="${o.n}" data-key="${esc(key)}" ` +
      `data-stream="${esc(s ? s.stream : '')}" data-ip="${esc(s ? s.ip : '')}"></button></td>`;
  };
  html += '<tr class="default"><th class="src">Own stream (OUT_&lt;serial&gt;)<span class="sub">the device default</span></th>' +
    outs.map(({ d, o }) => cell(d, o, null, !o.stream, !o.stream && playing(o))).join('') + '</tr>';
  html += '<tr class="default"><th class="src">None (splash screen)<span class="sub">a stream nothing sends</span></th>' +
    outs.map(({ d, o }) => { const r = isNone(o); return cell(d, o, NONE, r, r && d.online); }).join('') + '</tr>';
  // "HOST (STREAM)": the host ends at the first " (" (a bridge's stream names have brackets of their own).
  const host = s => { const m = /^(.*?) \((.*)\)$/.exec(s.name); return m ? m[1] : s.name; };
  html += srcs.map(s => `<tr><th class="src">${esc(s.stream)}${s.pindi ? '<span class="tag">PiNDI</span>' : ''}` +
    `${s.kind ? `<span class="tag">${s.kind}</span>` : ''}<span class="sub">${esc(s.kind ? s.name : host(s))} · ${esc(s.ip)}</span></th>` +
    outs.map(({ d, o }) => {
      const routed = !!o.stream && o.target.toLowerCase() === s.stream.toLowerCase() && (!o.source_ip || o.source_ip === s.ip);
      // An RTP or SRT output's source is the rtp:// or srt:// address it receives.
      return cell(d, o, s, routed, routed && playing(o) && o.source === (s.kind ? s.stream : s.name));
    }).join('') + '</tr>').join('');
  if (!srcs.length) {
    html += `<tr><td class="muted" colspan="${outs.length + 1}">${filter ? 'No source matches the filter.' : 'No NDI sources seen yet.'}</td></tr>`;
  }
  $('#matrix').innerHTML = html + '</tbody>';
}

async function route(btn) {
  const key = btn.dataset.key;
  if (busy.has(key)) return;
  busy.add(key);
  renderMatrix();
  try {
    await api('POST', '/api/route', {
      id: btn.dataset.dev, output: Number(btn.dataset.out), stream: btn.dataset.stream,
      source_ip: btn.dataset.ip === NONE.ip || $('#lock-ip').checked ? btn.dataset.ip : '',
    });
    await refresh();
  } catch (e) {
    toast('Switching failed: ' + e.message, true);
  } finally {
    busy.delete(key);
    renderMatrix();
  }
}

// ------------------------------------------------------------------ admin

function renderAdmin() {
  for (const id of [...selected]) if (!devices.some(d => d.id === id)) selected.delete(id);
  const all = devices.length && devices.every(d => selected.has(d.id));
  const updateText = { running: 'installing', failed: 'failed', done: 'done, rebooting' };
  $('#admin-table').innerHTML = `<thead><tr><th><input type="checkbox" id="sel-all" aria-label="Select all" ${all ? 'checked' : ''}></th>` +
    '<th>Device</th><th>Address</th><th>Serial</th><th>Version</th><th>Status</th><th>Software update</th>' +
    '<th>Last action</th><th></th></tr></thead><tbody>' + (devices.map(d => {
      const s = d.status, a = d.action, up = s && s.device.update;
      const action = a ? `<span class="badge ${a.state === 'ok' ? 'ok' : a.state === 'failed' ? 'bad' : 'warn'}">` +
        `${esc(a.action)}: ${esc(a.state)}</span> <span class="muted small">${esc(a.message)} · ${ago(a.t)}</span>` : '';
      return `<tr><td><input type="checkbox" data-sel="${esc(d.id)}" aria-label="Select ${esc(devName(d))}" ${selected.has(d.id) ? 'checked' : ''}></td>` +
        `<td>${esc(devName(d))}</td><td>${esc(d.address)}</td><td>${esc(d.serial)}</td>` +
        `<td>${esc(s ? s.device.version : '–')}</td><td>${deviceBadge(d)}</td>` +
        `<td>${up ? `<span class="badge ${up === 'failed' ? 'bad' : up === 'done' ? 'ok' : 'warn'}">${updateText[up] || esc(up)}</span>` : ''}` +
        `${s && s.api_version >= 2 ? ` <button class="link small" data-log="${esc(d.id)}">log</button>` : ''}</td>` +
        `<td class="wrap">${action}</td>` +
        `<td>${d.cert_changed ? `<button class="small danger" data-trust="${esc(d.id)}" title="New SHA-256 fingerprint: ${esc(d.cert_changed)}">Trust new certificate</button>` : ''}</td></tr>`;
    }).join('') || '<tr><td colspan="9" class="muted">No devices yet: add one below.</td></tr>') + '</tbody>';
  $('#sel-count').textContent = `${selected.size} selected`;
  $('#manager-info').textContent = `Signed in as ${session.user}. Manager ${session.version}.`;
}

function renderDiscover() {
  if (!discover) return;
  const scan = discover.scan, others = $('#scan-others').checked;
  if (!$('#scan-network').value) $('#scan-network').value = scan.network;
  $('#scan-button').disabled = scan.running;
  $('#scan-button').textContent = scan.running ? 'Scanning…' : 'Scan';
  const rows = scan.results.filter(r => others || r.kind !== 'ssh');
  for (const ip of [...scanSelected]) if (!rows.some(r => r.ip === ip)) scanSelected.delete(ip);
  const counts = k => scan.results.filter(r => r.kind === k).length;
  $('#scan-status').textContent = scan.error ? 'Scan failed: ' + scan.error : scan.running ? `Scanning ${scan.network}…` :
    scan.t ? `${scan.network}, ${ago(scan.t)}: ${counts('pindi')} PiNDI, ${counts('pi')} Raspberry Pi without PiNDI, ` +
      `${counts('ssh')} other SSH hosts${others ? '' : ' (hidden)'}.` : 'Finds PiNDI devices, and Raspberry Pis without PiNDI to install it on.';
  const what = { pindi: '<span class="badge ok">PiNDI</span>', pi: '<span class="badge warn">Raspberry Pi</span>',
    ssh: '<span class="badge">Linux with SSH</span>' };
  const stepText = { connecting: 'connecting', uploading: 'uploading', installing: 'installing', rebooting: 'rebooting',
    adding: 'adding', done: 'done' };
  $('#scan-table').innerHTML = rows.length ? '<thead><tr><th></th><th>Address</th><th>Found</th><th>MAC</th><th>SSH</th>' +
    '<th>Status</th></tr></thead><tbody>' + rows.map(r => {
      const job = discover.installs[r.ip];
      let status = r.managed ? '<span class="muted">managed</span>' :
        r.kind === 'pindi' ? '<span class="muted">not added</span>' : r.ssh ? '' : '<span class="muted">SSH is off: enable it to install</span>';
      if (job) {
        status = `<span class="badge ${job.state === 'ok' ? 'ok' : job.state === 'failed' ? 'bad' : 'warn'}">install: ` +
          `${job.state === 'running' ? stepText[job.step] || esc(job.step) : esc(job.state)}</span> ` +
          `<button class="link small" data-install-log="${esc(r.ip)}">log</button>`;
      }
      const selectable = !r.managed && (r.kind === 'pindi' || r.ssh);
      return `<tr><td>${selectable ? `<input type="checkbox" data-scan="${esc(r.ip)}" aria-label="Select ${esc(r.ip)}" ` +
        `${scanSelected.has(r.ip) ? 'checked' : ''}>` : ''}</td><td>${esc(r.ip)}</td><td>${what[r.kind]}</td>` +
        `<td class="muted">${esc(r.mac)}</td><td class="muted small">${esc(r.ssh)}</td><td>${status}</td></tr>`;
    }).join('') + '</tbody>' : '';
  $('#sdk-hint').textContent = !discover.ssh ? 'Installing needs paramiko: python -m pip install paramiko' :
    discover.sdk ? `Installs agent ${discover.agent_dir.split(/[\\/]/).pop()} with ${discover.sdk}.` :
      `Installing needs the NDI SDK: put Install_NDI_SDK_v6_Linux.tar.gz (ndi.video/sdk) in ${discover.agent_dir}.`;
}

async function refreshDiscover() {
  discover = await api('GET', '/api/discover');
  renderDiscover();
}

async function bulk(action) {
  const ids = [...selected];
  if (!ids.length) return toast('Select devices first.', true);
  if (action === 'reboot' && !confirm(`Reboot ${ids.length} device(s)? Their outputs go dark for about 20 seconds.`)) return;
  if (action === 'shutdown' && !confirm(`Shut down ${ids.length} device(s)? They can't be started again from here: ` +
    'each stays off until its power is unplugged and plugged back in.')) return;
  try {
    await api('POST', '/api/bulk', { action, ids });
    toast(`${action} sent to ${ids.length} device(s).`);
  } catch (e) {
    toast(e.message, true);
  }
}

async function showLog(id) {
  const d = devices.find(x => x.id === id);
  $('#log-title').textContent = `Update log · ${devName(d)}`;
  $('#log-text').textContent = 'Loading…';
  $('#log-dialog').showModal();
  try {
    const r = await api('GET', '/api/update-log?id=' + encodeURIComponent(id));
    $('#log-text').textContent = r.log || 'No update has run since the last reboot.';
  } catch (e) {
    $('#log-text').textContent = e.message;
  }
}

// ------------------------------------------------------------------ updates from GitHub

let updates = null; // settings and this manager's version (polled with the admin page)
let latest = null; // the last "Check for updates" answer
let repoEdited = false; // don't overwrite what is being typed

function cmpVersion(a, b) {
  const x = String(a || '').split('.').map(Number), y = String(b || '').split('.').map(Number);
  for (let i = 0; i < Math.max(x.length, y.length); i++) {
    const d = (x[i] || 0) - (y[i] || 0);
    if (d) return d;
  }
  return 0;
}

function renderUpdates() {
  if (!updates) return;
  if (!repoEdited) {
    $('#repo-url').value = updates.settings.repo;
    $('#repo-branch').value = updates.settings.branch;
  }
  $('#repo-url').placeholder = updates.settings.default_repo;
  const loc = updates.local, g = loc.git, job = loc.job;
  let html = `<p class="muted small">This manager: ${esc(loc.version)}, ` + (g ?
    `git checkout on ${esc(g.branch)} at <code>${esc(g.head.slice(0, 7))}</code>${g.dirty ? ' with local changes' : ''}.` :
    'not a git checkout: an update downloads the branch over its folder.') + '</p>';
  if (latest) {
    const a = latest.agent, m = latest.manager;
    if (!a) html += '<p class="muted">The repository has no agent release with an update package.</p>';
    else if (a.error) html += `<p class="error">Agent releases: ${esc(a.error)}</p>`;
    else {
      const older = devices.filter(d => d.status && cmpVersion(d.status.device.version, a.version) < 0).length;
      html += `<p>Newest agent: <strong>${esc(a.version)}</strong> <span class="muted small">${esc(a.published.slice(0, 10))}</span> ` +
        `<a href="${esc(a.url)}" target="_blank" rel="noopener">release notes</a> · ` +
        `${older ? `${older} device(s) run an older version` : 'every device online has it (or newer)'} ` +
        `<button type="button" class="small" id="install-release" data-tag="${esc(a.tag)}" data-version="${esc(a.version)}">` +
        `Install ${esc(a.version)} on selected</button></p>`;
    }
    if (m.error) html += `<p class="error">Manager: ${esc(m.error)}</p>`;
    else {
      const state = { current: ['ok', 'up to date'], ahead: ['ok', 'this checkout is newer'],
        behind: ['warn', 'update available'], unknown: ['', 'may differ'] }[m.state] || ['', ''];
      html += `<p>Manager in the repository: <strong>${esc(m.version || '?')}</strong>, latest change ` +
        `<code>${esc(m.commit.slice(0, 7))}</code> ${esc(m.message)} <span class="muted small">${esc(m.date.slice(0, 10))}</span> ` +
        `<span class="badge ${state[0]}">${state[1]}</span></p>`;
    }
  }
  if (job.state) {
    const cls = { running: 'warn', restarting: 'warn', failed: 'bad', done: 'ok' }[job.state] || '';
    html += `<p>Manager update: <span class="badge ${cls}">${esc(job.state)}</span> ` +
      '<button type="button" class="link small" id="manager-log">log</button></p>';
  }
  $('#updates-result').innerHTML = html;
  $('#update-manager').disabled = job.state === 'running' || job.state === 'restarting';
}

async function checkUpdates() {
  const btn = $('#check-updates');
  btn.disabled = true;
  try {
    latest = await api('POST', '/api/updates/check', {});
    renderUpdates();
  } catch (e) {
    toast(e.message, true);
  } finally {
    btn.disabled = false;
  }
}

async function installRelease(tag, version) {
  const ids = [...selected];
  if (!ids.length) return toast('Select the devices to update first.', true);
  if (!confirm(`Download PiNDI ${version} from GitHub and install it on ${ids.length} device(s)? Each one reboots ` +
    'when done (about 2 minutes); video stops meanwhile.')) return;
  try {
    await api('POST', '/api/updates/agent', { ids, tag, downgrade: $('#update-downgrade').checked,
      user: $('#update-user').value, password: $('#update-password').value });
    toast('Update started: progress shows in the table.');
  } catch (e) {
    toast(e.message, true);
  }
}

async function updateManager() {
  if (!confirm(`Update the manager from ${updates.settings.repo} (${updates.settings.branch}) and restart it? ` +
    'Everyone signed in has to sign in again.')) return;
  try {
    await api('POST', '/api/updates/manager', {});
  } catch (e) {
    return toast(e.message, true);
  }
  const wait = setInterval(async () => {  // faster than the page poll; the restart signs everyone out
    try {
      updates = await api('GET', '/api/updates');
      renderUpdates();
      if (updates.local.job.state === 'restarting') {
        clearInterval(wait);
        toast('The manager is restarting: sign in again.');
        setTimeout(() => location.reload(), 5000);
      } else if (updates.local.job.state !== 'running') {
        clearInterval(wait);
        toast(updates.local.job.state === 'done' ? 'The manager is already up to date.' : 'The manager update failed: see its log.',
          updates.local.job.state !== 'done');
      }
    } catch (e) {
      clearInterval(wait);
    }
  }, 1500);
}

// ------------------------------------------------------------------ pages, polling, sign-in

function render() {
  $$('header nav a').forEach(a => a.classList.toggle('active', a.dataset.page === page));
  $$('.page').forEach(p => p.classList.toggle('active', p.id === 'page-' + page));
  ({ monitor: renderMonitor, matrix: renderMatrix, admin: () => { renderAdmin(); renderDiscover(); renderUpdates(); } })[page]();
}

async function refresh() {
  const st = await api('GET', '/api/state');
  devices = st.devices;
  if (page === 'monitor') history = await api('GET', '/api/history');
  if (page === 'admin') [discover, updates] = await Promise.all([api('GET', '/api/discover'), api('GET', '/api/updates')]);
  $('#updated').textContent = 'Updated ' + new Date().toLocaleTimeString();
  render();
}

function schedule() {
  clearTimeout(pollTimer);
  pollTimer = setTimeout(async () => {
    try { await refresh(); } catch (e) { $('#updated').textContent = 'Not updating: ' + e.message; }
    if (session) schedule();
  }, (session.poll_sec || 5) * 1000);
}

function showLogin() {
  session = null;
  clearTimeout(pollTimer);
  $('#app').hidden = true;
  $('#login').hidden = false;
}

async function start() {
  try {
    session = await api('GET', '/api/session');
  } catch (e) {
    return;
  }
  $('#login').hidden = true;
  $('#app').hidden = false;
  $('#banner').hidden = !session.default_password;
  page = (location.hash || '#monitor').slice(1);
  if (!['monitor', 'matrix', 'admin'].includes(page)) page = 'monitor';
  await refresh().catch(e => toast(e.message, true));
  schedule();
}

function formData(form) {
  return Object.fromEntries(new FormData(form).entries());
}

window.addEventListener('hashchange', () => {
  const p = location.hash.slice(1);
  if (['monitor', 'matrix', 'admin'].includes(p) && session) {
    page = p;
    render();
    if (p !== 'matrix') refresh().catch(() => {});
  }
});

$('#login-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  $('#login-error').textContent = '';
  try {
    await api('POST', '/api/login', formData(ev.target));
    ev.target.password.value = '';
    start();
  } catch (e) {
    $('#login-error').textContent = e.message;
  }
});

$('#logout').addEventListener('click', async () => {
  await api('POST', '/api/logout', {}).catch(() => {});
  showLogin();
});

$('#matrix').addEventListener('click', ev => {
  const btn = ev.target.closest('button[data-dev]');
  if (btn && !btn.disabled) route(btn);
});
// A preview that isn't there yet (the device renders it on request): an empty frame, not a broken image.
$('#matrix').addEventListener('error', ev => { if (ev.target.tagName === 'IMG') ev.target.classList.add('missing'); }, true);
$('#matrix-filter').addEventListener('input', renderMatrix);
$('#show-thumbs').addEventListener('change', renderMatrix);

$('#admin-table').addEventListener('change', ev => {
  if (ev.target.id === 'sel-all') {
    devices.forEach(d => ev.target.checked ? selected.add(d.id) : selected.delete(d.id));
  } else if (ev.target.dataset.sel) {
    ev.target.checked ? selected.add(ev.target.dataset.sel) : selected.delete(ev.target.dataset.sel);
  }
  renderAdmin();
});
$('#admin-table').addEventListener('click', async ev => {
  const t = ev.target;
  if (t.dataset.log) return showLog(t.dataset.log);
  if (t.dataset.trust) {
    const d = devices.find(x => x.id === t.dataset.trust);
    if (!confirm(`Trust the new certificate of ${devName(d)}?\n\nSHA-256 ${d.cert_changed}\n\n` +
      'It changes when the device gets a new name or address. Compare it with Device settings → HTTPS certificate on the device.')) return;
    try {
      await api('POST', '/api/devices/trust', { id: t.dataset.trust });
      await refresh();
    } catch (e) {
      toast(e.message, true);
    }
  }
});
$$('[data-bulk]').forEach(b => b.addEventListener('click', () => bulk(b.dataset.bulk)));

$('#remove').addEventListener('click', async () => {
  const ids = [...selected];
  if (!ids.length) return toast('Select devices first.', true);
  if (!confirm(`Remove ${ids.length} device(s) from the manager? The devices themselves are not changed.`)) return;
  try {
    await api('POST', '/api/devices/remove', { ids });
    selected.clear();
    await refresh();
  } catch (e) {
    toast(e.message, true);
  }
});

$('#update-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  const ids = [...selected];
  const file = $('#update-file').files[0];
  if (!ids.length) return toast('Select the devices to update first.', true);
  if (!file) return;
  if (!confirm(`Install ${file.name} on ${ids.length} device(s)? Each one reboots when done (about 2 minutes); ` +
    'video stops meanwhile.')) return;
  const btn = ev.target.querySelector('button');
  btn.disabled = true;
  try {
    await api('POST', `/api/update?ids=${ids.map(encodeURIComponent).join(',')}&downgrade=${$('#update-downgrade').checked ? 1 : 0}`,
      undefined, file, { 'X-Device-User': $('#update-user').value, 'X-Device-Password': $('#update-password').value });
    toast('Update started: progress shows in the table.');
    $('#update-file').value = '';
  } catch (e) {
    toast(e.message, true);
  } finally {
    btn.disabled = false;
  }
});

$('#check-updates').addEventListener('click', checkUpdates);
$('#update-manager').addEventListener('click', updateManager);
$('#updates-result').addEventListener('click', ev => {
  const b = ev.target.closest('button');
  if (b && b.id === 'install-release') installRelease(b.dataset.tag, b.dataset.version);
  if (b && b.id === 'manager-log') {
    $('#log-title').textContent = 'Manager update log';
    $('#log-text').textContent = updates.local.job.log || 'Nothing yet.';
    $('#log-dialog').showModal();
  }
});
['#repo-url', '#repo-branch'].forEach(s => $(s).addEventListener('input', () => { repoEdited = true; }));
$('#repo-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  try {
    const r = await api('POST', '/api/updates/settings', { repo: $('#repo-url').value, branch: $('#repo-branch').value });
    updates.settings = r.settings;
    repoEdited = false;
    latest = null; // it was about the other repository
    renderUpdates();
    toast('Saved.');
  } catch (e) {
    toast(e.message, true);
  }
});

$('#scan-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  try {
    await api('POST', '/api/discover', { network: $('#scan-network').value });
    await refreshDiscover();
    const wait = setInterval(async () => {  // faster than the page poll while a scan runs
      await refreshDiscover().catch(() => {});
      if (!discover.scan.running) clearInterval(wait);
    }, 1500);
  } catch (e) {
    toast(e.message, true);
  }
});
$('#scan-others').addEventListener('change', renderDiscover);
$('#scan-table').addEventListener('change', ev => {
  const ip = ev.target.dataset.scan;
  if (ip) ev.target.checked ? scanSelected.add(ip) : scanSelected.delete(ip);
});
$('#scan-table').addEventListener('click', async ev => {
  const ip = ev.target.dataset.installLog;
  if (!ip) return;
  $('#log-title').textContent = `Install log · ${ip}`;
  $('#log-text').textContent = 'Loading…';
  $('#log-dialog').showModal();
  const r = await api('GET', '/api/install-log?ip=' + encodeURIComponent(ip)).catch(e => ({ log: e.message }));
  $('#log-text').textContent = r.log || '(empty)';
  $('#log-text').scrollTop = $('#log-text').scrollHeight;
});
const scanPicked = kind => discover ? discover.scan.results.filter(r => scanSelected.has(r.ip) && (kind === 'pindi') === (r.kind === 'pindi')) : [];
$('#add-selected').addEventListener('click', async () => {
  const picked = scanPicked('pindi');
  if (!picked.length) return toast('Select PiNDI devices in the list first.', true);
  let added = 0;
  for (const r of picked) {
    try {
      await api('POST', '/api/devices', { address: r.ip, user: $('#disc-user').value, password: $('#disc-password').value });
      added++;
      scanSelected.delete(r.ip);
    } catch (e) {
      toast(`${r.ip}: ${e.message}`, true);
    }
  }
  if (added) toast(`${added} device(s) added.`);
  await refresh().catch(() => {});
});
$('#install-selected').addEventListener('click', async () => {
  const picked = scanPicked('other');
  if (!picked.length) return toast('Select Raspberry Pis without PiNDI first.', true);
  if (!$('#disc-license').checked) return toast('Accept the NDI SDK license first.', true);
  if (!confirm(`Install PiNDI on ${picked.map(r => r.ip).join(', ')}?\n\nEach Pi gets the PiNDI agent and the NDI SDK, ` +
    'is set up as a PiNDI device (it boots to the PiNDI splash screen) and reboots. Hosts that turn out not to be ' +
    'a Raspberry Pi are left alone.')) return;
  try {
    await api('POST', '/api/install', {
      ips: picked.map(r => r.ip), user: $('#disc-user').value, password: $('#disc-password').value,
      skip_upgrade: $('#disc-skip-upgrade').checked, accept_ndi_license: true,
    });
    picked.forEach(r => scanSelected.delete(r.ip));
    toast('Install started: progress shows in the list.');
    await refreshDiscover();
  } catch (e) {
    toast(e.message, true);
  }
});

$('#add-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  const f = ev.target, btn = f.querySelector('button');
  $('#add-error').textContent = '';
  btn.disabled = true;
  btn.textContent = 'Connecting…';
  try {
    await api('POST', '/api/devices', formData(f));
    toast(`${f.address.value} added.`);
    f.address.value = '';
    f.token.value = '';
    await refresh();
  } catch (e) {
    $('#add-error').textContent = e.message;
  } finally {
    btn.disabled = false;
    btn.textContent = 'Add device';
  }
});

$('#password-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  const f = ev.target;
  $('#password-error').textContent = '';
  if (f.new.value !== f.confirm.value) {
    $('#password-error').textContent = 'The new passwords do not match.';
    return;
  }
  try {
    await api('POST', '/api/password', { current: f.current.value, new: f.new.value });
    f.reset();
    session.default_password = false;
    $('#banner').hidden = true;
    toast('Password changed.');
  } catch (e) {
    $('#password-error').textContent = e.message;
  }
});

start(); // a 401 shows the sign-in form
