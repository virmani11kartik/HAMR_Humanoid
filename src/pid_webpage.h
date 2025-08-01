const char* pid_webpage = R"=====(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PID Tuning – Sync Controller</title>
<style>
  :root { --fg:#111; --bg:#fafafa; --card:#fff; --muted:#666; --accent:#0a84ff; }
  * { box-sizing: border-box; font-family: system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
  body { margin: 0; background: var(--bg); color: var(--fg); }
  .wrap { max-width: 720px; margin: 24px auto; padding: 0 16px; }
  .card { background: var(--card); border-radius: 12px; padding: 20px; box-shadow: 0 4px 16px rgba(0,0,0,0.06); }
  h1 { font-size: 1.25rem; margin: 0 0 12px; }
  .row { display: grid; grid-template-columns: 120px 1fr 96px; gap: 12px; align-items: center; margin: 14px 0; }
  label { font-weight: 600; }
  input[type="range"] { width: 100%; }
  input[type="number"] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 8px; }
  .btns { display: flex; gap: 10px; margin-top: 16px; flex-wrap: wrap; }
  button { padding: 10px 14px; border: 0; border-radius: 10px; background: var(--accent); color: #fff; font-weight: 600; cursor: pointer; }
  button.light { background: #eee; color: #000; }
  .status { margin-top: 12px; color: var(--muted); white-space: pre-line; }
  .inline { display: inline-flex; gap: 8px; align-items: center; }
  .small { font-size: 0.9rem; color: var(--muted); }
</style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Sync PID Tuning</h1>
      <div class="small">Adjust Kp, Ki, Kd for wheel speed synchronization. Values update the controller immediately.</div>

      <div class="row">
        <label for="kp">Kp</label>
        <input id="kp_range" type="range" min="0" max="500" step="0.1">
        <input id="kp" type="number" min="0" max="500" step="0.1">
      </div>

      <div class="row">
        <label for="ki">Ki</label>
        <input id="ki_range" type="range" min="0" max="100" step="0.1">
        <input id="ki" type="number" min="0" max="100" step="0.1">
      </div>

      <div class="row">
        <label for="kd">Kd</label>
        <input id="kd_range" type="range" min="0" max="500" step="0.1">
        <input id="kd" type="number" min="0" max="500" step="0.1">
      </div>

      <div class="row">
        <label for="test">Test</label>
        <input id="test_range" type="range" min="0" max="4095" step="10">
        <input id="test" type="number" min="0" max="4095" step="10">
      </div>

      <div class="inline">
        <input id="autoApply" type="checkbox" checked>
        <label for="autoApply">Auto‑apply while sliding</label>
      </div>

      <div class="btns">
        <button id="applyBtn">Apply</button>
        <button id="refreshBtn" class="light">Refresh</button>
        <button id="defaultsBtn" class="light">Reset to Defaults (0, 0, 0, 0)</button>
      </div>

      <div id="status" class="status">Loading current PID…</div>
    </div>
  </div>

<script>
  const s = (id)=>document.getElementById(id);
  const statusEl = s('status');

  function setFields(kp, ki, kd, test=0) {
    s('kp').value = kp.toFixed(1);
    s('ki').value = ki.toFixed(1);
    s('kd').value = kd.toFixed(1);
    s('kp_range').value = kp;
    s('ki_range').value = ki;
    s('kd_range').value = kd;
    s('test').value = test.toFixed(0);
    s('test_range').value = test;
  }

  async function refreshPID() {
    status('Fetching /getPID …');
    try {
      const r = await fetch('/getPID', {cache:'no-store'});
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const j = await r.json();
      const kp = Number(j.Kp ?? j.kp ?? 0);
      const ki = Number(j.Ki ?? j.ki ?? 0);
      const kd = Number(j.Kd ?? j.kd ?? 0);
      const test = Number(j.Test ?? j.test ?? 0);
      setFields(kp, ki, kd, test);
      status(`Current PID:\nKp=${kp.toFixed(1)}  Ki=${ki.toFixed(1)}  Kd=${kd.toFixed(1)}\nTest=${test.toFixed(0)}`);
    } catch (e) {
      status('Failed to get PID: ' + e);
    }
  }

  async function applyPID(kp, ki, kd, test) {
    const body = new URLSearchParams({ Kp: kp, Ki: ki, Kd: kd, Test: test });
    status('Updating /updatePID …');
    try {
      const r = await fetch('/updatePID', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body });
      const t = await r.text();
      status((r.ok ? '✅ ' : '⚠️ ') + t);
    } catch (e) {
      status('Failed to update PID: ' + e);
    }
  }

  function status(msg){ statusEl.textContent = msg; }

  // Wire inputs: keep range and number in sync; auto-apply if enabled
  function bindPair(rangeId, numberId, decimals=1) {
    const r = s(rangeId), n = s(numberId);
    r.addEventListener('input', () => {
      n.value = Number(r.value).toFixed(decimals);
      if (s('autoApply').checked) sendFromFields();
    });
    n.addEventListener('change', () => {
      r.value = n.value;
      if (s('autoApply').checked) sendFromFields();
    });
  }

  function sendFromFields() {
    const kp = Number(s('kp').value);
    const ki = Number(s('ki').value);
    const kd = Number(s('kd').value);
    const test = Number(s('test').value);
    if ([kp, ki, kd, test].some(v => Number.isNaN(v))) {
      status('Please enter numeric values for Kp, Ki, Kd, Test.');
      return;
    }
    applyPID(kp, ki, kd, test);
  }

  s('applyBtn').addEventListener('click', sendFromFields);
  s('refreshBtn').addEventListener('click', refreshPID);
  s('defaultsBtn').addEventListener('click', () => {
    setFields(0.0, 0.0, 0.0, 0.0);
    sendFromFields();
  });

  bindPair('kp_range','kp', 1);
  bindPair('ki_range','ki', 1);
  bindPair('kd_range','kd', 1);
  bindPair('test_range','test', 0);

  // Initial load
  refreshPID();
</script>
</body>
</html>)=====";
