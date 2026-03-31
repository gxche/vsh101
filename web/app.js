// ============================================================
//  VSH101 Web BLE ECG Monitor
// ============================================================

// --- Constants ---
const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // notify (device → client)
const RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // write  (client → device)

const CMD_START = new Uint8Array([
  0x64, 0xC2, 0x64, 0x8A,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
]);

const CMD_STOP = new Uint8Array([
  0x65, 0xC2, 0x65, 0x8C,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
]);

const CMD_READ_TEMPLATE = new Uint8Array([
  0x6A, 0xC2, 0x6A, 0xE3,
  0x00, 0x00, 0x38, 0x02,
  0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
]);

const ACK_HEADER_SIZE = 8;
const TYPE1_ECG_SIZE = 400;   // 100 floats
const TYPE1_INFO_SIZE = 168;  // 42 floats
const TYPE1_TOTAL = 576;      // 8 + 568
const VSC_MODE_IDX_INVALID = 0x1F40;
const VSC_MODE_IDX_MAX = 8000;

const READ_INTERVAL_MS = 200;
const DATA_TIMEOUT_MS = 10000;

// Info block indices
const INFO = {
  INFOTYPE: 0, UTC: 1, TEMP: 2, HR: 3, LEADOFF: 4,
  GSEN_X: 5, GSEN_Y: 6, GSEN_Z: 7,
  BATT_SOC: 8, BATT_SEC: 9,
  HRV_SDNN: 10, HRV_NN50: 11, HRV_RMSSD: 12, HRV_RR: 13,
  HRV_VLF: 14, HRV_LF: 15, HRV_HF: 16,
  ATR_CODE: 27, ATR_TS: 28
};

// --- Utility ---
function safeFloat(dv, offset) {
  const v = dv.getFloat32(offset, true);
  return (Number.isNaN(v) || !Number.isFinite(v)) ? 0 : v;
}

// ============================================================
//  BLE Connection
// ============================================================
class BLEConnection {
  constructor() {
    this.device = null;
    this.server = null;
    this.txChar = null;
    this.rxChar = null;
    this.connected = false;
    this.onNotification = null;
    this.onDisconnect = null;
  }

  async connect() {
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: 'VSH101' }],
      optionalServices: [SERVICE_UUID]
    });

    this.device.addEventListener('gattserverdisconnected', () => {
      this.connected = false;
      if (this.onDisconnect) this.onDisconnect();
    });

    this.server = await this.device.gatt.connect();
    const service = await this.server.getPrimaryService(SERVICE_UUID);
    this.txChar = await service.getCharacteristic(TX_UUID);
    this.rxChar = await service.getCharacteristic(RX_UUID);

    await this.txChar.startNotifications();
    this.txChar.addEventListener('characteristicvaluechanged', (event) => {
      if (this.onNotification) this.onNotification(event);
    });

    this.connected = true;
  }

  async disconnect() {
    if (this.device && this.device.gatt.connected) {
      this.device.gatt.disconnect();
    }
    this.connected = false;
  }

  async write(data) {
    if (!this.rxChar) return;
    await this.rxChar.writeValueWithResponse(data);
  }
}

// ============================================================
//  Packet Assembler
// ============================================================
class PacketAssembler {
  constructor() {
    this.buffer = new Uint8Array(1024);
    this.length = 0;
    this.onPacket = null;
    this.fragmentCount = 0;
  }

  handleNotification(event) {
    const data = new Uint8Array(event.target.value.buffer);
    if (data.length === 0) return;
    this.fragmentCount++;

    // Detect new response header
    if (data.length >= 3 && data[1] === 0xC2 &&
        (data[0] === 0x6A || data[0] === 0x64 || data[0] === 0x65)) {
      // New response starting — discard old buffer
      this.length = 0;
    }

    // Append to buffer
    const space = this.buffer.length - this.length;
    const copyLen = Math.min(data.length, space);
    if (copyLen > 0) {
      this.buffer.set(data.subarray(0, copyLen), this.length);
      this.length += copyLen;
    }

    // Check if complete packet assembled
    if (this.length >= TYPE1_TOTAL && this.onPacket) {
      this.onPacket(this.buffer.slice(0, this.length));
      this.length = 0;
    }
  }
}

// ============================================================
//  VSH101 Protocol
// ============================================================
class VSH101Protocol {
  constructor(ble, assembler) {
    this.ble = ble;
    this.assembler = assembler;
    this.currentWId = 0;
    this.measuring = false;
    this.readIntervalId = null;
    this.lastDataTime = 0;
    this.timeoutCheckId = null;
    this.packetCount = 0;

    this.onECGData = null;
    this.onInfoUpdate = null;
    this.onStatsUpdate = null;
    this.onError = null;

    this.assembler.onPacket = (buf) => this.parsePacket(buf);
  }

  async startMeasurement() {
    try {
      await this.ble.write(CMD_START);
      this.currentWId = 0;
      this.measuring = true;
      this.packetCount = 0;
      this.assembler.fragmentCount = 0;
      this.assembler.length = 0;
      this.lastDataTime = Date.now();

      this.readIntervalId = setInterval(() => this.sendRead(), READ_INTERVAL_MS);
      this.timeoutCheckId = setInterval(() => this.checkTimeout(), 1000);
    } catch (e) {
      if (this.onError) this.onError('START 失敗: ' + e.message);
    }
  }

  async stopMeasurement() {
    this.measuring = false;
    if (this.readIntervalId) { clearInterval(this.readIntervalId); this.readIntervalId = null; }
    if (this.timeoutCheckId) { clearInterval(this.timeoutCheckId); this.timeoutCheckId = null; }
    try {
      await this.ble.write(CMD_STOP);
    } catch (e) {
      // Device may already be disconnected
    }
  }

  async sendRead() {
    if (!this.measuring) return;
    const cmd = new Uint8Array(CMD_READ_TEMPLATE);
    cmd[8] = this.currentWId & 0xFF;
    cmd[9] = (this.currentWId >> 8) & 0xFF;
    try {
      await this.ble.write(cmd);
    } catch (e) {
      if (this.onError) this.onError('READ 失敗: ' + e.message);
    }
  }

  async checkTimeout() {
    if (!this.measuring) return;
    if (this.lastDataTime > 0 && (Date.now() - this.lastDataTime) > DATA_TIMEOUT_MS) {
      if (this.onError) this.onError('資料超時，重啟測量...');
      try {
        await this.ble.write(CMD_STOP);
        await new Promise(r => setTimeout(r, 200));
        await this.ble.write(CMD_START);
        this.currentWId = 0;
        this.lastDataTime = Date.now();
      } catch (e) {
        if (this.onError) this.onError('重啟失敗: ' + e.message);
      }
    }
  }

  parsePacket(buf) {
    const len = buf.length;

    if (len >= ACK_HEADER_SIZE && buf[0] === 0x6A && buf[1] === 0xC2) {
      // ACK check
      if (buf[2] !== 0x41) {
        if (this.onError) this.onError('NACK: 0x' + buf[2].toString(16));
        return;
      }

      const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
      const dataIdx = dv.getUint16(4, true);
      const dataLen = dv.getUint16(6, true);

      // Invalid index — device not ready
      if (dataIdx === VSC_MODE_IDX_INVALID || dataLen === 0) {
        this.lastDataTime = Date.now();
        return;
      }

      // Index mismatch — sync to device
      if (dataIdx !== this.currentWId) {
        this.currentWId = dataIdx;
      }

      // Full 576-byte response
      if (len >= TYPE1_TOTAL) {
        const ecgOffset = ACK_HEADER_SIZE;
        const infoOffset = ACK_HEADER_SIZE + TYPE1_ECG_SIZE;

        // Parse ECG samples (100 float32)
        const ecgSamples = new Float32Array(100);
        for (let i = 0; i < 100; i++) {
          ecgSamples[i] = safeFloat(dv, ecgOffset + i * 4);
        }

        // Parse Info block (42 float32)
        const info = this.parseInfo(dv, infoOffset);

        this.packetCount++;
        this.lastDataTime = Date.now();
        this.currentWId = (this.currentWId + 1) % VSC_MODE_IDX_MAX;

        if (this.onECGData) this.onECGData(ecgSamples);
        if (this.onInfoUpdate) this.onInfoUpdate(info);
        if (this.onStatsUpdate) this.onStatsUpdate({
          packets: this.packetCount,
          fragments: this.assembler.fragmentCount,
          wId: this.currentWId
        });
      }
    }
  }

  parseInfo(dv, offset) {
    const f = (idx) => safeFloat(dv, offset + idx * 4);
    const hrRaw = f(INFO.HR);
    return {
      hr: (hrRaw >= 25 && hrRaw <= 250) ? hrRaw : 0,
      temp: f(INFO.TEMP),
      leadOff: f(INFO.LEADOFF),
      accelX: f(INFO.GSEN_X),
      accelY: f(INFO.GSEN_Y),
      accelZ: f(INFO.GSEN_Z),
      battSoc: f(INFO.BATT_SOC),
      battSec: f(INFO.BATT_SEC),
      hrvSDNN: f(INFO.HRV_SDNN),
      hrvNN50: f(INFO.HRV_NN50),
      hrvRMSSD: f(INFO.HRV_RMSSD),
      hrvRR: f(INFO.HRV_RR),
      hrvVLF: f(INFO.HRV_VLF),
      hrvLF: f(INFO.HRV_LF),
      hrvHF: f(INFO.HRV_HF),
      atrCode: f(INFO.ATR_CODE)
    };
  }
}

// ============================================================
//  ECG Canvas Renderer
// ============================================================
class ECGRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.bufferSize = 2500; // 5 seconds at 500Hz
    this.ecgBuffer = new Float32Array(this.bufferSize);
    this.writeIndex = 0;
    this.animFrameId = null;
    this.running = false;

    this.resize();
    window.addEventListener('resize', () => this.resize());
    this.draw();
  }

  resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = 300 * dpr;
    this.canvas.style.height = '300px';
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.displayWidth = rect.width;
    this.displayHeight = 300;
  }

  addSamples(samples) {
    for (let i = 0; i < samples.length; i++) {
      this.ecgBuffer[this.writeIndex] = samples[i];
      this.writeIndex = (this.writeIndex + 1) % this.bufferSize;
    }
  }

  start() {
    this.running = true;
  }

  stop() {
    this.running = false;
  }

  clear() {
    this.ecgBuffer.fill(0);
    this.writeIndex = 0;
  }

  draw() {
    const ctx = this.ctx;
    const w = this.displayWidth;
    const h = this.displayHeight;

    // Background
    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, w, h);

    // Grid
    ctx.strokeStyle = '#1a2a1a';
    ctx.lineWidth = 0.5;
    const gridSpacingX = w / 25; // ~200ms per division (5s / 25)
    const gridSpacingY = h / 6;
    ctx.beginPath();
    for (let x = 0; x <= w; x += gridSpacingX) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
    }
    for (let y = 0; y <= h; y += gridSpacingY) {
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    ctx.stroke();

    // Center line
    ctx.strokeStyle = '#2a3a2a';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();

    // ECG trace — sweep style
    const samplesVisible = this.bufferSize;
    const pixelsPerSample = w / samplesVisible;
    const yCenter = h / 2;
    const yScale = h / 4; // ~2mV full range maps to half height

    ctx.strokeStyle = '#00e676';
    ctx.lineWidth = 1.5;
    ctx.beginPath();

    let started = false;
    for (let i = 0; i < samplesVisible; i++) {
      const idx = (this.writeIndex + i) % this.bufferSize;
      const x = i * pixelsPerSample;
      const y = yCenter - this.ecgBuffer[idx] * yScale;

      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();

    // Sweep line
    const sweepX = ((this.writeIndex / this.bufferSize) * w);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sweepX, 0);
    ctx.lineTo(sweepX, h);
    ctx.stroke();

    // Erase zone ahead of sweep
    const eraseWidth = w * 0.04;
    const grad = ctx.createLinearGradient(sweepX, 0, sweepX + eraseWidth, 0);
    grad.addColorStop(0, 'rgba(10, 10, 10, 0.9)');
    grad.addColorStop(1, 'rgba(10, 10, 10, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(sweepX, 0, eraseWidth, h);

    this.animFrameId = requestAnimationFrame(() => this.draw());
  }

  destroy() {
    if (this.animFrameId) cancelAnimationFrame(this.animFrameId);
  }
}

// ============================================================
//  UI Controller
// ============================================================
class UIController {
  constructor() {
    this.els = {
      connStatus: document.getElementById('conn-status'),
      btnConnect: document.getElementById('btn-connect'),
      btnDisconnect: document.getElementById('btn-disconnect'),
      btnStart: document.getElementById('btn-start'),
      btnStop: document.getElementById('btn-stop'),
      elapsed: document.getElementById('elapsed-time'),
      hr: document.getElementById('val-hr'),
      temp: document.getElementById('val-temp'),
      leadoff: document.getElementById('val-leadoff'),
      battery: document.getElementById('val-battery'),
      batteryBar: document.getElementById('battery-bar'),
      sdnn: document.getElementById('val-sdnn'),
      rmssd: document.getElementById('val-rmssd'),
      nn50: document.getElementById('val-nn50'),
      rr: document.getElementById('val-rr'),
      vlf: document.getElementById('val-vlf'),
      lf: document.getElementById('val-lf'),
      hf: document.getElementById('val-hf'),
      accelX: document.getElementById('val-accel-x'),
      accelY: document.getElementById('val-accel-y'),
      accelZ: document.getElementById('val-accel-z'),
      packets: document.getElementById('val-packets'),
      fragments: document.getElementById('val-fragments'),
      wid: document.getElementById('val-wid'),
      errorLog: document.getElementById('error-log')
    };
    this.measureStartTime = null;
    this.elapsedIntervalId = null;
  }

  setConnectionState(state) {
    const s = this.els.connStatus;
    s.className = 'status-badge ' + state;
    const labels = { disconnected: '未連線', connecting: '連線中...', connected: '已連線' };
    s.textContent = labels[state] || state;

    this.els.btnConnect.disabled = (state === 'connected' || state === 'connecting');
    this.els.btnDisconnect.disabled = (state !== 'connected');
    this.els.btnStart.disabled = (state !== 'connected');
    this.els.btnStop.disabled = true;
  }

  setMeasuring(active) {
    this.els.btnStart.disabled = active;
    this.els.btnStop.disabled = !active;

    if (active) {
      this.measureStartTime = Date.now();
      this.elapsedIntervalId = setInterval(() => this.updateElapsed(), 1000);
    } else {
      if (this.elapsedIntervalId) { clearInterval(this.elapsedIntervalId); this.elapsedIntervalId = null; }
    }
  }

  updateElapsed() {
    if (!this.measureStartTime) return;
    const s = Math.floor((Date.now() - this.measureStartTime) / 1000);
    const hh = String(Math.floor(s / 3600)).padStart(2, '0');
    const mm = String(Math.floor((s % 3600) / 60)).padStart(2, '0');
    const ss = String(s % 60).padStart(2, '0');
    this.els.elapsed.textContent = `${hh}:${mm}:${ss}`;
  }

  updateVitals(info) {
    // Heart rate
    if (info.hr > 0) {
      this.els.hr.textContent = Math.round(info.hr);
      this.els.hr.parentElement.style.color =
        info.hr > 100 ? '#ff5252' : info.hr > 80 ? '#ffc107' : '#00e676';
    } else {
      this.els.hr.textContent = '--';
      this.els.hr.parentElement.style.color = '';
    }

    // Temperature
    const temp = info.temp;
    this.els.temp.textContent = (temp >= 15 && temp <= 45) ? temp.toFixed(1) : '--';

    // Lead-off
    if (info.leadOff > 0) {
      this.els.leadoff.textContent = '電極脫落';
      this.els.leadoff.className = 'lead-off';
    } else {
      this.els.leadoff.textContent = '正常';
      this.els.leadoff.className = 'lead-ok';
    }

    // Battery
    const soc = info.battSoc;
    if (soc > 0 && soc <= 100) {
      this.els.battery.textContent = Math.round(soc);
      this.els.batteryBar.style.width = soc + '%';
      this.els.batteryBar.className = 'battery-bar' +
        (soc <= 20 ? ' critical' : soc <= 50 ? ' low' : '');
    }

    // HRV
    this.els.sdnn.textContent = info.hrvSDNN ? info.hrvSDNN.toFixed(1) : '--';
    this.els.rmssd.textContent = info.hrvRMSSD ? info.hrvRMSSD.toFixed(1) : '--';
    this.els.nn50.textContent = info.hrvNN50 ? Math.round(info.hrvNN50) : '--';
    this.els.rr.textContent = info.hrvRR ? info.hrvRR.toFixed(0) : '--';
    this.els.vlf.textContent = info.hrvVLF ? info.hrvVLF.toFixed(1) : '--';
    this.els.lf.textContent = info.hrvLF ? info.hrvLF.toFixed(1) : '--';
    this.els.hf.textContent = info.hrvHF ? info.hrvHF.toFixed(1) : '--';

    // Accelerometer
    this.els.accelX.textContent = info.accelX.toFixed(3);
    this.els.accelY.textContent = info.accelY.toFixed(3);
    this.els.accelZ.textContent = info.accelZ.toFixed(3);
  }

  updateStats(stats) {
    this.els.packets.textContent = stats.packets;
    this.els.fragments.textContent = stats.fragments;
    this.els.wid.textContent = stats.wId;
  }

  showError(msg) {
    const el = this.els.errorLog;
    const line = document.createElement('div');
    line.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    el.appendChild(line);
    el.scrollTop = el.scrollHeight;
    // Auto remove after 10s
    setTimeout(() => { if (line.parentNode) line.remove(); }, 10000);
  }

  resetValues() {
    this.els.hr.textContent = '--';
    this.els.hr.parentElement.style.color = '';
    this.els.temp.textContent = '--';
    this.els.leadoff.textContent = '正常';
    this.els.leadoff.className = 'lead-ok';
    this.els.battery.textContent = '--';
    this.els.batteryBar.style.width = '0%';
    this.els.sdnn.textContent = '--';
    this.els.rmssd.textContent = '--';
    this.els.nn50.textContent = '--';
    this.els.rr.textContent = '--';
    this.els.vlf.textContent = '--';
    this.els.lf.textContent = '--';
    this.els.hf.textContent = '--';
    this.els.accelX.textContent = '--';
    this.els.accelY.textContent = '--';
    this.els.accelZ.textContent = '--';
    this.els.packets.textContent = '0';
    this.els.fragments.textContent = '0';
    this.els.wid.textContent = '0';
    this.els.elapsed.textContent = '00:00:00';
  }
}

// ============================================================
//  Application Entry Point
// ============================================================
document.addEventListener('DOMContentLoaded', () => {
  const ui = new UIController();
  const ble = new BLEConnection();
  const assembler = new PacketAssembler();
  const protocol = new VSH101Protocol(ble, assembler);
  const ecgRenderer = new ECGRenderer(document.getElementById('ecg-canvas'));

  // Wire callbacks
  ble.onNotification = (event) => assembler.handleNotification(event);
  ble.onDisconnect = () => {
    protocol.stopMeasurement();
    ecgRenderer.stop();
    ui.setConnectionState('disconnected');
    ui.setMeasuring(false);
    ui.showError('裝置已斷線');
  };

  protocol.onECGData = (samples) => ecgRenderer.addSamples(samples);
  protocol.onInfoUpdate = (info) => ui.updateVitals(info);
  protocol.onStatsUpdate = (stats) => ui.updateStats(stats);
  protocol.onError = (msg) => ui.showError(msg);

  // Button handlers
  document.getElementById('btn-connect').addEventListener('click', async () => {
    ui.setConnectionState('connecting');
    try {
      await ble.connect();
      ui.setConnectionState('connected');
    } catch (e) {
      ui.setConnectionState('disconnected');
      ui.showError('連線失敗: ' + e.message);
    }
  });

  document.getElementById('btn-disconnect').addEventListener('click', async () => {
    await protocol.stopMeasurement();
    await ble.disconnect();
    ecgRenderer.stop();
    ecgRenderer.clear();
    ui.setConnectionState('disconnected');
    ui.setMeasuring(false);
    ui.resetValues();
  });

  document.getElementById('btn-start').addEventListener('click', async () => {
    ui.resetValues();
    ecgRenderer.clear();
    ecgRenderer.start();
    ui.setMeasuring(true);
    await protocol.startMeasurement();
  });

  document.getElementById('btn-stop').addEventListener('click', async () => {
    await protocol.stopMeasurement();
    ecgRenderer.stop();
    ui.setMeasuring(false);
  });

  // Initial state
  ui.setConnectionState('disconnected');
});
