// ============================================================
//  VSH101 Web BLE IMU Monitor
//  IMU 加速度數據顯示於前端頁面，每 500ms 更新一次
//  BLE 連線相關 debug 訊息輸出至 DevTools console
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
const IMU_DISPLAY_INTERVAL_MS = 500;
const DATA_TIMEOUT_MS = 10000;

// Info block float indices (IMU only; other fields skipped)
const INFO_GSEN_X = 5;
const INFO_GSEN_Y = 6;
const INFO_GSEN_Z = 7;

// --- Utility ---
function safeFloat(dv, offset) {
  const v = dv.getFloat32(offset, true);
  return (Number.isNaN(v) || !Number.isFinite(v)) ? 0 : v;
}

// Latest IMU sample — updated on every parsed packet
const latestIMU = { x: 0, y: 0, z: 0, updated: false };

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
    console.debug('[BLE] requestDevice');
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: 'VSH101' }],
      optionalServices: [SERVICE_UUID]
    });

    this.device.addEventListener('gattserverdisconnected', () => {
      console.debug('[BLE] gattserverdisconnected');
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
    console.debug('[BLE] connected');
  }

  async disconnect() {
    if (this.device && this.device.gatt.connected) {
      this.device.gatt.disconnect();
    }
    this.connected = false;
  }

  async write(data) {
    if (!this.rxChar) return;
    if (this._writing) throw new Error('GATT operation already in progress');
    this._writing = true;
    try {
      await this.rxChar.writeValueWithResponse(data);
    } finally {
      this._writing = false;
    }
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
      this.length = 0;
    }

    const space = this.buffer.length - this.length;
    const copyLen = Math.min(data.length, space);
    if (copyLen > 0) {
      this.buffer.set(data.subarray(0, copyLen), this.length);
      this.length += copyLen;
    }

    if (this.length >= TYPE1_TOTAL && this.onPacket) {
      this.onPacket(this.buffer.slice(0, this.length));
      this.length = 0;
    }
  }
}

// ============================================================
//  VSH101 Protocol — 僅解析 IMU 欄位
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
    this.restarting = false;

    this.assembler.onPacket = (buf) => this.parsePacket(buf);
  }

  async startMeasurement() {
    try {
      await this.ble.write(CMD_START);
      this.currentWId = 0;
      this.measuring = true;
      this.assembler.fragmentCount = 0;
      this.assembler.length = 0;
      this.lastDataTime = Date.now();

      this.readIntervalId = setInterval(() => this.sendRead(), READ_INTERVAL_MS);
      this.timeoutCheckId = setInterval(() => this.checkTimeout(), 1000);
      console.debug('[PROTO] startMeasurement');
    } catch (e) {
      console.error('[PROTO] START 失敗:', e.message);
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
    console.debug('[PROTO] stopMeasurement');
  }

  async sendRead() {
    if (!this.measuring || this.restarting) return;
    const cmd = new Uint8Array(CMD_READ_TEMPLATE);
    cmd[8] = this.currentWId & 0xFF;
    cmd[9] = (this.currentWId >> 8) & 0xFF;
    try {
      await this.ble.write(cmd);
    } catch (e) {
      // Ignore write conflicts during normal operation
    }
  }

  async checkTimeout() {
    if (!this.measuring || this.restarting) return;
    if (this.lastDataTime > 0 && (Date.now() - this.lastDataTime) > DATA_TIMEOUT_MS) {
      this.restarting = true;
      console.warn('[PROTO] 資料超時，重啟測量...');

      if (this.readIntervalId) { clearInterval(this.readIntervalId); this.readIntervalId = null; }

      try {
        await new Promise(r => setTimeout(r, 300));
        await this.ble.write(CMD_STOP);
        await new Promise(r => setTimeout(r, 500));
        await this.ble.write(CMD_START);
        this.currentWId = 0;
        this.assembler.length = 0;
        this.lastDataTime = Date.now();

        this.readIntervalId = setInterval(() => this.sendRead(), READ_INTERVAL_MS);
      } catch (e) {
        console.error('[PROTO] 重啟失敗:', e.message);
        if (!this.readIntervalId && this.measuring) {
          this.readIntervalId = setInterval(() => this.sendRead(), READ_INTERVAL_MS);
        }
      } finally {
        this.restarting = false;
      }
    }
  }

  parsePacket(buf) {
    const len = buf.length;

    if (len >= ACK_HEADER_SIZE && buf[0] === 0x6A && buf[1] === 0xC2) {
      if (buf[2] !== 0x41) {
        console.debug('[PROTO] NACK: 0x' + buf[2].toString(16));
        return;
      }

      const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
      const dataIdx = dv.getUint16(4, true);
      const dataLen = dv.getUint16(6, true);

      if (dataIdx === VSC_MODE_IDX_INVALID || dataLen === 0) {
        this.lastDataTime = Date.now();
        return;
      }

      if (dataIdx !== this.currentWId) {
        this.currentWId = dataIdx;
      }

      if (len >= TYPE1_TOTAL) {
        const infoOffset = ACK_HEADER_SIZE + TYPE1_ECG_SIZE;
        latestIMU.x = safeFloat(dv, infoOffset + INFO_GSEN_X * 4);
        latestIMU.y = safeFloat(dv, infoOffset + INFO_GSEN_Y * 4);
        latestIMU.z = safeFloat(dv, infoOffset + INFO_GSEN_Z * 4);
        latestIMU.updated = true;

        this.lastDataTime = Date.now();
        this.currentWId = (this.currentWId + 1) % VSC_MODE_IDX_MAX;
      }
    }
  }
}

// ============================================================
//  Minimal UI Controller — 僅連線/量測狀態按鈕
// ============================================================
class UIController {
  constructor() {
    this.els = {
      connStatus: document.getElementById('conn-status'),
      btnConnect: document.getElementById('btn-connect'),
      btnDisconnect: document.getElementById('btn-disconnect'),
      btnStart: document.getElementById('btn-start'),
      btnStop: document.getElementById('btn-stop'),
      accelX: document.getElementById('val-accel-x'),
      accelY: document.getElementById('val-accel-y'),
      accelZ: document.getElementById('val-accel-z')
    };
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
  }

  updateIMU(x, y, z) {
    this.els.accelX.textContent = x.toFixed(3);
    this.els.accelY.textContent = y.toFixed(3);
    this.els.accelZ.textContent = z.toFixed(3);
  }

  resetIMU() {
    this.els.accelX.textContent = '--';
    this.els.accelY.textContent = '--';
    this.els.accelZ.textContent = '--';
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

  // IMU 前端顯示 — 每 500ms 更新一次
  let imuDisplayIntervalId = null;
  const startImuDisplay = () => {
    if (imuDisplayIntervalId) return;
    imuDisplayIntervalId = setInterval(() => {
      if (!latestIMU.updated) return;
      ui.updateIMU(latestIMU.x, latestIMU.y, latestIMU.z);
    }, IMU_DISPLAY_INTERVAL_MS);
  };
  const stopImuDisplay = () => {
    if (imuDisplayIntervalId) { clearInterval(imuDisplayIntervalId); imuDisplayIntervalId = null; }
  };

  // Wire callbacks
  ble.onNotification = (event) => assembler.handleNotification(event);
  ble.onDisconnect = () => {
    protocol.stopMeasurement();
    stopImuDisplay();
    latestIMU.updated = false;
    ui.resetIMU();
    ui.setConnectionState('disconnected');
    ui.setMeasuring(false);
    console.warn('[BLE] 裝置已斷線');
  };

  // Button handlers
  document.getElementById('btn-connect').addEventListener('click', async () => {
    ui.setConnectionState('connecting');
    try {
      await ble.connect();
      ui.setConnectionState('connected');
    } catch (e) {
      ui.setConnectionState('disconnected');
      console.error('[APP] 連線失敗:', e.message);
    }
  });

  document.getElementById('btn-disconnect').addEventListener('click', async () => {
    await protocol.stopMeasurement();
    stopImuDisplay();
    await ble.disconnect();
    latestIMU.updated = false;
    ui.resetIMU();
    ui.setConnectionState('disconnected');
    ui.setMeasuring(false);
  });

  document.getElementById('btn-start').addEventListener('click', async () => {
    latestIMU.updated = false;
    ui.resetIMU();
    ui.setMeasuring(true);
    await protocol.startMeasurement();
    startImuDisplay();
  });

  document.getElementById('btn-stop').addEventListener('click', async () => {
    await protocol.stopMeasurement();
    stopImuDisplay();
    ui.setMeasuring(false);
  });

  // Initial state
  ui.setConnectionState('disconnected');
  console.debug('[APP] ready');
});
