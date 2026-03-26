/*
 * ============================================================
 *  VSH101-ECG-IMU-Full.ino
 *  VSH101 心電圖感測器 + IMU 完整即時監測程式（v6 — 正確 ECG/Info 分界）
 * ============================================================
 *  開發板：DOIT ESP32 DEVKIT V1
 *  通訊：BLE (Bluetooth Low Energy) — Nordic UART Service
 *
 *  v6 重大修正（根據 VsApp 原始碼 VscMode.h / VscMode.cpp）：
 *    - TYPE1_DATA_SIZE = 400（100 個 ECG float），非 456！
 *    - Info = 42 個 float = 168 bytes，非 112！
 *    - 所有 Info 欄位均為 float（非混合格式）
 *    - 電池 SOC 在 fInfo[8]，電池秒在 fInfo[9]
 *
 *  TYPE1 資料結構（568 bytes 資料區）：
 *    前 400 bytes：ECG 波形取樣（100 個 float，@500Hz，200ms）
 *    後 168 bytes：Info 區（42 個 float = fInfo[0..41]）
 *      [0]  InfoType      [1]  UTC 時間戳
 *      [2]  溫度 (°C)     [3]  心率 HR (BPM)
 *      [4]  Lead-off      [5]  加速度 X (g)
 *      [6]  加速度 Y      [7]  加速度 Z
 *      [8]  電池 SOC (%)  [9]  電池剩餘秒數
 *      [10] HRV SDNN      [11] HRV NN50
 *      [12] HRV RMSSD     [13] HRV RR
 *      [14] HRV VLF       [15] HRV LF
 *      [16] HRV HF        [17] 氣壓
 *      [18] 氣壓溫度      [19-26] G-sensor 取樣
 *      [27-28] 心律不整    [29-30] VSC 計時
 *      [31] G-sensor 功率  [32-41] 保留
 * ============================================================
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

/* ============================================================
 *  使用者設定區
 * ============================================================ */

#define DEVICE_NAME_FILTER    "VSH101"
#define TARGET_MAC_ADDRESS    ""
#define USE_MAC_FILTER        false

#define READ_INTERVAL_MS      200       // READ 命令間隔 (ms)
#define DISPLAY_INTERVAL_MS   1000      // 顯示更新間隔 (ms)
#define RECONNECT_BASE_MS     2000
#define RECONNECT_MAX_MS      30000
#define RESET_STATS_ON_RECONNECT false

#define SERIAL_BAUD           115200
#define HR_ABNORMAL_THRESHOLD 30        // 心率突變閾值
#define DATA_TIMEOUT_MS       10000

// 除錯：印出前 N 筆重組封包的 HEX（0=關閉）
#define DEBUG_HEX_FIRST_N     20

/* ============================================================
 *  協定常數
 * ============================================================ */

#define ACK_HEADER_SIZE       8         // PacketAck 標頭
#define TYPE1_MISO_LEN        568       // wMISOLen for TYPE1
#define TYPE1_ECG_SIZE        400       // ECG 取樣區（100 floats × 4 = 400 bytes）
#define TYPE1_INFO_SIZE       168       // Info 區（42 floats × 4 = 168 bytes）
#define TYPE1_TOTAL           (ACK_HEADER_SIZE + TYPE1_MISO_LEN)  // 576

// fInfo 陣列索引（每個欄位 = 1 float = 4 bytes）
// 來源：VsApp/inc/VscMode.h
#define INFO_INFOTYPE     0     // float: InfoType（TYPE1=1）
#define INFO_UTC          1     // float: UTC 時間戳
#define INFO_TEMP         2     // float: 溫度 (°C)
#define INFO_HR           3     // float: 心率 HR (BPM)
#define INFO_LEADOFF      4     // float: Lead-off（0=正常，非0=脫落）
#define INFO_GSEN_X       5     // float: 加速度 X (g)
#define INFO_GSEN_Y       6     // float: 加速度 Y (g)
#define INFO_GSEN_Z       7     // float: 加速度 Z (g)
#define INFO_BATT_SOC     8     // float: 電池 SOC (%)
#define INFO_BATT_SEC     9     // float: 電池剩餘秒數
#define INFO_HRV_SDNN     10
#define INFO_HRV_NN50     11
#define INFO_HRV_RMSSD    12
#define INFO_HRV_RR       13
#define INFO_HRV_VLF      14
#define INFO_HRV_LF       15
#define INFO_HRV_HF       16
#define INFO_BARO         17    // float: 氣壓
#define INFO_BARO_TEMP    18    // float: 氣壓溫度
#define INFO_ATR_CODE     27    // float: 心律不整代碼
#define INFO_ATR_TS       28    // float: 心律不整時間戳
#define INFO_COUNT        42    // fInfo 陣列總長

/* ============================================================
 *  BLE UUID — Nordic UART Service
 * ============================================================ */

static BLEUUID serviceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static BLEUUID txUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
static BLEUUID rxUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");

/* ============================================================
 *  VSH101 命令（20 bytes）
 * ============================================================ */

static const uint8_t CMD_START[20] = {
  0x64, 0xC2, 0x64, 0x8A,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// READ 命令模板 — wId 在 bytes 8-9 會動態修改
// 注意：checksum(byte3) 保持 0xE3，裝置實測接受
static uint8_t cmdReadBuf[20] = {
  0x6A, 0xC2, 0x6A, 0xE3,
  0x00, 0x00, 0x38, 0x02,   // MISOLen = 568
  0x00, 0x00, 0x00, 0x00,   // bData[0-1]=wId, bData[2-3]=0
  0x01, 0x00, 0x00, 0x00,   // bData[4-7]=VscModeType=TYPE1
  0x00, 0x00, 0x00, 0x00
};

// VSC_MODE_IDX_INVALID — 裝置返回此值表示「無此索引資料」
#define VSC_MODE_IDX_INVALID 0x1F40   // 8000
#define VSC_MODE_IDX_MAX     8000     // wId 循環上限

// 目前的讀取索引（每次成功讀取後遞增）
static uint16_t currentWId = 0;

static const uint8_t CMD_STOP[20] = {
  0x65, 0xC2, 0x65, 0x8C,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* ============================================================
 *  狀態機
 * ============================================================ */

enum AppState {
  STATE_INIT, STATE_SCANNING, STATE_CONNECTING,
  STATE_RUNNING, STATE_DISCONNECTED
};
static AppState appState = STATE_INIT;

/* ============================================================
 *  統計追蹤器
 * ============================================================ */

struct StatTracker {
  float minVal, maxVal;
  double sum;
  uint32_t count;
  void reset()          { minVal = 999999; maxVal = -999999; sum = 0; count = 0; }
  void update(float v)  { if(v<minVal)minVal=v; if(v>maxVal)maxVal=v; sum+=v; count++; }
  float getAvg()        { return count>0 ? (float)(sum/count) : 0; }
  bool hasData()        { return count>0; }
};

struct BeatStats {
  uint32_t totalBeats, normalBeats, abnormalBeats;
  void reset() { totalBeats=normalBeats=abnormalBeats=0; }
  float normalPct()  { return totalBeats>0 ? normalBeats*100.0f/totalBeats : 0; }
  float abnormalPct() { return totalBeats>0 ? abnormalBeats*100.0f/totalBeats : 0; }
};

/* ============================================================
 *  量測會話
 * ============================================================ */

struct SessionState {
  bool running;
  unsigned long startTimeMs;

  uint32_t utcTimestamp;
  float ecgMv, hrFloat, tempC, leadOff;
  float accelX, accelY, accelZ;
  float battSoc, battSec;
  float hrvSDNN, hrvNN50, hrvRMSSD, hrvRR;
  float hrvVLF, hrvLF, hrvHF;
  uint8_t atrCode;
  uint32_t atrTimestamp;

  StatTracker hrStats, tempStats, batteryStats;
  BeatStats beatStats;
  uint32_t packetCount, fragmentCount;
  unsigned long lastDataTimeMs;
  float prevHr;

  void reset() {
    running=false; startTimeMs=0;
    utcTimestamp=0; ecgMv=0; hrFloat=0; tempC=0; leadOff=0;
    accelX=accelY=accelZ=0; battSoc=0; battSec=0;
    hrvSDNN=hrvNN50=hrvRMSSD=hrvRR=0;
    hrvVLF=hrvLF=hrvHF=0;
    atrCode=0; atrTimestamp=0;
    packetCount=0; fragmentCount=0; lastDataTimeMs=0; prevHr=0;
    hrStats.reset(); tempStats.reset(); batteryStats.reset(); beatStats.reset();
  }

  String elapsedStr() {
    if(!running||!startTimeMs) return "00:00:00";
    unsigned long s=(millis()-startTimeMs)/1000;
    char b[12]; snprintf(b,12,"%02d:%02d:%02d",(int)(s/3600),(int)((s%3600)/60),(int)(s%60));
    return String(b);
  }
};

/* ============================================================
 *  全域變數
 * ============================================================ */

static BLEAdvertisedDevice* targetDevice = nullptr;
static BLEClient* bleClient = nullptr;
static BLERemoteCharacteristic* txChar = nullptr;
static BLERemoteCharacteristic* rxChar = nullptr;

static volatile bool deviceFound = false;
static volatile bool connected = false;

static SessionState session;
static unsigned long lastReadMs = 0, lastDisplayMs = 0;
static unsigned long reconnectDelayMs = RECONNECT_BASE_MS;
static unsigned long disconnectTimeMs = 0;
static BLEScan* bleScan = nullptr;

/* ============================================================
 *  重組緩衝區
 *  所有 BLE 通知片段累積於此，在下一個 READ 前解析。
 * ============================================================ */

#define RX_BUF_SIZE 1024
static uint8_t rxBuf[RX_BUF_SIZE];
static volatile size_t rxBufLen = 0;

/* ============================================================
 *  二進位讀取
 * ============================================================ */

float safeReadFloat(const uint8_t* d) {
  float v; memcpy(&v,d,4);
  return (isnan(v)||isinf(v)) ? 0.0f : v;
}
uint32_t readU32(const uint8_t* d) { uint32_t v; memcpy(&v,d,4); return v; }
uint16_t readU16(const uint8_t* d) { uint16_t v; memcpy(&v,d,2); return v; }
int16_t  readS16(const uint8_t* d) { int16_t  v; memcpy(&v,d,2); return v; }

/* ============================================================
 *  HEX 傾印
 * ============================================================ */

void printHex(const uint8_t* d, size_t len, const char* label) {
  Serial.printf("[HEX][%s] %d bytes:\n", label, (int)len);
  for (size_t i=0; i<len; i++) {
    if (i>0 && i%32==0) Serial.println();
    if (d[i]<0x10) Serial.print("0");
    Serial.print(d[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\n");
}

/* ============================================================
 *  從 Info 區解析所有欄位
 * ============================================================ */

void parseInfoBlock(const uint8_t* info) {
  // 所有 fInfo 欄位均為 float，透過 index * 4 取得 byte offset

  /* InfoType（預期 = 1.0 for TYPE1）*/
  (void)safeReadFloat(&info[INFO_INFOTYPE * 4]);  // 目前不使用

  /* UTC 時間戳 */
  session.utcTimestamp = (uint32_t)safeReadFloat(&info[INFO_UTC * 4]);

  /* 溫度 (°C) */
  session.tempC = safeReadFloat(&info[INFO_TEMP * 4]);

  /* 心率 (BPM) — float；0 或 NaN = 無資料，< 25 視為雜訊 */
  float hrRaw = safeReadFloat(&info[INFO_HR * 4]);
  if (hrRaw >= 25 && hrRaw <= 250) {
    session.hrFloat = hrRaw;
  } else {
    session.hrFloat = 0;
  }

  /* Lead-off（0=正常，非0=電極脫落）*/
  session.leadOff = safeReadFloat(&info[INFO_LEADOFF * 4]);

  /* 加速度 X/Y/Z (g) — 直接為 float */
  session.accelX = safeReadFloat(&info[INFO_GSEN_X * 4]);
  session.accelY = safeReadFloat(&info[INFO_GSEN_Y * 4]);
  session.accelZ = safeReadFloat(&info[INFO_GSEN_Z * 4]);

  /* 電池 SOC (%) */
  session.battSoc = safeReadFloat(&info[INFO_BATT_SOC * 4]);

  /* 電池剩餘秒數 */
  session.battSec = safeReadFloat(&info[INFO_BATT_SEC * 4]);

  /* HRV 參數 */
  session.hrvSDNN  = safeReadFloat(&info[INFO_HRV_SDNN  * 4]);
  session.hrvNN50  = safeReadFloat(&info[INFO_HRV_NN50  * 4]);
  session.hrvRMSSD = safeReadFloat(&info[INFO_HRV_RMSSD * 4]);
  session.hrvRR    = safeReadFloat(&info[INFO_HRV_RR    * 4]);
  session.hrvVLF   = safeReadFloat(&info[INFO_HRV_VLF   * 4]);
  session.hrvLF    = safeReadFloat(&info[INFO_HRV_LF    * 4]);
  session.hrvHF    = safeReadFloat(&info[INFO_HRV_HF    * 4]);

  /* 心律不整 */
  session.atrCode     = (uint8_t)safeReadFloat(&info[INFO_ATR_CODE * 4]);
  session.atrTimestamp = (uint32_t)safeReadFloat(&info[INFO_ATR_TS * 4]);
}

/* ============================================================
 *  更新統計
 * ============================================================ */

void updateStats() {
  float hr = session.hrFloat;
  if (hr > 0 && session.leadOff < 1) {
    session.hrStats.update(hr);
    session.beatStats.totalBeats++;
    bool abnormal = false;
    if (session.prevHr > 0 && fabs(hr - session.prevHr) > HR_ABNORMAL_THRESHOLD)
      abnormal = true;
    if (abnormal) session.beatStats.abnormalBeats++;
    else          session.beatStats.normalBeats++;
    session.prevHr = hr;
  }
  if (session.tempC >= 15 && session.tempC <= 45)
    session.tempStats.update(session.tempC);
  if (session.battSoc > 0 && session.battSoc <= 100)
    session.batteryStats.update(session.battSoc);
}

/* ============================================================
 *  解析重組後的完整封包
 * ============================================================ */

void parseAssembled(const uint8_t* buf, size_t len) {

  // --- 偵測 ACK 回應 ---
  if (len >= ACK_HEADER_SIZE &&
      buf[0] == 0x6A && buf[1] == 0xC2) {

    // 檢查 ACK 或 NACK
    if (buf[2] != 0x41) {
      Serial.printf("[解析] NACK (0x%02X) wId=%d\n", buf[2], currentWId);
      return;
    }

    uint16_t dataIdx = readU16(&buf[4]);
    uint16_t dataLen = readU16(&buf[6]);

    // 無效索引 — 裝置尚未準備好此索引的資料
    if (dataIdx == VSC_MODE_IDX_INVALID || dataLen == 0) {
      // 不遞增 wId，下次重試
      session.lastDataTimeMs = millis();  // 避免超時
      return;
    }

    // 索引不符 — 裝置返回了不同的索引
    if (dataIdx != currentWId) {
      Serial.printf("[解析] 索引不符: 預期=%d 實際=%d，同步至裝置\n",
                    currentWId, dataIdx);
      currentWId = dataIdx;  // 同步至裝置的索引
    }

    // --- 方法 A：完整 576 bytes 回應 ---
    if (len >= TYPE1_TOTAL) {
      const uint8_t* ecg  = &buf[ACK_HEADER_SIZE];
      const uint8_t* info = &buf[ACK_HEADER_SIZE + TYPE1_ECG_SIZE];

      // ECG：從後往前找第一個非零取樣（末尾可能為零填充）
      int nSamples = TYPE1_ECG_SIZE / 4;
      session.ecgMv = 0;
      for (int i = nSamples - 1; i >= 0; i--) {
        float v = safeReadFloat(&ecg[i * 4]);
        if (v != 0.0f) { session.ecgMv = v; break; }
      }

      parseInfoBlock(info);
      updateStats();
      session.packetCount++;
      session.lastDataTimeMs = millis();

      // 成功讀取 → 遞增 wId
      currentWId = (currentWId + 1) % VSC_MODE_IDX_MAX;
      return;
    }

    // ACK 但資料不完整（只收到 8 bytes 標頭）
    Serial.printf("[解析] ACK 但資料不完整: idx=%d len=%d 實收=%d\n",
                  dataIdx, dataLen, (int)len);
    return;
  }

  // --- 方法 B：非 ACK 格式，嘗試直接解析 ---
  if (len >= TYPE1_MISO_LEN) {
    Serial.printf("[解析B] 非 ACK 格式: %d bytes\n", (int)len);
    const uint8_t* ecg  = &buf[0];
    const uint8_t* info = &buf[TYPE1_ECG_SIZE];
    int nSamples = TYPE1_ECG_SIZE / 4;
    session.ecgMv = 0;
    for (int i = nSamples - 1; i >= 0; i--) {
      float v = safeReadFloat(&ecg[i * 4]);
      if (v != 0.0f) { session.ecgMv = v; break; }
    }
    parseInfoBlock(info);
    updateStats();
    session.packetCount++;
    session.lastDataTimeMs = millis();
    currentWId = (currentWId + 1) % VSC_MODE_IDX_MAX;
    return;
  }

  // --- 方法 C：小封包 ---
  if (len >= 32) {
    Serial.printf("[解析C] 小封包: %d bytes\n", (int)len);
    session.utcTimestamp = readU32(&buf[0]);
    if (len > 18) { session.hrFloat = (float)buf[16]; session.battSoc = (float)buf[17]; }
    if (len >= 28) session.ecgMv = safeReadFloat(&buf[24]);
    if (len >= 12) {
      size_t p = len - 12;
      session.accelX = safeReadFloat(&buf[p]);
      session.accelY = safeReadFloat(&buf[p+4]);
      session.accelZ = safeReadFloat(&buf[p+8]);
    }
    updateStats();
    session.packetCount++;
    session.lastDataTimeMs = millis();
    return;
  }

  if (len > 0)
    Serial.printf("[解析] 無法解析: %d bytes\n", (int)len);
}

/* ============================================================
 *  處理重組緩衝區
 * ============================================================ */

void processRxBuffer() {
  if (rxBufLen == 0) return;

  size_t len = rxBufLen;
  uint8_t parseBuf[RX_BUF_SIZE];
  memcpy(parseBuf, rxBuf, len);
  rxBufLen = 0;

  // 除錯 HEX — 印出標頭 + Info 區
  #if DEBUG_HEX_FIRST_N > 0
  {
    static int dbgCnt = 0;
    if (dbgCnt < DEBUG_HEX_FIRST_N) {
      dbgCnt++;
      // 印出標頭（前 8 bytes）
      printHex(parseBuf, min(len, (size_t)8), "標頭");
      // 若封包夠大，印出 Info 區（bytes 464-575）
      if (len >= TYPE1_TOTAL) {
        Serial.printf("[HEX][Info區] offset %d-%d:\n",
                      ACK_HEADER_SIZE + TYPE1_ECG_SIZE,
                      ACK_HEADER_SIZE + TYPE1_ECG_SIZE + TYPE1_INFO_SIZE - 1);
        printHex(&parseBuf[ACK_HEADER_SIZE + TYPE1_ECG_SIZE],
                 TYPE1_INFO_SIZE, "Info");
      }
      // 印出關鍵欄位的原始值以供驗證（v6：全部使用 float 索引）
      if (len >= TYPE1_TOTAL) {
        const uint8_t* inf = &parseBuf[ACK_HEADER_SIZE + TYPE1_ECG_SIZE];
        Serial.printf("  [v6 解析] InfoType=%.0f UTC=%.0f Temp=%.2f HR=%.1f LeadOff=%.0f\n",
          safeReadFloat(&inf[INFO_INFOTYPE * 4]),
          safeReadFloat(&inf[INFO_UTC * 4]),
          safeReadFloat(&inf[INFO_TEMP * 4]),
          safeReadFloat(&inf[INFO_HR * 4]),
          safeReadFloat(&inf[INFO_LEADOFF * 4]));
        Serial.printf("  [G-sensor] X=%.3f Y=%.3f Z=%.3f\n",
          safeReadFloat(&inf[INFO_GSEN_X * 4]),
          safeReadFloat(&inf[INFO_GSEN_Y * 4]),
          safeReadFloat(&inf[INFO_GSEN_Z * 4]));
        Serial.printf("  [電池] SOC=%.1f%% 剩餘=%.0f秒\n",
          safeReadFloat(&inf[INFO_BATT_SOC * 4]),
          safeReadFloat(&inf[INFO_BATT_SEC * 4]));
        Serial.printf("  [心律不整] code=%.0f ts=%.0f\n",
          safeReadFloat(&inf[INFO_ATR_CODE * 4]),
          safeReadFloat(&inf[INFO_ATR_TS * 4]));
        // 印出 ECG 區最後 16 bytes（確認 ECG 資料正確性）
        printHex(&parseBuf[ACK_HEADER_SIZE + TYPE1_ECG_SIZE - 16], 16, "ECG末16B");
        // 印出 Info 前 40 bytes（fInfo[0..9] 的原始 hex）
        printHex(inf, 40, "Info[0-39](fInfo[0..9])");
      }
      Serial.printf("  重組長度=%d wId=%d 片段累計=%u\n\n",
                    (int)len, currentWId, session.fragmentCount);
    }
  }
  #endif

  parseAssembled(parseBuf, len);
}

/* ============================================================
 *  BLE 通知回調 — 片段累積
 * ============================================================ */

static void notifyCallback(
    BLERemoteCharacteristic* chr,
    uint8_t* data, size_t length, bool isNotify)
{
  if (length == 0) return;
  session.fragmentCount++;

  // 偵測新回應的開頭（ACK 標頭：已知命令碼 + GroupId=0xC2）
  if (length >= 3 && data[1] == 0xC2 &&
      (data[0] == 0x6A || data[0] == 0x64 || data[0] == 0x65)) {
    // 新回應開始 → 先處理舊緩衝區（如果有殘留）
    // 注意：這裡不呼叫 processRxBuffer 避免在回調中做太多事
    rxBufLen = 0;
  }

  // 累積片段
  size_t space = RX_BUF_SIZE - rxBufLen;
  size_t copy = (length <= space) ? length : space;
  if (copy > 0) {
    memcpy(&rxBuf[rxBufLen], data, copy);
    rxBufLen += copy;
  }
}

/* ============================================================
 *  BLE 回調
 * ============================================================ */

class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {
    Serial.println("[BLE] 已連線");
  }
  void onDisconnect(BLEClient* c) override {
    connected = false;
    appState = STATE_DISCONNECTED;
    disconnectTimeMs = millis();
    Serial.println("[BLE] 斷線，準備重連...");
  }
};

class MyScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    String name = dev.getName();
    String addr = dev.getAddress().toString();
    bool matched = false;
    #if USE_MAC_FILTER
      String tgt = TARGET_MAC_ADDRESS;
      tgt.toLowerCase(); addr.toLowerCase();
      if (addr == tgt) matched = true;
    #endif
    if (!matched && name.indexOf(DEVICE_NAME_FILTER) >= 0) matched = true;
    if (matched) {
      Serial.printf("[BLE] 找到: %s (%s)\n", name.c_str(), addr.c_str());
      if (targetDevice) delete targetDevice;
      targetDevice = new BLEAdvertisedDevice(dev);
      deviceFound = true;
      BLEDevice::getScan()->stop();
    }
  }
};

/* ============================================================
 *  命令
 * ============================================================ */

void sendStart() {
  if(!rxChar) return;
  rxChar->writeValue((uint8_t*)CMD_START, 20, true);
  currentWId = 0;  // 重設讀取索引
  Serial.println("[CMD] START (wId 重設為 0)");
}

void sendRead() {
  if(!rxChar) return;
  // 動態寫入 wId 至 READ 命令 bytes 8-9（小端序）
  cmdReadBuf[8] = (uint8_t)(currentWId & 0xFF);
  cmdReadBuf[9] = (uint8_t)((currentWId >> 8) & 0xFF);
  rxChar->writeValue(cmdReadBuf, 20, true);
}

void sendStop() {
  if(!rxChar) return;
  rxChar->writeValue((uint8_t*)CMD_STOP, 20, true);
  Serial.println("[CMD] STOP");
}

/* ============================================================
 *  連線
 * ============================================================ */

bool connectToDevice() {
  if (!targetDevice) return false;
  Serial.println("[BLE] 連線中...");

  if (!bleClient) {
    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(new MyClientCallbacks());
  }
  if (!bleClient->connect(targetDevice)) {
    Serial.println("[BLE] 連線失敗");
    return false;
  }

  bleClient->setMTU(517);
  Serial.printf("[BLE] MTU=%d\n", bleClient->getMTU());

  BLERemoteService* svc = bleClient->getService(serviceUUID);
  if (!svc) { Serial.println("[BLE] 服務未找到"); bleClient->disconnect(); return false; }

  txChar = svc->getCharacteristic(txUUID);
  rxChar = svc->getCharacteristic(rxUUID);
  if (!txChar || !rxChar) { Serial.println("[BLE] 特徵未找到"); bleClient->disconnect(); return false; }

  BLERemoteDescriptor* desc = txChar->getDescriptor(BLEUUID((uint16_t)0x2902));
  if (desc) { uint8_t on[]={0x01,0x00}; desc->writeValue(on,2,true); }
  txChar->registerForNotify(notifyCallback);
  Serial.println("[BLE] Notify 已啟用");

  rxBufLen = 0;
  currentWId = 0;
  sendStart();

  connected = true;
  session.running = true;
  session.startTimeMs = millis();
  session.lastDataTimeMs = millis();
  reconnectDelayMs = RECONNECT_BASE_MS;
  return true;
}

void startScan() {
  deviceFound = false;
  Serial.println("[BLE] 掃描中...");
  if (!bleScan) {
    bleScan = BLEDevice::getScan();
    bleScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks());
    bleScan->setActiveScan(true);
  }
  bleScan->clearResults();
  bleScan->start(10, false);
}

/* ============================================================
 *  時間
 * ============================================================ */

String timeStr() {
  unsigned long s = millis()/1000;
  char b[12]; snprintf(b,12,"%02d:%02d:%02d",(int)((s/3600)%24),(int)((s%3600)/60),(int)(s%60));
  return String(b);
}

/* ============================================================
 *  報告顯示
 * ============================================================ */

void displayReport() {
  Serial.println();
  Serial.println("========================================");
  Serial.printf("[%s] VSH101 即時監測報告\n", timeStr().c_str());
  Serial.println("========================================");

  if (session.running)
    Serial.printf("量測狀態: 執行中 | 已運行: %s\n", session.elapsedStr().c_str());
  else
    Serial.println("量測狀態: 停止");
  Serial.printf("封包: %u (片段: %u)\n", session.packetCount, session.fragmentCount);

  Serial.println("----------------------------------------");

  Serial.printf("ECG:     %.3f mV (10mm/mV)\n", session.ecgMv);
  Serial.printf("心律:    %.0f BPM\n", session.hrFloat);
  if (session.hrStats.hasData())
    Serial.printf("  最小:%.0f | 平均:%.0f | 最大:%.0f\n",
      session.hrStats.minVal, session.hrStats.getAvg(), session.hrStats.maxVal);

  Serial.printf("心拍:    共 %u 次\n", session.beatStats.totalBeats);
  if (session.beatStats.totalBeats > 0)
    Serial.printf("  正常:%u(%.1f%%) | 異常:%u(%.1f%%)\n",
      session.beatStats.normalBeats, session.beatStats.normalPct(),
      session.beatStats.abnormalBeats, session.beatStats.abnormalPct());

  Serial.println("----------------------------------------");

  if (session.tempStats.hasData()) {
    Serial.printf("溫度:    %.1f C\n", session.tempC);
    Serial.printf("  最小:%.1f | 平均:%.1f | 最高:%.1f\n",
      session.tempStats.minVal, session.tempStats.getAvg(), session.tempStats.maxVal);
  } else {
    Serial.println("溫度:    等待資料...");
  }

  if (session.battSoc > 0 && session.battSoc <= 100) {
    Serial.printf("電量:    %.0f %%\n", session.battSoc);
    if (session.batteryStats.hasData())
      Serial.printf("  最小:%.0f | 平均:%.0f | 最高:%.0f\n",
        session.batteryStats.minVal, session.batteryStats.getAvg(), session.batteryStats.maxVal);
    if (session.battSec > 0)
      Serial.printf("  預估剩餘: %.0f 秒 (%.1f 小時)\n", session.battSec, session.battSec / 3600.0f);
  } else {
    Serial.println("電量:    等待資料...");
  }

  Serial.println("----------------------------------------");

  Serial.printf("加速度(g):  X=%.3f Y=%.3f Z=%.3f\n",
    session.accelX, session.accelY, session.accelZ);

  Serial.println("----------------------------------------");

  Serial.printf("Lead-off: %s\n",
    session.leadOff > 0 ? "電極脫落" : "電極正常");

  if (session.atrCode > 0)
    Serial.printf("心律不整: 代碼=%d\n", session.atrCode);

  if (connected && bleClient) {
    Serial.printf("BLE: 已連線 | MTU:%d", bleClient->getMTU());
    int rssi = bleClient->getRssi();
    if (rssi!=0) Serial.printf(" | RSSI:%d dBm", rssi);
    Serial.println();
  } else {
    Serial.println("BLE: 未連線");
  }

  Serial.printf("UTC:%u | wId:%d\n",
    session.utcTimestamp, currentWId);
  Serial.println("========================================");
}

/* ============================================================
 *  狀態機
 * ============================================================ */

void handleInit() {
  Serial.println("\n========================================");
  Serial.println("  VSH101 ECG/IMU 完整監測 v6");
  Serial.println("  正確 ECG/Info 分界 + 電池顯示");
  Serial.printf("  過濾: %s | READ:%dms\n", DEVICE_NAME_FILTER, READ_INTERVAL_MS);
  Serial.println("========================================\n");
  BLEDevice::init("ESP32_ECG_Monitor");
  session.reset();
  startScan();
  appState = STATE_SCANNING;
}

void handleScanning() {
  if (deviceFound) { appState = STATE_CONNECTING; return; }
  static unsigned long t = 0;
  if (millis()-t > 12000) { t=millis(); Serial.println("[BLE] 重新掃描..."); startScan(); }
}

void handleConnecting() {
  if (connectToDevice()) { appState = STATE_RUNNING; Serial.println("[狀態] 量測中"); }
  else { appState = STATE_DISCONNECTED; disconnectTimeMs = millis(); }
}

void handleRunning() {
  if (millis()-lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = millis();
    processRxBuffer();  // 解析上一輪累積的資料
    sendRead();         // 發送新的 READ
  }

  if (millis()-lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = millis();
    displayReport();
  }

  if (session.lastDataTimeMs>0 && (millis()-session.lastDataTimeMs)>DATA_TIMEOUT_MS) {
    Serial.println("[警告] 超時，重啟量測...");
    sendStop(); delay(200); sendStart();
    session.lastDataTimeMs = millis();
  }

  if (!connected) { appState=STATE_DISCONNECTED; disconnectTimeMs=millis(); }
}

void handleDisconnected() {
  session.running = false;
  if (millis()-disconnectTimeMs < reconnectDelayMs) return;
  Serial.printf("[BLE] 重連（延遲 %lu ms）...\n", reconnectDelayMs);
  reconnectDelayMs = min(reconnectDelayMs*2, (unsigned long)RECONNECT_MAX_MS);
  #if RESET_STATS_ON_RECONNECT
    session.reset();
  #endif
  deviceFound = false;
  startScan();
  appState = STATE_SCANNING;
}

/* ============================================================
 *  Arduino
 * ============================================================ */

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  appState = STATE_INIT;
}

void loop() {
  switch (appState) {
    case STATE_INIT:         handleInit();         break;
    case STATE_SCANNING:     handleScanning();     break;
    case STATE_CONNECTING:   handleConnecting();   break;
    case STATE_RUNNING:      handleRunning();      break;
    case STATE_DISCONNECTED: handleDisconnected(); break;
  }
  delay(10);
}
