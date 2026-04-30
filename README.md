# VSH101

VSH101 是一套藍牙心電圖（ECG）與 IMU 感測系統，包含：

- **裝置端韌體**：`VSH101-ECG-IMU-Full/`（Arduino/ESP32），負責 BLE 通訊與感測資料輸出。
- **Web 前端**：`web/`，透過 Web Bluetooth 連線裝置並即時顯示 ECG 波形與生理數據。

## 專案結構

```
.
├── VSH101-ECG-IMU-Full/   # 裝置端 Arduino/ESP32 範例
└── web/                  # Web BLE 前端
```

---

# VSH101 Web BLE ECG Monitor

透過 Web Bluetooth API 連線 VSH101 藍牙心電圖裝置，即時顯示 ECG 波形與生理數據的前端網頁應用程式。

## 功能

- **BLE 無線連線** — 透過瀏覽器直接配對 VSH101 裝置，無需安裝任何軟體
- **即時 ECG 波形** — 500Hz 取樣率，5 秒滾動顯示，經典醫療監視器 sweep-line 風格
- **生理數值監測** — 心率 (BPM)、體溫 (°C)、電池電量、電極脫落偵測
- **HRV 心率變異分析** — SDNN、RMSSD、NN50、RR、VLF、LF、HF
- **三軸加速度計** — 即時顯示 X / Y / Z 軸數據
- **自動錯誤恢復** — 10 秒資料超時自動重啟測量

## 環境需求

| 項目 | 需求 |
|------|------|
| 瀏覽器 | Chrome 56+ / Edge 79+（支援 Web Bluetooth API） |
| 連線環境 | HTTPS 或 `localhost`（Web Bluetooth 安全性限制） |
| 作業系統 | Windows 10+、macOS、ChromeOS、Android（iOS 不支援 Web Bluetooth） |
| 硬體 | 電腦或手機需具備藍牙功能 |

## 快速開始

### 1. 啟動本地伺服器

在 `web/` 目錄下執行任一指令：

```bash
# Python 3
python -m http.server 8080

# Node.js (需安裝 http-server)
npx http-server -p 8080
```

### 2. 開啟網頁

使用 Chrome 瀏覽器開啟：

```
http://localhost:8080
```

### 3. 操作流程

1. 開啟 VSH101 裝置電源
2. 點擊 **「連線裝置」** → 在彈出視窗中選擇名稱含 `VSH101` 的裝置
3. 連線成功後，點擊 **「開始測量」**
4. ECG 波形與生理數據將即時更新於畫面上
5. 點擊 **「停止測量」** 結束量測
6. 點擊 **「斷開連線」** 中斷藍牙連線

## 檔案結構

```
web/
├── index.html   # 頁面結構與 dashboard 佈局
├── style.css    # 深色主題樣式、響應式排版
├── app.js       # BLE 通訊、協定解析、Canvas 繪圖、UI 控制
└── README.md    # 本說明文件
```

## 技術架構

### 模組說明

| 模組 | 說明 |
|------|------|
| `BLEConnection` | 管理 Web Bluetooth 連線、GATT 服務發現、通知訂閱 |
| `PacketAssembler` | 累積 BLE 通知片段，重組為完整 576-byte 封包 |
| `VSH101Protocol` | 發送 START/READ/STOP 命令、解析 ACK 回應、管理讀取索引 (wId) |
| `ECGRenderer` | Canvas 即時繪圖，circular buffer (2500 samples)，sweep-line 動畫 |
| `UIController` | 更新 DOM 顯示（心率、溫度、電量、HRV、加速度、狀態面板） |

### 資料流

```
VSH101 裝置
  │  BLE 通知（分片傳輸）
  ▼
PacketAssembler ──累積至 576 bytes──▶ VSH101Protocol.parsePacket()
                                        │
                                        ├─ ECG float32[100] ──▶ ECGRenderer ──▶ Canvas 波形
                                        │
                                        └─ Info float32[42] ──▶ UIController ──▶ DOM 數值

VSH101Protocol 每 200ms 發送 READ 命令 ──▶ 裝置回傳下一筆資料
```

## BLE 協定規格

### 服務與特徵

| 項目 | UUID |
|------|------|
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e`（Nordic UART Service） |
| TX（裝置→網頁，Notify） | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |
| RX（網頁→裝置，Write） | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |

### 命令格式（20 bytes）

| 命令 | Header (bytes 0-3) | 說明 |
|------|---------------------|------|
| START | `64 C2 64 8A` | 開始測量，wId 重設為 0 |
| STOP | `65 C2 65 8C` | 停止測量 |
| READ | `6A C2 6A E3` | 讀取資料，bytes 8-9 為 wId（小端序） |

### 回應封包結構（576 bytes）

```
┌─────────────────────────────────────────────────┐
│ ACK Header (8 bytes)                            │
│   [0] 0x6A  [1] 0xC2  [2] 0x41(ACK)  [3] CS   │
│   [4-5] wDataIdx (uint16, LE)                   │
│   [6-7] wDataLen (uint16, LE) = 568             │
├─────────────────────────────────────────────────┤
│ ECG 波形資料 (400 bytes)                         │
│   100 個 float32（小端序），取樣率 500Hz          │
│   每筆封包包含 200ms 的 ECG 資料                 │
├─────────────────────────────────────────────────┤
│ Info 區 (168 bytes)                              │
│   42 個 float32（小端序），生理參數               │
└─────────────────────────────────────────────────┘
```

### Info 區欄位索引

| 索引 | 欄位 | 單位 | 說明 |
|------|------|------|------|
| 0 | InfoType | - | 資料類型（TYPE1 = 1.0） |
| 1 | UTC | 秒 | Unix 時間戳 |
| 2 | 溫度 | °C | 有效範圍 15-45°C |
| 3 | 心率 | BPM | 有效範圍 25-250，0 = 無資料 |
| 4 | Lead-off | - | 0 = 電極正常，>0 = 脫落 |
| 5-7 | 加速度 X/Y/Z | g | 三軸加速度計 |
| 8 | 電池 SOC | % | 0-100 |
| 9 | 電池剩餘 | 秒 | 預估剩餘時間 |
| 10 | HRV SDNN | ms | 標準差 |
| 11 | HRV NN50 | 次 | NN 間期差 >50ms 的次數 |
| 12 | HRV RMSSD | ms | 連續差值均方根 |
| 13 | HRV RR | ms | 平均 RR 間期 |
| 14-16 | HRV VLF/LF/HF | ms² | 頻域分析 |
| 27 | 心律不整代碼 | - | 異常心律偵測 |

### wId 讀取索引管理

- 初始值為 `0`，每次成功讀取後遞增：`wId = (wId + 1) % 8000`
- 裝置回傳 `wDataIdx = 0x1F40`（8000）表示該索引尚無資料，不遞增 wId，下次重試
- 若裝置回傳的 `wDataIdx` 與預期不符，同步至裝置的索引值

## 疑難排解

| 問題 | 解決方式 |
|------|------|
| 看不到藍牙配對視窗 | 確認使用 Chrome/Edge，且網址為 `localhost` 或 HTTPS |
| 找不到 VSH101 裝置 | 確認裝置已開機、藍牙已開啟、距離在 10 公尺內 |
| 連線後無資料 | 點擊「開始測量」；確認電極已正確貼附 |
| 波形顯示平線 | 檢查電極狀態是否顯示「電極脫落」 |
| 資料中斷 | 系統會在 10 秒無資料後自動重啟測量 |
| iOS 無法使用 | iOS Safari 不支援 Web Bluetooth API，請使用 Android 或電腦 |

## 協定參考

本專案的 BLE 通訊協定基於 `VSH101-ECG-IMU-Full/VSH101-ECG-IMU-Full.ino`，該 Arduino 程式實作了 ESP32 作為 BLE client 連線至 VSH101 裝置的完整邏輯。Web 版本將相同協定移植至瀏覽器端。