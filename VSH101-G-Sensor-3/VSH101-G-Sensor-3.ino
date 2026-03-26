#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

/* BLE UUID */

BLEUUID serviceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLEUUID txUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
BLEUUID rxUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");

/* BLE objects */

BLEAdvertisedDevice* targetDevice;

BLEClient* client;
BLERemoteCharacteristic* txChar;
BLERemoteCharacteristic* rxChar;

bool deviceFound=false;
bool connected=false;

unsigned long lastRead=0;

/* ---------- Binary helper ---------- */

float readFloat(uint8_t *data)
{
  float v;
  memcpy(&v,data,4);
  if(isnan(v) || isinf(v)) return 0;
  return v;
}

uint32_t readUInt32(uint8_t *data)
{
  uint32_t v;
  memcpy(&v,data,4);
  return v;
}

/* ---------- Debug packet ---------- */

void printHex(uint8_t *data,size_t len)
{
  for(int i=0;i<len;i++)
  {
    if(data[i]<16) Serial.print("0");
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/* ---------- Packet parser ---------- */

void parsePacket(uint8_t *data,size_t len)
{

  if(len < 32) return;

  /* timestamp */

  uint32_t timestamp = readUInt32(&data[0]);

  /* ECG */

  float ecg = 0;

  if(len >= 28)
    ecg = readFloat(&data[24]);

  /* G sensor */

  float gx=0;
  float gy=0;
  float gz=0;

  if(len >= 12)
  {
    int pos = len - 12;

    gx = readFloat(&data[pos]);
    gy = readFloat(&data[pos+4]);
    gz = readFloat(&data[pos+8]);
  }

  /* HR + battery */

  uint8_t hr=0;
  uint8_t battery=0;

  if(len > 18)
  {
    hr = data[16];
    battery = data[17];
  }

  /* CSV output */

  Serial.print(timestamp);
  Serial.print(",");

  Serial.print(ecg,3);
  Serial.print(",");

  Serial.print(gx,3);
  Serial.print(",");

  Serial.print(gy,3);
  Serial.print(",");

  Serial.print(gz,3);
  Serial.print(",");

  Serial.print(hr);
  Serial.print(",");

  Serial.println(battery);

}

/* ---------- notify callback ---------- */

static void notifyCallback(
BLERemoteCharacteristic* chr,
uint8_t* data,
size_t length,
bool isNotify)
{

  if(length==0) return;

  parsePacket(data,length);

}

/* ---------- scan ---------- */

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // 將 std::string 改為 Arduino 的 String
    String name = advertisedDevice.getName();

    // 使用 indexOf() 來取代原本的 find()
    if(name.indexOf("VSH101") >= 0)
    {
      Serial.println("Found VSH101");

      targetDevice = new BLEAdvertisedDevice(advertisedDevice);

      deviceFound=true;

      BLEDevice::getScan()->stop();
    }
  }
};

/* ---------- START command ---------- */

void sendStart()
{

  uint8_t cmd[20] =
  {0x64,0xC2,0x64,0x8A,
   0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00};

  rxChar->writeValue(cmd,20,true);

  Serial.println("START SENT");

}

/* ---------- READ command ---------- */

void sendRead()
{

  uint8_t cmd[20] =
  {0x6A,0xC2,0x6A,0xE3,
   0x00,0x00,0x38,0x02,
   0x00,0x00,0x00,0x00,
   0x01,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00};

  rxChar->writeValue(cmd,20,true);

}

/* ---------- connect ---------- */

bool connectToServer()
{

  client = BLEDevice::createClient();

  Serial.println("Connecting...");

  if(!client->connect(targetDevice))
  {
    Serial.println("Connect fail");
    return false;
  }

  Serial.println("Connected");

  BLERemoteService* service = client->getService(serviceUUID);

  if(service==nullptr)
  {
    Serial.println("Service fail");
    return false;
  }

  txChar = service->getCharacteristic(txUUID);
  rxChar = service->getCharacteristic(rxUUID);

  if(txChar==nullptr || rxChar==nullptr)
  {
    Serial.println("Char fail");
    return false;
  }

  /* enable notify */

  BLERemoteDescriptor* desc =
  txChar->getDescriptor(BLEUUID((uint16_t)0x2902));

  if(desc!=nullptr)
  {
    uint8_t notifyOn[] = {0x01,0x00};
    desc->writeValue(notifyOn,2,true);
  }

  txChar->registerForNotify(notifyCallback);

  Serial.println("Notify enabled");

  sendStart();

  connected=true;

  return true;

}

/* ---------- setup ---------- */

void setup()
{

  Serial.begin(115200);

  Serial.println("VSH101 FULL Reader");

  Serial.println("timestamp,ecg,gx,gy,gz,hr,battery");

  BLEDevice::init("");

  BLEScan* scan = BLEDevice::getScan();

  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

  scan->setActiveScan(true);

  scan->start(10,false);

}

/* ---------- loop ---------- */

void loop()
{

  if(deviceFound && !connected)
  {
    connectToServer();
  }

  if(connected)
  {

    if(millis()-lastRead>200)
    {
      lastRead=millis();

      sendRead();
    }

  }

  delay(10);

}
