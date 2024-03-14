#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_TLA202x.h>
#include <math.h>

// Define DAC pins
#define DAC_DRAIN 25
#define DAC_GATE 26
#define DC_DC_CONV_EN 33 // LM27762 IC En PIN to enable the analog frontend

// Define Battery ADC and Led pins
#define BATTEY_PIN 34    // BATTERY ADC PIN to check the realtime voltage of battery
#define BATTERY_LED 13   // Battery charging status indicator, when the battery is Low
#define BLE_STATUS_LED 4 // BLE LED status PIN, when ble is connected its state is high else it will blinking

// Define DAC voltage values
// #define DAC_DRAIN_VOLT 2000
#define ADC_UPPER 300
#define ADC_LOWER 200
#define maxDAC_voltage 2700
#define minDAC_voltage 100

#define WINDOW_SIZE 5           // Moving Average Filter window size
#define clockFrequency 400000UL // Clock Frequency of I2C communication

#define Threshold_Value 3     //Threshold value for Time Stamp 

// DAC Variables Declaration
uint8_t DAC_D_Call_Value = 0;
uint8_t DAC_G_Call_Value = 0;
size_t DAC_GATE_VOLT = 800;
size_t DAC_DRAIN_VOLT = 800;

int battery_value = 0;

// Filter's Variables
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE] = {0};
int adc_avg_val = 0;

// Triggring Condition Variables
bool timer_notify = false; // timer notification variable
bool first_trigger = false;
bool positive_trigger = false;
bool second_trigger = false;

// BLE Objects and Variables
static NimBLEServer *pServer;
NimBLECharacteristic *tla2024_adc_read = NULL;
NimBLECharacteristic *tla2024_filter_adc_read = NULL;
NimBLECharacteristic *batteryCharacteristic = NULL;
NimBLECharacteristic *timeStampCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLEUUID declaration
NimBLEUUID SENSOR_SERVICE_UUID("79077f9e-6afb-437e-8bf8-3292ed8ea3a1");
NimBLEUUID TLA2024_ADC_UUID("dd0bef0c-6e8b-49cc-a82c-304b64384ec1");
NimBLEUUID TLA2024_FILTER_ADC_UUID("4f1dd4d4-94f6-49db-be0a-9c7b07a9aa85");
NimBLEUUID BATTERY_CHARACTERISTIC_UUID("bddd70d8-0108-43f3-87d4-92ca46997af2");
NimBLEUUID TIME_STAMP_CHARACTERISTIC_UUID("69fe73d8-c6db-4c15-afca-3217bf45792e");
//NimBLEUUID TIME_STAMP_SERVICE_UUID("cdeacb80-5235-4c07-8846-93a37ee6b86d");


Adafruit_TLA202x tla; // Adafruit_TLA2024 object
// int BLE_Status=0;

// Delay using millis() function for BLEstatus LED
unsigned long Y_trackedTime = 0;
bool LED_Y_checkDelay(int toCheck)
{
  unsigned long currentTime = millis();
  if (Y_trackedTime == 0)
  {
    Y_trackedTime = currentTime;
    return false;
  }
  else
  {
    unsigned long dif = currentTime - Y_trackedTime;
    if (dif >= toCheck)
    {
      Y_trackedTime = currentTime;
      return true;
    }
    else
    {
      return false;
    }
  }
}

// Delay using millis() function for BLEstatus LED
unsigned long trackedTime = 0;
bool checkDelay(int toCheck)
{
  unsigned long currentTime = millis();
  if (trackedTime == 0)
  {
    trackedTime = currentTime;
    return false;
  }
  else
  {
    unsigned long dif = currentTime - trackedTime;
    if (dif >= toCheck)
    {
      trackedTime = currentTime;
      return true;
    }
    else
    {
      return false;
    }
  }
}

// ble status function defination, connection status of ble
void ble_status_indicator()
{
  if (deviceConnected == 0)
  {
    if (LED_Y_checkDelay(200))
    {
      digitalWrite(BLE_STATUS_LED, !digitalRead(BLE_STATUS_LED));
    }
  }
  else if (deviceConnected == 1)
  {
    digitalWrite(BLE_STATUS_LED, HIGH);
  }
}

// TLA2024 ADC initialisetion
void tla2024_init()
{

  if (!tla.begin())
  {
    Serial.println("Failed to find TLA202x chip");
    while (1)
    {
      delay(10);
    }
  }

  // tla.setChannel(TLA202x_CHANNEL_0);
  tla.setDataRate(TLA202x_RATE_3300_SPS);
  tla.setMode(TLA202x_MODE_CONTINUOUS);
  tla.setMux(TLA202x_MUX_AIN0_GND);
  tla.setRange(TLA202x_RANGE_4_096_V);
}

// DAC callibration function defination
int DAC_Call(size_t Value)
{
  float DAC_Value_float = 0.0;
  DAC_Value_float = (Value - 102.36) / 12.08; // Practical Voltage equation: Voltage=12.08*Decimal_Value+102.36
  return round(DAC_Value_float);              // return round of voltage
}

bool error_flag = false;

// Sensor's DAC callibration function defination
void DAC_Callibration()
{
  DAC_G_Call_Value = DAC_Call(DAC_GATE_VOLT);  // DAC callibration function call
  DAC_D_Call_Value = DAC_Call(DAC_DRAIN_VOLT); // DAC callibration function call
  dacWrite(DAC_GATE, DAC_G_Call_Value);        // DAC Gate pin set
  dacWrite(DAC_DRAIN, DAC_D_Call_Value);       // DAC Drain pin set
  int current_adc_avg = 0;
  uint8_t DAC_G_New_Call_Value = 0;
  uint8_t DAC_D_New_Call_Value = 0;

  while (1)
  {
    current_adc_avg = tla.readOnce(TLA202x_CHANNEL_0) * 1000;
    Serial.println(current_adc_avg);
    // Serial.println(current_adc_avg - prev_adc_avg);
    delay(3000);

    if (current_adc_avg < ADC_LOWER)
    {
      DAC_GATE_VOLT += 100;
      DAC_DRAIN_VOLT += 100;
      DAC_G_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
      DAC_D_New_Call_Value = DAC_Call(DAC_DRAIN_VOLT);
      if (DAC_GATE_VOLT == maxDAC_voltage && DAC_DRAIN_VOLT == maxDAC_voltage)
      {
        DAC_GATE_VOLT = maxDAC_voltage;
        DAC_DRAIN_VOLT = maxDAC_voltage;
        DAC_G_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
        DAC_D_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
        dacWrite(DAC_GATE, DAC_G_New_Call_Value);
        dacWrite(DAC_DRAIN, DAC_D_New_Call_Value);
        //error_flag = true;
        break;
      }

      dacWrite(DAC_GATE, DAC_G_New_Call_Value);
      dacWrite(DAC_DRAIN, DAC_D_New_Call_Value);
      Serial.print("GATE voltage increase:");
      Serial.println(DAC_GATE_VOLT);
      Serial.print("DRAIN voltage increase:");
      Serial.println(DAC_DRAIN_VOLT);
      digitalWrite(BATTERY_LED, !digitalRead(BATTERY_LED));
    }

    else if (current_adc_avg >= ADC_LOWER && current_adc_avg <= ADC_UPPER)
    {
      // current_DAC_val = DAC_GATE_VOLT;
      // DAC_G_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
      dacWrite(DAC_GATE, DAC_G_New_Call_Value);
      dacWrite(DAC_DRAIN, DAC_D_New_Call_Value);
      Serial.print("GATE voltage stable:");
      Serial.println(DAC_GATE_VOLT);
      Serial.print("DRAIN voltage stable:");
      Serial.println(DAC_DRAIN_VOLT);
      digitalWrite(BATTERY_LED, LOW);
      break;
    }

    else if (current_adc_avg > ADC_UPPER)
    {
      DAC_GATE_VOLT -= 100;
      // DAC_DRAIN_VOLT-=100;
      DAC_G_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
      DAC_D_New_Call_Value = DAC_Call(DAC_DRAIN_VOLT);
      if (DAC_GATE_VOLT == minDAC_voltage)
      {
        DAC_GATE_VOLT = minDAC_voltage;
        DAC_DRAIN_VOLT = minDAC_voltage;
        DAC_G_New_Call_Value = DAC_Call(DAC_GATE_VOLT);
        DAC_D_New_Call_Value = DAC_Call(DAC_DRAIN_VOLT);
        dacWrite(DAC_GATE, DAC_G_New_Call_Value);
        // dacWrite(DAC_DRAIN, DAC_D_New_Call_Value);
        //error_flag = true;
        break;
      }

      dacWrite(DAC_GATE, DAC_G_New_Call_Value);
      dacWrite(DAC_DRAIN, DAC_D_New_Call_Value);
      Serial.print("GATE Voltage decrease:");
      Serial.println(DAC_GATE_VOLT);
      Serial.print("DRAIN Voltage Set:");
      Serial.println(DAC_DRAIN_VOLT);
      digitalWrite(BATTERY_LED, !digitalRead(BATTERY_LED));
    }
  }
}

// NimBLE Server class Callback Function
class MyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    Serial.println("Client connected");
    Serial.println("Multi-connect support: start advertising");
    deviceConnected = true;
    // digitalWrite(BLE_STATUS_PIN,HIGH);
    NimBLEDevice::startAdvertising();
  };
  void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("Client disconnected - start advertising");
    deviceConnected = false;
    // digitalWrite(BLE_STATUS_PIN,LOW);
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc)
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};


// BLE Initialization Function defination
void BLE_init()
{
  NimBLEDevice::init("SenFlexT");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService *SensorService = pServer->createService(SENSOR_SERVICE_UUID);
  //NimBLEService *timeStampService = pServer->createService(TIME_STAMP_SERVICE_UUID);

  tla2024_adc_read = SensorService->createCharacteristic(
      TLA2024_ADC_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  // tla2024_adc_read->addDescriptor(new NimBLE2902());

  tla2024_filter_adc_read = SensorService->createCharacteristic(
      TLA2024_FILTER_ADC_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  // tla2024_filter_adc_read->addDescriptor(new NimBLE2902());

  batteryCharacteristic = SensorService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  // tla2024_filter_adc_read->addDescriptor(new NimBLE2902());

  timeStampCharacteristic = SensorService->createCharacteristic(
      SENSOR_SERVICE_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  // tla2024_filter_adc_read->addDescriptor(new NimBLE2902());


  SensorService->start();
  //timeStampService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SENSOR_SERVICE_UUID);
  //pAdvertising->addServiceUUID(TIME_STAMP_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();

  Serial.println("Waiting a client connection to notify...");
}

// Battery Charging indicator
void battery_status_indicator()
{
  if (battery_value <= 3650)
  {
    digitalWrite(BATTERY_LED, HIGH);
  }
  else if (battery_value >= 3720)
  {
    digitalWrite(BATTERY_LED, LOW);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(clockFrequency);
  pinMode(BATTEY_PIN, INPUT);      // Initialize adc for Battery
  pinMode(BLE_STATUS_LED, OUTPUT); // BLE status PIN OUTPUT initialization
  pinMode(DC_DC_CONV_EN, OUTPUT);
  pinMode(BATTERY_LED, OUTPUT);
  digitalWrite(DC_DC_CONV_EN, HIGH);
  digitalWrite(BATTERY_LED, HIGH);
  // DAC_D_Call_Value = DAC_Call(DAC_DRAIN_VOLT); // DAC callibration function call
  //  DAC_G_Call_Value =DAC_Call(DAC_GATE_VOLT);
  // dacWrite(DAC_DRAIN, DAC_D_Call_Value); // DAC Drain pin set
  //  dacWrite(DAC_GATE, DAC_G_Call_Value);
  tla2024_init(); // Calling tla2024 delta sigma adc initialization function
  DAC_Callibration();
  BLE_init(); // Calling the BLE initialization function
}

// Declaration of Time Stamp Variables
float new_avg_val = 0;
float prev_avg_val = 0;
float dY = 0; //  dY is the difference between two values
double trigger_Val = 0;
unsigned int random_count = 0;
uint8_t positive_counter = 0, negative_counter = 0;

void loop()
{
  ble_status_indicator(); // ble_status function call to check the ble status

  battery_value = (analogReadMilliVolts(BATTEY_PIN))*2;//0.263291139;

  battery_status_indicator();

  /*****Movin Average Filter*****/

  int adcVal = tla.readOnce(TLA202x_CHANNEL_0) * 1000;

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = adcVal;                    // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  adc_avg_val = SUM / WINDOW_SIZE;   // Divide the sum of the window by the window size for the result

  /****Time Stamping Program****/

  if (random_count <= 1000)
    random_count++;
  else
  {
    new_avg_val = adc_avg_val; // Here store the new value of the sensor
    if (new_avg_val != 0 && prev_avg_val == 0)
    {
      prev_avg_val = new_avg_val;
    }
    dY = new_avg_val - prev_avg_val; // difference of the value
    // Serial.print("Avg Value Diff: ");
    // Serial.println(dY);

    // Negative slope detection
    // if (!first_trigger)
    // {
    //   // Condition to reset negative counter
    //   if (dY > 0)
    //   {
    //     negative_counter = 0;
    //     Serial.print("\t\t\t\t\tReset negative counter: ");
    //     Serial.println(negative_counter);
    //   }
    //   // Condition for increase negative counter
    //   if (dY <= -1)
    //   {
    //     negative_counter++;
    //     Serial.print("\t\t\t\tNegative Counter: ");
    //     Serial.println(negative_counter);
    //   }
    //   // Condition for first time stamp
    //   if (negative_counter >= 8)
    //   {
    //     first_trigger = true;
    //     Serial.println("\t\t\t\tFirst time Stamp");
    //   }
    // }

    // if (first_trigger && !second_trigger)
    // {

      // Condition to reset positive counter
      if (dY < 0)
      {
        positive_counter = 0;
        // Serial.print("\t\t\t\t\tReset positive counter: ");
        // Serial.println(positive_counter);
      }

      // Condition to check wheather the angle is less or equal to the value of Threshold
      if (dY >= 3) // && negative_trigger
      {
        // trigger_Val = prev_avg_val;
        positive_trigger = true;
        // Serial.println("\t\t\t\tSecond time Stamp");
      }

      if (dY >= 1 && positive_trigger)
      {
        positive_counter++;
        // Serial.print("\t\t\t\tCounter: ");
        // Serial.println(positive_counter);
      }

      if (!timer_notify && positive_counter >= 12) //&& prev_avg_val >= trigger_Val
      {
        timer_notify = true;
        second_trigger = true;
        // Serial.println("\t\t\t\tTimer Notify!");
      }
    //}
  }

  if (timer_notify  && checkDelay(6000))
  {
    // first_trigger = false;
    second_trigger = false;
    timer_notify = false;
    // Serial.println("\t\t\t\tTimer Notify false after 6sec!");
  }

  prev_avg_val = new_avg_val; // store old value of data

  // notify changed value
  if (pServer->getConnectedCount())
  {
    tla2024_adc_read->setValue(adcVal);
    tla2024_adc_read->notify();
    Serial.print(adcVal);
    Serial.print(",");
    tla2024_filter_adc_read->setValue(adc_avg_val);
    tla2024_filter_adc_read->notify();
    Serial.print(adc_avg_val);
    Serial.print(",");
    batteryCharacteristic->setValue(battery_value);
    batteryCharacteristic->notify();
    Serial.println(battery_value);
    timeStampCharacteristic->setValue(timer_notify);
    timeStampCharacteristic->notify();
    // Serial.println(timer_notify);

    delay(10);
  }
}
