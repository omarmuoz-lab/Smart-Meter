#include "SPI.h"
#include "ADE9000RegMap.h"
#include "ADE9000API.h"
#include "ADE9000CalibrationInputs.h"
#include "arduinoFFT.h"
#include <WiFi.h>
#include <SparkFun_RV8803.h>
#include "esp_task_wdt.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "FS.h"
#include "SD.h"

/*FFT ANALYSIS*/
#define samplesCoh 512
#define samplingFreqCoh 7680
#define samplesNoncoh 256
#define samplingFreqNoncoh 8000
arduinoFFT FFT = arduinoFFT();
/*Coherent FFT variables*/
double arrayCohA[samplesCoh] = {};
double arrayCohB[samplesCoh] = {};
double arrayCohC[samplesCoh] = {};
double sigA[samplesCoh] = {};
double sigB[samplesCoh] = {};
double sigC[samplesCoh] = {};
double imagCohA[samplesCoh] = {};
double imagCohB[samplesCoh] = {};
double imagCohC[samplesCoh] = {};
double binCohA[samplesCoh] = {};
double binCohB[samplesCoh] = {};
double binCohC[samplesCoh] = {};
double binCohSendA[64] = {};
double binCohSendB[64] = {};
double binCohSendC[64] = {};
String sampleTime, completeSample;
String electricSamplesA, electricSamplesB, electricSamplesC;

/*Basic initializations*/
ADE9000Class ade9000;

#define SPI_SPEED 5000000     //SPI Speed
#define CS_PIN 14             //8-->Arduino Zero. 16-->ESP8266
#define ADE9000_RESET_PIN 26  //Reset Pin on HW
#define PM_1 27               //PM1 Pin: 4-->Arduino Zero. 15-->ESP8266
#define EGY_REG_SIZE 6
int c = 0;
bool flag = false;

#define EGY_INTERRUPT_MASK0 0x00000001
#define EGY_INTERRUPT_MASK1 0x00000000

/*Structure decleration */
struct ActivePowerRegs powerRegs;             // Declare powerRegs of type ActivePowerRegs to store Active Power Register data
struct CurrentRMSRegs curntRMSRegs;           //Current RMS
struct VoltageRMSRegs vltgRMSRegs;            //Voltage RMS
struct VoltageTHDRegs voltageTHDRegsnValues;  //Voltage THD
struct CurrentTHDRegs CurrentTHDRegsnValues;
struct ResampledWfbDataNoncoh resampledDataNoncoh;  // Resampled Data
struct ResampledWfbDataCoh resampledData;
struct PeriodRegs freqRegs;  //period
struct VoltageRMSRegs tVrms;
struct CurrentRMSRegs tIrms;
struct ActivePowerRegs watts;
struct ApparentPowerRegs VA;
struct ReactivePowerRegs VAR;
struct WatthrRegValue wattHr;
struct VARhrRegValue varHr;
struct PfRegValue pf;
struct PowerFactorRegs pf2;
struct FundCurrentRMSRegs fIrms;
struct FundVoltageRMSRegs fVrms;
/*Function Decleration*/
void readRegisterData(void);
void readResampledData(void);
void resetADE9000(void);
/*Calibration values decleration*/
int32_t xIgain_registers[3] = { -175309, -80350, 795949};
int32_t xIgain_register_address[3] = {ADDR_AIGAIN, ADDR_BIGAIN, ADDR_CIGAIN};
int32_t xVgain_registers[3] = {90497, 85826, -270434};
int32_t xVgain_register_address[3] = {ADDR_AVGAIN, ADDR_BVGAIN, ADDR_CVGAIN};
int32_t xIoffset_registers[6] = { -314, -1144, -2255, 4, -816, -1955};
int32_t xIoffset_register_address[6] = {ADDR_AIRMSOS, ADDR_BIRMSOS, ADDR_CIRMSOS, ADDR_AIFRMSOS, ADDR_BIFRMSOS, ADDR_CIFRMSOS}; //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
int32_t xVoffset_registers[6] = {604060, 1075923, 1483149, 606638, 1080576, 1486636};
int32_t xVoffset_register_address[6] = {ADDR_AVRMSOS, ADDR_BVRMSOS, ADDR_CVRMSOS, ADDR_AVFRMSOS, ADDR_BVFRMSOS, ADDR_CVFRMSOS}; //order [AIGAIN, BIGAIN, CIGAIN, NIG
int32_t xPhcal_registers[3] = { -41807055, -42324166, -44065521};
int32_t xPhcal_register_address[3] = {ADDR_APHCAL0, ADDR_BPHCAL0, ADDR_CPHCAL0};
//int32_t xPgain_registers[3] = {-154816,115776 ,-630616};
int32_t xPgain_registers[3] = {336500, 1105465, 1554645};
int32_t xPgain_register_address[3] = {ADDR_APGAIN, ADDR_BPGAIN, ADDR_CPGAIN};
int32_t xPoffset1_registers[6] = {48, 38, 32, 98, 85, 76};
int32_t xPoffset_register_address[6] = {ADDR_AWATTOS, ADDR_BWATTOS, ADDR_CWATTOS, ADDR_AFWATTOS, ADDR_BFWATTOS, ADDR_CFWATTOS}; //order [AVGAIN, BVGAIN, CVGAIN, NVGAIN]
int32_t xPoffset2_registers[6] = {46, 37, 35, 127, 116, 114};
int32_t xPoffsetVAR_register_address[6] = {ADDR_AVAROS, ADDR_BVAROS, ADDR_CVAROS, ADDR_AFVAROS, ADDR_BFVAROS, ADDR_CFVAROS};


int32_t accumulatedActiveEnergy_registers[EGY_REG_SIZE];
int32_t xWATTHRHI_registers_address[6] = { ADDR_AWATTHR_HI, ADDR_BWATTHR_HI, ADDR_CWATTHR_HI, ADDR_AFWATTHR_HI, ADDR_BFWATTHR_HI, ADDR_CFWATTHR_HI };
int32_t accumulatedReactiveEnergy_registers[EGY_REG_SIZE];
int32_t xVARHRHI_registers_address[6] = { ADDR_AVARHR_HI, ADDR_BVARHR_HI, ADDR_CVARHR_HI, ADDR_AFVARHR_HI, ADDR_BFVARHR_HI, ADDR_CFVARHR_HI };


float newS, lastS = 0.00;
double wattHrASum = 0.00, wattHrBSum = 0.00, wattHrCSum = 0.00;
double varHrASum = 0.00, varHrBSum = 0.00, varHrCSum = 0.00;
void writeRegisters(int32_t *, int32_t *);
bool lastPage = false;
bool cohFlag = false;
bool noncohFlag = false;
uint32_t load, loadA, loadB, loadC;
/*Variables phase A*/
float freqA, vRmsA, iRmsA, wattA, apA, rpA, PfA, iFundRmsA, vFundRmsA, CurrentTHDValue_A;
float freqB, vRmsB, iRmsB, wattB, apB, rpB, PfB, iFundRmsB, vFundRmsB, CurrentTHDValue_B;
float freqC, vRmsC, iRmsC, wattC, apC, rpC, PfC, iFundRmsC, vFundRmsC, CurrentTHDValue_C;
double wattHrPerSecA = 0.00, wattHrPerSecB = 0.00, wattHrPerSecC = 0.00;
double varHrPerSecA = 0.00, varHrPerSecB = 0.00, varHrPerSecC = 0.00;
bool energyReady = false, harmonicsReady = false, isLoadA = false, isLoadB = false, isLoadC = false, sendingHarmonics = false, firsTimeHarmo = true, online = true, event_sent = true;
int counterSend = 1, lastSecond = 0, counter = 0;
char inputChar[1];
String inputString;
String inputWebPage;

String total[100][1];
String sampleTimeB;
String sampleTimeC;

//variables of event detection
#define mean_window_length 20
#define num_changes 50
#define record_window 500
double mean_window_array[mean_window_length], threshold = 0.300, threshold_min = 0.100, deviation, rising, falling;
double slide_window_active[mean_window_length], slide_window_reactive[mean_window_length], slide_window_apparent[mean_window_length], slide_window_ifund[mean_window_length];
int change_points[num_changes], positive_changes[num_changes], negative_changes[num_changes];
double positive_rec[record_window], positive_rec_finished[record_window], negative_rec[record_window], negative_rec_finished[record_window];
double positive_active[record_window], negative_active[record_window], positive_reactive[record_window], negative_reactive[record_window], positive_apparent[record_window], negative_apparent[record_window], positive_ifund[record_window], negative_ifund[record_window];
int counterPositive = 0, counterNegative = 0;
bool positive_flag = false, recording_positive = false, negative_flag = false, recording_negative = false, saved = false;
float mean_window = 0.00;
int sample_counter = 0, events_counter = 0, positive_counter = 0, negative_counter = 0, done_positive = 0, done_negative = 0;

const char *ssid = "INFINITUM3E94";  // Enter your WiFi name
const char *password = "g2EXf7D9xy";     // Enter WiFi password
//const char *ssid = "omarcell"; // Enter your WiFi name
//const char *password = "omarcell";  // Enter WiFi password

// Configuración del broker MQTT
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "smartMeterTest";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

RV8803 rtc;
int cont = 0;
void setup() {
  Serial.begin(500000);
  Serial.println("");
  if (!SD.begin(16)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  Wire.begin();
  Serial.println("*********************ADE9000 SETUP***************************\n ");
  ade9000.SPI_Write_16(ADDR_RUN, 0);
  pinMode(PM_1, OUTPUT);    //Set PM1 pin as output
  digitalWrite(PM_1, LOW);  //Set PM1 select pin low for PSM0 mode
  pinMode(ADE9000_RESET_PIN, OUTPUT);
  digitalWrite(ADE9000_RESET_PIN, HIGH);
  resetADE9000();
  ade9000.SPI_Init(SPI_SPEED, CS_PIN);  //Initialize SPI
  ade9000.SetupADE9000();               //Initialize ADE9000 registers according to values in ADE9000API.
  Serial.print("RUN Register: ");
  ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFF);
  ade9000.SPI_Write_32(ADDR_STATUS1, 0xFFFFFFFF);
  pinMode(32, INPUT_PULLUP);
  pinMode(15, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(32), updateEnergyRegisterFromInterrupt, FALLING);
  ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFF);
  /*calibration*/
  writeRegisters(xIgain_register_address, xIgain_registers);
  writeRegisters(xVgain_register_address, xVgain_registers);
  writeRegisters(xIoffset_register_address, xIoffset_registers);
  writeRegisters(xVoffset_register_address, xVoffset_registers);
  writeRegisters(xPhcal_register_address, xPhcal_registers);
  writeRegisters(xPgain_register_address, xPgain_registers);
  writeRegisters(xPoffset_register_address, xPoffset1_registers);
  writeRegisters(xPoffsetVAR_register_address, xPoffset2_registers);
  //coherentHarmonics();



  Serial.println("*********************Wifi and Broker conexion******************");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    counter++;
    if (counter > 10) {
      break;
    }
  }
  Serial.println("Connected to the WiFi network");
  if (rtc.begin() == false) {
    Serial.println("Something went wrong, check wiring");
  } else {
    Serial.println("RTC online!");
    setRtcNptTime();
    rtc.updateTime();
    sampleTime = (String)rtc.getYear() + ":" + (String)(rtc.getMonth() + 1) + ":" + (String)rtc.getDate() + ":" + (String)(rtc.getHours()) + ":" + (String)rtc.getMinutes() + ":" + (String)rtc.getSeconds() + ":" + (String)(rtc.getHundredths() * 10) + ",";
    Serial.println(sampleTime);
  }

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public EMQX MQTT broker connected");
      online = true;
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  client.subscribe("webPageStatus");
  client.publish("reconect", "reconecting");
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  firsTimeHarmo = true;
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
    inputChar[i] = (char)payload[i];
  }

  Serial.println();
  Serial.println("-----------------------");
}
float actualTime, lastTime = 0.00;
float actualTimeMillis, lastTimeMillis, lastTimeMillisSend, delta = 10;
void loop() {
  yield();
  checkedInterrupt();
  connect_wifi();
  if (!client.connected()) {
    online = false;
    connect_mqtt();        // Reconectar si la conexión se pierde
  }
  client.loop();           // Mantener la conexión MQTT
  online = true;
  if ((String)rtc.getYear() == "2000") {
    if (rtc.begin() == false) {
      //Serial.println("Something went wrong, check wiring");
    } else {
      //Serial.println("RTC online!");
      setRtcNptTime();
      rtc.updateTime();
      sampleTime = (String)rtc.getYear() + ":" + (String)(rtc.getMonth() + 1) + ":" + (String)rtc.getDate() + ":" + (String)(rtc.getHours()) + ":" + (String)rtc.getMinutes() + ":" + (String)rtc.getSeconds() + ":" + (String)(rtc.getHundredths() * 10) + ",";
      //Serial.println(sampleTime);
    }
  }
  load = (ade9000.SPI_Read_32(ADDR_PHNOLOAD));
  loadA = load & 0x0000003F;
  loadB = load & 0x00000FC0;
  loadC = load & 0x0003F000;
  ade9000.ReadPeriodRegsnValues(&freqRegs);
  ade9000.ReadVoltageRMSRegs(&tVrms);
  ade9000.ReadCurrentRMSRegs(&tIrms);
  ade9000.ReadActivePowerRegs(&watts);
  ade9000.ReadApparentPowerRegs(&VA);
  ade9000.ReadReactivePowerRegs(&VAR);
  ade9000.ReadPfValue(&pf);
  ade9000.ReadPowerFactorRegsnValues(&pf2);
  ade9000.ReadFundCurrentRMSRegs(&fIrms);
  ade9000.ReadFundVoltageRMSRegs(&fVrms);
  ade9000.ReadCurrentTHDRegsnValues(&CurrentTHDRegsnValues);

  if (loadA == 0x0000003F)  //0x0000003F
  {
    isLoadA = false;
    Serial.println("NO-load A");
    freqA = freqRegs.FrequencyValue_A;
    vRmsA = tVrms.vRmsA;
    iRmsA = 0;
    wattA = 0;
    apA = 0;
    rpA = 0;
    PfA = 0;
    iFundRmsA = 0;
    vFundRmsA = 0;
    CurrentTHDValue_A = 0;
  } else {
    isLoadA = true;
    freqA = freqRegs.FrequencyValue_A;
    vRmsA = tVrms.vRmsA;
    iRmsA = tIrms.iRmsA;
    wattA = watts.wattA;
    apA = VA.apA;
    rpA = VAR.rpA;
    PfA = pf.PfA;
    iFundRmsA = fIrms.iFundRmsA;
    vFundRmsA = fVrms.vFundRmsA;
    CurrentTHDValue_A = CurrentTHDRegsnValues.CurrentTHDValue_A;
  }
  if (loadB == 0x00000FC0) {
    isLoadB = false;
    Serial.println("NO-load B");
    freqB = freqRegs.FrequencyValue_B;
    vRmsB = tVrms.vRmsB;
    iRmsB = 0;
    wattB = 0;
    apB = 0;
    rpB = 0;
    PfB = 0;
    iFundRmsB = 0;
    vFundRmsB = 0;
    CurrentTHDValue_B = 0;
  } else {
    isLoadB = true;
    freqB = freqRegs.FrequencyValue_B;
    vRmsB = tVrms.vRmsB;
    iRmsB = tIrms.iRmsB;
    wattB = watts.wattB;
    apB = VA.apB;
    rpB = VAR.rpB;
    PfB = pf.PfB;
    iFundRmsB = fIrms.iFundRmsB;
    vFundRmsB = fVrms.vFundRmsB;
    CurrentTHDValue_B = CurrentTHDRegsnValues.CurrentTHDValue_B;
  }
  if (loadC == 0x0003F000) {
    isLoadC = false;
    Serial.println("NO-load C");
    freqC = freqRegs.FrequencyValue_C;
    vRmsC = tVrms.vRmsC;
    iRmsC = 0;
    wattC = 0;
    apC = 0;
    rpC = 0;
    PfC = 0;
    iFundRmsC = 0;
    vFundRmsC = 0;
    CurrentTHDValue_C = 0;
  } else {
    isLoadC = true;
    freqC = freqRegs.FrequencyValue_C;
    vRmsC = tVrms.vRmsC;
    iRmsC = tIrms.iRmsC;
    wattC = watts.wattC;
    apC = VA.apC;
    rpC = VAR.rpC;
    PfC = pf.PfC;
    iFundRmsC = fIrms.iFundRmsC;
    vFundRmsC = fVrms.vFundRmsC;
    CurrentTHDValue_C = CurrentTHDRegsnValues.CurrentTHDValue_C;
  }

  if (PfA > 0.9998) {
    PfA = 1.00;
    apA = wattA;
  }
  if (PfB > 0.9998) {
    PfB = 1.00;
    apB = wattB;
  }
  if (PfC > 0.9998) {
    PfC = 1.00;
    apC = wattC;
  }


  rtc.updateTime();
  actualTimeMillis = rtc.getHundredths() * 10;

  if (abs(actualTimeMillis - lastTimeMillis) >= delta)
  {
    checkedInterrupt();
    sampleTime = (String)rtc.getYear() + ":" + (String)(rtc.getMonth() + 1) + ":" + (String)rtc.getDate() + ":" + (String)(rtc.getHours()) + ":" + (String)rtc.getMinutes() + ":" + (String)rtc.getSeconds() + ":" + (String)(rtc.getHundredths() * 10) + ",";
    detect_events((iRmsA + iRmsB), (wattA + wattB), (rpA + rpB), (apA + apB), (iFundRmsA + iFundRmsB), sampleTime);
    sample_counter++;
    //electricSamplesA = (String)freqRegs.FrequencyValue_A + "," + (String)tVrms.vRmsA + "," + (String)iRmsA + "," + (String)(wattA + wattB) + "," + (String)(apA + apB) + "," + (String)(rpA / 1000) + "," + (String)(PfA) + "," + (String)(iFundRmsA) + "," + (String)(vFundRmsA) + "," + (String)(CurrentTHDRegsnValues.CurrentTHDValue_A) + ",";
    //electricSamplesB = (String)tVrms.vRmsB + "," + (String)iRmsB + "," + (String)(wattB / 1000) + "," + (String)(apB / 1000) + "," + (String)(rpB / 1000) + "," + (String)(PfB) + "," + (String)(iFundRmsB) + "," + (String)(vFundRmsB) + "," + (String)(CurrentTHDRegsnValues.CurrentTHDValue_B) + ",";
    //electricSamplesC = (String)tVrms.vRmsC + "," + (String)iRmsC + "," + (String)(wattC / 1000) + "," + (String)(apC / 1000) + "," + (String)(rpC / 1000) + "," + (String)(PfC) + "," + (String)(iFundRmsC) + "," + (String)(vFundRmsC) + "," + (String)(CurrentTHDRegsnValues.CurrentTHDValue_C);
    //completeSample += "\n" + sampleTime  + electricSamplesA + electricSamplesB + electricSamplesC;
    //Serial.println(electricSamplesA);
    lastTimeMillis = actualTimeMillis;
  }
  if (abs(actualTimeMillis - lastTimeMillisSend) >= 600 && *inputChar == 'b' && positive_flag == false && negative_flag == false && event_sent == true)
  {
    sampleTime = (String)rtc.getYear() + "," + (String)rtc.getMonth() + "," + (String)rtc.getDate() + "," + (String)(rtc.getHours() - 1) + "," + (String)rtc.getMinutes() + "," + (String)rtc.getSeconds() + "," + (String)(rtc.getHundredths() * 10);
    sendPhaJson("centralPhA", freqA, vRmsA, iRmsA, wattA, apA, rpA, vFundRmsA, iFundRmsA, PfA, CurrentTHDValue_A, sampleTime);
    sendPhaJson("centralPhB", freqB, vRmsB, iRmsB, wattB, apB, rpB, vFundRmsB, iFundRmsB, PfB, CurrentTHDValue_B, sampleTime);
    sendPhaJson("centralPhC", freqC, vRmsC, iRmsC, wattC, apC, rpC, vFundRmsC, iFundRmsC, PfC, CurrentTHDValue_C, sampleTime);
    lastTimeMillisSend = actualTimeMillis;
  }

  if (lastSecond != (int)rtc.getSeconds() && recording_positive == false && recording_negative == false)
  {
    sampleTime = (String)rtc.getYear() + "," + (String)rtc.getMonth() + "," + (String)rtc.getDate() + "," + (String)(rtc.getHours() - 1) + "," + (String)rtc.getMinutes() + "," + (String)rtc.getSeconds() + "," + (String)(rtc.getHundredths() * 10);
    sendsSumPower("sumPower", wattA, wattB, wattC, apA, apB, apC, rpA, rpB, rpC, sampleTime);
    lastSecond = (int)rtc.getSeconds();
  }
  if (energyReady == true)
  {
    if (ade9000.SPI_Read_16(ADDR_EGY_TIME) != ADE9000_EGY_TIME) {
      ade9000.SPI_Write_16(ADDR_EGY_TIME, ADE9000_EGY_TIME);
      Serial.println("Interrupt Restored");
    }
    IRAM_ATTR ade9000.ReadWatthrRegsnValue(&wattHr);
    IRAM_ATTR ade9000.ReadVARhrRegsnValue(&varHr);
    wattHrPerSecA = wattHr.wattHrA;
    wattHrPerSecB = wattHr.wattHrB;
    wattHrPerSecC = wattHr.wattHrC;

    varHrPerSecA = varHr.VARHrA;
    varHrPerSecB = varHr.VARHrB;
    varHrPerSecC = varHr.VARHrC;

    if (online == false) {
      wattHrASum += wattHrPerSecA;
      wattHrBSum += wattHrPerSecB;
      wattHrCSum += wattHrPerSecC;

      varHrASum += varHrPerSecA;
      varHrBSum += varHrPerSecB;
      varHrCSum += varHrPerSecC;


      wattHrPerSecA = wattHrASum;
      wattHrPerSecB = wattHrBSum;
      wattHrPerSecC = wattHrCSum;
      varHrPerSecA = varHrASum;
      varHrPerSecB = varHrBSum;
      varHrPerSecC = varHrCSum;
    } else {
      wattHrASum = 0;
      wattHrBSum = 0;
      wattHrCSum = 0;
      varHrASum = 0;
      varHrBSum = 0;
      varHrCSum = 0;
    }
    sampleTime = (String)rtc.getYear() + "," + (String)rtc.getMonth() + "," + (String)rtc.getDate() + "," + (String)(rtc.getHours() - 1) + "," + (String)rtc.getMinutes() + "," + (String)rtc.getSeconds() + "," + (String)(rtc.getHundredths() * 10);
    sendEnergyJson(sampleTime);
    energyReady = false;
  }
}
void resetADE9000(void) {
  yield();
  digitalWrite(ADE9000_RESET_PIN, LOW);
  delay(50);
  digitalWrite(ADE9000_RESET_PIN, HIGH);
  delay(1000);
  Serial.println("\nReset Done");
}

void writeRegisters(int32_t *regAddress, int32_t *valueAddress) {
  for (int c = 0; c < sizeof(regAddress) - 1; c++) {

    if (c == 0) {
      Serial.print("\nWriting Registers: ");
    }
    ade9000.SPI_Write_32(regAddress[c], valueAddress[c]);
    Serial.print(regAddress[c], HEX);
    Serial.print("|");
    Serial.print((signed)ade9000.SPI_Read_32(regAddress[c]));
    Serial.print("|");
  }
}

IRAM_ATTR void updateEnergyRegisterFromInterrupt() {

  energyReady = true;
  IRAM_ATTR ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFF);
}


void setRtcNptTime() {

  const char *ntpServer = "pool.ntp.org";
  const long gmtOffset_sec = 0;
  const int daylightOffset_sec = 0;
  int sec, minute, hour, date, month, year, weekday;
  struct tm timeinfo;

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  sec = timeinfo.tm_sec;
  minute = timeinfo.tm_min;
  //Serial.println(minute);
  hour = (timeinfo.tm_hour);
  date = timeinfo.tm_mday;
  month = timeinfo.tm_mon;
  year = timeinfo.tm_year + 1900;
  if (rtc.setTime(sec, minute, hour, weekday, date, month, year) == false) {
    Serial.println("Something went wrong setting the time");
  }

  rtc.set24Hour();
}




void appendFile(fs::FS & fs, const char *path, const char *message) {

  SPI.endTransaction();
  SD.begin(16);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
  file.close();
  SPI.endTransaction();
  ade9000.SPI_Init(SPI_SPEED, CS_PIN);  //Initialize SPI
}
double first_positive = 0, first_negative = 0;
bool false_event = false;
String time_event = "";

void detect_events(double new_value, double new_active, double new_reactive, double new_apparent, double new_ifund, String sampleTime) {
  if (sample_counter < mean_window_length) {
    mean_window_array[sample_counter] = new_value;
    slide_window_active[sample_counter] = new_active;
    slide_window_reactive[sample_counter] = new_reactive;
    slide_window_apparent[sample_counter] = new_apparent;
    slide_window_ifund[sample_counter] = new_ifund;
  }
  else {
    mean_window = calculateMean(mean_window_array, mean_window_length);
    deviation = new_value - mean_window;
    rising = max(0.00, deviation);
    falling = min(0.00, deviation);
    if (rising > threshold) {
      //Serial.println("Entro ");
      //Serial.println(sample_counter);
      events_counter++;
      positive_flag = true;
      recording_positive = true;
      event_sent = false;
      //Serial.println("Rising");
    }
    if (falling < -threshold / 2) {
      events_counter++;
      negative_flag = true;
      recording_negative = true;
      event_sent = false;
      //Serial.println("Falling");
    }
    if (recording_positive == true) {

      positive_counter++;
      //Serial.println(positive_counter);
      //      Serial.println("Recording positive");
      //      Serial.print(new_value);
      //      Serial.print(" : ");
      //      Serial.println(new_value-first_positive);
      if (positive_counter < record_window && negative_flag == true) {
        // this ensures that peaks in loads are not taken as negative events
        recording_negative = false;
      }
      if (positive_counter == 1) {
        // if it is the beginning of the recording save the first elements of the window then save the element that cause the event
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, (ADE9000_EGY_TIME + 0x0257));
        time_event = sampleTime;
        copyArray(mean_window_array, positive_rec, mean_window_length);
        copyArray(slide_window_active, positive_active, mean_window_length);
        copyArray(slide_window_reactive, positive_reactive, mean_window_length);
        copyArray(slide_window_apparent, positive_apparent, mean_window_length);
        copyArray(slide_window_ifund, positive_ifund, mean_window_length);

        positive_rec[mean_window_length] = new_value;
        positive_active[mean_window_length] = new_active;
        positive_reactive[mean_window_length] = new_reactive;
        positive_apparent[mean_window_length] = new_apparent;
        positive_ifund[mean_window_length] = new_ifund;

        first_positive = mean_window;
      }
      else if (positive_counter >= (record_window / 2) && positive_flag == true) {
        // if a second positive event happens while the recording was taking place, save the new one
        //copyArray(positive_rec,positive_rec_finished,record_window/2);
        Serial.println("2do Evento");
        memset(positive_rec, 0.00, sizeof(positive_rec));
        memset(positive_active, 0.00, sizeof(positive_active));
        memset(positive_reactive, 0.00, sizeof(positive_reactive));
        memset(positive_apparent, 0.00, sizeof(positive_apparent));
        memset(positive_ifund, 0.00, sizeof(positive_ifund));

        first_positive = 0;
        positive_counter = 1;
        copyArray(mean_window_array, positive_rec, mean_window_length);
        copyArray(slide_window_active, positive_active, mean_window_length);
        copyArray(slide_window_reactive, positive_reactive, mean_window_length);
        copyArray(slide_window_apparent, positive_apparent, mean_window_length);
        copyArray(slide_window_ifund, positive_ifund, mean_window_length);

        positive_rec[mean_window_length] = new_value;
        positive_active[mean_window_length] = new_active;
        positive_reactive[mean_window_length] = new_reactive;
        positive_apparent[mean_window_length] = new_apparent;
        positive_ifund[mean_window_length] = new_ifund;

        first_positive = mean_window;
      }
      /////////error aqui con el threshold
      else if (positive_counter < record_window && (new_value - first_positive) <= 0.050) {
        //if an event do not last the record window  because a falling event has ocurred then delet the recording
        Serial.println("Condicion");
        recording_positive = false;
        positive_flag = false;
        memset(positive_rec, 0.00, sizeof(positive_rec));
        memset(positive_active, 0.00, sizeof(positive_active));
        memset(positive_reactive, 0.00, sizeof(positive_reactive));
        memset(positive_apparent, 0.00, sizeof(positive_apparent));
        memset(positive_ifund, 0.00, sizeof(positive_ifund));
        first_positive = 0;
        positive_counter = 0;
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, (ADE9000_EGY_TIME));
        false_event = true;
        event_sent = true;

      }
      else if (positive_counter >= record_window) {
        //Serial.println("Record positivo");
        double minimum_value = findMinimum(positive_rec, mean_window_length);
        double minimum_value_active = findMinimum(positive_active, mean_window_length);
        double minimum_value_reactive = findMinimum(positive_reactive, mean_window_length);
        double minimum_value_apparent = findMinimum(positive_apparent, mean_window_length);
        double minimum_value_ifund = findMinimum(positive_ifund, mean_window_length);

        first_positive = 0;
        //Serial.println(minimum_value, 4);
        substractedArray(positive_rec, minimum_value, record_window);
        substractedArray(positive_active, minimum_value_active, record_window);
        substractedArray(positive_reactive, minimum_value_reactive, record_window);
        substractedArray(positive_apparent, minimum_value_apparent, record_window);
        substractedArray(positive_ifund, minimum_value_ifund, record_window);

        String rms_current_event = convertArrayToString(positive_rec, record_window);
        String watt_event = convertArrayToString(positive_active, record_window);
        String var_event = convertArrayToString(positive_reactive, record_window);
        String apparent_event = convertArrayToString(positive_apparent, record_window);
        String ifund_event = convertArrayToString(positive_ifund, record_window);
        done_positive++;

        detachInterrupt(digitalPinToInterrupt(32));
        if (ade9000.SPI_Read_16(ADDR_EGY_TIME) == ADE9000_EGY_TIME) {
          ade9000.SPI_Write_16(ADDR_EGY_TIME, 0x0907);
          Serial.println("interrupt change to 8 seconds");
        }


        appendFile(SD, "/events_irms.txt", String(time_event + rms_current_event + "1\n").c_str());
        appendFile(SD, "/events_watts.txt", String(time_event + watt_event + "1\n").c_str());
        appendFile(SD, "/events_var.txt", String(time_event + var_event + "1\n").c_str());
        //appendFile(SD, "/events_va.txt", String(time_event + apparent_event + "1\n").c_str());
        //appendFile(SD, "/events_ifund.txt", String(time_event + ifund_event + "1\n").c_str());
        rtc.updateTime();
        sampleTime = (String)rtc.getYear() + ":" + (String)(rtc.getMonth() + 1) + ":" + (String)rtc.getDate() + ":" + (String)(rtc.getHours()) + ":" + (String)rtc.getMinutes() + ":" + (String)rtc.getSeconds() + ":" + (String)(rtc.getHundredths() * 10) + ",";
        sendSignalJson("irmsEvent", positive_rec, sampleTime);
        delay(10);
        sendSignalJson("wattEvent", positive_active, sampleTime);
        delay(10);
        sendSignalJson("varEvent", positive_reactive, sampleTime);
        delay(600);
        attachInterrupt(digitalPinToInterrupt(32), updateEnergyRegisterFromInterrupt, FALLING);

        event_sent = true;

        Serial.println(String(time_event + rms_current_event + "1\n"));
        Serial.println(String(time_event + watt_event + "1\n"));
        Serial.println(String(time_event + var_event + "1\n"));
        Serial.print(time_event + " Record positive: ");
        Serial.println(done_positive);
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, ADE9000_EGY_TIME);

        recording_positive = false;
        false_event = false;
        recording_negative = false;
        positive_counter = 0;
        time_event = "";
      }
      else {
        positive_rec[mean_window_length + (positive_counter - 1)] = new_value;
        positive_active[mean_window_length + (positive_counter - 1)] = new_active;
        positive_reactive[mean_window_length + (positive_counter - 1)] = new_reactive;
        positive_apparent[mean_window_length + (positive_counter - 1)] = new_apparent;
        positive_ifund[mean_window_length + (positive_counter - 1)] = new_ifund;
        //        positive_rec[positive_counter] = new_value;
        //        positive_active[positive_counter] = new_active;
        //        positive_reactive[positive_counter] = new_reactive;
      }

    }
    if (recording_negative == true && false_event == false) {

      negative_counter++;
      //Serial.println("Recording negative");
      if (negative_counter == 1) {
        time_event = sampleTime;
        // if it is the beginning of the recording save the first elements of the window then save the element that cause the event
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, (ADE9000_EGY_TIME + 0x0257));
        copyArray(mean_window_array, negative_rec, mean_window_length);
        copyArray(slide_window_active, negative_active, mean_window_length);
        copyArray(slide_window_reactive, negative_reactive, mean_window_length);
        copyArray(slide_window_apparent, negative_apparent, mean_window_length);
        copyArray(slide_window_ifund, negative_ifund, mean_window_length);

        negative_rec[mean_window_length] = new_value;
        negative_active[mean_window_length] = new_active;
        negative_reactive[mean_window_length] = new_reactive;
        negative_apparent[mean_window_length] = new_apparent;
        negative_ifund[mean_window_length] = new_ifund;

        first_negative = new_value;

      }
      else if (negative_counter >= (record_window / 2) && negative_flag == true) {
        // if a second negative event happens while the recording was taking place, save the new one
        //copyArray(positive_rec,positive_rec_finished,record_window/2);
        memset(negative_rec, 0.00, sizeof(negative_rec));
        memset(negative_active, 0.00, sizeof(negative_active));
        memset(negative_reactive, 0.00, sizeof(negative_reactive));
        memset(negative_apparent, 0.00, sizeof(negative_apparent));
        memset(negative_ifund, 0.00, sizeof(negative_ifund));

        negative_counter = 1;

        copyArray(mean_window_array, negative_rec, mean_window_length);
        copyArray(slide_window_active, negative_active, mean_window_length);
        copyArray(slide_window_reactive, negative_reactive, mean_window_length);
        copyArray(slide_window_apparent, negative_apparent, mean_window_length);
        copyArray(slide_window_ifund, negative_ifund, mean_window_length);

        negative_rec[mean_window_length] = new_value;
        negative_active[mean_window_length] = new_active;
        negative_reactive[mean_window_length] = new_reactive;
        negative_apparent[mean_window_length] = new_apparent;
        negative_ifund[mean_window_length] = new_ifund;
      }
      else if (negative_counter < record_window && positive_flag == true) {
        //if an event do not last the record window  because a rising event has ocurred then delet the recording
        recording_negative = false;
        memset(negative_rec, 0.00, sizeof(negative_rec));
        memset(negative_active, 0.00, sizeof(negative_active));
        memset(negative_reactive, 0.00, sizeof(negative_reactive));
        memset(negative_apparent, 0.00, sizeof(negative_apparent));
        memset(negative_ifund, 0.00, sizeof(negative_ifund));
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, (ADE9000_EGY_TIME));
        negative_counter = 0;
        event_sent = true;
      }
      else if (negative_counter >= record_window) {
        double minimum_value = findMinimum(negative_rec, record_window);
        double minimum_value_active = findMinimum(negative_active, record_window);
        double minimum_value_reactive = findMinimum(negative_reactive, record_window);
        double minimum_value_apparent = findMinimum(negative_apparent, record_window);
        double minimum_value_ifund = findMinimum(negative_ifund, record_window);

        substractedArray(negative_rec, minimum_value, record_window);
        substractedArray(negative_active, minimum_value_active, record_window);
        substractedArray(negative_reactive, minimum_value_reactive, record_window);
        substractedArray(negative_apparent, minimum_value_apparent, record_window);
        substractedArray(negative_ifund, minimum_value_ifund, record_window);

        //Serial.println(minimum_value, 4);
        //copyArray(negative_rec,negative_rec_finished,record_window);
        //Serial.println("HOLA NEGATIVAS");
        String rms_current_event = convertArrayToString(negative_rec, record_window);
        String watt_event = convertArrayToString(negative_active, record_window);
        String var_event = convertArrayToString(negative_reactive, record_window);
        String apparent_event = convertArrayToString(negative_apparent, record_window);
        String ifund_event = convertArrayToString(negative_ifund, record_window);
        //Serial.println(String(time_event + rms_current_event + "0\n"));

        if (ade9000.SPI_Read_16(ADDR_EGY_TIME) == ADE9000_EGY_TIME) {
          ade9000.SPI_Write_16(ADDR_EGY_TIME, 0x03BF);
          Serial.println("interrupt change to 8 seconds");
        } else if (ade9000.SPI_Read_16(ADDR_EGY_TIME) == 0x03BF) {
          ade9000.SPI_Write_16(ADDR_EGY_TIME, 0x04AF);
          Serial.println("interrupt change to 10 seconds");
        }
        detachInterrupt(digitalPinToInterrupt(32));
        appendFile(SD, "/events_irms.txt", String(time_event + rms_current_event + "0\n").c_str());
        appendFile(SD, "/events_watts.txt", String(time_event + watt_event + "0\n").c_str());
        appendFile(SD, "/events_var.txt", String(time_event + var_event + "0\n").c_str());
        //appendFile(SD, "/events_va.txt", String(time_event + apparent_event + "0\n").c_str());
        //appendFile(SD, "/events_ifund.txt", String(time_event + ifund_event + "0\n").c_str());
        rtc.updateTime();
        sampleTime = (String)rtc.getYear() + ":" + (String)(rtc.getMonth() + 1) + ":" + (String)rtc.getDate() + ":" + (String)(rtc.getHours()) + ":" + (String)rtc.getMinutes() + ":" + (String)rtc.getSeconds() + ":" + (String)(rtc.getHundredths() * 10) + ",";

        sendSignalJson("irmsEvent", negative_rec, sampleTime);
        delay(10);
        sendSignalJson("wattEvent", negative_active, sampleTime);
        delay(10);
        sendSignalJson("varEvent", negative_reactive, sampleTime);
        delay(600);
        attachInterrupt(digitalPinToInterrupt(32), updateEnergyRegisterFromInterrupt, FALLING);
        done_negative++;
        Serial.println(String(time_event + rms_current_event + "1\n"));
        Serial.println(String(time_event + watt_event + "1\n"));
        Serial.println(String(time_event + var_event + "1\n"));
        Serial.print(time_event + "Record negative: ");
        Serial.println(done_negative);
        //ade9000.SPI_Write_16(ADDR_EGY_TIME, (ADE9000_EGY_TIME));
        recording_negative = false;
        event_sent = true;
        time_event = "";
        negative_counter = 0;

      }
      else {
        negative_rec[mean_window_length + (negative_counter - 1)] = new_value;
        negative_active[mean_window_length + (negative_counter - 1)] = new_active;
        negative_reactive[mean_window_length + (negative_counter - 1)] = new_reactive;
        negative_apparent[mean_window_length + (negative_counter - 1)] = new_apparent;
        negative_ifund[mean_window_length + (negative_counter - 1)] = new_ifund;
        //        negative_rec[negative_counter] = new_value;
        //        negative_active[negative_counter] = new_active;
        //        negative_reactive[negative_counter] = new_reactive;
      }
    }
    else {
      recording_negative = false;
    }
    positive_flag = false;
    negative_flag = false;
    add_and_shift(mean_window_array, new_value);
    add_and_shift(slide_window_active, new_active);
    add_and_shift(slide_window_reactive, new_reactive);
    add_and_shift(slide_window_apparent, new_apparent);
    add_and_shift(slide_window_ifund, new_ifund);
  }
}

void add_and_shift(double mean_window_array[], double new_value) {
  //shift  values in the array to the left
  for (int i = 0; i < mean_window_length; i++) {
    mean_window_array[i] = mean_window_array[i + 1];
  }
  // update the last element of the array with the new value
  mean_window_array[mean_window_length - 1] = new_value;
}
float calculateMean(double data[], int length) {
  float sum = 0.0;
  for (int i = 0; i < length; i++) {
    //Serial.println(data[i]);
    sum += data[i];
  }
  return sum / length;
}
void copyArray(double source[], double dest[], int size)
{
  for (int i = 0; i < size; i++)
  {
    dest[i] = source[i];
  }
}
void printArray(double arr[], int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(arr[i]);

    // Print a comma for all elements except the last one
    if (i < length - 1) {
      Serial.print(", ");
    }
  }

  // Print a new line after printing the array
  Serial.println();
}
double findMinimum(double arr[], int arrSize) {
  double minValue = arr[0];  // Initialize minValue with the first element of the array

  for (int i = 1; i < arrSize; i++) {
    if (arr[i] < minValue) {
      //Serial.println(arr[i]);
      minValue = arr[i];  // Update minValue if a smaller value is found
    }
  }
  return minValue;  // Return the minimum value
}
void substractedArray(double arr[], double value, int arrSize) {
  for (int i = 0; i < arrSize; i++) {
    arr[i] -= value;  // Subtract the specified value from each element in the array
  }
}

String convertArrayToString(double arr[], int length) {
  String result = "";

  for (int i = 0; i < length; i++) {
    result += String(arr[i]);
    result += ",";
  }
  return result;
}

void sendPhaJson(const char* topic, float freq, float vRms, float iRms, float watt, float ap, float  rp, float vFundRms, float iFundRms, float pf, float CurrentTHDValue, String sampleTime)
{
  client.loop();
  char jsonSend[500];
  StaticJsonDocument<256> doc;
  doc["freq"] = freq;
  doc["vRms"] = vRms;
  doc["iRms"] = iRms;
  doc["watt"] = watt;
  doc["ap"] = ap;
  doc["rp"] = rp;
  doc["vFundRms"] = vFundRms;
  doc["iFundRms"] = iFundRms;
  doc["Pf"] = pf;
  doc["CurrentTHDValue"] = CurrentTHDValue;
  //doc["wattHrASum"] = wattHrASum;
  doc["counterSend"] = counterSend;
  doc["date"] = sampleTime;
  serializeJson(doc, jsonSend);
  //Serial.println(jsonSend);
  client.publish(topic, jsonSend);

  ESP.getFreeHeap();
}
void sendsSumPower(const char* topic, float wattA, float wattB, float wattC, float apA, float apB, float  apC, float rpA, float rpB, float rpC, String sampleTime)
{
  client.loop();
  char jsonSend[500];
  StaticJsonDocument<256> doc;
  doc["wattA"] = wattA;
  doc["wattB"] = wattB;
  doc["wattC"] = wattC;
  doc["sumWatt"] = wattA + wattB + wattC;
  doc["sumRp"] = rpA + rpC + rpC;
  doc["sumAp"] = apA + apB + apC;
  //doc["counterSend"] = counterSend;
  doc["date"] = sampleTime;
  serializeJson(doc, jsonSend);
  client.publish(topic, jsonSend);
  ESP.getFreeHeap();
}
void sendEnergyJson(String sampleTime) {
  client.loop();
  char jsonSend[500];
  StaticJsonDocument<256> doc;
  doc["egyA"] = wattHrPerSecA;
  doc["egyB"] = wattHrPerSecB;
  doc["egyC"] = wattHrPerSecC;
  doc["sumEnergy"] = wattHrPerSecA + wattHrPerSecB + wattHrPerSecC;
  doc["counterSend"] = counterSend;
  doc["date"] = sampleTime;
  serializeJson(doc, jsonSend);
  client.publish("centralEnergyA", jsonSend);
  ESP.getFreeHeap();

}
void sendSignalJson(const char* topic, double signalX[], String sampleTime) {
  char arraySend[5000];
  DynamicJsonDocument docCurrentSig(9000);

  // Create a JSON array for the signal data and assign it to the key "signal"
  JsonArray arraySig = docCurrentSig.createNestedArray("signal");

  client.loop(); // Maintain MQTT connection

  // Add signal data to the JSON array
  for (int i = 0; i < record_window; i++) {
    arraySig.add(signalX[i]);
  }

  // Add the sampleTime to the JSON document
  docCurrentSig["sampleTime"] = sampleTime;

  // Serialize the JSON document to a string
  serializeJson(docCurrentSig, arraySend);

  // Publish the serialized JSON to the specified topic
  client.publish(topic, arraySend);

  // Optional: Clear the document to free up memory
  docCurrentSig.clear();

  // Check available heap memory
  ESP.getFreeHeap();
}


void connect_mqtt() {
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    checkedInterrupt();
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public EMQX MQTT broker connected");
      online = true;
      client.subscribe("webPageStatus");
      delay(50);
      client.publish("reconect", "reconecting");
      client.loop();
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
void connect_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();      // Disconnect from Wi-Fi if previously connected
    WiFi.begin(ssid, password);  // Attempt to reconnect

    // Wait for reconnection
    while (WiFi.status() != WL_CONNECTED) {
      connect_wifi();
      delay(250);
      Serial.println("Reconnecting to WiFi...");
    }
    Serial.println("Reconnected to WiFi!");
  }
}


void checkedInterrupt() {
  if (digitalRead(32) == LOW) {
    ade9000.SPI_Write_32(ADDR_STATUS0, 0xFFFFFFFF);
    Serial.println("STACKED");
  }
}
