#include "supercharger.h"

#include <ArduinoBLE.h>

#include <HardwareBLESerial.h>

#include <EEPROM.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <SPI.h>

#include <ZERO.h>
#include "Network.h"
#include "unions.h"
#include "battery_tables.h"

// chip select pins for the two CAN phys
#define CHARGER_PIN 7
#define BIKE_PIN 10

// set up a new serial object
HardwareBLESerial &btSerial = HardwareBLESerial::getInstance();

Zero zero;

// replace "delay iterations" shenanigans with timers
int last_report = millis();
int last_heartbeat = millis();
int last_ramp = millis();
int last_dash = millis();
uint32_t last_count = millis();

// wifi stuff
#include "WiFiS3.h"
#include "arduino_secrets.h"
#include <ArduinoMDNS.h>
#include <WiFiUdp.h>

bool wifiHasAddress = false;

// if the eeprom has been set up, this will exist in it at the appropriate offset.
// otherwise, use baked in defaults.
static const long EEPROM_SIGNATURE = 0xcafebabe;

WiFiUDP udp;
MDNS mdns(udp);

char ssid[32] = SECRET_SSID;  // default network SSID (name)
char pass[32] = SECRET_PASS;  // default network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);


#define VERSION 5
#define HeartbeatMessageID 0x1806E5F4
#define HeartbeatMessageLength 8

#define VoltageHighByte 0
#define VoltageLowByte 1
#define CurrentHighByte 2
#define CurrentLowByte 3

#define MaxCentiwattsAddress 30
#define TargetDecivoltsAddress 34
#define ChargerCountAddress 38
#define DeciampsPerCAddress 42
#define CPlusAddress 46
#define RampRateCwPerSecAddress 60
#define APmodeFlagAddress 128
#define WifiConfiguredAddress 129
#define SSIDAddress 130
#define PasswordAddress 163
#define VoltageHardStopAddress 196
#define DashDisplayAddress 197
#define EepromSignatureAddress 198

#define FAN_OVERHEAD_CENTIWATTS 600
#define SLOW_DOWN_VOLTAGE_MARGIN_DECIVOLTS 2

unsigned long CPlus = 0;

int apMode = 0;
int wifiConfigured = 0;
int voltageHardStop = 0;
int dashDisplay = 0;

#define OUTPUT_VOLTS_MIN 700
#define OUTPUT_VOLTS_MAX 1164

unsigned long targetDecivolts = OUTPUT_VOLTS_MAX;

unsigned long actualDecivolts = 0;

// Ramp-up rate, in centiwatts/sec.
// Ramp-down rate is 3x this number.
unsigned long rampRateCwPerSec = 5000;  // 50 W/s

// 1C Rate
long deciampsPerC = 960;

// Target wattage
unsigned long targetCentiwatts = 0;
// Maximum wattage allowed by the station.
unsigned long maxCentiwatts = 0;
// target amps to charge calculated by code
unsigned short chargingDeciamps = 0;

// coulomb counting
uint32_t coulombsSinceStart = 0;

// joule counting
uint32_t joulesSinceStart = 0;

int chargerCount = 0;

const int PROGMEM SPI_CS_PIN = 9;

MCP_CAN Bike(BIKE_PIN);
MCP_CAN Charger(CHARGER_PIN);

long delayIters = 550;
ByteToShort bikeThrottle;

struct BatteryData {
  int32_t millivolts;
  int32_t amps;
  int32_t ampHours;
  int32_t minTemp;
  int32_t maxTemp;
};

BatteryData monolith, powerTank;

unsigned char HeartbeatMessage[HeartbeatMessageLength] = {
  // Voltage, high then low byte in volts
  // default 116.4 not overwritten anywhere,
  //this could be dynamic
  0x04, 0x8c,
  // Current, high then low byte in amps
  //default 32.0 overwritten immediately unless no can messages recieved
  //this can happen on some older units when on 120v
  0x01, 0x40,
  // Reserved/status
  0x00, 0x00, 0x00, 0x00
};

void eep_writeLong(uint32_t addr, uint32_t qty) {
  for (uint32_t i = 0; i < sizeof(qty); i++) {
    EEPROM.write(addr + i, ((uint8_t *)&qty)[i]);
  }
}

uint32_t eep_readLong(uint32_t addr) {
  uint32_t qty;
  for (uint32_t i = 0; i < sizeof(qty); i++) {
    ((uint8_t *)&qty)[i] = EEPROM.read(i + addr);
  }
  return qty;
}

void eep_readString(uint32_t addr, char *retstr) {
  char c;
  for (uint32_t i = addr; i - addr < 33; i++) {
    c = EEPROM.read(i);
    retstr[i - addr] = c;
    if (0 == c)
      return;
  }
  retstr[32] = (char)0;
}

void eep_writeString(uint32_t addr, char *str) {
  for (uint32_t i = addr; i - addr < 33; i++) {
    EEPROM.write(i, str[i - addr]);
    if (str[i - addr] == 0)
      return;
  }
}

void readCommands() {
  if (!btSerial)
    return;

  btSerial.poll();
  char line[128];

  if (!btSerial.availableLines()) {
    return;
  }
  int got = btSerial.readLine(line, 128);
  if (!got) {
    return;
  }
  Serial.println(got);

  int elements;
  String commandString(line);
  String strs[20];
  Serial.print("got cmd ");
  Serial.println(commandString);

  // break it into tokens
  elements = 0;
  while (commandString.length() > 0) {
    int index = commandString.indexOf(',');
    if (index == -1)  // No comma found
    {
      strs[elements++] = commandString;
      break;
    } else {
      strs[elements++] = commandString.substring(0, index);
      commandString = commandString.substring(index + 1);
    }
  }
  if (elements == 1) {
    String cmd = strs[0];
    if (cmd.equalsIgnoreCase("wifiinfo")) {
      if (wifiHasAddress)
        printWiFiStatus();
      else {
        Serial.println("wifi not connected");
        btSerial.println("wifi not connected");
      }
    }
  }
  if (elements == 2) {
    String cmd = strs[0];
    String data = strs[1];
    if (cmd.equalsIgnoreCase("apmode")) {
      EEPROM.write(APmodeFlagAddress, data.toInt());
      apMode = data.toInt();
      NVIC_SystemReset();
    }
    if (cmd.equalsIgnoreCase("wificonfigured")) {
      EEPROM.write(WifiConfiguredAddress, data.toInt());
      wifiConfigured = data.toInt();
      NVIC_SystemReset();
    }
    if (cmd.equalsIgnoreCase("voltagehardstop")) {
      EEPROM.write(VoltageHardStopAddress, data.toInt());
      voltageHardStop = data.toInt();
    }
    if (cmd.equalsIgnoreCase("dashdisplay")) {
      EEPROM.write(DashDisplayAddress, data.toInt());
      dashDisplay = data.toInt();
    }
    if (cmd.equalsIgnoreCase("ssid")) {
      char buf[33];
      data.toCharArray(buf, 33);
      eep_writeString(SSIDAddress, buf);
      NVIC_SystemReset();
    }
    if (cmd.equalsIgnoreCase("pass")) {
      char buf[33];
      data.toCharArray(buf, 33);
      eep_writeString(PasswordAddress, buf);
      NVIC_SystemReset();
    }
    return;
  }

  if (elements == 6) {
    // there must be exactly six
    String commandVolts = strs[0];
    String commandAmpsPerC = strs[1];
    String commandChargers = strs[2];
    String commandWatts = strs[3];
    String command1CPlus = strs[4];
    String commandRampRateWPerSec = strs[5];

    unsigned long targetVolts = commandVolts.toInt();
    unsigned long targetAmpsPerC = commandAmpsPerC.toInt();
    unsigned long targetChargers = commandChargers.toInt();
    unsigned long targetWattage = commandWatts.toInt();
    unsigned long CPlus = command1CPlus.toInt();
    unsigned long targetRampRateWPerSec = commandRampRateWPerSec.toInt();

    btSerial.poll();

    if (targetVolts >= OUTPUT_VOLTS_MIN && targetVolts <= OUTPUT_VOLTS_MAX) {
      targetDecivolts = targetVolts;
      eep_writeLong(TargetDecivoltsAddress, targetDecivolts);
    }

    if (targetAmpsPerC >= 0) {
      deciampsPerC = targetAmpsPerC * 10;
      eep_writeLong(DeciampsPerCAddress, deciampsPerC);
    }

    if (targetChargers >= 0) {
      chargerCount = targetChargers;
      eep_writeLong(ChargerCountAddress, chargerCount);
    }

    if (targetWattage >= 0) {
      maxCentiwatts = targetWattage * 100;
      eep_writeLong(MaxCentiwattsAddress, maxCentiwatts);
    }

    if (CPlus >= 0) {
      CPlus = CPlus;
      eep_writeLong(CPlusAddress, CPlus);
    }

    if (targetRampRateWPerSec >= 0) {
      rampRateCwPerSec = targetRampRateWPerSec * 100;
      eep_writeLong(RampRateCwPerSecAddress, rampRateCwPerSec);
    }
  }
}

void setTargetPower(unsigned long targetPower) {
  maxCentiwatts = targetPower * 100;
  eep_writeLong(MaxCentiwattsAddress, maxCentiwatts);
}

void setTargetVoltage(unsigned long targetVoltage) {
  if (targetVoltage >= OUTPUT_VOLTS_MIN && targetVoltage <= OUTPUT_VOLTS_MAX) {
    targetDecivolts = targetVoltage;
    eep_writeLong(TargetDecivoltsAddress, targetDecivolts);
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  btSerial.print("SSID: ");
  btSerial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  btSerial.print("IP Address: ");
  btSerial.println((ip.toString()).c_str());

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

bool wifiCheck(void) {
  if (WiFi.localIP().toString().equals("0.0.0.0")) {
    wifiHasAddress = false;
  } else {
    wifiHasAddress = true;
  }
  return wifiHasAddress;
}

void wifiReconnect(void) {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    return;
  }
  if (!wifiConfigured) {
    Serial.println("WiFi not configured.");
    return;
  }
  if (apMode) {
    Serial.print("Recreating access point named: ");
    Serial.println(ssid);

    // Create WPA2 network.
    status = WiFi.beginAP(ssid, pass);
    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
    } else {
    Serial.print("Attempting to reconnect to WPA SSID: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("starting up!");

  if (eep_readLong(EepromSignatureAddress) != EEPROM_SIGNATURE) {
    Serial.println("No EEPROM signature found, initializing from defaults");
    EEPROM.write(APmodeFlagAddress, -1);
    EEPROM.write(WifiConfiguredAddress, 0);
    EEPROM.write(VoltageHardStopAddress, 0);
    EEPROM.write(DashDisplayAddress, -1);
    eep_writeString(SSIDAddress, "sc2.5");
    eep_writeString(PasswordAddress, "hiroprotagonist");
    eep_writeLong(TargetDecivoltsAddress, 1164);
    eep_writeLong(DeciampsPerCAddress, 1040);
    eep_writeLong(ChargerCountAddress, 2);
    eep_writeLong(MaxCentiwattsAddress, 660000);
    eep_writeLong(CPlusAddress, 0);
    eep_writeLong(RampRateCwPerSecAddress, 5000);
    // finally, write the signature and reboot
    eep_writeLong(EepromSignatureAddress, EEPROM_SIGNATURE);

    NVIC_SystemReset();
  }

  targetDecivolts = eep_readLong(TargetDecivoltsAddress);
  deciampsPerC = eep_readLong(DeciampsPerCAddress);
  chargerCount = eep_readLong(ChargerCountAddress);
  maxCentiwatts = eep_readLong(MaxCentiwattsAddress);
  CPlus = eep_readLong(CPlusAddress);
  rampRateCwPerSec = eep_readLong(RampRateCwPerSecAddress);

  apMode = EEPROM.read(APmodeFlagAddress);
  wifiConfigured = EEPROM.read(WifiConfiguredAddress);
  voltageHardStop = EEPROM.read(VoltageHardStopAddress);
  dashDisplay = EEPROM.read(DashDisplayAddress);

  btSerial.beginAndSetupBLE("SuperCharger 2.5 J Edition");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  if (wifiConfigured) {
    WiFi.setHostname("supercharger");
    eep_readString(SSIDAddress, ssid);
    eep_readString(PasswordAddress, pass);
  }

  if (apMode) {
    Serial.print("Creating access point named: ");
    Serial.println(ssid);

    // Create WPA2 network.
    status = WiFi.beginAP(ssid, pass);
    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
    }
  } else {
    int tries = 5;
    while (tries-- && status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);

      // Connect to WPA/WPA2 network:
      status = WiFi.begin(ssid, pass);

      // wait 1 second for connection:
      delay(1000);
    }
  }
  // wait another second for the wifi module to recombobulate itself
  delay(1000);

  wifiCheck();

  // start the web server on port 80
  server.begin();

  mdns.begin(WiFi.localIP(), "supercharger");
  mdns.addServiceRecord("SuperCharger J Edition Web UI._http",
                        80,
                        MDNSServiceTCP);
  mdns.run();

  // you're connected now, so print out the status
  printWiFiStatus();

  Serial.println(targetDecivolts);
  Serial.println(deciampsPerC);

  Serial.println(chargerCount);
  Serial.println(maxCentiwatts);
  Serial.println(CPlus);
  Serial.println(rampRateCwPerSec);
  Serial.println("charger CAN initializing.");

  int tries = 10;
  // initialize the charger CAN bus at baud rate 250kbps
  while (tries-- && CAN_OK != Charger.begin(CAN_250KBPS)) {
    Serial.println("charger CAN still initializing.");

    btSerial.poll();
    Serial.println("charger CAN initializing.");

    delay(10);
  }
  Serial.println("bike CAN initializing.");
  tries = 10;
  // initialize the bike CAN bus at baud rate 500kbps
  while (tries-- && CAN_OK != Bike.begin(CAN_500KBPS)) {
    btSerial.poll();
    Serial.println("bike CAN still initializing.");


    delay(10);
  }
  Serial.println("CAN initialized.");
}

void handleWifi() {
  char formbuf[2048];

  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                    // if you get a client,
    Serial.println("new client");  // print a message out the serial port
    String currentLine = "";       // make a String to hold incoming data from the client
    String requestLine = "";       // we only know we're handling a request after two newlines
    while (client.connected()) {   // loop while the client's connected
      delayMicroseconds(10);       // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {    // if there's bytes to read from the client,
        char c = client.read();    // read a byte, then
        Serial.write(c);           // print it out to the serial monitor
        if (c == '\n') {           // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            // what we do here depends on what the client requested.
            if (requestLine.startsWith("GET /favicon.ico")) {
              client.println("Content-type: image/svg+xml");
              client.println();
              client.write(favicon_turbo, sizeof(favicon_turbo));
              client.println();
            } else {
              client.println("Content-type:text/html; charset=utf-8");
              client.println();

              client.println("<title>SuperCharger v2.5 J Edition</title>");
              client.println("<body>");
              sprintf(formbuf, "<table><tr><td>Actual volts</td><td>%.1f V</td></tr>"
                               "<tr><td>Target volts</td><td>%.1f V</td></tr>"
                               "<tr><td>Monolith volts</td><td>%.3f V</td></tr>"
                               "<tr><td>Power tank volts</td><td>%.3f V</td></tr>"
                               "<tr><td>Voltage hard stop</td><td>%s</td></tr>"
                               "<tr><td>Dash display</td><td>%s</td></tr>"
                               "<tr><td>Total charging amps requested</td><td>%.1f A</td></tr>"
                               "<tr><td>Monolith current</td><td>%d A</td></tr>"
                               "<tr><td>Power tank current</td><td>%d A</td></tr>"
                               "<tr><td>Monolith capacity</td><td>%d Ah</td></tr>"
                               "<tr><td>Power tank capacity</td><td>%d Ah</td></tr>"
                               "<tr><td>Max charging current (1C)</td><td>%.1f A</td></tr>"
                               "<tr><td>Monolith min temp</td><td>%d °C</td></tr>"
                               "<tr><td>Monolith max temp</td><td>%d °C</td></tr>"
                               "<tr><td>Power tank min temp</td><td>%d °C</td></tr>"
                               "<tr><td>Power tank max temp</td><td>%d °C</td></tr>"
                               "<tr><td>Number of charger modules</td><td>%d</td></tr>"
                               "<tr><td>Actual charge power</td><td>%.2f W</td></tr>"
                               "<tr><td>Target charge power</td><td>%.2f W</td></tr>"
                               "<tr><td>Maximum charge power</td><td>%.2f W</td></tr>"
                               "<tr><td>Enable 1C+ charging?</td><td>%s</td></tr>"
                               "<tr><td>Ramp rate</td><td>%.2f W/s</td></tr>"
                               "<tr><td>Coulombs since start</td><td>%d C</td></tr>"
                               "<tr><td>Watt-hours since start</td><td>%.2f W.h</td></tr>"
                               "<tr><td>BLE protocol version</td><td>%d</td></tr>"
                               "</table>",
                      actualDecivolts / 10.0, targetDecivolts / 10.0, monolith.millivolts / 1000.0, powerTank.millivolts / 1000.0,
                      voltageHardStop ? "yes" : "no", dashDisplay ? "yes" : "no", (chargingDeciamps * chargerCount / 10.0),
                      monolith.amps, powerTank.amps, monolith.ampHours, powerTank.ampHours, deciampsPerC / 10.0,
                      monolith.minTemp, monolith.maxTemp, powerTank.minTemp, powerTank.maxTemp,
                      chargerCount, ((monolith.millivolts / 1000.0) * monolith.amps) + ((powerTank.millivolts / 1000.0) * powerTank.amps),
                      targetCentiwatts / 100.0, maxCentiwatts / 100.0, CPlus ? "yes" : "no",
                      rampRateCwPerSec / 100.0, coulombsSinceStart, joulesSinceStart / 3600.0, VERSION);
              client.println(formbuf);
              client.println("Click to change target power: <a href=\"/1200\">1200</a> <a href=\"/1400\">1400</a> <a href=\"/3000\">3000</a>");
              client.println("<a href=\"/5000\">5000</a> <a href=\"/5900\">5900</a> <a href=\"/6600\">6600</a> <a href=\"/7900\">7900</a> <br>");
              client.println("Click to change target voltage: <a href=\"/116.4\">116.4 (100%)</a>   <a href=\"/110.0\">110.0 (80%)</a><br>");
              client.println("<br><br><a href=\"/\">Reload this page</a><br>");
              client.println("</body>");
            }
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            if (currentLine.startsWith("GET"))
              requestLine = currentLine;
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was one of the power settings:
        if (currentLine.endsWith("GET /1200")) {
          setTargetPower(1200);
        }
        if (currentLine.endsWith("GET /1400")) {
          setTargetPower(1400);
        }
        if (currentLine.endsWith("GET /3000")) {
          setTargetPower(3000);
        }
        if (currentLine.endsWith("GET /5000")) {
          setTargetPower(5000);
        }
        if (currentLine.endsWith("GET /5900")) {
          setTargetPower(5900);
        }
        if (currentLine.endsWith("GET /6600")) {
          setTargetPower(6600);
        }
        if (currentLine.endsWith("GET /7900")) {
          setTargetPower(7900);
        }
        // Check to see if the client request was one of the voltage settings:
        if (currentLine.endsWith("GET /116.4")) {
          setTargetVoltage(1164);
        }
        if (currentLine.endsWith("GET /110.0")) {
          setTargetVoltage(1100);
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

uint32_t findTemperatureBasedCLimit() {
  uint32_t monolithLimit = findTemperatureBasedCLimitForPack(&monolith);
  if (powerTank.ampHours > 0) {
    uint32_t powerTankLimit = findTemperatureBasedCLimitForPack(&powerTank);
    return min(monolithLimit, powerTankLimit);
  } else {
    return monolithLimit;
  }
}

uint32_t findTemperatureBasedCLimitForPack(BatteryData *pack) {
  uint32_t coldLimit =
    find_cutback(pack->minTemp, CUTBACK_AT_OR_BELOW, COLD_CUTBACK);
  uint32_t hotLimit =
    find_cutback(pack->maxTemp, CUTBACK_AT_OR_ABOVE, HOT_CUTBACK);

  return min(coldLimit, hotLimit);
}

/**************************************************************************
 *
 * handle charge ramp up and cutback
 *
 **************************************************************************/

void setCurrentLimit() {
  actualDecivolts = max(monolith.millivolts / 100, actualDecivolts);
  actualDecivolts = max(powerTank.millivolts / 100, actualDecivolts);

  if (monolith.ampHours > 0 || powerTank.ampHours > 0) {
    deciampsPerC = (monolith.ampHours + powerTank.ampHours) * 10;
  }

  //if we are not running at full tilt
  //or if we are not yet started this will be a slow ramp
  if (targetCentiwatts < maxCentiwatts) {
    targetCentiwatts = min(targetCentiwatts + rampRateCwPerSec, maxCentiwatts);
  }

  uint32_t voltageCRateLimitThousandths =
    find_cutback(actualDecivolts, CUTBACK_AT_OR_ABOVE, VOLTAGE_CUTBACK);
  uint32_t packTempCRateLimitThousandths = findTemperatureBasedCLimit();
  uint32_t cRateLimitThousandths = min(voltageCRateLimitThousandths, packTempCRateLimitThousandths);

  if (cRateLimitThousandths != UINT32_MAX) {
    // We have a valid C-rate limit. Apply it now.
    // A = ((A/C) * C)
    // W = A * V = (A/C) * C * V
    // Since all are in tenths except the cutback in thousandths, scale down by 100,000.
    uint32_t limitCentiwatts =
      (deciampsPerC * cRateLimitThousandths * actualDecivolts) / 1000;
    targetCentiwatts = min(targetCentiwatts, limitCentiwatts);
  }

  // slow the charger for this cycle if the end voltage reached
  if ((actualDecivolts + SLOW_DOWN_VOLTAGE_MARGIN_DECIVOLTS) > targetDecivolts) {
    if (targetCentiwatts > rampRateCwPerSec * 3) {
      targetCentiwatts -= rampRateCwPerSec * 3;
    } else {
      // having it clip to 3x the ramp rate means we can't scale very low.
      // instead, if we're below 3x the ramp rate, divide the target by three.
      targetCentiwatts /= 3;
    }
  }
  // hard stop the charger for this cycle if the max voltage reached
  if (voltageHardStop && (actualDecivolts > OUTPUT_VOLTS_MAX || actualDecivolts < OUTPUT_VOLTS_MIN)) {
    targetCentiwatts = 0;
  }

  // Make sure we don't request more than the station can deliver.
  targetCentiwatts = min(
    targetCentiwatts,
    maxCentiwatts - (FAN_OVERHEAD_CENTIWATTS * chargerCount));

  chargingDeciamps = targetCentiwatts / actualDecivolts;
  if (CPlus < 1) {
    if (chargingDeciamps > deciampsPerC) {
      chargingDeciamps = deciampsPerC;
    }

    if (monolith.ampHours > 0 && monolith.amps > monolith.ampHours) {
      chargingDeciamps = min(chargingDeciamps, (monolith.ampHours - 2) * 10);
    }

    if (powerTank.ampHours > 0 && powerTank.amps > powerTank.ampHours) {
      chargingDeciamps = min(chargingDeciamps, (powerTank.ampHours - 2) * 10);
    }
  }

  // Divide target charge current among all paralleled chargers.
  chargingDeciamps /= chargerCount;
}


void loop() {
  char formbuf[1024];
  btSerial.poll();
  mdns.run();


  if (btSerial)
    readCommands();

  // wifi stuff
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed, update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  if (!wifiCheck())
    wifiReconnect();

  handleWifi();

  while (CAN_MSGAVAIL == Charger.checkReceive()) {
    // The charger sends out every second a CAN status message with voltage, current and status information.
    unsigned char ReceivedChargerMessageLength = 0;
    unsigned char ReceivedChargerMessage[8];
    Charger.readMsgBuf(&ReceivedChargerMessageLength, ReceivedChargerMessage);
    actualDecivolts = word(ReceivedChargerMessage[VoltageHighByte], ReceivedChargerMessage[VoltageLowByte]);
    mdns.run();
  }

  while (CAN_MSGAVAIL == Bike.checkReceive()) {
    byte len = 0;
    byte buf[zero.messageLength];

    Bike.readMsgBuf(&len, buf);

    short canId = Bike.getCanId();

    if (zero.hasMonolithVoltage(canId)) {
      monolith.millivolts = zero.voltage(len, buf);
    }

    if (zero.hasMonolithAmps(canId)) {
      monolith.amps = abs(zero.amps(len, buf));
    }

    if (zero.hasPowerTankVoltage(canId)) {
      powerTank.millivolts = zero.voltage(len, buf);
    }

    if (zero.hasPowerTankAmps(canId)) {
      powerTank.amps = abs(zero.amps(len, buf));
    }

    if (zero.hasMonolithPackConfig(canId)) {
      monolith.ampHours = zero.AH(len, buf);
    }

    if (zero.hasPowerTankPackConfig(canId)) {
      powerTank.ampHours = zero.AH(len, buf);
    }

    if (zero.hasMonolithPackActiveData(canId)) {
      monolith.maxTemp = zero.highestTemp(len, buf);
      monolith.minTemp = zero.lowestTemp(len, buf);
    }

    if (zero.hasPowerTankPackActiveData(canId)) {
      powerTank.maxTemp = zero.highestTemp(len, buf);
      powerTank.minTemp = zero.lowestTemp(len, buf);
    }

    if (zero.hasThrottle(canId)) {
      bikeThrottle.value = zero.throttle(len, buf);
    }

    if (bikeThrottle.value < 300) {
      if (canId == DASH_STATUS3.id) {
        ByteToLong displayVolts;
        displayVolts.value = actualDecivolts * 10 * KM_TO_MI;
        buf[2] = displayVolts.bytes[0];
        buf[3] = displayVolts.bytes[1];
        if (dashDisplay && (millis() - last_dash > 1000)) {
          last_dash = millis();
          Bike.sendMsgBuf(DASH_STATUS3.id, 0, len, buf);
        }
      }

      if (canId == DASH_STATUS2.id) {
        ByteToLong bikeWatts;
        bikeWatts.value = targetCentiwatts * MI_TO_KM * 100;
        buf[4] = bikeWatts.bytes[0];
        buf[5] = bikeWatts.bytes[1];
        if (dashDisplay && (millis() - last_dash > 1000)) {
          last_dash = millis();
          Bike.sendMsgBuf(DASH_STATUS2.id, 0, len, buf);
        }
      }
    }
  }

  // ramp once a second, but not until we see a bike with a battery
  if (millis() - last_ramp > 1000 && monolith.ampHours > 0) {
    last_ramp = millis();
    setCurrentLimit();
  }

  // coulomb and joule counting
  uint32_t now = millis();
  uint32_t duration = now - last_count;

  if (last_count > now) {
    // we have wrapped around, account for that
    duration = UINT32_MAX - last_count;
    duration += now;
  }

  coulombsSinceStart += (monolith.amps + powerTank.amps) * (duration / 1000.0);
  joulesSinceStart += (((monolith.millivolts / 1000.0) * monolith.amps) 
                      + ((powerTank.millivolts / 1000.0) * powerTank.amps))
                      * (duration / 1000.0);
  last_count = now;

  HeartbeatMessage[CurrentHighByte] = highByte(chargingDeciamps);
  HeartbeatMessage[CurrentLowByte] = lowByte(chargingDeciamps);

  mdns.run();
  btSerial.poll();

  if (btSerial && (millis() - last_report > 1000)) {

    last_report = millis();

    sprintf(formbuf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%d",
            actualDecivolts, monolith.millivolts / 100, powerTank.millivolts / 100, chargingDeciamps * chargerCount,
            monolith.amps, powerTank.amps, monolith.ampHours, powerTank.ampHours,
            deciampsPerC / 10, monolith.minTemp, monolith.maxTemp, powerTank.minTemp,
            powerTank.maxTemp, chargerCount, targetCentiwatts / 100, maxCentiwatts / 100, CPlus,
            rampRateCwPerSec / 100, coulombsSinceStart, joulesSinceStart / 3600.0, VERSION);

    btSerial.println(formbuf);

    btSerial.poll();
  }

  byte chargingBuf[8];

  chargingBuf[0] = 0x1;
  //    Bike.sendMsgBuf(CCU_CHARGE_STATUS.id, 0, zero.messageLength, chargingBuf);
  //    Bike.sendMsgBuf(CALEXMX_CHARGE_STATUS.id, 0, zero.messageLength, chargingBuf);
  //    Bike.sendMsgBuf(CALEX_CHARGE_STATUS.id, 0, zero.messageLength, chargingBuf);

  // frame status = extended in second arg
  if (millis() - last_heartbeat > 1000) {
    last_heartbeat = millis();
    Charger.sendMsgBuf(HeartbeatMessageID, 1, HeartbeatMessageLength, HeartbeatMessage);
  }
}
