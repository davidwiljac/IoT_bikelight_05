/********************************************************************* 
-> google geolocation API code provided by techiesms
-> Lorawan code provided by JP Meijers Date: 2016-10-20

* Transmit a packet via Loriot. This code scans for WiFi networks and sends the location
 * data obtained from Google's Geolocation API over LoRa using the RN2483 module.

* CHANGE ADDRESS!
 * Change the device address, network (session) key, and app (session) key to the values
 * that are registered via the TTN dashboard.
  * Connect the RN2xx3 as follows:
 * RN2xx3 -- ESP32
 * Uart TX -- GPIO16
 * Uart RX -- GPIO17
 * Reset   -- GPIO23
 * Vcc     -- 3.3V
 * Gnd     -- Gnd

*********************************************************************/

/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. Print the data issued by the LoRaWAN server to the serial port.
 * 
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

unsigned long WifiTimer = 0;

const char* myssid = "simsekkkk";  // your network SSID (name)
const char* mypass = "yavuz123";   // your network password

// Credentials for Google GeoLocation API...
const char* Host = "www.googleapis.com";
String thisPage = "/geolocation/v1/geolocate?key=";
String key = "AIzaSyD34mqo-C3ntDHC_uMehdqB96BXXXPxlMs";

String jsonString = "{\n";



float latitude = 0.0;
float longitude = 0.0;
float accuracy = 0.0;
int more_text = 1;  // set to 1 for more debug output

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC8 };
uint8_t appEui[] = { 0xBE, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x16, 0x47 };
uint8_t appKey[] = { 0x38, 0xAA, 0xBB, 0x98, 0xFA, 0xE1, 0x7B, 0x7B, 0x1E, 0xDC, 0x81, 0x92, 0x18, 0xD6, 0x3A, 0xB1 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x2C, 0x47, 0x8D, 0x1D, 0x5B, 0x51, 0x53, 0x33, 0xEC, 0xC9, 0x62, 0x25, 0x32, 0x5B, 0xFF, 0xFC };
uint8_t appSKey[] = { 0x53, 0xE9, 0x6C, 0xD4, 0x58, 0x01, 0x58, 0x06, 0xB7, 0xB4, 0x1D, 0xEE, 0xD1, 0x43, 0x4F, 0xC1 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_C;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;
uint32_t wifi_delay = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/

uint8_t LED_pin = 9;

uint8_t buffer = 0;
int last_time = 0;

uint8_t confirmedNbTrials = 4;

uint8_t received_data;
uint8_t data = 1;
uint8_t DataReceived[4];


/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port, float lat, float lon) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  unsigned char *puc;

  appDataSize = 0;
  appData[appDataSize++] = 0x00;
  appData[appDataSize++] = 0x01;
  appData[appDataSize++] = 0x02;
  appData[appDataSize++] = 0x04;

  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];  
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];

  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
}

//downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    DataReceived[i] = mcpsIndication->Buffer[i];
    received_data = mcpsIndication->Buffer[i];
    Serial.print("received data: ");
    Serial.println(received_data);
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (received_data != 0) {
    data = received_data;
  }
  Serial.print("stored data: ");
  Serial.println(data);

  if (data == 5) {
    digitalWrite(LED_pin, HIGH);
  } else if (data == 6) {
    digitalWrite(LED_pin, LOW);
  }
}



void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, HIGH);
  Serial.println("setup");

  //initialize WIFI
  initWiFi();
  googlemaps();
}


void loop() {


  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        //both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        Serial.println("join");
        break;
      }
    case DEVICE_STATE_SEND:
      {
        //Scans location every  minutes
        if ((millis() - last_time) > wifi_delay) {
          googlemaps();
          WifiTimer = millis();
        }
        
        prepareTxFrame(appPort, latitude, longitude);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;

        Serial.println("stored data: ");
        Serial.println(data);
        Serial.println("send");
        buffer = 0;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);

        deviceState = DEVICE_STATE_SLEEP;
        Serial.println("cycle");
        buffer = 0;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);

        if (data == 5) {
          digitalWrite(LED_pin, HIGH);
        } else if (data == 6) {
          digitalWrite(LED_pin, LOW);
        }
        // Serial.print("stored data: ");
        // Serial.println(data);
        if (buffer == 0) {
          Serial.println("sleep");
          Serial.println("");
          buffer = 1;
        }

        if ((millis() - last_time) > txDutyCycleTime) {
          deviceState = DEVICE_STATE_SEND;
          last_time = millis();
        }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}



//--------------------------------------------------------------------

void initWiFi() {
  // Set WiFi to station mode and disconnect from any previous AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");

  // Connect to WiFi
  Serial.print("Connecting ");
  WiFi.begin(myssid, mypass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
}


//--------------------------------------------------------------------

void googlemaps() {
  // Create a DynamicJsonDocument for parsing the response
  DynamicJsonDocument jsonBuffer(1024);

  Serial.println("Scan start");
  int n = WiFi.scanNetworks();
  Serial.println("Scan done");

  if (n == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found...");

    if (more_text) {
      // Output formatted JSON to Serial for debugging
      Serial.println("{");
      Serial.println("\"homeMobileCountryCode\": 234,");
      Serial.println("\"homeMobileNetworkCode\": 27,");
      Serial.println("\"radioType\": \"gsm\",");
      Serial.println("\"carrier\": \"Vodafone\",");
      Serial.println("\"wifiAccessPoints\": [");
      for (int i = 0; i < n; ++i) {
        Serial.println("{");
        Serial.print("\"macAddress\" : \"");
        Serial.print(WiFi.BSSIDstr(i));
        Serial.println("\",");
        Serial.print("\"signalStrength\": ");
        Serial.println(WiFi.RSSI(i));
        if (i < n - 1) {
          Serial.println("},");
        } else {
          Serial.println("}");
        }
      }
      Serial.println("]");
      Serial.println("}");
    }
    Serial.println();
  }

  // Build the JSON string for the API request
  jsonString = "{\n";
  jsonString += "\"homeMobileCountryCode\": 234,\n";
  jsonString += "\"homeMobileNetworkCode\": 27,\n";
  jsonString += "\"radioType\": \"gsm\",\n";
  jsonString += "\"carrier\": \"Vodafone\",\n";
  jsonString += "\"wifiAccessPoints\": [\n";
  for (int j = 0; j < n; ++j) {
    jsonString += "{\n";
    jsonString += "\"macAddress\" : \"";
    jsonString += WiFi.BSSIDstr(j);
    jsonString += "\",\n";
    jsonString += "\"signalStrength\": ";
    jsonString += WiFi.RSSI(j);
    jsonString += "\n";
    if (j < n - 1) {
      jsonString += "},\n";
    } else {
      jsonString += "}\n";
    }
  }
  jsonString += "]\n";
  jsonString += "}\n";

  Serial.println("");

  //Needs actual root ca
  WiFiClientSecure client;
  client.setInsecure();  // Disable certificate verification (use with caution)

  Serial.print("Requesting URL: ");
  Serial.println("https://" + String(Host) + thisPage + key);
  Serial.println("");

  if (client.connect(Host, 443)) {
    Serial.println("Connected");
    client.println("POST " + thisPage + key + " HTTP/1.1");
    client.println("Host: " + String(Host));
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("User-Agent: Arduino/1.0");
    client.print("Content-Length: ");
    client.println(jsonString.length());
    client.println();
    client.print(jsonString);
    delay(500);
  }

  // Read and parse the response from the server
  while (client.available()) {
    String line = client.readStringUntil('\r');
    if (more_text) {
      Serial.print(line);
    }
    DeserializationError error = deserializeJson(jsonBuffer, line);
    if (!error) {
      if (jsonBuffer.containsKey("location")) {
        latitude = jsonBuffer["location"]["lat"];
        longitude = jsonBuffer["location"]["lng"];
      }
      if (jsonBuffer.containsKey("accuracy")) {
        accuracy = jsonBuffer["accuracy"];
      }
    }
  }

  Serial.println("Closing connection");
  Serial.println();
  client.stop();

  Serial.println("TXing");
  // Send a comma-separated string of latitude, longitude, and accuracy
  //myLora.tx(String(latitude, 6) + "," + String(longitude, 6));


  Serial.print("Latitude = ");
  Serial.println(latitude, 6);
  Serial.print("Longitude = ");
  Serial.println(longitude, 6);
  Serial.print("Accuracy = ");
  Serial.println(accuracy);
}

//--------------------------------------------------------------------



//--------------------------------------------------------------
