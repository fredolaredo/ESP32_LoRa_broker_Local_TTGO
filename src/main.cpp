#include <Arduino.h>
#include "credentials/credentials.h"

#include <SPI.h>
#include <time.h>

#define DEBUG

// specifique TTGO lora OLED pins
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA   4
#define OLED_SCL  15
#define OLED_RST  16

// SPI LoRa Radio
#define LORA_SCK   5      // GPIO5  - SX1276 SCK
#define LORA_MISO 19      // GPIO19 - SX1276 MISO
#define LORA_MOSI 27      // GPIO27 - SX1276 MOSI
#define LORA_CS   18      // GPIO18 - SX1276 CS
#define LORA_RST  14      // GPIO14 - SX1276 RST
#define LORA_IRQ  26      // GPIO26 - SX1276 IRQ (interrupt request)

unsigned long sketchTime = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

////////////////////////////////
// Program state
////////////////////////////////
enum STATE {
  INIT,
  WAIT,
  WAKEUP,
  RECEIVING,
  MANAGE,
  SEND,
  BROKER,
  DATA,
  SLEEP,
  ERROR
};

volatile STATE state = INIT;
volatile STATE prevState;

void stateToStr(char *text){
  switch(state) {
    case INIT:      strcpy(text, "INIT"); break;
    case WAIT:      strcpy(text, "WAIT"); break;
    case WAKEUP:    strcpy(text, "WAKEUP"); break;
    case RECEIVING: strcpy(text, "RECEIVING"); break;
    case MANAGE:    strcpy(text, "MANAGE"); break;
    case SEND:      strcpy(text, "SEND"); break;
    case BROKER:    strcpy(text, "BROKER"); break;
    case DATA:      strcpy(text, "DATA"); break;
    case SLEEP:     strcpy(text, "SLEEP"); break;
    case ERROR:     strcpy(text, "ERROR"); break;
    default:        strcpy(text, "UNKNOWN"); break;
  }
}

//////////////////////////////////////
// LoRa 
//////////////////////////////////////
#include <SPI.h>
#include <LoRa.h>

#define LORA_BAND                                   870E6    // Hz
#define LORA_TX_POWER                               20        // dBm
#define LORA_BANDWIDTH                              125E3    
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE_DENOMINATOR                 5         // 4/5

String textSend, textRecv ;

const uint8_t LoRa_buffer_size = 128; // Define the payload size here
char txpacket[LoRa_buffer_size];

void IRAM_ATTR onSend() {
  portENTER_CRITICAL_ISR(&mux);
  state = SEND;
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void onTxDone() {
  portENTER_CRITICAL_ISR(&mux);
  //state = SENT;
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRa_sendMessage() {
  LoRa.beginPacket();                 // start packet
  LoRa.print(textSend);               // add payload
  LoRa.endPacket(true);               // finish packet and send it
  Serial.println("send message => "); 
  Serial.println(textSend);
}

void onReceive(int packetSize) {
  portENTER_CRITICAL_ISR(&mux);
  state = RECEIVING;
  portEXIT_CRITICAL_ISR(&mux);
}

void initLoRa(){
  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if (!LoRa.begin(LORA_BAND)) {
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setCodingRate4(LORA_CODINGRATE_DENOMINATOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setTxPower(LORA_TX_POWER);
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  //attachInterrupt(digitalPinToInterrupt(LORA_IRQ), LoraIRQ, RISING);
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
}

////////////////////////////////
// WiFi
////////////////////////////////
#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient wifiClient; 

void onWifiEvent (system_event_id_t event, system_event_info_t info) {
#ifdef DEBUG
    Serial.printf ("[WiFi-event] event: %d - ", event);
    switch (event) {
    case SYSTEM_EVENT_WIFI_READY:    
      Serial.printf ("WiFi ready\n"); 
      break;
    case SYSTEM_EVENT_STA_START:     
      Serial.printf ("WiFi start\n"); 
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.printf ("Connected to %s. Asking for IP address\n", info.connected.ssid);
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.printf ("Station Stop\n");
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.printf ("Lost IP\n");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.printf ("Got IP: %s\n", IPAddress (info.got_ip.ip_info.ip.addr).toString ().c_str ());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: 
      Serial.printf ("Disconnected from SSID: %s\n", info.disconnected.ssid);
      break;
		default:
      Serial.printf ("Unknown event\n");
      break;
    }
#endif
}


bool WiFiConnect() {
  #ifdef DEBUG
  Serial.println("--> WiFi Connect");
  #endif
  bool retval = false;
  if(!WiFi.isConnected()) WiFi.begin(AP_NAME, AP_PASSRHRASE);
  for (int t=0; t<1000; t++) {
    if (WiFi.isConnected()) {
      retval = true;
      break;
    }
    delay(10);
  }
  #ifdef DEBUG
  Serial.print("--> WiFi Connect End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}

////////////////////////////////
// JSon
////////////////////////////////
#include <ArduinoJson.h>

////////////////////////////////
// MQTT
////////////////////////////////
#include <PubSubClient.h>

const char* mqtt_server_ip = MQTT_SERVER;
const int   mqtt_server_port = MQTT_PORT;
const char* mqtt_server_username = MQTT_LOGIN ;
const char* mqtt_server_password = MQTT_PASSWD ;

PubSubClient mqttClient(wifiClient);

String message;
String device = "N/A";

// bool sendBrokerData();
bool sendMQTT();
bool managePacket(String message);

void callback(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
}

bool reconnect();

////////////////////////////////
// Interrupts
////////////////////////////////
#include <esp_sleep.h>
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define BUTTON_PIN 26

hw_timer_t *timer = NULL;

hw_timer_t *timer2 = NULL;

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {BUTTON_PIN, 0, false};

void IRAM_ATTR isr(void* arg) {
    Button* s = static_cast<Button*>(arg);
    s->numberKeyPresses += 1;
    s->pressed = true;
    state = SEND;
}


#define TIME_TO_SEND_BROKER_DATA 60

void IRAM_ATTR onBrokerTimer() {
  portENTER_CRITICAL_ISR(&mux);
  //numiter++;
  prevState=state;
  state = BROKER;
  portEXIT_CRITICAL_ISR(&mux);
}

#define TIME_TO_COLLECT_DATA 9 * 5

void IRAM_ATTR onDataTimer() {
  portENTER_CRITICAL_ISR(&mux);
  prevState = state;
  //numCollect++;
  state = DATA;
  portEXIT_CRITICAL_ISR(&mux);
}

void print_wakeup_reason(){

  #ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause(); 

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0      : Serial.println("Wakeup caused by external signal using RTC_IO");  break;
    case ESP_SLEEP_WAKEUP_EXT1      : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER     : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD  : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP       : Serial.println("Wakeup caused by ULP program"); break;
    case ESP_SLEEP_WAKEUP_UNDEFINED : Serial.println("Wakeup cause is undefined"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
    
  }
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  print_wakeup_reason();

  // WiFi init
  WiFi.begin(AP_NAME, AP_PASSRHRASE);
  WiFi.onEvent (onWifiEvent);

  // Init MQTT Client
  mqttClient.setServer(mqtt_server_ip,mqtt_server_port);
  
  initLoRa();
  LoRa_rxMode();

  #ifdef DEBUG
  Serial.println("LoRa started OK in recieve mode");
  #endif
  
  #ifdef DEBUG
  Serial.println("--------- SETUP ENDED ---------");
  #endif

}

////////////////////////////////
// LOOP
////////////////////////////////

void loop() {

  sketchTime = millis();

  switch (state) {
  case INIT:
    // delay(1000);
    state=WAIT;
    break;
  
  case ERROR:
    // if(WiFi.isConnected()) WiFi.disconnect();
    break;

  case WAIT:

    break;

  case RECEIVING:
    state = MANAGE;
    message.clear();
    while (LoRa.available()) {
      message += (char)LoRa.read();
      if(message.length() > 200) {
        #ifdef DEBUG
        Serial.println("Message too long !");
        #endif
        LoRa.flush();
        state=ERROR;
        break;
      }
    }
    #ifdef DEBUG
    Serial.println(message);
    #endif
    break;
  
  case MANAGE:
    state=SEND;
    if (!managePacket(message)) { 
      state = ERROR;
      device = "MSG Err";
    }
  break;

  case SEND:
    state = WAIT;
    if (!sendMQTT()) { 
      state = ERROR;
      device = "SEND Err";
    }
    // WiFi.disconnect(true);
    break;

  case BROKER:
    state=WAIT;
    // WiFi.disconnect(true);
    break;

  case DATA:
    state = prevState;
    break;

  case SLEEP:
    Serial.flush();
    LoRa.flush();
    LoRa.end();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32,1);
    esp_deep_sleep_start();
    state=WAIT;
    break;

  default:
    #ifdef DEBUG
      Serial.println("State unknown !");
    #endif
    break;
  }

} 

////////////////////////////////
// Manage LoRa / JSon Packet
////////////////////////////////

bool managePacket(String message) {

  #ifdef DEBUG
  Serial.println("--> Manage Packet");
  Serial.printf("RSSI: %d SNR: %.2f\n",LoRa.packetRssi(), LoRa.packetSnr());
  #endif
  bool retval=true;

  StaticJsonDocument<200> doc;
  DeserializationError Derror = deserializeJson(doc, message);
  #ifdef DEBUG
  Serial.print("Deserialization : "); Serial.println(Derror.c_str());
  #endif

  if (Derror) {
    #ifdef DEBUG
    Serial.println("Deserialization FAILED !");
    #endif
    device = "unknown";
    retval=false;
  } else {
    String dev = doc["device"];
    device = dev;  
  }

  #ifdef DEBUG  
  Serial.print("device = "); Serial.print(device);
  size_t docsize = doc.size();
  Serial.printf(" with %d elements\n",docsize);
  Serial.print("--> Manage Packet End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif

  return retval;
}

////////////////////////////////
// Send data
////////////////////////////////

#define WAIT 150

bool sendMQTT() {
  #ifdef DEBUG
  Serial.println("--> Publish MQTT");
  #endif
  bool retval = true;
  StaticJsonDocument<200> doc;
  char topic[64];
  //char topicLocal[64];
  char payload[64];

  if (!mqttClient.connected()) reconnect();
  //if (!mqttLocalClient.connected()) reconnect();

  DeserializationError Derror = deserializeJson(doc, message);
  if (Derror) {
    #ifdef DEBUG
    Serial.println("Deserialization FAILED ");
    #endif
    return false;
  } else {
    Serial.println("Deserialization OK ");
  }

  String dev = doc["device"];
  device = dev;

  JsonObject obj = doc.as<JsonObject>();

  for (JsonPair p : obj) {
    if (strcmp(p.key().c_str(),"device") == 0) continue; 

    //sprintf(topic,"%s/f/%s.%s",mqtt_server_username,device.c_str(),p.key().c_str());
    sprintf(topic,"home/%s/%s",device.c_str(),p.key().c_str());

    #ifdef DEBUG
    //Serial.printf("topic External = %s\n",topic);
    Serial.printf("topic = %s\n",topic);
    #endif

    if (p.value().is<char*>()) {
      const char* s = p.value();
      sprintf(payload,"%s",s);
    }

    if(p.value().is<float>()) {
      float f = p.value();
      sprintf(payload,"%.4f",f);
    }

    #ifdef DEBUG
    Serial.printf("payload External = %s\n",payload );
    #endif

    if (!mqttClient.publish(topic,payload)) {
      Serial.println("external publish failed !");
      retval=false;
    }

    /*if (!mqttLocalClient.publish(topicLocal,payload)) {
      Serial.println("local publish failed !");
      retval=false;
    }*/
  }
  delay(WAIT);


  sprintf(topic,"home/%s/snr",device.c_str());
  sprintf(payload,"%.2f",LoRa.packetSnr());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);

  sprintf(topic,"home/%s/rssi", device.c_str());
  sprintf(payload,"%d",LoRa.packetRssi());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);

  sprintf(topic,"home/%s/frequencyerror", device.c_str());
  sprintf(payload,"%li",LoRa.packetFrequencyError());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);


  #ifdef DEBUG
  Serial.print("--> Publish MQTT End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}

////////////////////////////////
// MQTT specific
////////////////////////////////

bool reconnect() {
  bool retval = true;

  #ifdef DEBUG
  Serial.println("--> reconnect MQTT");
  #endif

  WiFiConnect();

  Serial.print("Attempting MQTT connection...");

  String clientId = "ESP32Logger-";
  clientId += String(random(0xffff), HEX);

  for (int i=1 ; i < 11 ; i++) {
    if (mqttClient.connect(clientId.c_str(),MQTT_LOGIN,MQTT_PASSWD )) {
      #ifdef DEBUG
        Serial.println("external connected");
      #endif
      retval=true;
      break;
    } else {
      #ifdef DEBUG
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.print(" try="); Serial.print(i);
        Serial.println(" try again in 1 seconds");
      #endif
      retval = false;
      delay(1000);
    }
    return retval;
  }

  #ifdef DEBUG
  Serial.print("--> reconnect MQTT End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif

  return true;
}

////////////////////////////////
// Display Statistics
////////////////////////////////
// 
// void displayStats() {
//   //display.ssd1306_command(SSD1306_DISPLAYON);
//   // #ifdef DEBUG
//   // Serial.printf(".");
//   // #endif
//   display.clearDisplay();
//   //display.stopscroll();
//   display.invertDisplay(invert_display);
//   display.setRotation(2);
//   display.setCursor(0,0);
//   char strstate[16]; stateToStr(strstate);
//   display.printf("Broker - %s\n",strstate);
// 
//   display.printf("Dev  %s\n",device.c_str());
// 
//   if (timeClient.getSeconds() % 2) display.fillCircle(display.width()-4, display.height()-4,2,0x1);  // 
//   time_t nowTime = timeClient.getEpochTime();
//   tm *n = localtime(&nowTime);
//   display.printf("Time %02d:%02d:%02d %02d/%02d\n",n->tm_hour,n->tm_min,n->tm_sec,n->tm_mday,n->tm_mon+1);
// 
//   tm *myTimeLocal = localtime(&lastReceived);
//   display.printf("Last %02d:%02d:%02d\n", myTimeLocal->tm_hour, myTimeLocal->tm_min, myTimeLocal->tm_sec );
//   display.printf("Rx   %3db\n",message.length() );
//   display.printf("RSNR %d %.2f\n",LoRa.packetRssi(),LoRa.packetSnr() );
//   //display.printf("FqErr: %li\n",LoRa.packetFrequencyError());
// 
//   if (LoRa.available()) display.drawBitmap(display.width() - LORA_WIDTH, /*display.height()/2*/ 40 - LORA_HEIGHT/2, LoRaLogo, LORA_WIDTH, LORA_HEIGHT, 0x1);
//   // if (!mqttClient.connected()) { display.printf("MQTT : No Cnx\n"); } else {display.printf("MQTT : connected\n"); }
//   // switch (mqttClient.state()) {
//   //   case -4 /*MQTT_CONNECTION_TIMEOUT*/    : display.printf("MQTT cnx timeout!\n"); break;
//   //   case -3 /*MQTT_CONNECTION_LOST*/       : display.printf("MQTT cnx lost!\n"); break;
//   //   case -2 /*MQTT_CONNECT_FAILED*/        : display.printf("MQTT cnx failed!\n"); break;
//   //   case -1 /*MQTT_DISCONNECTED*/          : display.printf("MQTT disconnected\n"); break;
//   //   case 0 /*MQTT_CONNECTED*/              : display.printf("MQTT connected\n"); break;
//   //   case 1 /*MQTT_CONNECT_BAD_PROTOCOL*/   : display.printf("MQTT Bad Protocol!\n"); break;
//   //   case 2 /*MQTT_CONNECT_BAD_CLIENT_ID*/  : display.printf("MQTT BAD ID\n"); break;
//   //   case 3 /*MQTT_CONNECT_UNAVAILABLE*/    : display.printf("MQTT unavailable\n"); break;
//   //   case 4 /*MQTT_CONNECT_BAD_CREDENTIALS*/: display.printf("MQTT BAD CRED!\n"); break;
//   //   case 5 /*MQTT_CONNECT_UNAUTHORIZED*/   : display.printf("MQTT Unauthorized!\n"); break; 
//   //   default:                                 display.printf("MQTT Unknown\n"); break;
//   // }
// 
//   if (WiFi.isConnected()) { 
//     display.drawBitmap(display.width() - WIFI_WIDTH, 0, WiFiLogo,WIFI_WIDTH, WIFI_HEIGHT, 0x1); 
//     // display.printf("WiFi %s\r\n",WiFi.localIP().toString().c_str());
//   } else { 
//     display.drawBitmap(display.width() - NOWIFI_WIDTH, 0, noWiFiLogo,NOWIFI_WIDTH, NOWIFI_HEIGHT, 0x1); 
//     // display.printf("WiFi No IP\r\n");
//   }
// 
//   int16_t VCC     = adc0.getConversionP0GND();;
//   int16_t vGlobal = adc0.getConversionP2GND();
// 
//   float GlobalmA  = (VCC - vGlobal) * adc0.getMvPerCount();
//   display.printf("Curr %3.0fmA %3.0fmA\n", GlobalmA,averageCurrent);
//   // if (GlobalmA > maxGlobalCurrent) maxGlobalCurrent = (int16_t) GlobalmA;
//   // display.printf("Max   %5dmA\n", maxGlobalCurrent);
//   // int16_t vESP32  = adc0.getConversionP3GND();
//   // float ESP32mA  = (vGlobal - vESP32) * adc0.getMvPerCount();
//   // display.printf("uC    %5.0fmA\n", ESP32mA);
//   display.printf("VCC %5.0fmV\n", VCC * adc0.getMvPerCount() );
//   display.display();
//   delay(20);
//   // #ifdef DEBUG
//   // Serial.printf("\r");
//   // #endif
// }