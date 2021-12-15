#include <ArduinoJson.h>
#include <WiFi.h>
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialMon Serial
#define SerialAT Serial1  //SERIAL 2 ESP32
#define TINY_GSM_DEBUG SerialMon


#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
//const char apn[] = "movistar.pe";
//const char gprsUser[] = "movistar@datos";
//const char gprsPass[] = "movistar";

const char apn[] = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "";
const char wifiPass[] = "";

// MQTT details
const char* broker = "ec2-34-204-68-112.compute-1.amazonaws.com";  //aqui habiamos creado broker mosquitto con aitenticacion
String username_mqtt = "";
String password_mqtt = "";
 int port_mqtt = 1883;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;


#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <TinyGsmClientSIM70xx.h>

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient mqtt(client);


#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define UART_BAUD   9600
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define LED_PIN 12

uint32_t lastReconnectAttempt = 0;

//VARIABLES DE SISTEMA
  String modemInfo;
  String modemIMEI;
  String modemOP;
  String StrACK;
  String TopicOut;
  String TopicIn;
  int cont_desc;
////////////////////


  
void publicar_JSON(String ACK=""){
  StrACK= ACK;
StaticJsonDocument<200> doc;   
    doc["SENSOR-ID"] = modemIMEI;
    doc["LUM-%"] = random(0,100);
    doc["SIR"] = random(0,1);
    doc["ESTR"] = random(0,1);
    doc["T-LED"] = random(0,1);
    doc["ACK"] = StrACK;
    
    String output ;
    serializeJson(doc, output);
    Serial.println(output);
    mqtt.publish(TopicOut.c_str(), output.c_str());
}


void callback(char* topic, byte* payload, unsigned int length) {
 Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String str_topic = String(topic); //de char arra a string contienen nombre de topico
  String str_payload; //contiene payload e string
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    str_payload += (char)payload[i];
  }
  Serial.println();

StaticJsonDocument<96> doc;
DeserializationError error = deserializeJson(doc, str_payload);
if (error) {
  Serial.print(F("deserializeJson() failed: "));
  Serial.println(error.f_str());
  return;
}

String str_command = doc["COMMAND"]; 
Serial.println(str_command);

int index_LUM = str_command.indexOf("AT+LUM");
if(index_LUM==1) //AT+LUM-%:
{
    publicar_JSON("OK:LUM-%");  
    return;
}

if(str_command=="AT")
{
    publicar_JSON("OK");  
    return;
}else if(str_command=="AT+RST")
{
    publicar_JSON("OK:RST");
    ESP.restart();  
    return;
}
else if(str_command=="AT+CAL:PH:10")
{
   //do..
    publicar_JSON("OK:PH:10");  
    return;
}else if(str_command=="AT+CAL:PH:7")
{
    publicar_JSON("OK:PH:7");  
    return;
}else if(str_command=="AT+CAL:PH:4")
{
   //do..
    publicar_JSON("OK:PH:4");  
    return;
}else if(str_command=="AT+CAL:PH:ERASE")
{
    publicar_JSON("OK:PH:ERASE");  
    return;
}else{
    publicar_JSON("ERROR");  
    return;
}
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = modemIMEI;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(),username_mqtt.c_str() , password_mqtt.c_str() )) {
      //if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // ... and resubscribe     
      mqtt.subscribe(TopicIn.c_str());
      cont_desc=0;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      
      // MQTT Broker BACKUP
      cont_desc ++;
      if(cont_desc==3){
              Serial.println(">>Reconectando a broker.mqtt-dashboard");
              mqtt.setServer("broker.mqtt-dashboard.com", port_mqtt);
              username_mqtt="";
              password_mqtt="";
      }
      if(cont_desc==10){ //No conexion, Reiniciar
        Serial.println("No RED");       
        Serial.println(">>Reiniciar");
        ESP.restart();
      }
      delay(5000);
    }
  }
}

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  Serial1.begin(9600,SERIAL_8N1, 18, 5 ); // A GPS NEO 6M
  pinMode(LED_PIN, OUTPUT);
   SerialMon.println("");
  SerialMon.println("INICIANDO...");
  
  WiFi.mode( WIFI_MODE_NULL );
  btStop();
  WiFi.disconnect(true);  // Disconnect from the network
  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  setCpuFrequencyMhz(10);
  SerialMon.print("CPU Freq: ");
  SerialMon.println(getCpuFrequencyMhz());
  
  pinMode(PWR_PIN, OUTPUT);
  for(int x=0;x<=10;x++)
  {
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));
   delay(100);
  }
  digitalWrite(LED_PIN,HIGH);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);  
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  SerialMon.println(">>Modem Encendido");
   SerialMon.println(">>Inicializando...");
  
  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  modem.restart();
  for(int x=0;x<=10;x++)
  {
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));
   delay(100);
  }
  digitalWrite(LED_PIN,HIGH);
    // 2 Automatic
    // 13 GSM only
    // 38 LTE only
    // 51 GSM and LTE only
    Serial.println( modem.setNetworkMode(51));
     SerialMon.println(" ");
     Serial.print("Network Mode: ");
     Serial.println(modem.getNetworkModes());
     Serial.println(modem.getNetworkMode());
     // 1 CATM
     // 2 NB-IOT
     // 3 CAT-M & NBIOT
     //Serial.println( modem.setPreferredMode);
     Serial.print("Prefered  Mode: ");
     Serial.println(modem.getPreferredModes());
     Serial.println(modem.getPreferredMode());
     SerialMon.println(" ");
  // modem.init();
  SerialMon.println(">>Modem Inicializado");
  modemInfo = modem.getModemInfo();
  SerialMon.print(">>Modem Info: ");
  SerialMon.println(modemInfo);
  modemIMEI = modem.getIMEI();
  SerialMon.print("Modem IMEI: ");
  SerialMon.println(modemIMEI);
  modemOP = modem.getOperator();
  SerialMon.print("Modem OPERATOR: ");
  SerialMon.println(modemOP);
  TopicOut = "data/smartlux/" + modemIMEI + "/data";
  TopicIn = "data/smartlux/" + modemIMEI + "/req";
  Serial.print("Topic out: ");Serial.println(TopicOut);
  Serial.print("Topic in: ");Serial.println(TopicIn);
//  String modemBAT = modem.getBattVoltageImpl();
//  SerialMon.print("Modem Voltage: ");
//  SerialMon.println(modemBAT);
  
  
#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
#endif

  SerialMon.print(">>Esperando Conexion a Red...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    digitalWrite(LED_PIN,LOW);
    delay(10000);
    ESP.restart();
    return;
  }
  SerialMon.println(">>CONECTADO! ");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
    SerialMon.print(F(">>Conectando A:  "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(">>No Conexion..");
      digitalWrite(LED_PIN,LOW);
      delay(10000);
      ESP.restart();
      return;
    }
    SerialMon.println(" Conectado!");

  if (modem.isGprsConnected()) {
    SerialMon.println(">>CONECTADo A RED MOVIL");
  }
#endif
  // MQTT Broker setup
  mqtt.setServer(broker, port_mqtt);
  mqtt.setCallback(callback);
    for(int x=0;x<=5;x++)
  {
    digitalWrite(LED_PIN,!digitalRead(LED_PIN));
    delay(300);
  }
   digitalWrite(LED_PIN,HIGH);
   
}

void loop() {

  if (!client.connected()) {
    reconnect();
    publicar_JSON("BOOT");  
  }
  mqtt.loop();
  unsigned long now = millis();
  //if (now - lastMsg > 600000) {//60segundos
  if (now - lastMsg > 60000) {//60segundos
  lastMsg = now;
  Serial.println("PUBLICAR: ");
  digitalWrite(LED_PIN,LOW);
  publicar_JSON();  
  digitalWrite(LED_PIN,HIGH);
  }
  
}
