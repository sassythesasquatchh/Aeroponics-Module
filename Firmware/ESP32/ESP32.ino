#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#define NAME "capstone"
#define publish_topic "capstone/pub"
#define subscribe_topic "capstone/sub"

const char UPDATE_SETPOINTS =0x01;
const char SEND_READINGS =0x02;

const char* ssid = "nyuadguest";

const uint32_t MILLIS_PER_SECOND = 1000;
const uint16_t SECONDS_PER_MINUTE = 60;

char tx[5] ={0x00};


WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

const uint16_t connectTimeoutMs = 10000;
uint64_t lastPublish = millis();
const uint32_t messageTimer = 60*SECONDS_PER_MINUTE*MILLIS_PER_SECOND; // Message interval in ms 

const char* awsEndpoint = "a25w67mn0bb0hm-ats.iot.us-east-1.amazonaws.com";

static const char certificatePemCrt[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
INSERT YOUR CERTIFICATE HERE
-----END CERTIFICATE-----
)EOF";

static const char privatePemKey[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
INSERT YOUR PRIVATE KEY HERE
-----END RSA PRIVATE KEY-----
)EOF";

static const char caPemCrt[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";


void msgReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  DynamicJsonDocument doc(128);
  deserializeJson(doc, payload);
  String out_msg;

  uint8_t command = doc["command"];

  switch(command) {
    case UPDATE_SETPOINTS:{
      tx[0]=UPDATE_SETPOINTS;
      Serial.println("Updating setpoints");
      Serial2.write(tx,1);
       
      Serial2.print((String)(float) doc["phSetpoint"]+'\n');
      Serial2.print((String)(int)doc["conductivitySetpoint"]+'\n');
      Serial2.print((String)(int)doc["period"]+'\n');
      Serial2.print((String)(int)doc["dutyCycle"]+'\n');
      break;
    }
      
    case SEND_READINGS:{
      Serial.println("Sending Readings");
      if(send_readings()){
        lastPublish = millis();
      }
      break;
    }

    default:{
      Serial.println("Unrecognised command code");
    }
  }
}

bool send_readings(){

  while(Serial2.available()>0){
    Serial2.read();
  }
  
  String out_msg;
  tx[0]=SEND_READINGS;
  Serial2.write(tx,1);
  delay(500);

  if(Serial2.available()>0)
  {
    out_msg = Serial2.readString();
  }else{
    Serial.println("Error reading from arduino");
    return false;
  }

  if (client.publish(publish_topic, out_msg.c_str())){
    Serial.print("Published: "); Serial.println(out_msg);
    return true;
  } else{
    Serial.println("Error publishing message");
    return false;
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: "); Serial.print(asctime(&timeinfo));
}


void connectAWS()
{
  delay(1000);
  initWiFi();
 
  setCurrentTime();
 
  net.setCACert(caPemCrt);
  net.setCertificate(certificatePemCrt);
  net.setPrivateKey(privatePemKey);
 
  client.setServer(awsEndpoint, 8883);
  client.setCallback(msgReceived);
 
 
  Serial.println("Connecting to AWS IOT");

  int i = 0;
 
  while (!client.connect(NAME)&& (i<60))
  {
    Serial.print(".");
    delay(1000);
    i++;
  }
 
  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(subscribe_topic);
 
  Serial.println("AWS IoT Connected!");
}

 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Serial Connected");
  Serial2.begin(9600);
  connectAWS();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (!client.connected()){
    connectAWS();
  }else{
    client.loop();
  }

  if (millis() - lastPublish > messageTimer) {
    if (send_readings()){
      lastPublish = millis();
    }
  }
}
