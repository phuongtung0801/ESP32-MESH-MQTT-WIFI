//************************************************************
// this is a simple example that uses the painlessMesh library to
// connect to a another network and relay messages from a MQTT broker to the nodes of the mesh network.
// To send a message to a mesh node, you can publish it to "painlessMesh/to/12345678" where 12345678 equals the nodeId.
// To broadcast a message to all nodes in the mesh you can publish it to "painlessMesh/to/broadcast".
// When you publish "getNodes" to "painlessMesh/to/gateway" you receive the mesh topology as JSON
// Every message from the mesh which is send to the gateway node will be published to "painlessMesh/from/12345678" where 12345678
// is the nodeId from which the packet was send.
//************************************************************

#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define MESH_PREFIX "tung"
#define MESH_PASSWORD "phuongtung0801"
#define MESH_PORT 5555

#define STATION_SSID "TUNG"
#define STATION_PASSWORD "123456789"
//WiFiEventHandler wifiConnectHandler;
//WiFiEventHandler wifiDisconnectHandler;

#define HOSTNAME "MQTT_Bridge"

String path = "/";
unsigned long t1 = 0;
const unsigned long eventInterval = 5000;
unsigned long previousTime = 0;

const char *mqtt_server = "m14.cloudmqtt.com";
const char *mqtt_username = "bxpalvco";
const char *mqtt_password = "UUILhS73phGV";
const char *clientID = "TUNG";
const char *endTopic = "phuongtung0801/LWT";

// Prototypes
void receivedCallback(const uint32_t &from, const String &msg);
void mqttCallback(char *topic, byte *payload, unsigned int length);

/*void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("Connecting to: ");
  Serial.println(WiFi.localIP());
} */

/*void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi sucessfully.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi, trying to connect...");
  WiFi.disconnect();
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
}*/

IPAddress getlocalIP();

IPAddress myIP(0, 0, 0, 0);
//IPAddress mqttBroker("m2m.eclipse.org");

painlessMesh mesh;
WiFiClient wifiClient;
//tạo một instance để kết nối. Sử dụng cú pháp PubSubClient (server, port, [callback], client, [stream])

//PubSubClient mqttClient(wifiClient);
PubSubClient mqttClient(mqtt_server, 12321, mqttCallback, wifiClient);

void setup()
{
  Serial.begin(9600);

  /*wifi checking
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);*/
  /*Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());*/

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // set before init() so that you can see startup messages

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);

  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  WiFi.begin(STATION_SSID, STATION_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  //kết nối mqtt broker
  mqttClient.setServer(mqtt_server, 12321);
  mqttClient.setCallback(mqttCallback);
  //mqttClient.connect(clientID, "", "", "theEndTopic", 1, true, "offline");
  if (mqttClient.connect(
          clientID, mqtt_username, mqtt_password, endTopic, 1, true, "Sensor disconnected from mqtt"))
  {
    Serial.println("Connected to MQTT Broker!");
    mqttClient.publish(endTopic, "Sensor connected!", true);
  }
  else
  {
    Serial.println("Connection to MQTT Broker failed...");
  }

  /*while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(
      clientID, mqtt_username, mqtt_password, endTopic, 1, true, "Sensor disconnected from mqtt" ))
    {
      Serial.println("connected");  
    } 
    else 
    { 
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      delay(2000); 
    }
  }*/

  mqttClient.subscribe("tungtran");
  mqttClient.subscribe(endTopic);

}

void loop()
{
  mesh.update();
  mqttClient.loop();
  /*firebase test
  if (millis() - t1 > 1000)
  { Firebase.setInt(firebaseData, path + "/data", random(0,100));
    t1 = millis();
  }*/

  /* Updates frequently */
  unsigned long currentTime = millis();

  //wifi checking
  if (currentTime - previousTime >= eventInterval)
  {
    switch (WiFi.status())
    {
    case WL_CONNECTED:
      //Serial.println("Wifi Connection successfully established");
      if (!mqttClient.connected())
      {
        Serial.println("Reconnecting to MQTT...");
        if (mqttClient.connect(
                clientID, mqtt_username, mqtt_password, endTopic, 1, true, "Sensor disconnected from mqtt"))
        {
          Serial.println("Reconnect to MQTT success!");
          mqttClient.setServer(mqtt_server, 12321);
          mqttClient.setCallback(mqttCallback);
          mqttClient.subscribe("tungtran");
          mqttClient.publish(endTopic, "Sensor connected!", true);
        }
        else
        {
          Serial.println("MQTT connection is lost!");
        }
      }
      break;

    default:
      Serial.println("Failed to connect Wifi");
      break;
    }
    //Update timing for next round
    previousTime = currentTime;
  }

  if (myIP != getlocalIP())
  {
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());

    /*if (mqttClient.connect("painlessMeshClient"))
    {
      mqttClient.publish("painlessMesh/from/gateway", "Ready!");
      mqttClient.subscribe("tungtran");
    }*/
    //mqttClient.subscribe("tungtran");
  }
}

void receivedCallback(const uint32_t &from, const String &msg)
{
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  //String topic = "painlessMesh/from/2131961461";
  String topic = "painlessMesh/from/" + String(from);
  mqttClient.publish(topic.c_str(), msg.c_str());
  Serial.println("Data received");
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{

  String cmd = "";

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    cmd += (char)payload[i];
  }
  Serial.println(cmd);
  if (cmd == "on")
  {
    mesh.sendSingle(110231767, cmd);
  }
  if (cmd == "off")
  {
    mesh.sendSingle(110231767, cmd);
  }
}
//end mqttCallback

IPAddress getlocalIP()
{
  return IPAddress(mesh.getStationIP());
}
