/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/

#include <WiFi.h>
#include <WiFiMulti.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
  uint8_t temprature_sens_read();
}
#include "../include/config.h"
#include <AsyncMqttClient.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Update.h>

#include "DallasTemperature.h"
#include "OneWire.h"

#define ONE_WIRE_BUS 22

#define MQTT_HOST IPAddress(10, 1, 1, 201)
#define MQTT_PORT 1883

uint8_t temprature_sens_read();

IPAddress local_IP(10, 1, 1, 80);
IPAddress gateway(10, 1, 1, 1);
IPAddress subnet(255, 0, 0, 0);
IPAddress primaryDNS(10, 1, 1, 1);
IPAddress secondaryDNS(8, 8, 4, 4); //optional

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t mqttTempTimer;
TimerHandle_t wifiReconnectTimer;
AsyncWebServer server(80);

const char *PARAM_MESSAGE = "message";

const char *MQTT_TOPIC = "tele/roof";
const char *MQTT_TOPIC_STATE = "tele/roof/STATE";
const char *MQTT_TOPIC_LWT = "tele/roof/LWT";

void notFound(AsyncWebServerRequest *request) {
	request->send(404, "text/plain", "Not found");
}

void connectToWifi() {
	Serial.println("Connecting to Wi-Fi...");
	WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
	Serial.println("Connecting to MQTT...");
	mqttClient.connect();
}

void tempToMqtt() {
	Serial.println("Sending Temp...");
	Serial.print((temprature_sens_read() - 32) / 1.8);
	float tempInternal = (temprature_sens_read() - 32) / 1.8;
  sensors.requestTemperatures();
	String json = "{";
	json += "\"temp\":" + String(sensors.getTempCByIndex(0));
	json += ",\"rssi\":" + String(WiFi.RSSI());
	json += ",\"ssid\":\"" + WiFi.SSID() + "\"";
	json += ",\"tempInternal\":" + String(tempInternal);
	json += "}";
	char jsonStr[100];
	json.toCharArray(jsonStr, sizeof(jsonStr));
	uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC_STATE, 1, true, jsonStr);
	json = String();
}

void WiFiEvent(WiFiEvent_t event) {
	Serial.printf("[WiFi-event] event: %d\n", event);
	switch (event) {
	case SYSTEM_EVENT_STA_GOT_IP:
		Serial.println("WiFi connected");
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
		connectToMqtt();
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		Serial.println("WiFi lost connection");
		xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		xTimerStop(mqttTempTimer, 0); // 
		xTimerStart(wifiReconnectTimer, 0);
		break;
	}
}

void onMqttConnect(bool sessionPresent) {
	Serial.println("Connected to MQTT.2");
	Serial.print("Session present: ");
	Serial.println(sessionPresent);
	uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC, 2);
	Serial.print("Subscribing at QoS 2, packetId: ");
	Serial.println(packetIdSub);
	mqttClient.publish(MQTT_TOPIC_LWT, 0, true, "Online");
	Serial.println("Publishing at QoS 0");
	xTimerStart(mqttTempTimer, pdMS_TO_TICKS(10000));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
	Serial.println("Disconnected from MQTT.");
	xTimerStop(mqttTempTimer, 0);
	if (WiFi.isConnected()) {
		xTimerStart(mqttReconnectTimer, 0);
	}
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
	Serial.println("Subscribe acknowledged.");
	Serial.print("  packetId: ");
	Serial.println(packetId);
	Serial.print("  qos: ");
	Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
	Serial.println("Unsubscribe acknowledged.");
	Serial.print("  packetId: ");
	Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
	Serial.println("Publish received.");
	Serial.print("  topic: ");
	Serial.println(topic);
	Serial.print("  qos: ");
	Serial.println(properties.qos);
	Serial.print("  dup: ");
	Serial.println(properties.dup);
	Serial.print("  retain: ");
	Serial.println(properties.retain);
	Serial.print("  len: ");
	Serial.println(len);
	Serial.print("  index: ");
	Serial.println(index);
	Serial.print("  total: ");
	Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
	Serial.println("Publish acknowledged.");
	Serial.print("  packetId: ");
	Serial.println(packetId);
}

void setup() {
	Serial.begin(115200);
	Serial.println();
	Serial.println();

	mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
	mqttTempTimer = xTimerCreate("mqttTempTimer", pdMS_TO_TICKS(60000), pdTRUE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(tempToMqtt));
	wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

	WiFi.onEvent(WiFiEvent);

	sensors.begin();

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.onSubscribe(onMqttSubscribe);
	mqttClient.onUnsubscribe(onMqttUnsubscribe);
	mqttClient.onMessage(onMqttMessage);
	mqttClient.onPublish(onMqttPublish);
	mqttClient.setServer(MQTT_HOST, MQTT_PORT);

	connectToWifi();

	server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/index.html", String(), false);
	});

	// Send a GET request to <IP>/get?message=<message>
	server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
		String message;
		if (request->hasParam(PARAM_MESSAGE)) {
			message = request->getParam(PARAM_MESSAGE)->value();
		} else {
			message = "No message sent";
		}
		request->send(200, "text/plain", "Hello, GET: " + message);
	});

	// Send a POST request to <IP>/post with a form field message set to <message>
	server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {
		String message;
		if (request->hasParam(PARAM_MESSAGE, true)) {
			message = request->getParam(PARAM_MESSAGE, true)->value();
		} else {
			message = "No message sent";
		}
		request->send(200, "text/plain", "Hello, POST: " + message);
	});

	server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request) {
		sensors.requestTemperatures();
		String json = "{";
		json += "\"temp\":" + String(sensors.getTempCByIndex(0));
		json += "}";
		request->send(200, "application/json", json);
		json = String();
	});

	server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
		String json = "[";
		int n = WiFi.scanComplete();
		if (n == -2) {
			WiFi.scanNetworks(true);
		} else if (n) {
			for (int i = 0; i < n; ++i) {
				if (i)
					json += ",";
				json += "{";
				json += "\"rssi\":" + String(WiFi.RSSI(i));
				json += ",\"ssid\":\"" + WiFi.SSID(i) + "\"";
				json += ",\"bssid\":\"" + WiFi.BSSIDstr(i) + "\"";
				json += ",\"channel\":" + String(WiFi.channel(i));
				json += ",\"secure\":" + String(WiFi.encryptionType(i));
				json += "}";
			}
			WiFi.scanDelete();
			if (WiFi.scanComplete() == -2) {
				WiFi.scanNetworks(true);
			}
		}
		json += "]";
		request->send(200, "application/json", json);
		json = String();
	});

	// Simple Firmware Update Form
	server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
	});

	server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError()) ? "OK" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
    ESP.restart(); }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index)
    {
      Serial.printf("Update Start: %s\n", filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
      {
        Update.printError(Serial);
      }
    }
    if (!Update.hasError())
    {
      if (Update.write(data, len) != len)
      {
        Update.printError(Serial);
      }
    }
    if (final){
      if (Update.end(true)) {
        Serial.printf("Update Success: %uB\n", index + len);
      } else {
        Update.printError(Serial);
      }
    } });

	if (!SPIFFS.begin(true)) {
		Serial.println("An Error has occurred while mounting SPIFFS");
		return;
	}

	server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

	server.onNotFound(notFound);

	server.begin();
}

void loop() {
}