//#include <SparkFunLSM9DS1.h>
//#include <LSM9DS1_Types.h>
//#include <LSM9DS1_Registers.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <BME280I2C.h>
#include <Wire.h>
#include "Max44009.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <HX711.h>
#include "Settings.h"
#include <SPI.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
//#include "../../../../AppData/Local/arduino15/packages/esp8266/hardware/esp8266/2.5.0/cores/esp8266/Arduino.h"
#include <Arduino.h>

//#include <Update.h>

// Define Function Prototypes that use User Types below here or use a .h file
boolean checkAkku(float);
void start_ota();
void readAkku(int);
void readLux();
void readScale();
void readBME();
bool reconnect();
void debug_menu();
void calibrate_scales();
void startBME();
void loop();

uint16_t start_time = millis();
uint16_t volatile current_time = millis();

//*****Sensors:*****
//Lux:
Max44009 SensorLux(0x4A, 4, 5);
#define luxMid 5
char clux[16];

//BME:
BME280I2C SensorBme;
bool startup = true;
bool metric = true;
float temp(NAN), hum(NAN), pres(NAN);
char ctemp[16];
char chum[16];
char cpres[16];
uint8_t pressureunit(4);
#define a_len 6 //len of FilterArray
float temp_arr[a_len] = { 0 };
float hum_arr[a_len] = { 0 };
float pres_arr[a_len] = { 0 };
float temp_arr_med[a_len] = { 0 };
float hum_arr_med[a_len] = { 0 };
float pres_arr_med[a_len] = { 0 };
int a_i = 0;
int a_i_last = a_len;

// HX711:
struct Scale {
	HX711* main;
	const char dout;
	const char sck;
	const float offset;
	const float scale;
	const char gain;
	char cscale[16];
	char const * mqttUri;
};
#define ScaleSCK 13
HX711 SensorScale1;
HX711 SensorScale2;
HX711 SensorScale3;
HX711 SensorScale4;
HX711 SensorScale5;
HX711 SensorScale6;
HX711 SensorScale7;
HX711 SensorScale8;
Scale Scale1 = { &SensorScale1,  2, ScaleSCK, -391750 , 23200, 128, "",  MQTT_TOPIC_PLANT1 };
Scale Scale2 = { &SensorScale2,  2, ScaleSCK, -391750 , 23200, 32, "", MQTT_TOPIC_PLANT2 };
Scale Scale3 = { &SensorScale3, 12, ScaleSCK, -391750 , 23200, 128, "",  MQTT_TOPIC_PLANT3 };
Scale Scale4 = { &SensorScale4, 12, ScaleSCK, -391750 , 23200, 32, "", MQTT_TOPIC_PLANT4 };
Scale Scale5 = { &SensorScale5, 10, ScaleSCK, -391750 , 23200, 128, "",  MQTT_TOPIC_PLANT5 };
Scale Scale6 = { &SensorScale6, 10, ScaleSCK, -391750 , 23200, 32, "", MQTT_TOPIC_PLANT6 };
Scale Scale7 = { &SensorScale7, 14, ScaleSCK, -391750 , 23200, 128, "",  MQTT_TOPIC_PLANT7 };
Scale Scale8 = { &SensorScale8, 14, ScaleSCK, -391750 , 23200, 32, "", MQTT_TOPIC_PLANT8 };
#ifndef NrOfScales
#elif   NrOfScales == 1 
			Scale* Scales[NrOfScales] = { &Scale1 };
#elif   NrOfScales == 2 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2 };
#elif   NrOfScales == 3 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3 };
#elif   NrOfScales == 4 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3, &Scale4 };
#elif   NrOfScales == 5 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3, &Scale4, &Scale5 };
#elif   NrOfScales == 6 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3, &Scale4, &Scale5, &Scale6 };
#elif   NrOfScales == 7 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3, &Scale4, &Scale5, &Scale6, &Scale7 };
#elif   NrOfScales == 8 
			Scale* Scales[NrOfScales] = { &Scale1, &Scale2, &Scale3, &Scale4, &Scale5, &Scale6, &Scale7, &Scale8 };
#else   
			#error "Unexpected value of NrOfScales."
#endif

//Akku
char cakku[16];
float akku = 0;
#define SHUTDOWN_VOLTAGE 3*1000
#define R_GND 300000
#define R_BAT 1000000
#define R_AKKU (R_BAT + R_GND) / R_GND
#define O_AKKU 0.3

//WIFI:
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASS;
WiFiClient espClient;
PubSubClient client(espClient);
//PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);

//#define mqtt_topics_len 6
//char const* mqtt_topics[mqtt_topics_len] = { "/home/balkon/temp/", "/home/balkon/hum/", "/home/balkon/pres/", "/home/balkon/lux/", "/home/balkon/akku/", "/home/balkon/haengekasten_rechts/" };


void setup()
{
	/*WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	delay(1);

	WiFi.forceSleepWake();
	WiFi.mode(WIFI_STA);
	wifi_station_connect();
	*/
	WiFi.begin(ssid, password);
	WiFi.persistent(false);
	WiFi.mode(WIFI_OFF);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	client.setServer(MQTT_SERVER, 1883);

	#ifdef debug
		Serial.begin(115200);
	#endif

	//int volatile * const p_reg = (int *) 0x60000808;
	//*p_reg = 0b000010010 | GPOS;
	//*p_reg = 0b000010010 | GPOS;
	//digitalWrite(12, HIGH);
	//digitalWrite(13, HIGH);
	//delay(3000);

	readAkku(10);
	readLux();
	readScale();
	readBME();

	volatile bool connected = false;
	current_time = millis();
	start_time = millis();
	volatile uint8_t retries = 0;
	while (connected == false && retries <= 2)
	{
		connected = reconnect();
		delay(500);
#ifdef debug
		Serial.print("Number of retries: ");
		Serial.println(retries);
		Serial.println(connected);
#endif
		retries++;
	}

	if (connected) {
		//Publish all values:
		//TODO Add Timeout 
		client.publish(MQTT_TOPIC_AKKU, cakku);
		client.publish(MQTT_TOPIC_AKKU_LOW, "0");
		for (int i = 0; i < NrOfScales; i++) {
			client.publish(Scales[i]->mqttUri, Scales[i]->cscale);
#ifdef debug
			Serial.println(Scales[i]->cscale);
#endif
		}
		client.publish(MQTT_TOPIC_LUX, clux);
		client.publish(MQTT_TOPIC_TEMPERATURE, ctemp);
		client.publish(MQTT_TOPIC_HUMIDITY, chum);
		client.publish(MQTT_TOPIC_PRESSURE, cpres);
#ifdef debug
		Serial.print("Published. Akku:");
		Serial.println(cakku);
		Serial.println("Sleeping...");
#endif
		//client.loop();
	}

	if (digitalRead(0) == 0) debug_menu();

	//DeepSleep:
	if (checkAkku(akku)) {
		float akku_cum = 0;
		for (int i = 0; i < 5; i++)
		{
			readAkku(10);
			akku_cum += akku;
		}
		if (checkAkku(akku_cum / 5)) {
#ifdef debug
			Serial.print("Low Voltage detected: ");
			Serial.println((akku * R_AKKU + O_AKKU) / 1000);
			Serial.println("Sleeping forever...");
#endif
			client.publish(MQTT_TOPIC_AKKU_LOW, "1");
			digitalWrite(ScaleSCK, HIGH);
			delay(500);  // wait for safety
			ESP.deepSleep(0);
		}
	}

	//client.flush();
	digitalWrite(ScaleSCK, HIGH);	// set scales to sleep
	delay(500);						// wait for safety
	
	ESP.deepSleep(UPDATE_INTERVAL);
}


void loop()
{
	ESP.deepSleep(UPDATE_INTERVAL);
	/*publishBME();
	publishLux();
	publishScale();
	publishAkku(10);
	delay(600000);
	for (int i = 1; i < 1024; i++) {
		//analogWrite(A0, i);
		//Serial.print(i);
		//Serial.print("Battery: ");
		Serial.println(SensorScale1.read_average(10));
		delay(2000);
	}*/
}

void callback(char* topic, byte* payload, unsigned int length) {}

boolean checkAkku(float akku_measure)
{
	return ((akku_measure * R_AKKU + O_AKKU) < SHUTDOWN_VOLTAGE);
}

void inline readAkku(int iter) {
	akku = 0;
	for (int i = 1; i < iter; i++) {
		akku += analogRead(A0);
		delay(5);
	}
	akku = akku / iter;
	snprintf(cakku, sizeof(cakku), "%.1f", akku);
#ifdef debug
	Serial.print("Akku: ");
	Serial.println(cakku);
#endif
}

void inline readScale() {
	for (int i = 0; i < NrOfScales; i++) {
		Scales[i]->main->begin(Scales[i]->dout, Scales[i]->sck);
		Scales[i]->main->set_gain(Scales[i]->gain);
		float scale1 = (Scales[i]->main->read_average(3) - Scales[i]->offset) / Scales[i]->scale;
		snprintf(Scales[i]->cscale, sizeof(Scales[i]->cscale), "%.2f", scale1);
#ifdef debug
		Serial.print("Scale Nr ");
		Serial.print(i+1);
		Serial.print(": ");
		Serial.println(scale1);
#endif
	}
}

void inline readLux() {
	float lux_mid = 0;
	if (SensorLux.getError() == 0) {
		for (int i = 1; i < luxMid; i++) {
			lux_mid += SensorLux.getLux();
			delay(5);
		}
		lux_mid = lux_mid / luxMid;
		snprintf(clux, sizeof(clux), "%.1f", lux_mid);
#ifdef debug
		Serial.print("Updating Lux: ");
		Serial.println(lux_mid);
#endif
	}
}

void inline readBME() {
	startBME();
	SensorBme.read(pres, temp, hum, metric, pressureunit);
	//72.4 Humidity / Pressure Exception:
	char retries = 0;
	while (pres <= 0.65 && retries <= 5) {
		startBME();
		SensorBme.read(pres, temp, hum, metric, pressureunit);
		retries++;
	}
	snprintf(ctemp, sizeof(ctemp), "%.1f", temp);
	snprintf(chum, sizeof(chum), "%.1f", hum);
	snprintf(cpres, sizeof(cpres), "%.4f", pres);
#ifdef debug
	Serial.print("Updating bme (T, H, P): ");
	Serial.print(ctemp);
	Serial.print(", ");
	Serial.print(chum);
	Serial.print(", ");
	Serial.println(cpres);
#endif
}

void inline startBME()
{
	if (!SensorBme.begin()) {
		SensorBme.begin();
	}
}

void debug_menu()
{
	Serial.begin(115200);
	pinMode(0, OUTPUT);
	delay(10);
	digitalWrite(0, HIGH);
	start_ota();
	calibrate_scales();
}

void start_ota()
{
	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
}

void calibrate_scales() {
	Serial.println("");
	Serial.println("Starting calibration.");
	Serial.println("Available commands:");
	Serial.println("get <scaleNr>");
	Serial.println("set <scaleNr> <offset>");
	Serial.println("gain <scaleNr> <gain>");
	Serial.println("tar <scaleNr>");
	Serial.println("stop");
	Serial.println("");
	Serial.println("Type start or any other command within 10s to begin!");
	for (int i = 0; i < NrOfScales; i++) {
		Scales[i]->main->begin(Scales[i]->dout, Scales[i]->sck);
		Scales[i]->main->set_gain(128);
		Scales[i]->main->set_scale();
	}
	const int start_time = millis();
	bool started = false;
	while (start_time + 10*1000 < millis() || started) {
		ArduinoOTA.handle();
		if (Serial.available()) {
			String input = Serial.readString();
			//Serial.print("You typed: " );
			//Serial.println(input);
			int index = input.indexOf(" ");
			if (index <= 0) index = 3;
			String cmd = input.substring(0, index);
			String params = input.substring(index + 1);
			int scaleNr = params.substring(0, params.indexOf(" ")).toInt();
			float cmdValue = params.substring(params.indexOf(" ") + 1).toInt();
			/*Serial.print("cmd: "); Serial.println(cmd);
			Serial.print("params: "); Serial.println(params);
			Serial.print("scaleNr: "); Serial.println(scaleNr);
			Serial.print("cmdValue: "); Serial.println(cmdValue);*/
			if (cmd == "get") {
				started = true;
				Serial.print("Value from scale "); Serial.print(scaleNr); Serial.print(" : ");
				Serial.println(Scales[scaleNr]->main->get_units(30));
			}
			if (cmd == "set") {
				started = true;
				Serial.print("Setting scale "); Serial.print(scaleNr); Serial.print(" to: "); Serial.println(cmdValue);
				Scales[scaleNr]->main->set_scale(cmdValue);
			}
			if (cmd == "tar") {
				started = true;
				Serial.print("Taring scale: "); Serial.print(scaleNr);
				Scales[scaleNr]->main->set_scale();
				Scales[scaleNr]->main->tare();
				Serial.println(" - Finished.");
			}
			if (cmd == "gain") {
				started = true;
				Serial.print("Setting scale "); Serial.print(scaleNr); Serial.print(" gain to: ");
				Serial.println(cmdValue);
				Scales[scaleNr]->main->set_gain(int(cmdValue));
			}
			if (cmd == "stop") {
				started = true;
				Serial.print("Finished calibrating. Restarting...");
				ESP.restart();
			}
			if (cmd == "start") {
				started = true;
				Serial.print("Starting.");

			}
		}
	}
}

///attempt to connect to the wifi if connection is lost
bool reconnect() {
	if (WiFi.status() != WL_CONNECTED) {
#ifdef debug
		Serial.print("Connecting to ");
		Serial.print(ssid);
#endif
		start_time = millis();
		current_time = millis();
		while (WiFi.status() != WL_CONNECTED && start_time + 15*1000 > current_time) {
			delay(500);
			current_time = millis();
#ifdef debug
			Serial.print(".");
#endif
		}
#ifdef debug
		if (WiFi.status() == WL_CONNECTED) {
			Serial.println("");
			Serial.println("WiFi connected");
			Serial.print("IP address: ");
			Serial.println(WiFi.localIP());
		}
		else
		{
			Serial.println("Timed out while connecting to Wifi...");
		}
#endif
	}
	//make sure we are connected to WIFI before attemping to reconnect to MQTT
	if (WiFi.status() == WL_CONNECTED) {
		start_time = millis();
		current_time = millis();
		volatile uint8_t retries = 0;
		while (!client.connected() && (retries <= 3)) {
			retries++;
			Serial.print("Attempting MQTT connection...");
			String clientId = MQTT_CLIENT_ID;  // Set Client ID
			// Attempt to connect to MQTT Server
			if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
#ifdef debug
				Serial.println("connected");
#endif
			}
			else {
#ifdef debug
				Serial.print("failed, rc=");
				Serial.print(client.state());
				Serial.println(" trying again in 1 seconds");
#endif
				delay(1000);  // Wait 1 seconds before retrying
			}
		}
	}
	if ((WiFi.status() == WL_CONNECTED) && client.connected()) {
		return true;
	}
	else {
		return false;
	}
}
/*
//generate unique name from MAC addr
String macToStr(const uint8_t* mac) {
	String result;
	for (int i = 0; i < 6; ++i) {
		result += String(mac[i], 16);

		if (i < 5) {
			result += ':';
		}
	}
	return result;
}*/