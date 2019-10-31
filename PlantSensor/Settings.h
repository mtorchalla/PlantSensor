// Settings.h
#include "Secrets.h"

#ifndef _SETTINGS_h
#define _SETTINGS_h


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//For debug-messages / comment out for power-saving!
//#define debug

#define UPDATE_INTERVAL_MINUTES 5

//WIFI settings:
#define WIFI_SSID	SECRET_WIFI_SSID
#define WIFI_PASS	SECRET_WIFI_PASS

//MQTT connection settings:
#define MQTT_SERVER	SECRET_MQTT_SERVER
#define MQTT_USER	SECRET_MQTT_USER
#define MQTT_PASS	SECRET_MQTT_PASS

//Topics for MQTT:
#define MQTT_CLIENT_ID			"EspBalkon"
#define MQTT_TOPIC_AKKU			"/home/balkon/akku/"
#define MQTT_TOPIC_AKKU_LOW		"/home/balkon/akkulow/"
#define MQTT_TOPIC_LUX			"/home/balkon/lux/"
#define MQTT_TOPIC_TEMPERATURE	"/home/balkon/temp/"
#define MQTT_TOPIC_HUMIDITY		"/home/balkon/hum/"
#define MQTT_TOPIC_PRESSURE		"/home/balkon/pres/"
#define NrOfScales 2			//Number of used plants (2 per scale sensor!) 
#define MQTT_TOPIC_PLANT1		"/home/balkon/pflanze1/" //Scale sensor 1, Pins A+ A-
#define MQTT_TOPIC_PLANT2		"/home/balkon/pflanze2/" //Scale sensor 1, Pins B+ B-
#define MQTT_TOPIC_PLANT3		"/home/balkon/pflanze3/" //Scale sensor 2, Pins A+ A-
#define MQTT_TOPIC_PLANT4		"/home/balkon/pflanze4/" //Scale sensor 2, Pins B+ B-
#define MQTT_TOPIC_PLANT5		"/home/balkon/pflanze5/" //Scale sensor 3, Pins A+ A-
#define MQTT_TOPIC_PLANT6		"/home/balkon/pflanze6/" //Scale sensor 3, Pins B+ B-
#define MQTT_TOPIC_PLANT7		"/home/balkon/pflanze7/" //Scale sensor 4, Pins A+ A-
#define MQTT_TOPIC_PLANT8		"/home/balkon/pflanze8/" //Scale sensor 4, Pins B+ B-

#define UPDATE_INTERVAL 1000000*60*UPDATE_INTERVAL_MINUTES

#endif