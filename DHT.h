/*!
 *  @file DHT.h
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  This library is inspired from Adafruit DHT Sensor Library library for arduinos.
 *
 *  Written by ArduinoEasy.
 *
 *  Twitter: @ArduinoEasy
 *
 */

#include "cycfg.h"

/* Define types of sensors. */
#define DHT11 11 /**< DHT TYPE 11 */
#define DHT12 12 /**< DHY TYPE 12 */
#define DHT22 22 /**< DHT TYPE 22 */
#define DHT21 21 /**< DHT TYPE 21 */
#define AM2301 21 /**< AM2301 */

typedef enum {
	DHT_ERROR = 0,
	DHT_SUCCESS,
	INVALID_SENSOR,
	INVALID_PORT,
	INVALID_PIN,
	INVALID_PIN_CONFIGURATION
}DHT_status;


typedef struct {
    float temperature;
    float humidity;
    float heat_index;
    GPIO_PRT_Type* portNum;
    uint32_t pinNum;
    uint32_t MAX_cycles;	// 1 millisecond timeout for reading pulses from DHT sensor.
    uint8_t CtoF;
    uint8_t sensor_type;
    uint8_t updated;
}DHT_config;

DHT_status DHT_begin(DHT_config* DHT);

void read_Temperature_Humidity(DHT_config* DHT);

float convertCtoF(float);
float convertFtoC(float);

void readHeatIndex(DHT_config* DHT);
void computeHeatIndex(DHT_config* DHT);

uint8_t DHT_read (DHT_config* DHT);
