#ifndef NMEA0183_H
#define NMEA0183_H

#include "stm32f1xx_hal.h"

typedef struct {
    long utc_time;
    double latitude;
    char latitude_direction;
    double longitude;
    char longitude_direction;
    char state;
    uint8_t satellite_available;
    uint8_t bds_satellite_total;
    uint8_t gal_satellite_total;
    uint8_t glo_satellite_total;
    uint8_t gps_satellite_total;
} NmeaInfo;

void NMEA_Init(UART_HandleTypeDef *huart);

void NMEA_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

void NMEA_GetInfo(NmeaInfo *nmeaInfo);

#endif //NMEA0183_H
