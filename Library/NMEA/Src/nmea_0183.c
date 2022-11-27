#include <memory.h>
#include <stdlib.h>
#include <time.h>
#include "nmea_0183.h"

UART_HandleTypeDef *gNmeaHuart;

char gNmeaDataBuffer[2048] = {0};
const uint16_t BUFFER_SIZE = sizeof(gNmeaDataBuffer) / sizeof(char);

NmeaInfo gNmeaInfo = {0};

void NMEA_ParseData(char data[1024], NmeaInfo *info);
void NMEA_ParseCommand(char command[], size_t length, NmeaInfo *info);
void NMEA_ParseGSA(char command[], NmeaInfo *info);
void NMEA_ParseGSV(char command[], NmeaInfo *info);
void NMEA_ParseRMC(char command[], NmeaInfo *info);

uint8_t NMEA_ParseNextValue(char **pointer, char value[], uint8_t *length, uint8_t *index);
uint8_t NMEA_ParseValueCount(char *command);

long NMEA_ParseTime(const char text[]);
long NMEA_ParseDate(const char text[]);
double NMEA_ParseLocation(char *text);

void NMEA_RxCommandCallback(const char *command, uint16_t size);
void NMEA_RxCallback(NmeaInfo *info);

void NMEA_Init(UART_HandleTypeDef *huart) {
    gNmeaHuart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(gNmeaHuart,(uint8_t *) gNmeaDataBuffer,BUFFER_SIZE);
}

void NMEA_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance != gNmeaHuart->Instance) {
        return;
    }
    HAL_UART_DMAStop(gNmeaHuart);
    if (Size < BUFFER_SIZE) {
        gNmeaDataBuffer[Size] = '\0';
    }
    NMEA_ParseData(gNmeaDataBuffer, &gNmeaInfo);
    HAL_UARTEx_ReceiveToIdle_DMA(gNmeaHuart,(uint8_t *) gNmeaDataBuffer,BUFFER_SIZE);

    NMEA_RxCommandCallback(gNmeaDataBuffer, Size);
    NMEA_RxCallback(&gNmeaInfo);
}

void NMEA_GetInfo(NmeaInfo *nmeaInfo) {
    memcpy(nmeaInfo, &gNmeaInfo, sizeof(NmeaInfo));
}

void NMEA_ParseData(char data[], NmeaInfo *info) {
    memset(info, 0, sizeof(NmeaInfo));
    info->latitude_direction = '*';
    info->longitude_direction = '*';
    info->state = 'V';

    size_t length;
    char command[100] = {0};
    char *index = data, *index_begin, *index_end, *index_next_begin;
    for (;;) {
        index_begin = strchr(index, '$');
        if (index_begin == NULL) break;
        index_end = strchr(index_begin, '\n');
        if (index_end == NULL) break;
        index_next_begin = strchr(index_begin + 1, '$');
        if (index_next_begin != NULL && index_next_begin < index_end) {
            index = index_next_begin;
            continue;
        }

        length = index_end - index_begin + 1;
        memcpy(command, index_begin, length);
        command[length] = '\0';
        NMEA_ParseCommand(command, length, info);

        if (index_next_begin == NULL) break;
        index = index_next_begin;
    }
}

/**
 * https://blog.csdn.net/weixin_43854928/article/details/120423640
 * @param command
 * @param length
 * @param info
 */
void NMEA_ParseCommand(char command[], size_t length, NmeaInfo *info) {
    if (command[0] != '$' || length < 3) return;
    if (command[length - 2] != '\r' || command[length - 1] != '\n') return;
    // 计算校验值
    char check_text[] = {
            command[length - 4],
            command[length - 3],
            0
    };
    uint8_t expected_check = (uint8_t) strtol(check_text, NULL, 16);
    uint8_t actual_check = 0;
    char *index = command + 1;
    while (*index != '*' && *index != '\0') {
        actual_check ^= (uint8_t) *index;
        index++;
    }
    if (expected_check != actual_check) return;
    // 当前卫星信息
    if (strstr(command, "GSA") != NULL) {
        NMEA_ParseGSA(command, info);
        return;
    }
    // 可见卫星信息
    if (strstr(command, "GSV") != NULL) {
        NMEA_ParseGSV(command, info);
        return;
    }
    // 推荐定位信息
    if (strstr(command, "RMC") != NULL) {
        NMEA_ParseRMC(command, info);
        return;
    }
}

void NMEA_ParseGSA(char command[], NmeaInfo *info) {
    if (strstr(command, "GNGSA") == NULL) return;
    char *pointer = command; char value[20] = {0}; uint8_t length = 0; uint8_t index = 0;
    while (NMEA_ParseNextValue(&pointer, value, &length, &index) > 0) {
        if (index <= 2 || index >= 15) continue;
        if (length == 0) continue;
        info->satellite_available++;
    }
}

void NMEA_ParseGSV(char command[], NmeaInfo *info) {
    char *pointer = command; char value[20] = {0}; uint8_t length = 0; uint8_t index = 0;
    while (NMEA_ParseNextValue(&pointer, value, &length, &index) > 0) {
        if (index != 3) continue;
        if (strstr(command, "BD") != NULL) {
            info->bds_satellite_total = strtol(value, NULL, 10);
        }
        if (strstr(command, "GA") != NULL) {
            info->gal_satellite_total = strtol(value, NULL, 10);
        }
        if (strstr(command, "GP") != NULL) {
            info->gps_satellite_total = strtol(value, NULL, 10);
        }
        if (strstr(command, "GL") != NULL) {
            info->glo_satellite_total = strtol(value, NULL, 10);
        }
        break;
    }
}

void NMEA_ParseRMC(char command[], NmeaInfo *info) {
    char *pointer = command; char value[20] = {0}; uint8_t length = 0; uint8_t index = 0;
    while (NMEA_ParseNextValue(&pointer, value, &length, &index) > 0) {
        switch (index) {
            case 1:
                if (length > 0) {
                    info->utc_time = NMEA_ParseTime(value);
                }
                break;
            case 9:
                if (length > 0) {
                    info->utc_time += NMEA_ParseDate(value);
                }
                break;
            case 2:
                if (length > 0) {
                    info->state = value[0];
                }
                break;
            case 3:
                info->latitude = NMEA_ParseLocation(value);
                break;
            case 4:
                if (length > 0) {
                    info->latitude_direction = value[0];
                }
                break;
            case 5:
                info->longitude = NMEA_ParseLocation(value);
                break;
            case 6:
                if (length > 0) {
                    info->longitude_direction = value[0];
                }
                break;
            default:
                break;
        }
    }
}

uint8_t NMEA_ParseValueCount(char *command) {
    size_t count = 0;
    char *index_begin = command;
    char *index_end;
    for (;;) {
        if (*index_begin != ',') {
            index_begin = strchr(index_begin, ',');
        }
        if (index_begin == NULL) break;

        count++;

        index_end = strchr(index_begin + 1, ',');
        if (index_end == NULL) {
            index_end = strchr(index_begin + 1, '*');
        }
        if (index_end == NULL) break;
        index_begin = index_end;
    }

    return count;
}

uint8_t NMEA_ParseNextValue(char **pointer, char value[], uint8_t *length, uint8_t *index) {
    char *index_begin = *pointer;
    if (*index_begin != ',') {
        index_begin = strchr(index_begin, ',');
    }
    if (index_begin == NULL) return 0;

    char *index_end = strchr(index_begin + 1, ',');
    if (index_end == NULL) {
        index_end = strchr(index_begin + 1, '*');
    }
    if (index_end == NULL) return 0;

    size_t size = index_end - (index_begin + 1);
    memcpy(value, index_begin + 1, size);
    value[size] = '\0';

    *pointer = index_end;
    *length = size;
    (*index)++;
    return 1;
}

/**
 * HHmmss.sss 转 timestamp
 * @param value
 * @return
 */
long NMEA_ParseTime(const char text[]) {
    if (strlen(text) != 10) return 0;
    char value[3] = {0};
    long result = 0;
    value[0] = text[0];
    value[1] = text[1];
    result += strtol(value, NULL, 10) * 60 * 60;
    value[0] = text[2];
    value[1] = text[3];
    result += strtol(value, NULL, 10) * 60;
    value[0] = text[4];
    value[1] = text[5];
    result += strtol(value, NULL, 10);
    return result;
}

/**
 * yyMMdd 转 timestamp
 * @param value
 * @return
 */
long NMEA_ParseDate(const char text[]) {
    if (strlen(text) != 6) return 0;

    // 使用标准库需要大约 12kb 空间
    //char value[3] = {0};
    //struct tm time = {0};
    //value[0] = text[4];
    //value[1] = text[5];
    //time.tm_year = 2000 + strtol(value, NULL, 10) - 1900;
    //value[0] = text[2];
    //value[1] = text[3];
    //time.tm_mon = strtol(value, NULL, 10) - 1;
    //value[0] = text[0];
    //value[1] = text[1];
    //time.tm_mday = strtol(value, NULL, 10);
    //return (long) mktime(&time);

    char value[3] = {0};
    value[0] = text[4];
    value[1] = text[5];
    int tm_year = 2000 + strtol(value, NULL, 10);
    value[0] = text[2];
    value[1] = text[3];
    int tm_mon = strtol(value, NULL, 10);
    value[0] = text[0];
    value[1] = text[1];
    int tm_mday = strtol(value, NULL, 10);

    int day = 0;
    for (int i = 1970; i < tm_year; ++i) {
        if (i % 100 == 0) {
            if (i % 400) {
                day += 366;
            } else {
                day += 365;
            }
            continue;
        }
        if (i % 4 == 0) {
            day += 366;
        } else {
            day += 365;
        }
    }

    switch (tm_mon - 1) {
        case 1:day += 31;break;
        case 2:day += 59;break;
        case 3:day += 90;break;
        case 4:day += 120;break;
        case 5:day += 151;break;
        case 6:day += 181;break;
        case 7:day += 212;break;
        case 8:day += 243;break;
        case 9:day += 273;break;
        case 10:day += 304;break;
        case 11:day += 334;break;
        case 12:day += 365;break;
        default:break;
    }
    if (tm_mon > 2) {
        if (tm_year % 100 == 0) {
            if (tm_year % 400) {
                day++;
            }
        } else {
            if (tm_year % 4 == 0) {
                day++;
            }
        }
    }

    day += tm_mday;

    return (long) day * 24 * 60 * 60;
}

/**
 * dddmm.mmmmm char 转 dd.ddddddd float
 * @param value
 * @return
 */
double NMEA_ParseLocation(char *text) {
    double value = strtod(text, NULL) / 100;
    int integer = (int) value / 1;
    double decimal = value - integer;
    decimal = decimal / 60 * 100;
    return integer + decimal;
}

__weak void NMEA_RxCommandCallback(const char *command, uint16_t size) {
    UNUSED(command);
}

__weak void NMEA_RxCallback(NmeaInfo *info) {
    UNUSED(info);
}