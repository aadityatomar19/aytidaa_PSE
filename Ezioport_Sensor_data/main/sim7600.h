/*
 * sim7600.h
 *
 *  Created on: Mar 11, 2022
 *      Author: Admin
 */

#ifndef MAIN_SIM7600_H_
#define MAIN_SIM7600_H_
#include "common.h"
#include <time.h>

void sendATcmd(uart_port_t uart_controller, char* text);
char* read_line(uart_port_t uart_controller);
void string_len(char* line);
int sendATcommand(char* ATcommand, char* expected_answer,unsigned int timeout);
char sendATcommand2(const char* ATcommand, const char* expected_answer1, const char* expected_answer2, unsigned int timeout);
int sendATcommand3(char* ATcommand, char* expected_answer,unsigned int timeout,char* response1);
void Soft_Reset();
bool PowerOn();
bool MQTT_Start(const char*MQTT_server, const char* MQTT_username, const char* MQTT_password, const char* MQTT_client_ID);
bool Internet(const char* APN);
bool MQTT_Pub_data(char *Topic, char *Payload);
bool MQTT_Stop();
bool GPSPositioning();
float GPSlat();
float GPSlog();
void getTime(struct tm *rtcTime);

#define SIM7600_uart UART_NUM_1
char RecMessage[200];
char LatDD[3], LatMM[10], LogDD[4], LogMM[10], DdMmYy[7], UTCTime[7];
char* tok;
char* tok1;
char* tok2;
char* Date;
char* Time;
int Month;
int Year;
int Day;
int Hour;
int Minute;
int Second;
#endif /* MAIN_SIM7600_H_ */
