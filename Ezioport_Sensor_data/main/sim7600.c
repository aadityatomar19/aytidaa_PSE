/*
 * sim7600.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Admin
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>
#include "driver/gpio.h"
#include "sim7600.h"
float Lat = 0;
float Log = 0;
float Latdata=0;
float logdata=0;
#define RESET_PIN GPIO_NUM_13
#define GPIO_OUTPUT_PIN (1ULL << RESET_PIN)

void sendATcmd(uart_port_t uart_controller, char* text){
	uart_write_bytes(uart_controller, text, strlen(text));
	return;
}

// read a line from the UART controller
char* read_line(uart_port_t uart_controller) {
	static char line[2048];
	char *ptr = line;
	while(1) {
		int num_read = uart_read_bytes(uart_controller, (unsigned char *)ptr, 1, 1000/portTICK_RATE_MS);//portMAX_DELAY);
		if(num_read == 1) {
			// new line found, terminate the string and return
			if(*ptr == '\n') {
				ptr++;
				*ptr = '\0';
				return line;
			}
			// else move to the next char
			ptr++;
		} else {
			*ptr=10;
			ptr++;
			*ptr=0;
			return line;
		}
	}
}

void string_len(char* line){
	if(strlen(line)>1){
		printf("%s",line);
	}
}


int sendATcommand(char* ATcommand, char* expected_answer,unsigned int timeout){
	uint8_t answer = 0;
	char *response;
	unsigned long previous;
	vTaskDelay(100 / portTICK_RATE_MS);
	previous = millis();
	uart_flush(SIM7600_uart);
	sendATcmd(SIM7600_uart, ATcommand);
	printf("\nSending command: -- %s\n", ATcommand);
	sendATcmd(SIM7600_uart, "\r"); // Send the AT command
	previous = millis();
	// this loop waits for the answer
	do
	{
		// if there are data in the UART input buffer, reads it and checks for the asnwer
		response= read_line(SIM7600_uart);
		// check if the desired answer  is in the response of the module
		if (strstr(response, expected_answer) != NULL)
		{
			printf("\nresponse: -- %s\n", response);
			answer = 1;
		}
		// Waits for the asnwer with time out
	} while ((answer == 0) && ((millis() - previous) < timeout));
	return answer;
}

char sendATcommand2(const char* ATcommand, const char* expected_answer1, const char* expected_answer2, unsigned int timeout){
	uint8_t answer = 0;
	char *response;
	unsigned long previous;
	vTaskDelay(100 / portTICK_RATE_MS);
	previous = millis();
	uart_flush(SIM7600_uart);
	sendATcmd(SIM7600_uart, ATcommand);
	printf("\nSending command: -- %s\n", ATcommand);
	sendATcmd(SIM7600_uart, "\r"); // Send the AT command
	previous = millis();
	// this loop waits for the answer
	do{
		// if there are data in the UART input buffer, reads it and checks for the asnwer
		response= read_line(SIM7600_uart);
		// check if the desired answer 1  is in the response of the module
		if (strstr(response, expected_answer1) != NULL)
		{
			printf("\nresponse: -- %s\n", response);
			answer = 1;
		}
		// check if the desired answer 2 is in the response of the module
		else if (strstr(response, expected_answer2) != NULL)
		{
			printf("\nresponse: -- %s\n", response);
			answer = 2;
		}

	}
	// Waits for the asnwer with time out
	while((answer == 0) && ((millis() - previous) < timeout));
	return answer;
}

int sendATcommand3(char* ATcommand, char* expected_answer,unsigned int timeout,char* response1){
	uint8_t answer = 0;
	unsigned long previous;
	char *response;
	vTaskDelay(100 / portTICK_RATE_MS);
	previous = millis();
	uart_flush(SIM7600_uart);
	sendATcmd(SIM7600_uart, ATcommand);
	printf("\nSending command: -- %s\n", ATcommand);
	sendATcmd(SIM7600_uart, "\r"); // Send the AT command
	previous = millis();
	// this loop waits for the answer
	do
	{
		// if there are data in the UART input buffer, reads it and checks for the asnwer
		response= read_line(SIM7600_uart);
		// check if the desired answer  is in the response of the module
		if (strstr(response, expected_answer) != NULL)
		{
			printf("\nresponse: -- %s\n", response);
			strncpy(response1, response, strlen(response));
			answer = 1;
		}
		// Waits for the asnwer with time out
	} while ((answer == 0) && ((millis() - previous) < timeout));
	return answer;
}
void Soft_Reset(){
	sendATcommand("AT+CRESET", "PB DONE", 30000);
}
void Hard_Reset()
{
	gpio_set_level(RESET_PIN,1);
	vTaskDelay(1000 / portTICK_RATE_MS);
	gpio_set_level(RESET_PIN,0);
	vTaskDelay(1000 / portTICK_RATE_MS);
}

bool PowerOn()
{
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
	//  Reset_key=Rst;
	//  pinMode(Reset_key, OUTPUT);
	unsigned long previous;
	uint8_t answer = 0;
	uint8_t res = 0;
	Soft_Reset();
	vTaskDelay(1000/portTICK_RATE_MS);
	answer = sendATcommand("AT", "OK", 5000);
	if (answer == 0)
	{
		printf("Starting up...\n");
		  Hard_Reset();
		vTaskDelay(30000/portTICK_RATE_MS);
		previous = millis();
		while ((answer == 0) && ((millis() - previous) < 10000))
		{ // Send AT every two seconds and wait for the answer
			answer = sendATcommand("AT", "OK", 2000);
			vTaskDelay(1000/portTICK_RATE_MS);
		}
	}
	if(answer == 1){
		sendATcommand("AT+CGDRT=43,1", "OK", 10000);
		sendATcommand("AT+CGSETV=43,1", "OK", 10000);
		vTaskDelay(1000/portTICK_RATE_MS);
		sendATcommand("AT+CGSETV=43,0", "OK", 10000);
		sendATcommand("AT+CGPSAUTO=1", "OK", 1000);
		previous = millis();
		while((!res)&& ((millis() - previous) < (10*60*1000))){
			res = sendATcommand2("AT+CREG?", "+CREG: 0,1","+CREG: 0,5", 500);
			vTaskDelay(250/portTICK_RATE_MS);
			//			GPSPositioning();
			vTaskDelay(250/portTICK_RATE_MS);
		}
	}
	if(res){
		sendATcommand("AT+CGSETV=43,1", "OK", 1000);
		return true;
	}
	else{
		sendATcommand("AT+CGSETV=43,0", "OK", 1000);
		return false;
	}
}
/**************************MQTT**************************/
bool MQTT_Start(const char*MQTT_server, const char* MQTT_username, const char* MQTT_password, const char* MQTT_client_ID){
	if (sendATcommand("AT+CMQTTSTART", "+CMQTTSTART: 0", 10000)) {
		vTaskDelay(500/portTICK_RATE_MS);
		char aux_str[200];
		sprintf(aux_str, "AT+CMQTTACCQ=0,\"%s\",0,4", MQTT_client_ID);
		sendATcommand(aux_str, "OK", 5000);
		vTaskDelay(500/portTICK_RATE_MS);
		memset(aux_str, '\0', 200);
		sprintf(aux_str, "AT+CMQTTCONNECT=0,\"tcp://%s\",120,1,\"%s\",\"%s\"", MQTT_server, MQTT_username, MQTT_password);
		int answer=sendATcommand(aux_str, "+CMQTTCONNECT: 0,0", 60000);
		if(answer){
			printf("\nconnected to server\n");
			return true;
		}
		else{
			printf("\nfailed to connected to server\n");
			return false;
		}
	}
	else{
		return false;
	}
}

bool Internet(const char* APN){
	char aux_str[50];
	sprintf(aux_str, "AT+CGDCONT=1,\"IP\",\"%s\"", APN);
	if (sendATcommand(aux_str, "OK", 5000)) {
		sendATcommand("AT+CIPMODE=1", "OK", 5000);
		sendATcommand("AT+CGATT=1", "OK", 5000);
		sendATcommand("AT+CGACT=1,1", "OK", 5000);
		return true;
	}
	else
		return false;
}
bool MQTT_Pub_data(char *Topic, char *Payload){
	char aux_str[200];
	int topic_len = strlen(Topic) ;
	sprintf(aux_str, "AT+CMQTTTOPIC=0,%d",topic_len);
	sendATcommand(aux_str, "OK", 1000);
	sendATcmd(SIM7600_uart, Topic);
	sendATcmd(SIM7600_uart, "\r"); // Send the AT command
	int Payload_len = strlen(Payload);
	memset(aux_str, '\0', 200);
	sprintf(aux_str, "AT+CMQTTPAYLOAD=0,%d", Payload_len);
	sendATcommand(aux_str, "OK", 1000);
	sendATcmd(SIM7600_uart, Payload);
	sendATcmd(SIM7600_uart, "\r"); // Send the AT command
	if (sendATcommand("AT+CMQTTPUB=0,1,60", "+CMQTTPUB: 0,0", 10000)) {
		return true;
	}
	else
		return false;
}

bool MQTT_Stop(){
	if (sendATcommand("AT+CMQTTDISC=0,120", "OK", 1000)) {
		sendATcommand("AT+CMQTTREL=0", "OK", 1000);
		sendATcommand("AT+CMQTTSTOP", "OK", 1000);
		return true;
	}
	else
		return false;
}
/**************************GPS positoning**************************/
bool GPSPositioning()
{
	unsigned long previous;
	uint8_t answer = 0;
	Lat = 0;
	Log = 0;
	char response1[200];
	memset(LatDD, '\0', 3);        // Initialize the string
	memset(LatMM, '\0', 10);       // Initialize the string
	memset(LogDD, '\0', 4);        // Initialize the string
	memset(LogMM, '\0', 10);       // Initialize the string
	memset(DdMmYy, '\0', 7);       // Initialize the string
	memset(UTCTime, '\0', 7);      // Initialize the string
	printf("Start GPS session...\n");
	answer = sendATcommand3("AT+CGPSINFO", "+CGPSINFO: ", 1000, &response1); // start GPS session, standalone mode
	if (answer == 1)
	{
		answer = 0;
		printf("\n gps response: -- %s\n", response1);
		if ((strstr(response1, "+CGPSINFO:") != NULL)&&(strstr(response1, ",,,,,,,,") == NULL))
		{
			answer = 1;
			printf("\n Got gps data: -- %s\n", response1);
		}
		else if (strstr(response1, ",,,,,,,,") != NULL)
		{
			printf("\n NO gps data: -- %s\n", response1);
			return false;
		}
		strncpy(LatDD, response1 + 11, 2);
		LatDD[2] = '\0';
		strncpy(LatMM, response1 + 13, 9);
		LatMM[9] = '\0';
		Lat = atoi(LatDD) + (atof(LatMM) / 60);
		strncpy(LogDD, response1 + 25, 3);
		LogDD[3] = '\0';
		strncpy(LogMM, response1 + 28, 9);
		LogMM[9] = '\0';
		Log = atoi(LogDD) + (atof(LogMM) / 60);
		memset(response1, '\0', 200);
	}
	return true;
}

float GPSlat()
{
	Latdata = Lat;
	return Latdata;
}

float GPSlog()
{
	logdata = Log;
	return logdata;
}
void getTime(struct tm *rtcTime){
	uint8_t answer = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	int l=  0;
	char Clock[50];
	Month = 0;
	Year = 0;
	Day = 0;
	Hour = 0;
	Minute = 0;
	Second = 0;
	char* Time_data= 0;
	sendATcommand("AT+CTZU=1", "OK", 5000);
	answer = sendATcommand3("AT+CCLK?", "+CCLK:", 1000, &Clock); // start GPS session, standalone mode
	if (answer == 1)
	{
		/*Separate +cclk: \" and 22/03/10,18:43:52+22\"*/
		tok = strtok(Clock, "\"");
		while (tok != 0 && l != 2) {
			if (l == 0)
			{
				printf("\n");
				printf("\nunused data =%s\n", tok);
			}
			if (l == 1)
			{
				Time_data = tok;
				printf("Time and date =%s\n", tok);
			}
			tok = strtok(0, "\"");
			l++;
		}

		/*parse date and time from variable eg.22/03/10,18:43:52+22*/
		tok = strtok(Time_data, ",");
		while (tok != 0 && i != 2) {
			if (i == 0)
			{
				Date = tok;
				printf("date =%s\n", Date);
			}
			if (i == 1)
			{
				Time = tok;
				printf("time =%s\n", Time);
			}
			tok = strtok(0, "+");
			i++;
		}
		/* parse year, month,day from date variable eg. 22/03/10*/
		tok1 = strtok(Date, "/");
		while (tok1 != 0 && j != 3) {
			if (j == 0)
			{
				Year = atoi(tok1);
				rtcTime->tm_year =Year + 2000;
				printf("year =%d\n", rtcTime->tm_year);
			}
			if (j == 1)
			{
				Month = atoi(tok1);
				rtcTime->tm_mon = Month;
				printf("month =%d\n", rtcTime->tm_mon);
			}
			if (j == 2)
			{
				Day = atoi(tok1);
				rtcTime->tm_mday = Day;
				printf("day =%d\n", rtcTime->tm_mday);
			}
			tok1 = strtok(0, "/");
			j++;
		}
		/* Parse hour, minute, second from Time variable eg.18:43:52*/
		tok2 = strtok(Time, ":");
		while (tok2 != 0 && k != 3) {
			if (k == 0)
			{
				Hour = atoi(tok2);
				rtcTime->tm_hour = Hour;
				printf("hour =%d\n", rtcTime->tm_hour);
			}
			if (k == 1)
			{
				Minute = atoi(tok2);
				rtcTime->tm_min = Minute;
				printf("minute =%d\n", rtcTime->tm_min);
			}
			if (k == 2)
			{
				Second = atoi(tok2);
				rtcTime->tm_sec = Second;
				printf("second =%d\n", rtcTime->tm_sec);
			}
			tok2 = strtok(0, ":");
			k++;

		}
	}
}


