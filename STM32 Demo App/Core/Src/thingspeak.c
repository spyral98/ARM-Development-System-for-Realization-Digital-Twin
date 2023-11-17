#include "thingspeak.h"

#include <stdio.h>
#include <string.h>
#include "AT_commands.h"

uint8_t TS_establish_connection(const char* SSID, const char* pass)
{
	AT_send_command("AT\r\n", "OK");
	osDelay(100);
	AT_send_command("AT+GMR\r\n", "OK");
	osDelay(100);
	AT_send_command("AT+CWMODE=1\r\n", "OK");
	osDelay(100);
	AT_send_command("AT+CWLAP\r\n", "OK");
	osDelay(100);

	char buf[64];
	sprintf(buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, pass);

	AT_send_command(buf, "OK");
	osDelay(100);
	AT_send_command("AT+CIFSR\r\n", "OK");
	osDelay(100);
	AT_send_command("AT+CIPMUX=0\r\n", "OK");

	return 0;
}

uint8_t TS_send_data(const char* API_key, float* value_buf, uint8_t nFields)
{
	char data_buf[128];
	char field_buf[32];
	char length_buf[32];

	AT_send_command("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n", "OK");
	osDelay(100);

	sprintf(data_buf, "GET /update?api_key=%s", API_key);
	for (int i=0; i<nFields; i++)
	{
		sprintf(field_buf, "&field%d=%f",i+1, value_buf[i]);
		strcat (data_buf, field_buf);
	}

	strcat(data_buf, "\r\n");

	sprintf (length_buf, "AT+CIPSEND=%d\r\n", strlen(data_buf));

	AT_send_command(length_buf, ">");
	osDelay(100);
	AT_send_command(data_buf, "CLOSED");
	osDelay(100);

	return 0;
}
