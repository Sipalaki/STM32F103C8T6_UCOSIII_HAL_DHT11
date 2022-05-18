#ifndef __DHT11_H
#define __DHT11_H	 
#include "main.h"

#define DHT11PORT	GPIOA	//定义IO接口
#define DHT11_IO	GPIO_PIN_15	//定义IO接口


void DHT11_IO_OUT (void);
void DHT11_IO_IN (void);
void DHT11_RST (void);
uint8_t Dht11_Check(void); 	   
uint8_t Dht11_ReadBit(void); 
uint8_t Dht11_ReadByte(void); 
uint8_t DHT11_Init (void);
uint8_t DHT11_ReadData(uint8_t *h);
		 				    
#endif