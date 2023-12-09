/*
 *  09/12/2023
 * Archivo: main.c
 *
 * Aquí se presenta el uso del adc mediante la variación de voltaje manual de unos potenciometros 
 *
 * Desarrollado por:
 * - Iving Iván Castillo Pérez 19060759
 *
 * Todos los derechos de autor reservados.
 * El mal uso de estos Drivers no es responsabilidad del desarrollador.
 */



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <unistd.h> //Pasa uso del delay (Usleep)
#include "adc_drivers.h"

///Definiciones
#define VOLTAJE 3.28
#define VOLTAJE2 5.00
#define CANTIDAD_MUESTRAS 11
#define RANGO_CANALES 2


void app_main(void)
{
	//Declaracion de variables para guardar la conversion FINAL que hace el ADC y mostrarlo en consola
	float v0=0;
	float v1=0;
	float v2=0;
	//float v3=0;
	//float v4=0;
	float v5=0;
	//float v6=0;

	//Arregloas para guardar los valores que LEE el ADC (Antes de tomar en cuenta la resolucion y el voltaje de referencia)
	uint32_t lectura[RANGO_CANALES];
	uint32_t lectura2[RANGO_CANALES];
	//uint32_t lectura3[RANGO_CANALES];
	uint32_t lectura4[RANGO_CANALES];
	//uint32_t lectura5[RANGO_CANALES];

	// LLamara a varios canales  //Prende los canales del ADC, configuramos su resolucion, atenuacion, canal y en caso de ser necesario el rango de canales a leer
	//ADC1
	INICIAR_CANALES_ADC(ADC_MODULO_1, ADC_12_BITS_RESOLUCION, ADC_ATENUACION_11_Db, CANAL_6, RANGO_CANALES);
	// LLamara a un solo canal //Prende el canal ADC a usar
	INICIAR_CANAL_ADC(ADC_MODULO_1, ADC_12_BITS_RESOLUCION, ADC_ATENUACION_11_Db, CANAL_0);


	//ADC2
	INICIAR_CANALES_ADC(ADC_MODULO_2, ADC_12_BITS_RESOLUCION, ADC_ATENUACION_11_Db, CANAL_5, RANGO_CANALES);
	INICIAR_CANAL_ADC(ADC_MODULO_2, ADC_12_BITS_RESOLUCION, ADC_ATENUACION_11_Db, CANAL_4);

	// Leyendo constantemente los valores del ADC con un while (creando un ciclo)
    while (true) {

		//Aqui pausamos el adc que eligamos
    	//PAUSAR_ADC(2);

    										//Anteriormente habiamos prendido y configurado el ADC
    										//Ahora vamos a realizar la lectura en los ADC prendidos
    	//Aqui leemos varios canales del 6 al 7
    	ADC_SECUENCIA_READ(ADC_MODULO_1, ADC_12_BITS_RESOLUCION, CANAL_6, RANGO_CANALES, lectura, CANTIDAD_MUESTRAS);
    	//LEER 1 SOLO CANAL (Canal 0)
    	ADC_SECUENCIA_READ(ADC_MODULO_1, ADC_12_BITS_RESOLUCION, CANAL_0, 1, lectura2, CANTIDAD_MUESTRAS);

    	//Aqui leemos varios canales del 5 al 6
    	//ADC_SECUENCIA_READ(ADC_MODULO_2, ADC_12_BITS_RESOLUCION, CANAL_5, RANGO_CANALES, lectura3, CANTIDAD_MUESTRAS);
    	//LEER 1 SOLO CANAL (Canal 4)
    	ADC_SECUENCIA_READ(ADC_MODULO_2, ADC_12_BITS_RESOLUCION, CANAL_4, 1, lectura4, CANTIDAD_MUESTRAS);

    	//Como el voltaje de referencia que estamos usando es el de la ESP32 y el voltaje que maneja es de 3.3
    	//Hacemos uso de la fomrula para calcular el valor real de el voltaje de entrada (VEASE LA FORMULA-FOTO).
		v0 = (lectura[0]*VOLTAJE)/4096; // 2^12 = una resolucion de 12 BITS = 4096
		v1 = (lectura[1]*VOLTAJE)/4096;	// 2^12 = una resolucion de 12 BITS = 4096
		v2 = (lectura2[0]*VOLTAJE)/4096;	// 2^12 = una resolucion de 12 BITS = 4096
		//v3 = (lectura3[0]*VOLTAJE)/4096; // 2^12 = una resolucion de 12 BITS = 4096
		//v4 = (lectura3[1]*VOLTAJE)/4096;	// 2^12 = una resolucion de 12 BITS = 4096
		v5 = (lectura4[0]*VOLTAJE)/4096; // 2^12 = una resolucion de 12 BITS = 4096



		//Se imprime los valores obtenidos
		printf("ADC1\n");
		printf("Voltaje Canal 6 ADC1: %.2f v\n", v0);
		printf("Voltaje Canal 7 ADC1: %.2f V\n", v1);
		printf("Voltaje Canal 0 ADC1: %.2f V\n\n", v2);

		printf("ADC2\n");
		//printf("Voltaje Canal 5 ADC2: %.2f v\n", v6);
		//printf("Voltaje Canal 7 ADC2: %.2f V\n", v4);
		printf("Voltaje Canal 4 ADC2: %.2f V\n", v5);


		usleep(3000000);				//delay para la visualizacion de los resultados en la consola de una manera optima

    			  }
}
