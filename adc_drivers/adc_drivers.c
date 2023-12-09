/*
 *  09/12/2023
 * Archivo: adc_drivers.c
 *
 * Desarrollado por:
 * - Iving Iván Castillo Pérez 19060759
 *
 * Todos los derechos de autor reservados.
 * El mal uso de estos Drivers no es responsabilidad del desarrollador.
 */


#include "adc_drivers.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "soc/sens_struct.h"

//Variables
int G;
//Registros

volatile uint32_t * const SENS_SAR  = (uint32_t*)0x3FF4882C;	//PAG 643- Controles del ADC
volatile uint32_t * const INV_ADC1CLK  = (uint32_t*)0x3FF48800;		// Declara la tasa de muestreo de nuestro adc1
volatile uint32_t * const DAC_OFF  = (uint32_t*)0x3FF48910; //APAGA EL DAC
volatile uint32_t * const INV_ADC2CLK  = (uint32_t*)0x3FF48890; // Declara la tasa de muestreo de nuestro adc2
volatile uint32_t * const DAC_DownR  = (uint32_t*)0x3FF4889E;  //Fuerza el apagado del tec
volatile uint32_t * const ON_DAC1  = (uint32_t*)0x3FF48484;		//ON DAC
volatile uint32_t * const ON_DAC2  = (uint32_t*)0x3FF48488;		//ON DAC
volatile uint32_t * const lee  = (uint32_t*)0x3FF4881c; // 0x3FF4881c
volatile uint32_t * const READ_CTRL_REG  = (uint32_t*)0x3FF48800; //Permite seleccionar el modo RTC o digital
volatile uint32_t * const START1_REG  = (uint32_t*)0x3FF48854; //Controla el incio del ADC tipo SAR
//MAPEO CANALES CON PINES

const int adc1_maping_canales_ent_sal[MAX_CANTIDAD_CANALES_ADC1] = {ADC1_CANAL_0_GPIO_NUMERO, ADC1_CANAL_1_GPIO_NUMERO, ADC1_CANAL_2_GPIO_NUMERO, ADC1_CANAL_3_GPIO_NUMERO,
																	ADC1_CANAL_4_GPIO_NUMERO, ADC1_CANAL_5_GPIO_NUMERO, ADC1_CANAL_6_GPIO_NUMERO, ADC1_CANAL_7_GPIO_NUMERO};

const int adc2_maping_canales_ent_sal[MAX_CANTIDAD_CANALES_ADC2] = {ADC2_CANAL_0_GPIO_NUMERO, ADC2_CANAL_1_GPIO_NUMERO, ADC2_CANAL_2_GPIO_NUMERO, ADC2_CANAL_3_GPIO_NUMERO,
																	ADC2_CANAL_4_GPIO_NUMERO, ADC2_CANAL_5_GPIO_NUMERO, ADC2_CANAL_6_GPIO_NUMERO, ADC2_CANAL_7_GPIO_NUMERO,
																	ADC2_CANAL_8_GPIO_NUMERO, ADC2_CANAL_9_GPIO_NUMERO};




void INICIAR_CANAL_ADC(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_atenuacion_t ATENUACION, adc_canal_t CANAL)
{
	TURN_OFF_DAC; //APAGA EL DAC para que no interfiera con el ADC

	if(MODULO == ADC_MODULO_1)
	{
				RESOLUCION_ADC1(RESOLUCION);//•	Función para escribir en registro cual es la resolución que se quiere utilizar
				Confi_PIN1(CANAL);	 //Configura el pin y desactiva GPIO en el canal deseado

				SENS.sar_atten1 = ( SENS.sar_atten1 & ~(0x3 << (CANAL * 2)) ) | ((ATENUACION & 0x3) << (CANAL * 2));
				//SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = false; //0x3FF48910
				Invert_SAR1(1);
				//SENS.sar_read_ctrl.sar1_data_inv = 1; //0x3FF48800
				CLK_DIV1(2);
				//SENS.sar_read_ctrl.sar1_clk_div = 2;  //0x3FF48800
	}
	if(MODULO == ADC_MODULO_2)
	{
								RESOLUCION_ADC2(RESOLUCION);//•	Función para escribir en registro cual es la resolución que se quiere utilizar
								Confi_PIN2(CANAL);	 //Configura el pin y desactiva GPIO en el canal deseado
								SENS.sar_atten2 = ( SENS.sar_atten2 & ~(0x3 << (CANAL * 2)) ) | ((ATENUACION & 0x3) << (CANAL * 2));
								//SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = false; //0x3FF48910
								Invert_SAR2(1);// //Invierte los valores recibidos por el adc
								//SENS.sar_read_ctrl.sar1_data_inv = 1; //0x3FF48800
								CLK_DIV2(2);// Declara la velocidad del reloj que nos ayudara a leer los datos
								//SENS.sar_read_ctrl.sar1_clk_div = 2;  //0x3FF48800
								   // return ESP_OK;
								//SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = true; //0x3FF48910
								DAC_Down(1);	//Desactiva la funcion de DAC  NO SE QUE HAGA ESTO NI SI SEA NECESARIO
	}
}

void INICIAR_CANALES_ADC(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_atenuacion_t ATENUACION, adc_canal_t CANAL_INICIO, int NUM_CANALES)
{
	TURN_OFF_DAC; // Apaga el DAC para que no interfiera con el ADC
	// Variables de tipo adc1_canal_t (Enumeración)
	adc1_canal_t ADC1_CANAL_INICIO = 0;
	adc1_canal_t ADC1_CANAL_FINAL = 0;   //Declara variables para medir los rangos de los canales del ADC
	adc2_canal_t ADC2_CANAL_INICIO = 0;
	adc2_canal_t ADC2_CANAL_FINAL = 0;

	//Seleccion de modulo
	if(MODULO == ADC_MODULO_1) //Revisa si se va a utilizar el modulo 1 del adc
	{
		ADC1_CANAL_INICIO = CANAL_INICIO; //Es igual a 0  Toma el valor previavemnte declarado y lo iguala
		ADC1_CANAL_FINAL = (ADC1_CANAL_INICIO+NUM_CANALES)-1;	// Es igual a 4 Declara el canal maximo a utilizar y resta 1 por que también tomamos en cuenta el 0

		for(adc1_canal_t CANAL = ADC1_CANAL_INICIO; CANAL <= ADC1_CANAL_FINAL; CANAL++) //Hace la configuración de cada uno de los canales, por medio de un for y utiliza la función ADC1_Configuracion_canal
		{
			RESOLUCION_ADC1(RESOLUCION); //•	Función para escribir en registro cual es la resolución que se quiere utilizar
			Confi_PIN1(CANAL);  //Configura el pin y desactiva GPIO en el canal deseado

			SENS.sar_atten1 = ( SENS.sar_atten1 & ~(0x3 << (CANAL * 2)) ) | ((ATENUACION & 0x3) << (CANAL * 2)); //SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = false; //0x3FF48910

			Invert_SAR1(1);
			//SENS.sar_read_ctrl.sar1_data_inv = 1; //0x3FF48800
			CLK_DIV1(2);
			//SENS.sar_read_ctrl.sar1_clk_div = 2;  //0x3FF48800
		}
	}
	if(MODULO == ADC_MODULO_2) //Revisa si se va a utilizar el modulo 2 del adc
	{
		ADC2_CANAL_INICIO = CANAL_INICIO; //Toma el valor previavemnte declarado y lo iguala
		ADC2_CANAL_FINAL = (ADC2_CANAL_INICIO+NUM_CANALES)-1; //Declara el canal maximo a utilizar y resta 1 por que también tomamos en cuenta el 0

		for(adc2_canal_t CANAL = ADC2_CANAL_INICIO; CANAL <= ADC2_CANAL_FINAL; CANAL++) //Hace la configuración de cada uno de los canales, por medio de un for y utiliza la función ADC1_Configuracion_canal
		{
						RESOLUCION_ADC2(RESOLUCION);
						Confi_PIN2(CANAL);
						SENS.sar_atten2 = ( SENS.sar_atten2 & ~(0x3 << (CANAL * 2)) ) | ((ATENUACION & 0x3) << (CANAL * 2));
						//SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = false; //0x3FF48910
						Invert_SAR2(1);// //Invierte los valores recibidos por el adc
						//SENS.sar_read_ctrl.sar1_data_inv = 1; //0x3FF48800
						CLK_DIV2(2);// Declara la velocidad del reloj que nos ayudara a leer los datos
						//SENS.sar_read_ctrl.sar1_clk_div = 2;  //0x3FF48800
						//SENS.sar_meas_ctrl2.sar1_dac_xpd_fsm = true; //0x3FF48910
						 DAC_Down(1);	//Desactiva la funcion de DAC
		}
	}
}

					//funcion para hacer lectura de el canal ADC

esp_err_t ADC_SECUENCIA_READ(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_canal_t CANAL_INICIO, int NUM_CANALES, uint32_t *lectura, int NUM_MUESTRAS)
{
	if(MODULO == ADC_MODULO_1)//Se elige si la lectura se hara en el ADC 1
		{
				//FOR para mandar a llamar la funcion que leera el valor de un canal e ira guardando los valores en un arreglo
			for(adc1_canal_t i=0; i < NUM_CANALES; i++)
			{
				*(lectura+i) = ADC1_READ(CANAL_INICIO+i, NUM_MUESTRAS);
			}
		}
		if(MODULO == ADC_MODULO_2)
		{

			for(adc2_canal_t i=0; i < NUM_CANALES; i++)
			{
				*(lectura+i) = ADC2_READ(CANAL_INICIO+i, RESOLUCION, NUM_MUESTRAS);
				//se(G)++; // Incrementa el valor al que apunta G en una unidad
			}
		}
		return G;
}


			//lectura de un canal específico
uint32_t ADC1_TOMAR_LECTURA(adc1_canal_t ADC1_CANAL)
{
	uint32_t lectura;/// Variable
	//Entramos a la configuracion para leer datos y valores del ADC
	NOS1;	//SENS.sar_meas_wait2.force_xpd_sar = UPP;
	   SENS.sar_meas_wait2.force_xpd_amp = 2; //No hay registro en PDF -  encargado de activar o desactivar  funcionalidad del amplificador (AMP)
	    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;	//No hay registro en PDF - relacionado con el control o la configuración de la etapa de reset o realimentación del amplificador (AMP) dentro del FSM (Finite State Machine)
	    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;	//No hay registro en PDF - control o la configuración de la referencia corta del amplificador (AMP) dentro del FSM (Finite State Machine)
	    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;	//No hay registro en PDF - configuración de la referencia corta a tierra del amplificador (AMP) dentro del FSM (Finite State Machine)
	    DIG_FORCE; //#642  permitir seleccionar el control digital (RTC) en lugar del control analógico para el canal SAR1.
	    //SENS.sar_read_ctrl.sar1_dig_force       = 0;    // 1: Select digital control;       0: Select RTC control.
	    START_SAR; //  controla el inicio del controlador SAR ADC1 (Analog-to-Digital Converter 1)
		//PAD_FORCE;   #644			#650
	    //determina la manera en que se controla el mapeo de los pines del ADC (Analog-to-Digital Converter) SAR1 (Successive Approximation Register)
		SENS.sar_meas_start1.sar1_en_pad_force  = 1;    // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
		//configurando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital) en el ESP32 para seleccionar un canal específico.
		SENS.sar_meas_start1.sar1_en_pad = (1 << ADC1_CANAL); //only one channel is selected.
		SENS.sar_meas_start1.meas1_start_sar = 0; //conversión analógico a digital en el ESP32 utilizando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital).
		SENS.sar_meas_start1.meas1_start_sar = 1;  //conversión analógico a digital en el ESP32 utilizando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital).
		while ((_Bool)SENS.sar_meas_start1.meas1_done_sar != 1); //verifica constantemente si la variable meas1_done_sar en SENS.sar_meas_start1 indica que la conversión SAR1 ha finalizado.
	//leyendo un campo de registro de 32 bits desde el registro meas1_data_sar que es donde se obtiene el valor que tiene el canal a leer y devuelve el valor leído como un entero sin signo de 32 bits
	lectura = HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas_start1, meas1_data_sar);
	//Nos salimos de la configuracion para leer datos y valores del ADC
	NOS2; //SENS.sar_meas_wait2.force_xpd_sar = DOWNN;
	return lectura;
}

uint32_t ADC1_READ(adc_canal_t CANAL, int NUM_MUESTRAS)
{
	adc1_canal_t ADC1_CANAL = 0; //Crea una variable de tipo estructura adc1_canal_t llamada ADC1_CANAL y le da un valor de 0
	uint32_t valor_adc=0; // Crea una variable de tipo estructura uint32_t llamada valor_adc y le da el valor de 0
	ADC1_CANAL = CANAL; // Da el valor que tengamos de CANAL en ADC1_READ y le da ese valor a la variable previamente creada llamada ADC1_CANAL
	printf("%d\n",ADC1_CANAL);
	for(int i=0; i<NUM_MUESTRAS; i++)
	{
		valor_adc += ADC1_TOMAR_LECTURA(ADC1_CANAL);  //SE va sumando los valores capturados por el ADC
	}
	valor_adc /= NUM_MUESTRAS;  //Se hace un promedio de los valores optenidos para ser mas precisos con el valor real
	return valor_adc;		//Se retorna el valor promedio final optenido para que despues pueda ser impreso y vizualizado por el usuario
}



			//FUNCION QUE USA LA FUNCION "ADC_SECUENCIA_READ"
uint32_t ADC2_TOMAR_LECTURA(adc2_canal_t ADC2_CANAL, adc_ancho_resolucion_t RESOLUCION)
{
	uint32_t lectura;

	NOS1; //SENS.sar_meas_wait2.force_xpd_sar = UPP;
	SENS.sar_meas_start2.meas2_start_force  = 1;    // 1: SW control RTC ADC start;     0: ULP control RTC ADC start.
			SENS.sar_meas_start2.sar2_en_pad_force  = 1;    // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
			SENS.sar_read_ctrl2.sar2_dig_force      = 0;    // 1: Select digital control;       0: Select RTC control.
			SENS.sar_read_ctrl2.sar2_pwdet_force    = 0;    // 1: Select power detect control;  0: Select RTC control.
			//SYSCON.saradc_ctrl.sar2_mux             = 1;    // 1: Select digital control;       0: Select power detect control.
			//configurando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital) en el ESP32 para seleccionar un canal específico.
			SENS.sar_meas_start2.sar2_en_pad = (1 << ADC2_CANAL); //only one channel is selected.
			SENS.sar_meas_start2.meas2_start_sar = 0; //start force 0  -  //conversión analógico a digital en el ESP32 utilizando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital).
			SENS.sar_meas_start2.meas2_start_sar = 1; //start force 1 -  //conversión analógico a digital en el ESP32 utilizando el ADC SAR1 (Analog-to-Digital Converter, Convertidor Analógico a Digital).
	while ((_Bool)SENS.sar_meas_start2.meas2_done_sar != 1); //verifica constantemente si la variable meas1_done_sar en SENS.sar_meas_start1 indica que la conversión SAR1 ha finalizado.
	//eyendo un campo de registro de 32 bits desde el registro meas1_data_sar que es donde se obtiene el valor que tiene el canal a leer y devuelve el valor leído como un entero sin signo de 32 bits
	lectura = HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas_start2, meas2_data_sar);
	//Nos salimos de la configuracion para leer datos y valores del ADC
	NOS2; //SENS.sar_meas_wait2.force_xpd_sar = DOWNN;

	return lectura;

}

			//FUNCION QUE USA LA FUNCION "ADC_SECUENCIA_READ"
uint32_t ADC2_READ(adc_canal_t CANAL, adc_ancho_resolucion_t RESOLUCION, int NUM_MUESTRAS)
{
	adc2_canal_t ADC2_CANAL = 0; //Crea una variable de tipo estructura adc1_canal_t llamada ADC1_CANAL y le da un valor de 0
	uint32_t valor_adc = 0; // Crea una variable de tipo estructura uint32_t llamada valor_adc y le da el valor de 0
	ADC2_CANAL = CANAL;  // Da el valor que tengamos de CANAL en ADC1_READ y le da ese valor a la variable previamente creada llamada ADC1_CANAL
	printf("%d\n",ADC2_CANAL);

	for(int i = 0; i<NUM_MUESTRAS; i++)
	{
		valor_adc += ADC2_TOMAR_LECTURA(ADC2_CANAL, RESOLUCION); //Se va sumando los valores capturados por el ADC
	}
	valor_adc /= NUM_MUESTRAS;  //Se hace un promedio de los valores optenidos para ser mas precisos con el valor real

	return valor_adc; //Se retorna el valor promedio final optenido para que despues pueda ser impreso y vizualizado por el usuario
}

			//Configura el GPIO para usar el ADC1
esp_err_t Confi_PIN1(adc1_canal_t ADC_CANAL)    //Configura el pin y desactiva GPIO en el canal deseado
{
	pin_gpio_num_t numero_gpio = 0;
	numero_gpio = OBTENER_ADC1_GPIO_NUM_PIN(ADC_CANAL);
	rtc_gpio_init(numero_gpio);
	rtc_gpio_set_direction(numero_gpio, RTC_GPIO_MODE_DISABLED);
	rtc_gpio_pullup_dis(numero_gpio);
	rtc_gpio_pulldown_dis(numero_gpio);
	return 0;
}

//Configura el GPIO para usar el ADC2
esp_err_t Confi_PIN2(adc2_canal_t ADC_CANAL)
{
	pin_gpio_num_t numero_gpio = 0;
	numero_gpio = OBTENER_ADC2_GPIO_NUM_PIN(ADC_CANAL);
	rtc_gpio_init(numero_gpio);
	rtc_gpio_set_direction(numero_gpio, RTC_GPIO_MODE_DISABLED);
	rtc_gpio_pullup_dis(numero_gpio);
	rtc_gpio_pulldown_dis(numero_gpio);
	return 0;
}
