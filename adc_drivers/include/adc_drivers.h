/*
 *  09/12/2023
 * Archivo: adc_drivers.h
 *
 * Desarrollado por:
 * - Iving Iván Castillo Pérez 19060759
 *
 * Todos los derechos de autor reservados.
 * El mal uso de estos Drivers no es responsabilidad del desarrollador.
 */

#include <math.h>
#pragma once
#include "esp_err.h"
#define ESP_FAILure        -1

			//Definiciones de las direcciones de los registros
extern volatile uint32_t * const SENS_SAR; //0x3FF4882C
extern volatile uint32_t * const INV_ADC1CLK; //0x3FF48800
extern volatile uint32_t * const DAC_OFF; //0x3FF48910
extern volatile uint32_t * const INV_ADC2CLK; //0x3FF48890
extern volatile uint32_t * const DAC_DownR; //0x3FF4889E======
extern volatile uint32_t * const ON_DAC1; //0x3FF48484
extern volatile uint32_t * const ON_DAC2; //0x3FF48488
extern volatile uint32_t * const lee; //0x3FF4881c
extern volatile uint32_t * const READ_CTRL_REG; //0x3FF48800
extern volatile uint32_t * const START1_REG; //0x3FF48854

#define HAL_SWAP16(d) __builtin_bswap16((d))
#define HAL_SWAP32(d) __builtin_bswap32((d))
#define HAL_SWAP64(d) __builtin_bswap64((d))


#define HAL_FORCE_MODIFY_U32_REG_FIELD(base_reg, reg_field, field_val)    \
{                                                           \
    uint32_t temp_val = base_reg.val;                       \
    typeof(base_reg) temp_reg;                              \
    temp_reg.val = temp_val;                                \
    temp_reg.reg_field = (field_val);                       \
    (base_reg).val = temp_reg.val;                          \
}

/**
 * @brief Macro to force a 32-bit read on a peripheral register
 *
 * @note This macro should only be called on register fields of xxx_struct.h type headers. See description above for
 *       more details.
 * @note Current implementation reads into a uint32_t. See description above for more details.
 */
#define HAL_FORCE_READ_U32_REG_FIELD(base_reg, reg_field) ({    \
    uint32_t temp_val = base_reg.val;                       \
    typeof(base_reg) temp_reg;                              \
    temp_reg.val = temp_val;                                \
    temp_reg.reg_field;                                     \
})

					//Escritura directmanete en los registros

/* PAUSAR ADC */
#define PAUSAR_ADC(B)	   if(B==1) /* PAUSA EL ADC1 */ *SENS_SAR |= (int)pow((double)2,(double)23); \
						   if(B==2)  /* PAUSA EL ADC2 */ *SENS_SAR  |= (int)pow((double)2,(double)22);
/* RESOLUCION PARA ADC1 */
#define RESOLUCION_ADC1(B)	if(B==0) /* RESOLUCION 9 BITS */ *SENS_SAR 	  &= ~(int)pow((double)2,(double)0)+((int)pow((double)2,(double)1)); \
		                    if(B==1) /* RESOLUCION 10 BITS */ *SENS_SAR 	  |= (int)pow((double)2,(double)0); \
							if(B==2)  /* RESOLUCION 11 BITS */ *SENS_SAR  |= (int)pow((double)2,(double)1); \
							if(B==3)  /* RESOLUCION 12 BITS */ *SENS_SAR  |= (int)pow((double)2,(double)0)+((int)pow((double)2,(double)1));
/* RESOLUCION PARA ADC2 */
#define RESOLUCION_ADC2(B)	if(B==0) /* RESOLUCION 9 BITS */ *SENS_SAR 	  &= ~(int)pow((double)2,(double)2)+((int)pow((double)2,(double)3)); \
		                    if(B==1) /* RESOLUCION 10 BITS */ *SENS_SAR 	  |= (int)pow((double)2,(double)2); \
							if(B==2)  /* RESOLUCION 11 BITS */ *SENS_SAR  |= (int)pow((double)2,(double)3); \
							if(B==3)  /* RESOLUCION 12 BITS */ *SENS_SAR  |= (int)pow((double)2,(double)2)+((int)pow((double)2,(double)3));


#define DAC_Down(B)	if(B==0) /* Resolucion del canal;*/ *DAC_DownR &= ~ 15; \
						else if (B==1)   *DAC_DownR |= 15; //Apaga el DAC
/* Apaga DAC */		/* Enciende DAC Force */

#define TURN_OFF_DAC /* Selecciona cual DAC Apagar */ \
    *ON_DAC1 &= ~(((int)pow((double)2,(double)DacON))+((int)pow((double)2,(double)DacONF))+15); \
    *ON_DAC2 &= ~(((int)pow((double)2,(double)DacON))+((int)pow((double)2,(double)DacONF))+15);

#define Invert_SAR1(B)	if(B==1) /* Invert SAR ADC2 data*/ *INV_ADC1CLK |= (int)pow((double)2,(double)28);// 29???

#define Invert_SAR2(B)	if(B==1) /* Invert SAR ADC2 data*/ *INV_ADC2CLK |= (int)pow((double)2,(double)29);

#define CLK_DIV2(B)	if(B==2) /* Clock divider*/ *INV_ADC2CLK |= (int)pow((double)2,(double)1);

#define CLK_DIV1(B)	   if(B==2) /* Clock divider*/ *INV_ADC1CLK |= (int)pow((double)2,(double)1);

#define NOS1 /* Invert SAR ADC2 data*/ *lee |= (int)pow((double)2,(double)H);
#define NOS2 /* Invert SAR ADC2 data*/ *lee |= (int)pow((double)2,(double)H2);
//permitir seleccionar el control digital (RTC) en lugar del control analógico para el canal SAR1.
#define DIG_FORCE /* Permite seleccionar el modo de ADC*/ *READ_CTRL_REG &= ~ (int)pow((double)2,(double)27);
#define START_SAR /* Controla el incio del ADC*/ *START1_REG |= (int)pow((double)2,(double)18);
#define PAD_FORCE/*Control manual(Software) o control por procesador ULP.*/ *START1_REG |= (int)pow((double)2,(double)31);
#define EN_PAD(B)	   if(B<=12) /* Clock divider*/ *START1_REG |= ((1048576)*(B));



//Definiciones para los registros
#define DacONF 10
#define DacON 18
#define H 1835008	//PONE EN 1 (3 Byte) 2^17,18,19,20 - HOJA #1
#define H2 1048576


//Enumeraciones para los DRIVERS

typedef enum {
    ADC_MODULO_NUM_1 = 0,          /*!< SAR ADC 1 */
    ADC_MODULO_NUM_2 = 1,          /*!< SAR ADC 2 */
    ADC_MODULO_NUM_MAX,
} adc_ll_modulo_num_t;

typedef enum {
    ADC_ENERGIA_POR_FSM,   /*!< ADC XPD controlled by FSM. Used for polling mode */
    ADC_ENERGIA_SW_ON,    /*!< ADC XPD controlled by SW. power on. Used for DMA mode */
    ADC_ENERGIA_SW_OFF,   /*!< ADC XPD controlled by SW. power off. */
    ADC_ENERGIA_MAX,      /*!< For parameter check. */
} adc_ll_energia_t;

typedef enum {
    ADC_LL_CONTROL_RTC   = 0,    ///< For ADC1 and ADC2. Select RTC controller.
    ADC_LL_CONTROL_ULP   = 1,    ///< For ADC1 and ADC2. Select ULP controller.
    ADC_LL_CONTROL_DIG   = 2,    ///< For ADC1 and ADC2. Select DIG controller.
    ADC_LL_CONTROL_PWDET = 3,    ///< For ADC2. Select PWDET controller.
} adc_ll_controlador_t;

typedef enum {
    ADC_MODULO_1 = 1,          /*!< SAR ADC 1. */
    ADC_MODULO_2 = 2,          /*!< SAR ADC 2. */
	ADC_MAX_MODULO,
} adc_modulo_t;

typedef enum {
    ADC_9_BITS_RESOLUCION  = 0, /*!< ADC capture width is 9Bit. */
	ADC_10_BITS_RESOLUCION = 1, /*!< ADC capture width is 10Bit. */
	ADC_11_BITS_RESOLUCION = 2, /*!< ADC capture width is 11Bit. */
	ADC_12_BITS_RESOLUCION = 3, /*!< ADC capture width is 12Bit. */
	ADC_MAX_RESOLUCION,
} adc_ancho_resolucion_t;

typedef enum {
    ADC_ATENUACION_0_Db   = 0,  /*!<No input attenumation, ADC can measure up to approx. 800 mV. */
    ADC_ATENUACION_2_5_Db = 1,  /*!<The input voltage of ADC will be attenuated extending the range of measurement by about 2.5 dB (1.33 x) */
    ADC_ATENUACION_6_Db   = 2,  /*!<The input voltage of ADC will be attenuated extending the range of measurement by about 6 dB (2 x) */
    ADC_ATENUACION_11_Db  = 3,  /*!<The input voltage of ADC will be attenuated extending the range of measurement by about 11 dB (3.55 x) */
    ADC_ATENUACION_MAX,
} adc_atenuacion_t;

typedef enum {
	CANAL_0 = 0, 	 /*!< ADC channel */
	CANAL_1 = 1,     /*!< ADC channel */
	CANAL_2 = 2,     /*!< ADC channel */
	CANAL_3 = 3,     /*!< ADC channel */
	CANAL_4 = 4,     /*!< ADC channel */
	CANAL_5 = 5,     /*!< ADC channel */
	CANAL_6 = 6,     /*!< ADC channel */
	CANAL_7 = 7,     /*!< ADC channel */
	CANAL_8 = 8,     /*!< ADC channel */
	CANAL_9 = 9,     /*!< ADC channel */
    ADC_CANAL_MAX,
} adc_canal_t;

typedef enum {
	ADC1_CANAL_0 = 0,  	  /*!< ADC1 channel 0 is GPIO36 */
    ADC1_CANAL_1 = 1,     /*!< ADC1 channel 1 is GPIO37 */
    ADC1_CANAL_2 = 2,     /*!< ADC1 channel 2 is GPIO38 */
    ADC1_CANAL_3 = 3,     /*!< ADC1 channel 3 is GPIO39 */
    ADC1_CANAL_4 = 4,     /*!< ADC1 channel 4 is GPIO32 */
    ADC1_CANAL_5 = 5,     /*!< ADC1 channel 5 is GPIO33 */
    ADC1_CANAL_6 = 6,     /*!< ADC1 channel 6 is GPIO34 */
    ADC1_CANAL_7 = 7,     /*!< ADC1 channel 7 is GPIO35 */
    ADC1_CANAL_MAX,
} adc1_canal_t;

typedef enum {
    ADC2_CANAL_0 = 0,	  /*!< ADC2 channel 0 is GPIO4 */
    ADC2_CANAL_1 = 1,     /*!< ADC2 channel 1 is GPIO0 */
    ADC2_CANAL_2 = 2,     /*!< ADC2 channel 2 is GPIO2 */
    ADC2_CANAL_3 = 3,     /*!< ADC2 channel 3 is GPIO15 */
    ADC2_CANAL_4 = 4,     /*!< ADC2 channel 4 is GPIO13 */
    ADC2_CANAL_5 = 5,     /*!< ADC2 channel 5 is GPIO12 */
    ADC2_CANAL_6 = 6,     /*!< ADC2 channel 6 is GPIO14 */
    ADC2_CANAL_7 = 7,     /*!< ADC2 channel 7 is GPIO27 */
    ADC2_CANAL_8 = 8,     /*!< ADC2 channel 6 is GPIO25 */
    ADC2_CANAL_9 = 9,     /*!< ADC2 channel 7 is GPIO26 */
    ADC2_CANAL_MAX,
} adc2_canal_t;

typedef enum {
    GPIO_PIN_NUM_SIN_CONECCION = -1,    /*!< Use to signal not connected to S/W */
    GPIO_PIN_NUM_0 = 0,     /*!< GPIO0, input and output */
    GPIO_PIN_NUM_1 = 1,     /*!< GPIO1, input and output */
    GPIO_PIN_NUM_2 = 2,     /*!< GPIO2, input and output */
    GPIO_PIN_NUM_3 = 3,     /*!< GPIO3, input and output */
    GPIO_PIN_NUM_4 = 4,     /*!< GPIO4, input and output */
    GPIO_PIN_NUM_5 = 5,     /*!< GPIO5, input and output */
    GPIO_PIN_NUM_6 = 6,     /*!< GPIO6, input and output */
    GPIO_PIN_NUM_7 = 7,     /*!< GPIO7, input and output */
    GPIO_PIN_NUM_8 = 8,     /*!< GPIO8, input and output */
    GPIO_PIN_NUM_9 = 9,     /*!< GPIO9, input and output */
    GPIO_PIN_NUM_10 = 10,   /*!< GPIO10, input and output */
    GPIO_PIN_NUM_11 = 11,   /*!< GPIO11, input and output */
    GPIO_PIN_NUM_12 = 12,   /*!< GPIO12, input and output */
    GPIO_PIN_NUM_13 = 13,   /*!< GPIO13, input and output */
    GPIO_PIN_NUM_14 = 14,   /*!< GPIO14, input and output */
    GPIO_PIN_NUM_15 = 15,   /*!< GPIO15, input and output */
    GPIO_PIN_NUM_16 = 16,   /*!< GPIO16, input and output */
    GPIO_PIN_NUM_17 = 17,   /*!< GPIO17, input and output */
    GPIO_PIN_NUM_18 = 18,   /*!< GPIO18, input and output */
    GPIO_PIN_NUM_19 = 19,   /*!< GPIO19, input and output */
    GPIO_PIN_NUM_20 = 20,   /*!< GPIO20, input and output */
    GPIO_PIN_NUM_21 = 21,   /*!< GPIO21, input and output */
    GPIO_PIN_NUM_22 = 22,   /*!< GPIO22, input and output */
    GPIO_PIN_NUM_23 = 23,   /*!< GPIO23, input and output */
    GPIO_PIN_NUM_25 = 25,   /*!< GPIO25, input and output */
    GPIO_PIN_NUM_26 = 26,   /*!< GPIO26, input and output */
    GPIO_PIN_NUM_27 = 27,   /*!< GPIO27, input and output */
    GPIO_PIN_NUM_28 = 28,   /*!< GPIO28, input and output */
    GPIO_PIN_NUM_29 = 29,   /*!< GPIO29, input and output */
    GPIO_PIN_NUM_30 = 30,   /*!< GPIO30, input and output */
    GPIO_PIN_NUM_31 = 31,   /*!< GPIO31, input and output */
    GPIO_PIN_NUM_32 = 32,   /*!< GPIO32, input and output */
    GPIO_PIN_NUM_33 = 33,   /*!< GPIO33, input and output */
    GPIO_PIN_NUM_34 = 34,   /*!< GPIO34, input mode only */
    GPIO_PIN_NUM_35 = 35,   /*!< GPIO35, input mode only */
    GPIO_PIN_NUM_36 = 36,   /*!< GPIO36, input mode only */
    GPIO_PIN_NUM_37 = 37,   /*!< GPIO37, input mode only */
    GPIO_PIN_NUM_38 = 38,   /*!< GPIO38, input mode only */
    GPIO_PIN_NUM_39 = 39,   /*!< GPIO39, input mode only */
    GPIO_PIN_NUM_MAX,
} pin_gpio_num_t;


//RE-DEFINIONES

#define ADC1_GPIO36_CANAL     ADC1_CANAL_0
#define ADC1_CANAL_0_GPIO_NUMERO 36

#define ADC1_GPIO37_CANAL     ADC1_CANAL_1
#define ADC1_CANAL_1_GPIO_NUMERO 37

#define ADC1_GPIO38_CANAL     ADC1_CANAL_2
#define ADC1_CANAL_2_GPIO_NUMERO 38

#define ADC1_GPIO39_CANAL     ADC1_CANAL_3
#define ADC1_CANAL_3_GPIO_NUMERO 39

#define ADC1_GPIO32_CANAL     ADC1_CANAL_4
#define ADC1_CANAL_4_GPIO_NUMERO 32

#define ADC1_GPIO33_CANAL     ADC1_CANAL_5
#define ADC1_CANAL_5_GPIO_NUMERO 33

#define ADC1_GPIO34_CANAL     ADC1_CANAL_6
#define ADC1_CANAL_6_GPIO_NUMERO 34

#define ADC1_GPIO35_CANAL     ADC1_CANAL_7
#define ADC1_CANAL_7_GPIO_NUMERO 35

#define ADC2_GPIO4_CANAL      ADC2_CANAL_0
#define ADC2_CANAL_0_GPIO_NUMERO 4

#define ADC2_GPIO0_CANAL      ADC2_CANAL_1
#define ADC2_CANAL_1_GPIO_NUMERO 0

#define ADC2_GPIO2_CANAL      ADC2_CANAL_2
#define ADC2_CANAL_2_GPIO_NUMERO 2

#define ADC2_GPIO15_CANAL     ADC2_CANAL_3
#define ADC2_CANAL_3_GPIO_NUMERO 15

#define ADC2_GPIO13_CANAL     ADC2_CANAL_4
#define ADC2_CANAL_4_GPIO_NUMERO 13

#define ADC2_GPIO12_CANAL     ADC2_CANAL_5
#define ADC2_CANAL_5_GPIO_NUMERO 12

#define ADC2_GPIO14_CANAL     ADC2_CANAL_6
#define ADC2_CANAL_6_GPIO_NUMERO 14

#define ADC2_GPIO27_CANAL     ADC2_CANAL_7
#define ADC2_CANAL_7_GPIO_NUMERO 27

#define ADC2_GPIO25_CANAL     ADC2_CANAL_8
#define ADC2_CANAL_8_GPIO_NUMERO 25

#define ADC2_GPIO26_CANAL     ADC2_CANAL_9
#define ADC2_CANAL_9_GPIO_NUMERO 26

#define MAX_CANTIDAD_CANALES_ADC1 (8)
#define MAX_CANTIDAD_CANALES_ADC2 (10)

#define SOC_ADC1_INVERSION_DEFAULT_DATOS (1)
#define SOC_ADC2_INVERSION_DEFAULT_DATOS (1)

#define OBTENER_ADC1_GPIO_NUM_PIN(canal) (adc1_maping_canales_ent_sal[canal])
#define OBTENER_ADC2_GPIO_NUM_PIN(canal) (adc2_maping_canales_ent_sal[canal])

#define DOWNN 2 // Force power down
#define UPP  3 // Force power up


extern void INICIAR_CANALES_ADC(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_atenuacion_t ATENUACION, adc_canal_t CANAL_INICIO, int NUM_CANALES);

extern void INICIAR_CANAL_ADC(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_atenuacion_t ATENUACION, adc_canal_t CANAL);

extern esp_err_t ADC_SECUENCIA_READ(adc_modulo_t MODULO, adc_ancho_resolucion_t RESOLUCION,adc_canal_t CANAL_INICIO, int NUM_CANALES, uint32_t *lectura, int NUM_MUESTRAS);

extern uint32_t ADC1_READ(adc_canal_t CANAL, int NUM_MUESTRAS);

extern uint32_t ADC2_READ(adc_canal_t CANAL, adc_ancho_resolucion_t RESOLUCION, int NUM_MUESTRAS);

extern esp_err_t Confi_PIN1(adc1_canal_t ADC_CANAL);

extern esp_err_t Confi_PIN2(adc2_canal_t ADC_CANAL);
