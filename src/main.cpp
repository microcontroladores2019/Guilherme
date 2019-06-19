/* Includes ------------------------------------------------------------------*/

// Must be included if using STM32F4 Discovery board or processor
#include "stm32f4xx.h"


#include "stm32f4_discovery.h"
#include "utils/commandline.h"
#include <usb/usb_stm32.h>
#include <usb/usb_device_class_audio.h>
#include <usb/usb_device_class_cdc_vcp.h>
#include <usb/usb_device_class_cdc_rndis.h>
#include <utils/io_pin_stm32.h>
#include <utils/serialnumber.h>
#include <utils/interrupt_stm32.h>

extern "C"{
#include "usb_dcd_int.h"
#include "usb_hcd_int.h"
}

// Must be included if using GPIO (General purpose I/O) peripheral
#include "stm32f4xx_gpio.h"

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// Must be included to setup timers
#include "stm32f4xx_tim.h"

// Must be included to setup the A/D
#include "stm32f4xx_adc.h"

extern CommandLine cmdline;

IO_Pin_STM32 USB_DP(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_11, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_OTG_FS);
IO_Pin_STM32 USB_DM(IO_Pin::IO_Pin_Mode_SPECIAL, GPIOA, GPIO_Pin_12, GPIO_PuPd_NOPULL, GPIO_OType_PP, GPIO_AF_OTG_FS);

uint8_t USB_DEVICE_CLASS::_numinterfaces=0;
uint8_t USB_DEVICE_CLASS::_numinendpoints=1;
uint8_t USB_DEVICE_CLASS::_numoutendpoints=1;
uint8_t USB_DEVICE_CLASS::_numdescriptorstrings=6;
std::list<USB_DEVICE_CLASS*> USB_DEVICE_CLASS::_usb_device_classes_list;

USB_DEVICE_CLASS_CDC_RNDIS usb_device_class_cdc_rndis(0);
USB_DEVICE_CLASS_CDC_VCP usb_device_class_cdc_vcp(1);
USB_DEVICE_CLASS_AUDIO usb_device_class_audio(0);

USB_STM32 usb(0x29BC, 0x0002, "IME", "Microcontroladores 2017", SerialNumberGetHexaString());

INTERRUPT_STM32 usb_otg_fs_interrupt(OTG_FS_IRQn, 0x0D, 0x0D, ENABLE);

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/**********************************************************************************
 *
 * The GPIO_InitTypeDef is a structure defined in the stm32f4xx_gpio.h file.
 * This is a simple way to use ST's library to configure the I/O.
 *
 * This is a common setup for the other peripherals as well.  Each peripheral will have
 * different properties which can be found in the header files and source files.
 *
**********************************************************************************/

GPIO_InitTypeDef  GPIO_InitStruct;


/**********************************************************************************
 *
 * The TIM_TimeBaseInitTypeDef is a structure defined in the stm32f4xx_tim.h file.
 *
**********************************************************************************/

TIM_TimeBaseInitTypeDef TIM_InitStruct;


/**********************************************************************************
 *
 * The ADC_InitTypeDef is a structure defined in the stm32f4xx_adc.h file.
 * The ADC_CommonInitTypeDef is a structure defined in the stm32f4xx_adc.h file.
 *
 **********************************************************************************/

ADC_InitTypeDef ADC_InitStruct;
ADC_CommonInitTypeDef ADC_CommonInitStruct;



int main(void)
{

	usb.Init();
        unsigned int _btn_count = 0;
        float TemperatureValue = 0;

    /**********************************************************************************
     *
     * This enables the peripheral clock to the GPIOD module.  This is stated in
     * the beginning of the stm32f4xx.gpio.c source file.
     *
    **********************************************************************************/

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /**********************************************************************************
     *
     * This block of code defines the properties of the GPIO port.
     * The different options of each item can be found in the stm32f4xx_gpio.h header file
     * Every GPIO configuration should have the following initialized
     *
     * GPIO_InitStruct.GPIO_Pin
     * GPIO_InitStruct.GPIO_Mode
     * GPIO_InitStruct.GPIO_Speed
     * GPIO_InitStruct.GPIO_OType
     * GPIO_InitStruct.GPIO_PuPd
     * GPIO_Init(GPIOx, &GPIO_InitStruct); (x represents port - A, B, C, D, E, F, G, H, I)
     *
    **********************************************************************************/

    //GPIO_InitStruct.GPIO_Pin configures the pins that will be used.
    //In this case we will use the LED's off of the discovery board which are on
    //PortD pins 12, 13, 14 and 15
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;

    //PIO_InitStruct.GPIO_Mode configures the pin mode the options are as follows
    // GPIO_Mode_IN (Input Mode)
    // GPIO_Mode_OUT (Output Mode)
    // GPIO_Mode_AF (Alternate Function)
    // GPIO_Mode_AN (Analog Mode)
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    //GPIO_InitStruct.GPIO_Speed configures the clock speed, options are as follows
    // GPIO_Speed_2MHz
    // GPIO_Speed_25MHz
    // GPIO_Speed_50MHz
    // GPIO_Speed_100MHz
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    //GPIO_InitStruct.GPIO_OType configures the pin type, options are as follows
    // GPIO_OType_PP (Push/Pull)
    // GPIO_OType_OD (Open Drain)
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

    //Configures pullup / pulldown resistors on pin, options are as follows
    // GPIO_PuPd_NOPULL (Disables internal pullup and pulldown resistors)
    // GPIO_PuPd_UP (Enables internal pullup resistors)
    // GPIO_PuPd_DOWN (Enables internal pulldown resistors)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    //This finally passes all the values to the GPIO_Init function
    //which takes care of setting the corresponding bits.
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /**********************************************************************************
     *
     * This enables the peripheral clock to the GPIOA module.  This is stated in
     * the beginning of the stm32f4xx.gpio.c source file.
     *
    **********************************************************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /**********************************************************************************
     *
     * This block of code defines the properties of the GPIOA port.
     * We are defining Pin 0 as a digital input with a pulldown resistor
     * to detect a high level.  Pin 0 is connected to the 3.3V source
     *
   **********************************************************************************/

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);


  /**********************************************************************************
  *
  * This enables the peripheral clock to the TIM2 (Timer 2) module.
  * The TIM2 module clock is at 84MHz.  The prescaler will be set
  * to 42000-1 which will configure the clock for 84MHz/42kHz = 2kHz
  * Then we will use the period to define as 2000-1.  This will trigger
  * an event every 1 second (or it should).
  *
  * Since we are using the STM32F4 Discovery board that has a 8MHz crystal
  * we need to adjust the following:
  *
  * 1)  Use the clock configuration tool to setup the system_stm32f4xx.c file
  *
  * 2) In stm32f4xx.h file
  *    a) Set the #define HSE_VALUE to the following
  *      #define HSE_VALUE    ((uint32_t)8000000)
  *
  * 3) In startup_stm32f4xx.c file
  *    a)  Uncomment the line
  *      extern void SystemInit(void);
  *
  *    b)  Add the following line under Default_Reset_Handler()
  *      just before the line of code that jumps to main();
  *      SystemInit();
  *
  * **********************************************************************************/

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_InitStruct.TIM_Prescaler = 42000 - 1;    // This will configure the clock to 2 kHz
 TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;  // Count-up timer mode
 TIM_InitStruct.TIM_Period = 2000 - 1;     // 2 kHz down to 1 Hz = 1 second
 TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;  // Divide clock by 1
 TIM_InitStruct.TIM_RepetitionCounter = 0;    // Set to 0, not used
 TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
 /* TIM2 enable counter */
 TIM_Cmd(TIM2, ENABLE);


  /**********************************************************************************
  *
  * This enables the A/D converter for sampling the on board
  * temperature sensor.
  *
  * You must first enable the CommonInitStructure
  * then enable the specific InitStructure for AD1, AD2 or AD3
  *
  * Review reference manual RM0090 for register details.
  *
  * 1) Deinitialize the ADC
  * 2) Set the ADC1 peripheral clock
  * 2) Set the Common Structure of ADC
  *   a) Mode = Configures the ADC to operate in independent or multi mode.
  *   b) Prescaler = Select the frequency of the clock to the ADC. The clock is common for all the ADCs.
  *       ADCCLK = PCLK2/Prescaler
  *   c) DMA = Configures the Direct memory access mode for multi ADC mode.
  *   d) Two Sampling Delay = Configures the Delay between 2 sampling phases.
  *         # * Time(ADCCLK) where # is between 5 and 20
  * 3) Configure The ADC Initialization Settings
  * 4) Configure the channels for for ADC1, internal temperature sensor is on channel 16
  * 5) Enable the internal temperature sensor
  * 6) Enable the A/D conversion for ADC1
  *
  * **********************************************************************************/

  ADC_DeInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
 ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div8;
 ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
 ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
 ADC_CommonInit(&ADC_CommonInitStruct);

  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStruct.ADC_ScanConvMode = DISABLE;
 ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
 ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
 ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
 ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStruct.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStruct);

  // ADC1 Configuration, ADC_Channel_TempSensor is actual channel 16
 ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1,
   ADC_SampleTime_144Cycles);

  // Enable internal temperature sensor
 ADC_TempSensorVrefintCmd(ENABLE);

  // Enable ADC conversion
 ADC_Cmd(ADC1, ENABLE);

    /**********************************************************************************
     *
     * This block of code blinks all four LED's on initial startup
     *
     * **********************************************************************************/
    GPIO_SetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    Delay(0xFFFFF);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

    CircularBuffer<uint8_t> buffer(0,1024);
    while(1)
    {


		uint16_t size=usb_device_class_cdc_vcp.GetData(buffer,256);
		if(size) {
			cmdline.In(buffer);
		}

		uint8_t buffer[32];
		do{
			size=cmdline.Out(buffer, 32);
			if(size){
				usb_device_class_cdc_vcp.SendData(buffer, size);
			}
		} while(size==32);

    	//Reads the status of the push-button on the Discovery board
        //If button is pressed blue LED is toggled

        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1)
        {
            _btn_count++;
                if (_btn_count > 0xFFFF)
                {
                    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
                    _btn_count = 0;
                }
        }

        //Monitors the TIM2 flag status.  If the TIM2 period is reached
        //The TIM_FLAG_Update = 1.  If this is true, we clear the flag
        //and toggle the green LED.

      if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
     {
       TIM_ClearFlag(TIM2, TIM_IT_Update);
       GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
     }


   // ADC Conversion to read temperature sensor
  // Temperature (in °C) = ((Vsense – V25) / Avg_Slope) + 25
  // Vense = Voltage Reading From Temperature Sensor
  // V25 = Voltage at 25°C, for STM32F407 = 0.76V
  // Avg_Slope = 2.5mV/°C
  // This data can be found in the STM32F407VF Data Sheet

   ADC_SoftwareStartConv(ADC1); //Start the conversion
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
   ; //Processing the conversion
  TemperatureValue = ADC_GetConversionValue(ADC1); //Return the converted data
  TemperatureValue *= 3300;
  TemperatureValue /= 0xfff; //Reading in mV
  TemperatureValue /= 1000.0; //Reading in Volts
  TemperatureValue -= 0.760; // Subtract the reference voltage at 25°C
  TemperatureValue /= .0025; // Divide by slope 2.5mV
  TemperatureValue += 25.0; // Add the 25°C

    } //End of while(1) loop
} //End of main loop

extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern "C" void OTG_FS_IRQHandler(void){
	USBD_OTG_ISR_Handler (&USB_OTG_dev);
	USBH_OTG_ISR_Handler (&USB_OTG_dev);
}

extern "C" void OTG_FS_WKUP_IRQHandler(void){
	if(USB_OTG_dev.cfg.low_power){
		*(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
		SystemInit();
		USB_OTG_UngateClock(&USB_OTG_dev);
	}
	EXTI_ClearITPendingBit(EXTI_Line18);
}


extern "C" void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
	/* TODO, implement your code here */
	return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
extern "C" uint16_t EVAL_AUDIO_GetSampleCallBack(void){
	/* TODO, implement your code here */
	return -1;
}

