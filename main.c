#include "MKM34Z5.h" 
#include "adc.h" 
#include "sim.h"
//#include "adc.c"
//#include "sim.c"

#define GREEN_LED   (1<<5)
#define GREEN_LEDON   (0<<5)
#define GREEN_ON  GPIOE_PDOR=GREEN_LEDON;
#define GREEN_LEDOFF  (1<<5)
#define GREEN_OFF  GPIOE_PDOR=GREEN_LEDOFF;

#define RED_LED   (1<<1)
#define RED_LEDON   (0<<1)
#define RED_ON  GPIOF_PDOR=RED_LEDON;
#define RED_LEDOFF  (1<<1)
#define RED_OFF  GPIOF_PDOR=RED_LEDOFF;

#define ORANGE_LED   (1<<1)
#define ORANGE_LEDON   (0<<1)
#define ORANGE_ON  GPIOD_PDOR=ORANGE_LEDON;
#define ORANGE_LEDOFF  (1<<1)
#define ORANGE_OFF  GPIOD_PDOR=ORANGE_LEDOFF;


#define YELLOW_LED   (1<<1)
#define YELLOW_LEDON   (0<<1)
#define YELLOW_ON  GPIOC_PDOR=YELLOW_LEDON;
#define YELLOW_LEDOFF  (1<<1)
#define YELLOW_OFF  GPIOC_PDOR=YELLOW_LEDOFF;

#define CUSTOM_LED   (1<<0)
#define CUSTOM_LEDON   (0<<0)
#define CUSTOM_ON  GPIOG_PDOR=CUSTOM_LEDON;
#define CUSTOM_LEDOFF  (1<<0)
#define CUSTOM_OFF  GPIOG_PDOR=CUSTOM_LEDOFF;




void delay1(int a);

//code execution begins here
int main(void)
{
            
         static volatile uint16 tmp16;
	static volatile uint16 tmp17=2500;

	SIM_Init (SIM_MODULE_ALL_PERIPH_ON_CONFIG); 
      ADC_Init (ADC_MODULE_16B_SWTRG_XREF_CONFIG ,
                    HWAVG_32,
                    ADC_CH_SE_POLL_CONFIG(AD10),
                    ADC_CH_DISABLE_CONFIG,
                    ADC_CH_DISABLE_CONFIG,
                    ADC_CH_DISABLE_CONFIG,
                    PRI_LVL0, NULL); 

		    delay1(10000);
     PORTE_PCR5= PORT_PCR_MUX(1); //PTE5 AS GPIO
      PORTF_PCR1= PORT_PCR_MUX(1); //PTF1 AS GPIO
      PORTD_PCR1= PORT_PCR_MUX(1); //PTD1 AS GPIO
      PORTC_PCR1= PORT_PCR_MUX(1); //PTC1 AS GPIO
     PORTG_PCR0= PORT_PCR_MUX(1); //PTG0 AS GPIO
      
      //PORTG_PCR1= PORT_PCR_MUX(0); //PTG1 AS ADC PORT
      GPIOE_PDDR|=(1<<5);  
      GPIOF_PDDR|=(1<<1);
      GPIOC_PDDR|=(1<<1);
      GPIOD_PDDR|=(1<<1);
      GPIOG_PDDR|=(1<<0);


	 

                   //GREEN_OFF
                   //RED_OFF
                   //YELLOW_OFF
                   //ORANGE_OFF
                   //delay1(1000000);
                   //RED_ON
	
	for(;;) 
	{
	   
	 
         // delay1(100000);
	 //GREEN_ON
	 //ORANGE_ON
	//delay1(100000);
	
	 if (ADC_Ready(CHA))
          {
          tmp16 = ADC_Read(CHA);
         ADC_Start(CHA,AD10);
          } 

	if(tmp16<2000)
        {
      delay1(50000);
	CUSTOM_ON
	//ORANGE_OFF
	 delay1(50000);
	CUSTOM_OFF
	//ORANGE_ON
        }
        if(tmp16>2000)
	{
	delay1(100000);	
	CUSTOM_ON
	//GREEN_OFF
	delay1(100000);
	CUSTOM_OFF
	//GREEN_ON	
        }
       
	}
	
	return 0;
}


void delay1(int a)
{
	while(a>=0)
		a--;
}





