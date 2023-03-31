//myscopemain
//2019.02.13-Kenrick Chin
//Modified by Mohammadreza Shahzadeh
//-----------------------------------------------------------------
//ReadMe
//-----------------------------------------------------------------
//myscope main.c code works with the MATLAB myscope.m and
//myscope.fig files under the following conditions:
//MSP430 is configured for 16MHz clock.
//UART is set for 115200baud,using the x16 clock mode.
//NPOINTS=400
//ADC input is on P1.4
//myscope.m mustbe changed to connect to the proper COMx port.
//-----------------------------------------------------------------


#include "io430.h"
#include "math.h"

#define ON 1
#define OFF 0
#define DELAY 20000
#define ASCII_CR 0x0D
#define ASCII_LF 0x0A
#define BUTTON P1IN_bit.P3

#define GREEN_LED P1OUT_bit.P0
#define RED_LED P1OUT_bit.P6

#define SAMPLEPOINTS 400 //

//--------------------------------------------------------
//GlobalVariables
//--------------------------------------------------------

unsigned char v[SAMPLEPOINTS];


//--------------------------------------------------------
//Miscellaneous Functions:

void delay (unsigned long d)
{
  while (d--);
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
  GREEN_LED = OFF;
  P1IFG_bit.P3 = 0;    // clear the interrupt request flag
}


//--------------------------------------------------------
//UART Module
//--------------------------------------------------------

void Init_UART(void)
{

  //initialize the USCI
  //RXD is on P1.1
  //TXD is on P1.2

  //configure P1.1and P1.2 for secondary peripheral function
  P1SEL_bit.P1 = 1;
  P1SEL2_bit.P1 = 1;
  P1SEL_bit.P2 = 1;
  P1SEL2_bit.P2 = 1;

  // divide by  104 for 9600b with  1MHz clock
  // divide by 1667 for 9600b with 16MHz clock
  // divide by  9 for 115200b with 16MHz clock

  UCA0BR1 = 0;
  UCA0BR0 = 9;

  // use x16 clock
  UCA0MCTL_bit.UCOS16 = 1;

  //select UART clock source
  UCA0CTL1_bit.UCSSEL1 = 1;
  UCA0CTL1_bit.UCSSEL0 = 0;

  //release UART RESET
  UCA0CTL1_bit.UCSWRST = 0;
}

unsigned char getc(void)
{
  while (!IFG2_bit.UCA0RXIFG);
  return (UCA0RXBUF);
}

void putc(unsigned char ch)
{
  while (!IFG2_bit.UCA0TXIFG);
  UCA0TXBUF = ch;
}

void puts(char *s)
{
  while (*s) putc(*s++);
}

void newline(void)
{
  putc(ASCII_CR);
  putc(ASCII_LF);
}

void itoa(unsigned int n)
{
  unsigned int i;
  char s[6] = "    0";
  i = 4;
  while (n)
  {
    s[i--] = (n % 10) + '0';
    n = n / 10;
  }
  puts(s);
}

void Send3Digit(float T) //for sending floats
{
  int temp = (int)(T*10); //send first 3 digits of the temperature

  putc( (char)(temp * 0.01) % 10); //tens column
  putc( (char)(temp *  0.1) % 10); //ones column
  putc( (char)temp         % 10); //tenths column
}

void Print3Digit(float T) //For TeraTerm testing
{
  int temp = (int)(T*10); //send first 3 digits of the temperature
  unsigned char d2 = (char)(temp * 0.01) % 10;
  unsigned char d1 = (char)(temp *  0.1) % 10;
  unsigned char d0 = (char)(temp) % 10;
  putc(d2 + '0'); //tens column
  putc(d1 + '0'); //ones column
  putc('.');
  putc(d0 + '0'); //tenths column
  newline();
}

//--------------------------------------------------------
//ADCModule
//--------------------------------------------------------
void Init_ADC(void) // TO COMPLETE
{
  // initialize 10-bit ADC using input channel 4 on P1.4
  // use Mode 2 - Repeat single channel

  ADC10CTL1 = INCH_4 + CONSEQ_2; // use P1.4 (channel 4)
  ADC10AE0 |= BIT4; // enable analog input channel 4

  //select sample-hold time, multisample conversion and turn
  ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON;

  // start ADC
  ADC10CTL0 |= ADC10SC + ENC;

}

void Sample(int n) // TO COMPLETE
{
  for (int i = n-1; i >=0; i--){
    int temp = ADC10MEM;
    temp = temp >> 2;
    v[i] = (unsigned char)temp;
  }
  // Write a simple program to enable the MCU to digitize an analog input signal by
  // reading the content of the ADC register in a loop.

}

void Send(int n) // TO COMPLETE
{
  for (int i = n-1; i >=0; i--){
    putc(v[i]);
  }
  // Write a simple program to enable the MCU to to transmit the sampled data
  // via the USB interface.
}
//--------------------------------------------------------
// PWM
//--------------------------------------------------------
void Init_PWM(void){
  P2DIR_bit.P5 = 1; //Set pin 2.7 to be the H-Bridge mode.
  P2OUT_bit.P5 = 0;

  P2DIR |= BIT1; //Set pin 2.1 to the output direction.
  P2SEL |= BIT1; //Select pin 2.1 as our PWM output.

  TA1CCR0 = 1000; //Set the period in the Timer A0 Capture/Compare 0register to 1000 us.
  TA1CCTL1 = OUTMOD_7;
  TA1CCR1 = 0; //The period in microseconds that the power is ON. It'shalf the time, which translates to a 50% duty cycle.
  TA1CTL = TASSEL_2 + MC_1; //TASSEL_2 selects SMCLK as the clock source,and MC_1 tells it to count up to the value in TA0CCR0.
}

void PWM_fun(float duty){
  if(duty<0){ //cooling
     //negative duty cycle + 1 is inverse of abs(negative duty cycle)
    P2OUT_bit.P5 = 0; //Cooling direction on H-Bridge
    duty = -duty; //remove negative sign
  }
  else {
    duty = 1-duty; //invert duty cycle
    P2OUT_bit.P5 = 1; //Heating direction on H-Bridge
  }

  TA1CCR1 = (int)(1000*duty); //The period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
}

//--------------------------------------------------------
//Initialization
//--------------------------------------------------------

void Init(void)
{
  //Stop watchdog timer to prevent timeout reset
  WDTCTL = WDTPW + WDTHOLD;

  DCOCTL = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;

  P1REN = 0x08; //enable output resistor
  P1OUT = 0x08; //enable P1.3 pullup resistor
  P1DIR = 0x41; //setup LEDs as output
  P1IE_bit.P3 = 1; //enable interrupts on P1.3 input
}

float byteToDuty(signed char byte){
  float pwm;
  pwm = byte/127.0;
  return pwm;
}

void main(){
  //initialization
  Init();
  Init_UART();
  Init_ADC();
  Init_PWM();
  Sample(SAMPLEPOINTS);

  while(1){

    getc();
    Send(SAMPLEPOINTS);

    signed char dutyByte = getc(); //convert to signed number
    float duty = byteToDuty(dutyByte); //convert to float
    putc(dutyByte); //return the byte for debug in MATLAB
    //PWM_fun(duty);
    PWM_fun(duty); //set duty cycle and H-Bridge mode
    Sample(SAMPLEPOINTS);

  }
}

