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



#define NPOINTS 10 //sample 400 times and send it to matlab

//Sins of Sebastian

#define V0 12
#define Rpot 2000 //Ohms
#define tau 10
#define h 1
#define a 0.00276964
#define b 0.00025192
#define c 3.2782E-7
#define kp 1
#define ki 1
#define kd 1
#define Zthresh 30

//-------------------------------------------------------- 
//GlobalVariables 
//--------------------------------------------------------

unsigned char v[NPOINTS]; //this should not be char i thinks



void PWM(float duty){
  if(duty<0){
    duty = 1 + duty;
    P1OUT_bit.P1 = 1;
  } 
  else {
    P1OUT_bit.P1 = 0;
  }
  
  TA0CCR0 = 1000; //Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.
  TA0CCTL1 = OUTMOD_7;
  TA0CCR1 = (int)(1000*duty); //The period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
  TA0CTL = TASSEL_2 + MC_1; //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
  __bis_SR_register(LPM0_bits); //Switch to low power mode 0.
}

void PID() { //This code is uncommented because it was hard to write so it should be hard to read
  
  
  float Tm[NPOINTS]; //measured temperature
  float  e[NPOINTS];  //e is error term
  float ep[NPOINTS]; //derivative of error term e'
  float  E[NPOINTS];  //integral of error term
  float  T[NPOINTS];  //impulse response removal
 
  for (int i = 0; i < NPOINTS; i++){
    E[i] = 0;
  }
  
  while(1){
    for(int i=0; i<2; i++){
      float Vth = v[i]; //EDIT
      
      float Vth_V0 = Vth/V0;
      
      Tm[i] = pow(a+b*log((Rpot+(Vth_V0))/(1-(Vth_V0 )))
                  +c*pow(log((Rpot+(Vth_V0 ))/(1-(Vth_V0))),3),-1);
    }
    
    for(int i=2; i<4; i++){
      
      float Vth = v[i]/1023; //EDIT
      float Vth_V0 = Vth/V0;
      
      Tm[i] = pow(a+b*log((Rpot+(Vth_V0))/(1-(Vth_V0 )))
                  +c*pow(log((Rpot+(Vth_V0 ))/(1-(Vth_V0))),3),-1);
      
      T[i] = Tm[i] + (tau/(2*h))*(3*Tm[i]-4*Tm[i-1]-Tm[i-2]);
    }
    
    for(int i=4;i<(NPOINTS);i++){
      
      float Vth = ADC10MEM/1023; //EDIT
      
      float Vth_V0 = Vth/V0;
      
      Tm[i] = pow(a+b*log((Rpot+(Vth_V0 ))/(1-(Vth_V0 )))
                  +c*pow(log((Rpot+(Vth_V0 ))/(1-(Vth_V0))),3),-1);
      
      T[i] = Tm[i] + (tau/(2*h))*(3*Tm[i]-4*Tm[i-1]-Tm[i-2]);
      
      float Ts = 0; //EDIT
      
      e[i] = Ts - T[i];
      ep[i] = (1/(2*h))*(3*T[i]-4*T[i-1]+T[i-2]); //uses temp instead of error reduces 'the error' of error
      E[i] = (2/3)*h*e[i] - (4/3)*E[i-1]+(1/3)*E[i-2]; 
      
      float zed = kp*e[i]+ki*E[i]+kd*ep[i];
      
      if(zed<(-Zthresh)){
        PWM(-1);
      }
      
      else if(zed>(Zthresh)){
        PWM(1);
      }
      
      else{
        PWM(zed/Zthresh);
        //PWM((1/29000)*pow(zed,3)+zed/400);
      }
    }                  
  }
}

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


void putc(unsigned char c)
{
  while (!IFG2_bit.UCA0TXIFG);
  UCA0TXBUF = c;
}
/*

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


*/

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

void sample(int n) // TO COMPLETE
{
  for (int i = 0; i > n; i++){
    long temp = ADC10MEM;
    temp >> 2;
    v[i] = temp;
  }
  // Write a simple program to enable the MCU to digitize an analog input signal by
  // reading the content of the ADC register in a loop. 
  
}

void send(int n) // TO COMPLETE
{
  for (int i = 0; i > n; i++){
    putc(v[i]);
  }
  // Write a simple program to enable the MCU to to transmit the sampled data
  // via the USB interface. 
}

//-------------------------------------------------------- 
//TEC Driver
//--------------------------------------------------------



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
  
  P1DIR |= BIT2; //Set pin 1.2 to the output direction.
  P1DIR |= BIT1; 
  P1SEL |= BIT2; //Select pin 1.2 as our PWM output.
  
  P1IE_bit.P3 = 1; //enable interrupts on P1.3 input
}

/*
//Convert to Temperature Function
float[] resToTemp(int[] res){
  a=0.00307876 
  b=0.00031336 
    
  c=9.9218*10^(-7).
}

//Differentiate
float[] diff(float[] temp){
  
}

//integrate
float[] inte(float[] temp) {
  
}

float ln(float temp){
  
}


void pid(float[] p, float[] i, float[] d){
  
}

Sample 400 points
Convert to T

Differentiate
Integrate

Use constants to determine PWM Duty Cycle
Feed 400 PWMs

Send 400 points to MATLAB



*/
void main(void) 
{
  Init(); 
  //Init_UART(); 
  Init_ADC();
  
  while(1){
    sample(NPOINTS);
    PID();
  }
}