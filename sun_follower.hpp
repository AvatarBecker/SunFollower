#ifndef  __SUN_FOLLOWER_HPP__
#define  __SUN_FOLLOWER_HPP__

#include <math.h>

/* --- CONSTANTS --- */
const char cSayMyName[] = "SunFollower";

//const float   k[2][4] = 
//{ 
//  { 250.2567, -0.0393, 3.9761, -0.4429 },        // Parameters for Sigmoid of axis 0
//  { 38.6182, 0.1136, -0.2768, -11.2364 }         // ditto, axis 1
//};

const float   k[2][4] = 
{ 
  { 250.2567, 0.0607, 3.9761, -0.4429 },        // Parameters for Sigmoid of axis 0
  { 38.6182, 0.1136, -0.2768, -11.2364 }         // ditto, axis 1
};

const int power_of_10[] = {1, 10, 100, 1000, 10000};

const int max_array_size = 8;

/* --- FUNCTIONS --- */

void ConfigureHardware()
{
    //WDTCTL  = WDTPW + WDTHOLD;      // Stop watchdog timer to prevent time out reset

    DCOCTL  = CALDCO_16MHZ;         // Wir müssen schneller!!
    BCSCTL1 = CALBC1_16MHZ;         // Sonst verpassen wir den Zug!

//    DCOCTL  = CALDCO_1MHZ;
//    BCSCTL1 = CALBC1_1MHZ;

    P1DIR    = BIT7;
    P1OUT   &= ~(BIT7);

    P2DIR   |= BIT1 + BIT4;          // Signal outputs for the servos
    P2SEL   |= BIT1 + BIT4;
}

void ConfigureADC()
{
    ADC10CTL0   =   ADC10ON + SREF_1 + ADC10SHT_0 + REF2_5V + REFON;
    ADC10CTL1   |=   ADC10SSEL_3;
    ADC10AE0    |=   BIT0 + BIT6 + BIT4 + BIT5;
}

void ConfigureTimers()
{
    // TA1 -- PWM for both servos
    TA1CCR0  =   40000;           // 20ms period (or 50Hz)
    TA1CCR1  =   3000;            // 1.5ms duty (or 0 deg for the servo)
    TA1CCR2  =   3000;            // 2000 to 4000...
    TA1CCTL1 =   OUTMOD_3;        // set/reset
    TA1CCTL2 =   OUTMOD_3;        // ditto
    TA1CTL   =   TASSEL_2 + ID_3 + MC_1;

    // TA0  -- Assures a precise cycle time
    TA0CCR0 =   60000;            // 5 ms cycle time
    TACCTL0 |=  CCIE;
    TA0CTL  =   TASSEL_2 + ID_3 + MC_1;
}

void ConfigureCommunication()
{

    P1SEL       |=  BIT1 + BIT2;
    P1SEL2      |=  BIT1 + BIT2;
    UCA0CTL1    |=  UCSWRST;
    UCA0CTL1    |=  UCSSEL_2;

//    UCA0BR0     =   138;            // 115200 bps
//    UCA0BR1     =   138;
//    UCA0MCTL    =   14;

    UCA0BR1     =   6;            // 9600 bps
    UCA0BR0     =   130;
    UCA0MCTL    =   12;

    UCA0CTL1    &=  ~UCSWRST;

    UC0IE       |=   UCA0RXIE;
}

void SendByte(char data_byte)
{
    while( !( UC0IFG & UCA0TXIFG ) );
    UCA0TXBUF   =   data_byte;
}

void Print(char* data_array)
{
    while(*data_array)
    {
        SendByte(*data_array);
        data_array++;
    }
}

int NumberOfDigits(unsigned data_int)
{
    if( data_int <= 1 )
    {
        return 1;
    }
    else
    {
        int number_of_digits_temp = (int)floor( log10( float(data_int) ) ) + 1;
        return number_of_digits_temp;
    }
}

char* IntToChar(unsigned data_int)
{
    unsigned array_size = NumberOfDigits((unsigned)data_int);   // Gets final size of the array
    static char   data_char_temp[max_array_size] = { 0 };
    unsigned     data_int_temp       =   data_int;

    for(int i = 0; i<array_size; i++)
    {
        int remainder       =  data_int_temp%10;
        data_int_temp      -=  remainder;
        data_int_temp      /=  10;
        data_char_temp[array_size-1-i] =  (char)remainder + '0';
    }

    data_char_temp[array_size] =   '\0';

    return  data_char_temp;
}

char* FloatToChar(float data_float)
{
    static char   data_char_temp[max_array_size] = "";
    
    int is_negative = signbit(data_float) ? 1 : 0;
    
    float data_float_abs = fabs(data_float);
    
    int data_int_temp    =   (int)floor(data_float_abs);
    int data_dec_temp    =   (int)round( 100*(data_float_abs - (float)data_int_temp) );
  
    int array_size_int = NumberOfDigits((unsigned)data_int_temp) + is_negative;
    int array_size_dec = 2;
    
    int array_size  =   array_size_int + array_size_dec + 1; // Final size (with the point already)
    
//    Print("\n\r");
//    Print(IntToChar(array_size_int));
//    Print("\n\r");
//    Print(IntToChar(array_size_dec));
//    Print("\n\r");
//    Print(IntToChar(array_size));
//    Print("\n\r");
    
    for(int i = 0; i<array_size_int; i++)
    {
        int remainder       =  data_int_temp%10;
        data_int_temp      -=  remainder;
        data_int_temp      /=  10;
        data_char_temp[array_size_int-1-i] =  (char)remainder + '0';
    }
    
    if(is_negative)
    {
        data_char_temp[0] = '-';
    }
    
    data_char_temp[array_size_int] = '.';
    
    for(int i = 0; i<array_size_dec; i++)
    {
        int remainder       =  data_dec_temp%10;
        data_dec_temp      -=  remainder;
        data_dec_temp      /=  10;
        data_char_temp[array_size-1-i] =  (char)remainder + '0';
    }
    
    data_char_temp[array_size] = '\0';
    
    return data_char_temp;
}

int CharToInt(char *data_char)
{
    int data_int_temp = 0;
    int index_of_null = 0;
    
    for(int i = 0; i<max_array_size; i++)
    {
        if( data_char[i] == '\0' )
        {
            index_of_null = i;
            break;
        }
    }
    
    for(int i = 0; i<index_of_null; i++)
    {
        data_int_temp += (int)( ( data_char[i] - '0' )*power_of_10[ index_of_null - 1 - i ] );
    }
                            
    return  data_int_temp;
}

void SayMyName()
{
    for(int i = 0; i<sizeof(cSayMyName); i++) SendByte(cSayMyName[i]);
    Print("\n\r");
}

int   GetMeasure(int axis, int sensor)
{
    // Axis 0 is the one the PCB turns about; Seeing the logo upwards,
    // Sensors 0 are the one to the right and the one facing the logo.

    int   measure_temp = 0;

    if( axis == 0 )
    {
        if( sensor == 0)
        {
            ADC10CTL1   |=  INCH_0;
        }
        else
        {
            ADC10CTL1   |=  INCH_4;
        }
    }
    else
    {
        if( sensor == 0)
        {
            ADC10CTL1   |=  INCH_5;
        }
        else
        {
            ADC10CTL1   |=  INCH_6;
        }
    }

    ADC10CTL0   |=  ENC + ADC10SC;
    while( ADC10CTL1 & ADC10BUSY );
    measure_temp    =   (int)ADC10MEM;
    
    ADC10CTL0   &= ~(ENC + ADC10SC);
    ADC10CTL1   &= ~INCH_15;

    return  measure_temp;
}

float   Sigmoid( float x, int axis )
{
    float angle_temp = 0;
    float x_shifted = x + k[axis][1];
    
    angle_temp = k[axis][0]*x_shifted/( 1 + k[axis][2]*abs(x_shifted) ) + k[axis][3];
    
    return angle_temp;
}

#endif //__SUN_FOLLOWER_HPP__