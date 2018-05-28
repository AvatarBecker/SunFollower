#include "msp430g2553.h"
#include "sun_follower.hpp"
#include <cstdlib>

/* --- VARIABLES ---*/
char    position_servo1_literal[]  =   "500";
char    position_servo2_literal[]  =   "400";

int     position_servo1_int          =   500;
int     position_servo2_int          =   400;

int     servo1_register_temp         =   3000;
int     servo2_register_temp         =   3000;

float   error[2]    =   { 0.0 };
float   error_integral[2]   =   { 0.0 };
float   angle_diff[2]       =   { 0.0 };

const   float   cSetPoint   =   0.0;

const   float   cKP[2]      =   {-20, -20};
const   float   cKI[2]      =   {-.5, -.8};
const   float   cKD[2]      =   {0, 150};

int     cursor_char    =   0;
int     cursor_servo   =   0;

int     measure_raw[2][2]       = {0};
float   measure_accumulated[2][2]   = {0.0};
float   measure_avg[2][2]   = {0.0};

float   measure_diff[2] = { 0.0, 0.0 };             // Diference between raw values (Digital measure of Sensor...)
float   measure_sum[2]  = { 0.0, 0.0 };             // Sum of raw values
float   measure_norm[2] = { 0.0, 0.0 };             // Nomalised difference

float   angle[2]     = { 0.0, 0.0 };
float   angle_int[2] = { 0.0, 0.0 };
float   angle_old[2] = { 0.0, 0.0 };

int     test_leds = 0;

const   int     n_samples       =   10;
//const   float   alpha           =   0.9811;
const   float   alpha[2]           =   {0.9, 0.96};
const   float   one_minus_alpha[2] =   {1-alpha[0], 1-alpha[1]};

int       do_it             =   1;
int       use_led           =   1;
int       send_measure      =   0;
float     angle_now         =   45.0;
unsigned  pacemaker         =   0;
unsigned  pacemaker_old     =   0;

/* --- FUNCTIONS --- */

void UpdateActuators(void)
{
    servo1_register_temp  = (int)( alpha[0]*(float)servo1_register_temp + one_minus_alpha[0]*(2*(float)position_servo1_int + 2000) );
    servo2_register_temp  = (int)( alpha[1]*(float)servo2_register_temp + one_minus_alpha[1]*(2*(float)position_servo2_int + 2000) );
    
//    TA1CTL  |=  TAIE;
    TA1CCTL0 |=   CCIE;
}

void    SendData(void)
{
    // Servo values
//    Print("M:\t");
//    Print( IntToChar(position_servo1_int) );
//    Print("\t");
//    Print( IntToChar(position_servo2_int) );
//    Print("\t");
//    Print("\n\r");
    
    // Read values
    //Print("S:\t");
//    Print("\n\r");
//    Print(FloatToChar(measure_avg[0][0]));
//    Print("\t");
//    Print(FloatToChar(measure_avg[0][1]));
//    Print("\t");

    Print(FloatToChar(measure_avg[1][0]));
    Print("\t");
    Print(FloatToChar(measure_avg[1][1]));
    Print(";");
    Print("\n\r");
    Print(FloatToChar(angle_now));
    Print("\t");
}

void SendReading()
{ 
//    Print(FloatToChar(angle[0]));
//    Print("\t");
//    Print(IntToChar(position_servo1_int));
//    Print(";");
//    Print("\t");
//    Print("\n\r");
    
    Print(FloatToChar(angle[1]));
    Print("\t");
    Print(IntToChar(position_servo2_int));
    Print(";");
    Print("\n\r");
}

/* --- MAIN FUNCTION --- */
int main()
{
    WDTCTL  = WDTPW + WDTHOLD;      // Stop watchdog timer to prevent time out reset

    ConfigureHardware();
    ConfigureADC();
    ConfigureCommunication();
    ConfigureTimers();

    _BIS_SR(GIE);
    
    //float dummy_float = 15.0;
    
    while(1)
    {
        if(do_it)
        {
            Print("\n\r");
            SayMyName();
            do_it = 0;
            
//            Print("\n\r");
//            Print(FloatToChar(angle_now));
//            Print("\t");  
        }
        
        measure_accumulated[0][0] =   0.0;
        measure_accumulated[0][1] =   0.0;
        measure_accumulated[1][0] =   0.0;
        measure_accumulated[1][1] =   0.0;
        
        measure_avg[0][0] =   0.0;
        measure_avg[0][1] =   0.0;
        measure_avg[1][0] =   0.0;
        measure_avg[1][1] =   0.0;
        
        for(int i=0; i<n_samples; i++)
        {
//            measure_raw[0][0] =  GetMeasure(0,0);
//            measure_raw[0][1] =  GetMeasure(0,1);
//            measure_raw[1][0] =  GetMeasure(1,0);
//            measure_raw[1][1] =  GetMeasure(1,1);
          
            measure_accumulated[0][0] +=  (float)GetMeasure(0,0);
            measure_accumulated[0][1] +=  (float)GetMeasure(0,1);
            measure_accumulated[1][0] +=  (float)GetMeasure(1,0);
            measure_accumulated[1][1] +=  (float)GetMeasure(1,1);
        }
        
        measure_avg[0][0] = measure_accumulated[0][0]/n_samples;
        measure_avg[0][1] = measure_accumulated[0][1]/n_samples;
        measure_avg[1][0] = measure_accumulated[1][0]/n_samples;
        measure_avg[1][1] = measure_accumulated[1][1]/n_samples;
        
        // Differences:
        measure_diff[0] =   measure_avg[0][1] - measure_avg[0][0];
        measure_diff[1] =   measure_avg[1][1] - measure_avg[1][0];
        
        // Sums:
        measure_sum[0] =   measure_avg[0][1] + measure_avg[0][0];
        measure_sum[1] =   measure_avg[1][1] + measure_avg[1][0];
        
        // Nomalised differences:
        measure_norm[0]   =   measure_diff[0]/measure_sum[0];
        measure_norm[1]   =   measure_diff[1]/measure_sum[1];
        
        angle[0]    =   Sigmoid(measure_norm[0],0);
        angle[1]    =   Sigmoid(measure_norm[1],1);
        
        // Computation for the control
        error[0]    =    cSetPoint - angle[0];
        error[1]    =    cSetPoint - angle[1];
        
        error_integral[0] += error[0];
        error_integral[1] += error[1];
        
        angle_diff[0]   =   angle[0] - angle_old[0];
        angle_diff[1]   =   angle[1] - angle_old[1];
        
        angle_old[0]    =   angle[0];
        angle_old[0]    =   angle[0];
        
        position_servo1_int = (int)( error[0]*cKP[0] + error_integral[0]*cKI[0] - cKD[0]*angle_diff[0] );
        position_servo2_int = (int)( error[1]*cKP[1] + error_integral[1]*cKI[1] );
        
        // Forced actuator boundaries
        if(position_servo1_int>999)
        {
            position_servo1_int = 999;
        }
        else if(position_servo1_int<0)
        {
            position_servo1_int = 0;
        }
        
        if(position_servo2_int>999)
        {
            position_servo2_int = 999;
        }
        else if(position_servo2_int<0)
        {
            position_servo2_int = 0;
        }
        
        // Update
        UpdateActuators();
        
        // Feedback for the user
        if( pacemaker > 10 )
        { 
            send_measure = 0;   
            pacemaker = 0;
        }
        
        SendReading();

        LPM0;   
    }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void Receive_ISR()
{
    LPM0_EXIT;
    
    if( cursor_servo == 0 )
    {
        cursor_servo = UCA0RXBUF - '0';
        
        if( UCA0RXBUF == 'l' )
        {
            P1OUT   ^=  BIT7;
            //use_led   = ~use_led;
        }
        else if( UCA0RXBUF == 's' )
        {
            send_measure = 1;
            angle_now += 5;
        }
        
    }
    else if( cursor_servo == 1 )
    {
        position_servo1_literal[ cursor_char ] = UCA0RXBUF;
        cursor_char++;
    }
    else if( cursor_servo == 2 )
    {
        position_servo2_literal[ cursor_char ] = UCA0RXBUF;
        cursor_char++;
    }
    else
    {
        cursor_char = 0;
        cursor_servo = 0;
    }
    
    if( cursor_char == 3 )
    {
        if( cursor_servo == 1 )
        {
            position_servo1_literal[cursor_char]   =   '\0';
            position_servo1_int =   CharToInt( position_servo1_literal );
        }
        else if( cursor_servo == 2 )
        {
            position_servo2_literal[cursor_char]   =   '\0';
            position_servo2_int =   CharToInt( position_servo2_literal );
        }

        cursor_char  = 0;
        cursor_servo = 0;
    }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0_ISR()
{
    LPM0_EXIT;
    pacemaker++;
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1_ISR()
{
	TA1CCR1 = servo1_register_temp;
	TA1CCR2 = servo2_register_temp;

    TA1CCTL0 &=  ~CCIE;
}