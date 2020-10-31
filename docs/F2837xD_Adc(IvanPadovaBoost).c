//###########################################################################
//###########################################################################

// ***************** CURRENT LOOP TESTS

//###########################################################################
//###########################################################################

#pragma CODE_SECTION(adcd1_isr, ".TI.ramfunc");
#pragma CODE_SECTION(Conv_OFF, ".TI.ramfunc");
//#pragma CODE_SECTION(epwm4_isr, ".TI.ramfunc");

//---------------------------------------------------------------------------
// Included Files
//---------------------------------------------------------------------------
#include "F28x_Project.h"
#include "can.h"

#include "sfra_f32.h"
#include "math.h"

#include "Constants.h"
#include "pid_reg3.h"
#include "filter_notch.h"
#include "filtro_struct.h"

// test variables - ivan
int LPF_on=0;
int N_int=0;
int ctr_comp=4;
int PWM1_dir=0;     //1 - up, 2 - down
int PWM1_up=0;
int dir_change=0;
int cross_margin=7;             // below 7 there are still those vertical crossing problems...
Uint16 bit_cut=0;
float current_err_bin=0;      // 356 corresponds to 3.4766 A, and is divisible by 4 (last two bits are 0)



Uint16 pwm1TBCTR_increment=0;
Uint16 counter_pwm4=0;
Uint16 counter_pwm1=0;
Uint16 counter_pwm1max=0;
Uint16 counter_pwm1min=0;
Uint16 counter_pwmVirt=0;
Uint16 duty_test=0;
Uint16 size=3600;
//Uint16 store_duty[2000];
Uint16 store_iL[4000];
//Uint16 store_Vout[2000];
Uint16 CMPA_fast=0;
Uint16 CMPB_fast=TBPWM_SA+1;
double balance[5] = {1000.0, 2.0, 3.4, 7.0, 50.0};
//---------------------------------------------------------------------------
// Protection
//---------------------------------------------------------------------------
Uint16 stopxx = 0;
float LIMIT_Vin = 210;      // 210 V
float LIMIT_I = 20;         // 20 A
float LIMIT_Vout = 410;     // 410 V


//---------------------------------------------------------------------------
// Manual signals
//---------------------------------------------------------------------------
Uint16 start_output = 0;
Uint16 save_data = 0;
Uint16 soft_start = 0;
Uint16 start_uC[Total_uCNum] = {0, 0, 0, 0, 0};

char ADCoffCompF=0;
char ADCoffCompfinishF=0;

//---------------------------------------------------------------------------
// SFRA
//---------------------------------------------------------------------------
Uint16 start_SFRA = 0;
Uint16 SFRA_ope = 0;
Uint16 start_sweep = 0;
Uint32 counter = 0;
Uint16 update_amp = 0;
Uint16 SFRA_trigger = 0;
float sig_err = 0;
float sig_fdb = 0;

#if ADC_DOUBLE_SAMPLE
    #define SFRA_ISR_FREQ 40000
#else
    #define SFRA_ISR_FREQ 20000
#endif
#define SFRA_FREQ_START 10
//SFRA step Multiply = 10^(1/No of steps per decade(40))
#define SFRA_FREQ_STEP_MULTIPLY (float)1.2589254118
#define SFRA_FREQ_LENGTH 28

float SFRA_AMPLITUDE = 5;
SFRA_F32 sfra1;

// SFRA Variables storage
float32_t plantMagVect[SFRA_FREQ_LENGTH];
float32_t plantPhaseVect[SFRA_FREQ_LENGTH];
float32_t olMagVect[SFRA_FREQ_LENGTH];
float32_t olPhaseVect[SFRA_FREQ_LENGTH];
float32_t freqVect[SFRA_FREQ_LENGTH];

//extern to access tables in ROM
extern long FPUsinTable[];


//---------------------------------------------------------------------------
// Measurements
//---------------------------------------------------------------------------
float IL_O[2];
float IL;
float IIN_O[2];
float IIN;
float IO_O[2];
float IO;
float VOUT_O[2];
float VOUT;
float VIN_O[2];
float VIN;
float VC_LPF=0;           // for my lpf test



//---------------------------------------------------------------------------
// HRPWM
//---------------------------------------------------------------------------
Uint16 CMPA_reg_val = 0;
Uint16 CMP_fast = 0;
Uint16 CMPAHR_reg_val = 0;
Uint32 CMP_TEMP = 0;
extern int MEP_ScaleFactor;


//---------------------------------------------------------------------------
// Controllers and related variables
//---------------------------------------------------------------------------

//
// 0-->IV droop; 1-->VI droop; 2-->VI droop w/ low-pass filter
//
Uint16 TypeDroop = 1;

float duty_cycle = 0.95;
float Duty=0.5;
//ivan - for open loop duty changing
float Duty_manual=0.5;
float iLRef_manual=3.5;

float IOEstimate=0;
float V0 = 380;            // Voltage reference
float VREF1 = 380;
float VREF_LPF = 1.5;
float IREF1 = 0;
float IOREF1 = 0;
float Vd = 0;


// PI of RC voltage (test)
PI_REG PIvRC = PI_DEFAULTS;

//filter coefficients, for different N. Filter is first order LPF, with cutoff equal to switching freq - 20 kHz

structFiltro FilterLPFv = structFiltro_DEFAULT;  // Filter1 for voltage loop
structFiltro FilterLPFi = structFiltro_DEFAULT;  // Filter1 for current loop

float afilt,bfilt,cfilt,dfilt;


// PI of current loop
PI_REG PIi1 = PI_DEFAULTS;
float Kpi_double = 0.032504282376894;          // Double sampling, 2000 Hz, 60 degree
float Kii_double = 0.0004677319927178197;      // Double sampling
float Kpi_single = 0.034240059528030;          // Single sampling, 2000 Hz, 50 degree
float Kii_single = 0.001594722695840;          // Single sampling



// *********************** CURRENT CONTROLLER GAINS IVAN **********************************************
/*
// my gains IVAN - low bandwidth loop
float kpi_const =  0.010588403435013;           //values for 500Hz crossover, 50 degree at single sampling
float kii_const =  16.196880500472990;
*/

/*
// my gains IVAN - high bandwidth loop          // Single sampling, 2000 Hz, 50 degree
float kpi_const =  0.034240059528030;
float kii_const =  31.894453916800000;
*/

int high_bw = 0;                               // 1 for 2 kHz cross over freq, 0 for 500 Hz crossover
float kpi_const,kii;


// ********************** END OF CONTROLLER GAINS IVAN  ****************************************


float Iinit = 0.5;

/* Impedance measurement
// PI of voltage loop
PI_REG PIv1 = PI_DEFAULTS;
float Kpv_double = 0.600139443738351;          // Double sampling, 600 Hz, 60 degree
float Kiv_double = 0.007995284412373;          // Double sampling
float Kpv_single = 0.751230420275460;          // Single sampling, 550 Hz, 60 degree *1.2
float Kiv_single = 0.003866820979152;          // Single sampling
float Imax = 15;
float Imin = -5;

float Kd = 2.53;                               // Droop coefficient, =20/3000*(380-20)
structFiltro FilterD1 = structFiltro_DEFAULT;  // Filter1 for droop loop
float FD1a = 0.019050797031570;
float FD1b = -0.006044401768844;
float FD1c = 1;
float FD1d = -0.994865896606819;
structFiltro FilterD2 = structFiltro_DEFAULT;  // Filter2 for droop loop
float FD2a = 0.006503197631363;
float FD2b = 0.006503197631363;
float FD2c = 1;
float FD2d = -0.994865896606819;*/


// PI of voltage loop
PI_REG PIv1 = PI_DEFAULTS;
float Kpv_double = 0.600139443738351;          // Double sampling, 600 Hz, 60 degree
float Kiv_double = 0.007995284412373;          // Double sampling
float Kpv_single = 0.863914983316779;          // Single sampling, 550 Hz, 60 degree *1.2
float Kiv_single = 0.004369507706442;          // Single sampling
float kpv_const = 0.863914983316779;
float kiv_const = 87.390154128840010;
float Imax = 15;
float Imin = -5;

float Kd = 2.53;                               // Droop coefficient, =20/3000*(380-20)
structFiltro FilterD1 = structFiltro_DEFAULT;  // Filter1 for droop loop
float FD1a = 0.019050797031570;
float FD1b = -0.006044401768844;
float FD1c = 1;
float FD1d = -0.994865896606819;
structFiltro FilterD2 = structFiltro_DEFAULT;  // Filter2 for droop loop
float FD2a = 0.006503197631363;
float FD2b = 0.006503197631363;
float FD2c = 1;
float FD2d = -0.994865896606819;


//---------------------------------------------------------------------------
// CAN
//---------------------------------------------------------------------------
Uint16 Variable_index;
unsigned char ucTXMsgData[MESSAGE_SIZE] = {0, 0, 0, 0, 0};   // TX Data
unsigned char ucRXMsgData[MESSAGE_SIZE] = {0x25, 0x34, 0x56, 0x78, 0x12};   // RX Data
float CanRXVariables[CANVarNum] = {2.3, 1, -112.4, 7.8};
float CanTXVariables[CANVarNum] = {311, 11.2, -26.7, 0.5};
extern unsigned char canCommands[][MESSAGE_SIZE];

// Flags
Uint16 MaxResponseTimes = 3;
Uint16 ON_flag[Total_uCNum] = {0, 0, 0, 0, 0};
Uint16 OFF_flag[Total_uCNum] = {0, 0, 0, 0, 0};
Uint16 Request_flag[Total_uCNum][CANVarNum] = {{0, 0, 0, 0},
                                               {0, 0, 0, 0},
                                               {0, 0, 0, 0},
                                               {0, 0, 0, 0},
                                               {0, 0, 0, 0}};
Uint16 Set_flag[Total_uCNum][CANVarNum] = {{0, 0, 0, 0},
                                           {0, 0, 0, 0},
                                           {0, 0, 0, 0},
                                           {0, 0, 0, 0},
                                           {0, 0, 0, 0}};

// DER converter's states
//
// 0: DER MCU hasn't been powered up yet
// 1: DER MCU is powered up, and PWM is off
// 2: DER MCU is powered up, and PWM is on
// 3: DER converter is not available
//
Uint16 uC_state[Total_uCNum] = {0, 0, 0, 0, 0};


//---------------------------------------------------------------------------
// Other variables
//---------------------------------------------------------------------------
float tempf = 0;
Uint16 for_index = 0;



interrupt void adcd1_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO10 = 1;

    ADCoffCompF = 1;

    //============================================================================
    //                      Read ADC results
    //============================================================================
        if (ADCoffCompfinishF)
        {
            IL_O[0] = AdcaResultRegs.ADCPPB1RESULT.bit.PPBRESULT-32768.0;
            IL = IL_O[0]*0.009765625;               //  (x/4096*3 - 1.5)/3*40;              PPBRESULT will offset readings for 4096/2 = 1.5 V, because current measurements have 1.5V offset!

            IIN_O[0] = AdcbResultRegs.ADCPPB1RESULT.bit.PPBRESULT-32768.0;
            IIN = IIN_O[0]*0.009765625;                   // (x/4096*3 - 1.5)/3*40;

            IO_O[0] = AdccResultRegs.ADCPPB1RESULT.bit.PPBRESULT-32768.0;
            IO = IO_O[0]*0.009765625;                     // -(x/4096*3 - 1.5)/3*40;

            #if(ADCD_SIGNALMODE_DIFFERENTIAL)
                {
            //        VOUT_O = AdcdResultRegs.ADCRESULT0;                      // 16BIT ADC
            //        VOUT = VOUT_O*0.001674745141006 + 325.1219512195122;
            //
            //        VIN_O = AdcdResultRegs.ADCRESULT1;                       // 16BIT ADC
            //        VIN = VIN_O*0.001127661728277 + 163.8319130179254;
                }
            #else


            VIN_O[0] = 1.0*AdcdResultRegs.ADCRESULT2-AdcdResultRegs.ADCRESULT3;                      // 12BIT ADC
            VIN = VIN_O[0]*0.009021293826 + 200;              // x/4096*3*258 + 200;

            #if(currentUC==1)
                 VOUT_O[0] = 1.0*AdcdResultRegs.ADCRESULT0-AdcdResultRegs.ADCRESULT1;                     // 12BIT ADC
                 VOUT = VOUT_O[0]*0.013810388820073 + 379.9533597800944;        // x/4096*3*258 + 380;
            #endif
            #if(currentUC==2)
                 VOUT_O[0] = 1.0*AdcdResultRegs.ADCRESULT0-AdcdResultRegs.ADCRESULT1;                     // 12BIT ADC
                 VOUT = VOUT_O[0]*0.013800240272253 + 380.4381594140814;       // x/4096*3*258 + 380;
            #endif
            #if(currentUC==3)
                 VOUT_O[0] = 1.0*AdcdResultRegs.ADCRESULT0-AdcdResultRegs.ADCRESULT1;                     // 12BIT ADC
                 VOUT = VOUT_O[0]*0.013757463838157+ 379.5220150769027;       // x/4096*3*258 + 380;
            #endif
          #endif

          //low pass filtering, output voltage
//        FilterLPFv.uk=VOUT;
//        FilterLPFv.calc(&FilterLPFv);
          //low pass filtering, inductor current
          FilterLPFi.uk=IL;
          FilterLPFi.calc(&FilterLPFi);

          if(LPF_on)
          {
//              VOUT=FilterLPFv.yk;
              IL=FilterLPFi.yk;
          }
        }


        IOEstimate=0.5*VIN*(IIN+IL)/VOUT;

        // Update CAN arrays
        CanTXVariables[0] = VOUT;
        CanTXVariables[1] = IL;

    //============================================================================
    //                      Calculation and PWM update
    //============================================================================

        if (start_output==0 && stopxx==0)
        {
            if(VOUT>345) {                           //slave
                soft_start = 1;
                duty_cycle  = 0.499999;
                PIi1.ui_reg = 1-(VIN/VOUT);
            } else {                                //master
                soft_start = 0;
                PIi1.reset(&PIi1);
                duty_cycle  = 0.95;
            }
            CMPB_fast=TBPWM_SA+1;
            CMPA_fast=0;
        }


        if((start_output==1) && (stopxx==0))
        {
            // PWM output enable
            EALLOW;
            EPwm1Regs.TZCLR.bit.OST = 1;
            EDIS;


            #if (!OpenLoop)
            //====================================================================
            //
            // Closed-loop operation
            //
            //====================================================================
            if((soft_start == 0))
            {

                if( VOUT < (V0-2) ) {
                    duty_cycle -= 0.000005;
                } else if( VOUT> (V0+2) ) {
                    duty_cycle += 0.000005;
                } else {
                    soft_start = 1;
                    VREF1 = V0;

                    // Set initial state for Integrators
                    PIv1.ui_reg = IL;
                    PIi1.ui_reg = 1-duty_cycle;
                }

                if(duty_cycle < 0.02)
                {
                    duty_cycle = 0.02;
                } else if(duty_cycle > 0.98) {
                    duty_cycle = 0.98;
                }

            }
            else
            {

//                PIv1.e_reg = VREF1 - VOUT;
//                PIv1.calc(&PIv1);
                //PIi1.e_reg = PIv1.pi_out - IL;             // voltage loop active
                PIi1.e_reg = iLRef_manual - IL;              // here I can just set current reference, for only current loop active
                PIi1.calc(&PIi1);

                duty_cycle = 1-PIi1.pi_out;
            }


            #else
            //====================================================================
            //
            // For open loop test
            //
            //====================================================================

            //Duty=0.45;

            if (soft_start == 0) {
                if (duty_cycle > (1 - Duty)) {
                    duty_cycle -= 0.000005;
                } else {
                    duty_cycle = 1 - Duty;
                    soft_start = 1;
                }
            }else {
                duty_cycle = 1 - Duty;
            }
            #endif

            //calculate compare value
            CMP_fast = (Uint16) (duty_cycle * TBPWM);

            // MULTISAMPLING CARRIER HANDLING

            #if (MULTISAMPLING || ADC_DOUBLE_SAMPLE)
                        //multisampling handling
                        N_int++;
                        if(N_int>Ntot)
                            N_int=1;

                        if(N_int>Ntot/2)
                            PWM1_dir=2;
                        else
                            PWM1_dir=1;

                        //CMPA_fast is shadow register, used to clear PWM1 signal
                        if(PWM1_dir==1) //be careful to separate cases for last interupts before direction change
                        {
                            CMPB_fast=TBPWM_SA+1; //never set pwm on up count (except for the last interrupt in up count mode);

                            if (N_int!=Ntot/2)              //not one before starting to count down
                            {
                                counter_pwm1max=(N_int)*(TBPWM_SA+1)+TBPWM_SA;           //Virtual pwm counter at the end of the next interrupt
                                counter_pwm1min=(N_int)*(TBPWM_SA+1);                    //Virtual pwm counter at the begining of the next interrupt

                                if((CMP_fast>=counter_pwm1min+cross_margin))             // && (CMP_fast<=counter_pwm1max) I do not need, because CMPA_fast will be bigger than TBPWM_SA, so it will never trigger
                                    CMPA_fast=CMP_fast-counter_pwm1min;                  //we are always watching for next interrupt, because of the shadow register
                                else                                                     //vertical cross handling with some cross margin to ensure cmp loading before compare
                                    CMPA_fast=cross_margin;
                            }
                            else
                            {
                                CMPA_fast=TBPWM_SA+1;                         //do not allow clear on down count
                                CMPB_fast=TBPWM-CMP_fast;                     //there can be no vertical crossing here, due to the duty cycle upper limitation
                            }
                        }
                        else                                                  //CMPB_fast is written in shadow register, and used to clear PWM1 signal
                        {
                            CMPA_fast=TBPWM_SA+1;                             //never clear pwm on up count (except for the last interrupt in up count mode);

                            if (N_int!=Ntot)                                  //not one before starting to count up
                            {
                                counter_pwm1max=TBPWM-(N_int-Ntot/2)*(TBPWM_SA+1);              //Virtual pwm counter at the beginning of the next interrupt
                                counter_pwm1min=TBPWM-(N_int-Ntot/2)*(TBPWM_SA+1)-TBPWM_SA;     //Virtual pwm counter at the end of the next interrupt

                                if((CMP_fast<=counter_pwm1max-cross_margin))
                                    CMPB_fast=counter_pwm1max-CMP_fast;       //we are always watching for next interrupt, because of the shadow register
                                else                                          //vertical cross handling with some cross margin to ensure cmp loading before compare
                                    CMPB_fast=cross_margin;
                            }
                            else
                            {
                                CMPB_fast=TBPWM_SA+1;                         // do not allow set on up count
                                CMPA_fast=CMP_fast;                           //we are always watching for next interrupt, because of the shadow register
                            }
                        }

                        //update shadow registers
                        EPwm1Regs.CMPA.bit.CMPA = CMPA_fast;
                        EPwm1Regs.CMPB.bit.CMPB = CMPB_fast;

            #elif (!ADC_DOUBLE_SAMPLE)       //single sampling
                        EPwm1Regs.CMPA.bit.CMPA=CMP_fast;
                        EPwm1Regs.CMPB.bit.CMPB=TBPWM_SA+1-CMP_fast;
            #endif
        }
        else        //controller off
        {
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST = 1;
            EDIS;
            start_output=0;
            PIv1.reset(&PIv1);
        }


//============================================================================
//                      DAC output
//============================================================================

//    tempf = VOUT_O;
//    if(tempf>4096) {
//        tempf = 4095;
//    } else if(tempf<0) {
//        tempf = 0;
//    }
//    DacaRegs.DACVALS.all = (Uint16)(tempf);


//============================================================================
//                      Protection
//============================================================================
     if(VOUT > LIMIT_Vout)
     {
        stopxx = 1;
     }
     if(IL > LIMIT_I  || IL < -LIMIT_I)
     {
        stopxx = 2;
     }
     if(VIN > LIMIT_Vin)
     {
        stopxx = 3;
     }


//=============================================================================
//                 DATA ACQUISITION
//=============================================================================

     if(save_data)
     {
         if(counter_pwm4<4000)
         {
             //store_duty[counter_pwm4] = CMP_fast;              // this is actually Dprim
//             store_Vout[counter_pwm4] = (Uint16) (VOUT_O[0]+200);
             store_iL[counter_pwm4++] = (Uint16) IL_O[0];      // in case of LPF_ON, I still want to store just the acquired current
         }
     }
     else
     {
         counter_pwm4=0;
     }


//=============================================================================
//              clear registers
//=============================================================================

    DacaRegs.DACVALS.all=0;
    // Clear INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
}





void Conv_OFF(Uint16 Conv_num)
{

    Uint16 index = 0;
    for (index = 0; index < CANVarNum; index++)
    {
        Request_flag[Conv_num][index] = 0;    // clear all flags
        Set_flag[Conv_num][index] = 0;
    }

    ON_flag[Conv_num] = 0;
    OFF_flag[Conv_num] = 0;

    uC_state[Conv_num] = 3;     // DER in no-availability state
    start_uC[Conv_num] = 0;     // open switch

}



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//

//                               Initialization functions

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void InitADC(void)
{


    if (high_bw)
    {
        kpi_const = 0.03424006;                                  // IVAN float 32 can have 23 bits for significant bits (after 0.00... ) so I will put numbers in that form now
    }
    else
    {
        kpi_const = 0.010588403;
    }

    // integral gains and LPF coefficients for noise + multisampling
    switch (Ntot)
    {
    case 1:
        //N=1;

        if (high_bw)
        {
            kii = 0.0015921711;
        }
        else
        {
            kii = 0.0008085483;
        }


        afilt=0.758546992994776;
        bfilt=0.758546992994776;
        cfilt=1;
        dfilt=0.517093985989552;

        break;
    case 2:
        //N=2;

        if (high_bw)
        {
            kii = 0.0007960856;
        }
        else
        {
            kii = 0.00040427414;
        }

        afilt=0.611015470351657;
        bfilt=0.611015470351657;
        cfilt=1;
        dfilt=0.222030940703315;

        break;
    case 4:
        //N=4;

        if (high_bw)
        {
            kii = 0.0003980428;
        }
        else
        {
            kii = 0.00020213707;
        }


        afilt=0.439900846488443;
        bfilt=0.439900846488443;
        cfilt=1;
        dfilt=-0.120198307023115;

        break;
    case 8:
        //N=8;

        if (high_bw)
        {
            kii = 0.0001990214;
        }
        else
        {
            kii = 0.00010106853;
        }

        afilt=0.281969800123466;
        bfilt=0.281969800123466;
        cfilt=1;
        dfilt=-0.436060399753068;

        break;
    case 16:
        //N=16;

        if (high_bw)
        {
            kii = 0.0000995107;
        }
        else
        {
            kii = 0.00005053427;
        }

        afilt = 0.1641239;
        bfilt = 0.1641239;
        cfilt = 1;
        dfilt = -0.6717522;
        break;
    }


    //---------------------------------------------------------------------------
    // Adding my PI for BOOST CONVERTER, constant gains, with change of multisampling ratio
    //---------------------------------------------------------------------------
    PIi1.Kp_reg = kpi_const;
    PIi1.Ki_reg = kii;
    PIi1.pi_out_max = 0.975;
    PIi1.pi_out_min = 0.025;

    PIv1.Kp_reg = kpv_const;
    PIv1.Ki_reg = kiv_const*Tint;
    PIv1.pi_out_max = Imax;
    PIv1.pi_out_min = Imin;

    // -------------------------------------------------------------------------------
    /* END OF MY GAINS  */
    // -------------------------------------------------------------------------------


    FilterLPFv.a=afilt;
    FilterLPFv.b=bfilt;
    FilterLPFv.c=cfilt;
    FilterLPFv.d=dfilt;

    FilterLPFi.a=afilt;
    FilterLPFi.b=bfilt;
    FilterLPFi.c=cfilt;
    FilterLPFi.d=dfilt;

    FilterLPFv.reset(&FilterLPFv);
    FilterLPFi.reset(&FilterLPFi);


    InitControl();  // SFRA initialization


    //---------------------------------------------------------------------------
    // ADC Initialization
    //---------------------------------------------------------------------------
    ConfigureADC();

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;

    // SetupADC_A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;                  //SOC0 will convert pin A3, IL
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;                //trigger on EPWM1_SOCA

    // SetupADC_B
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;                  //SOC0 will convert pin B2, Iin
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

    // SetupADC_C
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 3;                  //SOC0 will convert pin C3, Io
    AdccRegs.ADCSOC0CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

//    AdccRegs.ADCSOC1CTL.bit.CHSEL = 2;                  //SOC1 will convert pin C2
//    AdccRegs.ADCSOC1CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
//    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 1;                //trigger on CPUTimer0

    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;          // SOC0 is high priority

    // SetupADC_D
    #if(!ADCD_SIGNALMODE_DIFFERENTIAL)

        AdcdRegs.ADCSOC0CTL.bit.CHSEL = 2;                  //SOC0 will convert pin D2
        AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
        AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

        AdcdRegs.ADCSOC1CTL.bit.CHSEL = 3;                  //SOC1 will convert pin D3
        AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
        AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

        AdcdRegs.ADCSOC2CTL.bit.CHSEL = 0;                  //SOC2 will convert pin D2
        AdcdRegs.ADCSOC2CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
        AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

        AdcdRegs.ADCSOC3CTL.bit.CHSEL = 1;                  //SOC3 will convert pin D3
        AdcdRegs.ADCSOC3CTL.bit.ACQPS = ADCACQ_12Bit;       //sample window definition
        AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 5;             //trigger on EPWM1_SOCA

        AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;          // SOC0-SOC2 are high priority

        AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 3;              //end of SOC3 will set INT1 flag
        AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;                //enable INT1 flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //make sure INT1 flag is cleared

     #else

        AdcdRegs.ADCSOC0CTL.bit.CHSEL = 2;                  //SOC0 will convert pin D2, D3, Vout
        AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADCACQ_16Bit;       //sample window definition
        AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;                //trigger on EPWM4_SOCA

        AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0;                  //SOC1 will convert pin D0, D1, Vin
        AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADCACQ_16Bit;       //sample window definition
        AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;                //trigger on EPWM4_SOCA

        AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;          // SOC0 is high priority

        AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1;              //end of SOC1 will set INT1 flag
        AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;                //enable INT1 flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //make sure INT1 flag is cleared

    #endif


    PieVectTable.ADCD1_INT = &adcd1_isr;
    EDIS;

    IER |= M_INT1; //Enable group 1 interrupts

    //enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    // Clear INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//

//                                ADC configuration

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    // Set to 12 bit mode
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    if(ADCD_SIGNALMODE_DIFFERENTIAL) {
        AdcSetMode(ADC_ADCD, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_DIFFERENTIAL);
    } else {
        AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    }


    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
//
// AdcSetMode - Set the resolution and signalmode for a given ADC. This will
//              ensure that the correct trim is loaded.
//
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void AdcSetMode(Uint16 adc, Uint16 resolution, Uint16 signalmode)
{
    Uint16 adcOffsetTrimOTPIndex; //index into OTP table of ADC offset trims
    Uint16 adcOffsetTrim;         //temporary ADC offset trim

    //
    //re-populate INL trim
    //
    CalAdcINL(adc);

    if(0xFFFF != *((Uint16*)GetAdcOffsetTrimOTP))
    {
        //
        //offset trim function is programmed into OTP, so call it
        //

        //
        //calculate the index into OTP table of offset trims and call
        //function to return the correct offset trim
        //
        adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
        adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
    }
    else
    {
        //
        //offset trim function is not populated, so set offset trim to 0
        //
        adcOffsetTrim = 0;
    }

    //
    //Apply the resolution and signalmode to the specified ADC.
    //Also apply the offset trim and, if needed, linearity trim correction.
    //
    switch(adc)
    {
        case ADC_ADCA:
            AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCB:
            AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCC:
            AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCD:
            AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
    }
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
//
// CalAdcINL - Loads INL trim values from OTP into the trim registers of the
//             specified ADC. Use only as part of AdcSetMode function, since
//             linearity trim correction is needed for some modes.
//
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void CalAdcINL(Uint16 adc)
{
    switch(adc)
    {
        case ADC_ADCA:
            if(0xFFFF != *((Uint16*)CalAdcaINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcaINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCB:
            if(0xFFFF != *((Uint16*)CalAdcbINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcbINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCC:
            if(0xFFFF != *((Uint16*)CalAdccINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdccINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCD:
            if(0xFFFF != *((Uint16*)CalAdcdINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcdINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
    }
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//

//                              SFRA Initialization

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//


void InitControl(void)
{
    //Resets the internal data of sfra module to zero
    SFRA_F32_reset(&sfra1);
    //Configures the SFRA module
    SFRA_F32_config(&sfra1,
                    SFRA_ISR_FREQ,
                    SFRA_AMPLITUDE,
                    SFRA_FREQ_LENGTH,
                    SFRA_FREQ_START,
                    SFRA_FREQ_STEP_MULTIPLY,
                    plantMagVect,
                    plantPhaseVect,
                    olMagVect,
                    olPhaseVect,
                    freqVect);
    //Resets the response arrays to all zeroes
    SFRA_F32_resetFreqRespArray(&sfra1);
    //Initializes the frequency response array ,
    //The first element is SFRA_FREQ_START
    //The subsequent elements are freqVect[n-1]*SFRA_FREQ_STEP_MULTIPLY
    //This enables placing a fixed number of frequency points
    //between a decade of frequency.
    // The below routine can be substituted by a routine that sets
    // the frequency points arbitrarily as needed.
    SFRA_F32_initFreqArrayWithLogSteps(&sfra1,
                                       SFRA_FREQ_START,
                                       SFRA_FREQ_STEP_MULTIPLY);

    sfra1.state = 1;
    sfra1.status = 1;
}

//ADCoffComp function enables timer interrupt before the measure, disable timer interrupt after the measure
//uses the post processing block to subtract the measured offset at the input of the adc
void ADCoffComp(void)
{
    int cyct=0;
    DELAY_US(1000000);

    //variables uset to calculate the offset
    long off0=00;
    long off1=00;
    long off2=00;
    long off3=00;
    long off4=00;
    long off5=00;
    long off6=00;

    for(cyct=0; cyct<4096; cyct++)   //sum 32768 samples for every adc input
    {
        DELAY_US(50);

        while(ADCoffCompF == 0);
        ADCoffCompF = 0;

        off0 += AdcaResultRegs.ADCRESULT0;
        off1 += AdcbResultRegs.ADCRESULT0;
        off2 += AdccResultRegs.ADCRESULT0;
        off3 += AdcdResultRegs.ADCRESULT0;
        off4 += AdcdResultRegs.ADCRESULT1;
        off5 += AdcdResultRegs.ADCRESULT2;
        off6 += AdcdResultRegs.ADCRESULT3;

        // Clear INT1 flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
        // Acknowledge this interrupt to receive more interrupts from group 1
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    }

    //mean calculation on 4096 samples
    off0 >>= 12;
    off1 >>= 12;
    off2 >>= 12;
    off3 >>= 12;
    off4 >>= 12;
    off5 >>= 12;
    off6 >>= 12;

    off0 -= 32768;
    off1 -= 32768;
    off2 -= 32768;
    off3 -= 32768;
    off4 -= 32768;
    off5 -= 32768;
    off6 -= 32768;

    //OFFSET SUBTRACTION USING ADC POST PROCESSING BLOCK
    EALLOW;

    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //ADC_A PPB1 is associated with SOC0
    AdcaRegs.ADCPPB1CONFIG.bit.TWOSCOMPEN=0; //this bit enables the post conversion hardware processing circuit: 0 ADCPPBxRESULT = ADCRESULTx - ADCSOCxOFFREF
    AdcaRegs.ADCPPB1OFFREF = off0;         //ADC_A PPB1 will subtract OFFCAL value to associated SOC

    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;
    AdcbRegs.ADCPPB1CONFIG.bit.TWOSCOMPEN=0;
    AdcbRegs.ADCPPB1OFFREF = off1;

    AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;
    AdccRegs.ADCPPB1CONFIG.bit.TWOSCOMPEN=0;
    AdccRegs.ADCPPB1OFFREF = off2;

    AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 0;
    AdcdRegs.ADCPPB1CONFIG.bit.TWOSCOMPEN=0;
    AdcdRegs.ADCPPB1OFFREF = off3;

    AdcdRegs.ADCPPB2CONFIG.bit.CONFIG = 1;
    AdcdRegs.ADCPPB2CONFIG.bit.TWOSCOMPEN=0;
    AdcdRegs.ADCPPB2OFFREF = off4;

    AdcdRegs.ADCPPB3CONFIG.bit.CONFIG = 2;
    AdcdRegs.ADCPPB3CONFIG.bit.TWOSCOMPEN=0;
    AdcdRegs.ADCPPB3OFFREF = off5;

    AdcdRegs.ADCPPB4CONFIG.bit.CONFIG = 3;
    AdcdRegs.ADCPPB4CONFIG.bit.TWOSCOMPEN=0;
    AdcdRegs.ADCPPB4OFFREF = off6;

    EDIS;
    ADCoffCompfinishF=1;
}

//
// End of file
//
