#include "F28x_Project.h"
#include "math.h"
#include "C28x_FPU_FastRTS.h"
#include <string.h>

#define UR 2                                        // Update rate (UR>2 -> multisampling algorithm)
#define OVERSAMPLING 1                              // Logic variable to differentiate between case with and without oversampling
#define NOS 16                                      // Number of samples to be measured on PWM period (if oversampling==1 NOS is oversampling factor)
#define NOS_UR (NOS/UR)                             // Ratio between NOS and UR
#define LOG2_NOS_UR (log2(NOS_UR))                  // Used for averaging on regulation period (if OVERSAMPLING==1)
#define INV_UR (1/(float32)UR)                      // Used for averaging on switching period (if OVERSAMPLING==1)
#define FTB 100e6                                   // FTB=EPWMCLK=SYSCLKOUT/2 (time base clock ratio to EPWM clock = 1 assumed)
#define FPWM 10e3                                   // Switching frequency
// Period of virtual switching counter, up-down mode assumed; closest to (Uint16)(FTB/(2*FPWM)-1) so that PWM_TBPRD%16=0
#define PWM_TBPRD 4992
#define TPWM (2*PWM_TBPRD/FTB)                      // Switching period
#define ADC_TBPRD (PWM_TBPRD/NOS)                   // Counter period for ePWM used for ADC triggering, up-down mode assumed
#define MS_TBPRD (2*PWM_TBPRD/UR - 1)               // Counter period of ePWM used to implement multisampling algorithm, up mode;
#define TS (TPWM/UR)                                // Regulation period

#define E 550.0f                                    // Available DC voltage
#define EINVERSE (1/E)                              // Inverse of E
#define D_MAX (PWM_TBPRD - 5)                       // Maximum duty cycle (to avoid unnecessary switching)
#define D_MIN 5                                     // Minimum duty cycle (to avoid unnecessary switching)
#define PI 3.141593f                                // PI
#define DEADTIME 100                                                     // Dead time in number of EPWM clocks (see EPwmXRegs.TBPRD) 100 --> 1us
#define UDT ((float32)(DEADTIME)/(float32)(PWM_TBPRD)*E*4.0f/PI)         // Constant for dead time compensation (4/pi for first harmonic's peak)

// Defines for measurements (position & current)
#define P 2.0f                                      // Machine's number of pole pairs
#define ENC_LINE 1000                               // Number of encoder lines
#define ANG_CNV (2*PI*P/(float32)(4*ENC_LINE))      // Constant for angle calculation (conversion from QEP counter)
#define INV_UR_1 (1/(float32)(UR+1))                // Used for angle averaging on switching period
#define ADC_SCALE 0.0007326f                        // ADC scaling: 3.0 --> 4095 (zero ADC offset assumed)
#define ISENSE_SCALE 10.0f                          // [A] --> [V] (ISENSE_SCALE)A=1V
#define LSB_offset_a 0
#define LSB_offset_b 0
#define ISENSE_OFFSET_A (1.50f + LSB_offset_a*ADC_SCALE) //(1.50f + 0.00309f + 0.00013f - 0.00573f - 0.00529f + 0.00028f) // - 0.01443f) //96f                     // 0A --> 1.5V + offset ADC-a
#define ISENSE_OFFSET_B (1.50f + LSB_offset_b*ADC_SCALE) //(1.50f + 0.00448f + 0.00012f - 0.00567f - 0.00563f + 0.00032f) // - 0.01492f)//111f                     // 0A --> 1.5V + offset ADC-a

#define MAX_data_count 443 //850                       // Size of an array used for data storage
#define DATACNT_REF 200
#define DMACNT_REF 10869  //3478                         // Set reference after DMACNT_REF regulation periods
#define DMACNT_PRNT 10600  //3000                        // Start printing after DMACNT_PRNT regulation periods

// Defines for IREG
#define R 0.09871f                                     // Motor resistance
#define L 0.0025f                                   // Motor inductance
#define INV_TAU (R/L)                               // 1/(L/R)

// defines for slip calculation
#define RR 0.108f                                  // Rotor phase resistance referred to stator
#define LR 0.0484f                                  // Rotor phase inductance referred to stator (Lr = Llr + Lm)
#define ID_NOM 18.55f                                 // Nominal d current
#define INV_TAUR_ID (RR/LR/ID_NOM)                  // Inverse of rotor time constant (for slip calculation)

#pragma CODE_SECTION(dmach1_isr, ".TI.ramfunc");    // Allocate code (dmach1_isr) in RAM
#pragma CODE_SECTION(adca1_isr, ".TI.ramfunc");     // Allocate code (adca1_isr) in RAM

// First array for the ping pong algorithm
#pragma DATA_SECTION(DMAbuffer1,"ramgs0");          // Allocate data (DMAbuffer1) in RAM
volatile Uint16 DMAbuffer1[2*NOS_UR];

// Second array for the ping pong algorithm
#pragma DATA_SECTION(DMAbuffer2,"ramgs0");          // Allocate data (DMAbuffer2) in RAM
volatile Uint16 DMAbuffer2[2*NOS_UR];

void Configure_ePWM(void);                     // Configure EPWM
void Configure_ADC(void);                      // Configure ADC
void Configure_DMA(void);                      // Configure DMA
void Clear_DMAbuffer(void);                    // Clear DMA buffers
void Configure_GPIO(void);                     // Configure GPIO
void Configure_eQEP(void);                     // Configure EQEP
void PrintData(void);                          // Data storage used to export monitored variables to .dat file
void Init_VARS(void);                          // Initialize variables

__interrupt void adca1_isr(void);              // JUST FOR DEBIGGING to check sampling process
__interrupt void dmach1_isr(void);             // Regulation takes place in dmach1_isr

// Variables for multisampling handling
Uint16 n_seg = 1;                              // Indicates a current segment of the virtual carrier
Uint16 PWM_dir;                                // Direction of the virtual carrier (1 -> up, 2-> down)
Uint16 PWM_nextSeg_max;                        // Maximum value of the virtual carrier in the following control period (n_seg + 1)
Uint16 PWM_nextSeg_min;                        // Minimum value of the virtual carrier in the following control period (n_seg + 1)
Uint16 cross_margin = 7;                       // Cross margin to avoid glitches in EPWM output (experimentally determined)

// Variables for motor control

// Currents
volatile Uint16 Ia_sum_Ts, Ib_sum_Ts;           // Measurements (take data from DMAbuffer)
float32 Ia_avg_Ts, Ib_avg_Ts;                   // if OVERSAMPLING= 1 averaged data from DMAbuffer (on TS); else last measurement from DMAbuffer
float32 Ic_avg_Ts;                              // Ic = - Ia - Ib
float32 Ialpha_avg_Ts, Ibeta_avg_Ts;            // Alpha/beta currents (averaged on Ts)
float32 Id_avg_Ts[UR] = {};                     // D current (averaged on Ts)
float32 Iq_avg_Ts[UR] = {};                     // Q current (averaged on Ts)
float32 Sum_Id_avg_Ts, Sum_Iq_avg_Ts;           // Sum averaged quantities in dq frame (to perform averaging on switching period)
float32 Id, Iq;                                 // Currents in dq frame (averaged on Tpwm)
float32 dId[2] = {};                            // Current error in q axis; dId[0] -> current, dId[1] -> previous
float32 dIq[2] = {};                            // Current error in d axis; dIq[0] -> current, dIq[1] -> previous
float32 Id_ref = 0.0f;                          // Reference d current
float32 Iq_ref = 0.0f;                          // Reference q current
float32 IMAX = 35.0f;                           // Limit for over-current protection

// IREG
float32 alpha = 0.2f; //0.0636f; //0.087f;                            // Gain for IREG
float32 K1, K2;                                  // Constants used for IREG

// Voltages
float32 Ud[2]={};                               // D voltage (IREG output - control signal)
float32 Uq[2]={};                               // Q voltage (IREG output - control signal)
float32 Ualpha, Ubeta;                          // Voltages in alpha/beta frame
float32 Ua, Ub, Uc;                             // A,B,C voltages
float32 Udq_max = E/(2.0f);                     // Maximum available voltage

// Angles
float32 theta[UR+1] = {};                       // Machine electrical angle - theta[0]=current, theta[1]=previous, etc.
float32 theta_avg_Ts;                           // Averaged angle on Ts
float32 theta_enc;                              // Mechanical angle calcualted using QEP counter
float32 dtheta;                                 // Angle difference used in IREG (w*Ts)
float32 _sin[4];                                // _sin[0]=sin(theta_avg_Ts), _sin[1]=sin(theta[0]), _sin[2]=sin(dtheta), _sin[3]=sin(2*dtheta)
float32 _cos[4];                                // _cos[0]=cos(theta_avg_Ts), _cos[1]=sin(theta[0]), _cos[2]=sin(dtheta), _cos[3]=sin(2*dtheta)
float32 theta_k = 0.0f;                         // Slip angle
float32 omega_k;
float32 theta_s[2]={};;

// Variables for CMP registers

Uint16 PWM_CMP_a, PWM_CMP_b, PWM_CMP_c;         // CMP value for the virtual carrier (duty cycle)
Uint16 MS_CMPA_a, MS_CMPA_b, MS_CMPA_c;         // CMPA value for the multisampling counter
Uint16 MS_CMPB_a, MS_CMPB_b, MS_CMPB_c;         // CMPB value for the multisampling counter

// Data storage
float32 dataOut_1[MAX_data_count] = {};
float32 dataOut_2[MAX_data_count] = {};
//float32 dataOut_3[MAX_data_count] = {};
//float32 dataOut_4[MAX_data_count] = {};
//float32 dataOut_5[MAX_data_count] = {};
//float32 dataOut_6[MAX_data_count] = {};
Uint16 canPrint = 0;                            // Logic signal used to trigger data storage
long int data_count = 0;                        // Counter for data storage
Uint16 reg_enabled = 0;                         // Logic variable used to start output
Uint16 error_flag = 0;                          // Indicate if an error occurred (overcurrent protection, etc)

// For debugging with f=const
float32 f_ref = 270.0f;                         // Frequency of electrical quantities
#define TWOPI_TS (2*PI*TS)                      // 2*pi*Ts for angle calculation

float32 pom1, pom2;

long int dma_count = 0;                         // Counter to check dmach1_isr
//long int adc_count_a = 0;                     // Counter to check adca1_isr

void main(void)
{
    // To run from FLASH uncomment the following and include in Linker ...FLASH...cmd instead of ...RAM...cmd

/*
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t) &RamfuncsLoadSize);
    InitFlash();
*/

    InitSysCtrl();
    InitGpio();

    Configure_GPIO();

    DINT;                           // Disable ST1.INTM
    IER = 0x0000;                   // Disable CPU interrupts
    IFR = 0x0000;                   // Clear all CPU interrupt flags

    InitPieCtrl();                  // DINT, PIECTRL, PIEIER.x, .PIEIFR.x (disable all, clear all)
    InitPieVectTable();             // PIECTRL (enable) & what else ("setup to a known state")?

    Init_VARS();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Stop the TBCLK clock
    EDIS;

    Configure_ePWM();
    Configure_ADC();
    Clear_DMAbuffer();
    DMAInitialize();
    Configure_DMA();
    Configure_eQEP();

    // Write the ISR vector for each interrupt to the appropriate location in the PIE vector table
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    PieVectTable.DMA_CH1_INT = &dmach1_isr;
    EDIS;

    // Enable interrupts on CPU level
    IER |= M_INT1;                  // PIE group1 --> CPU INT1
    IER |= M_INT7;                  // PIE group7 --> CPU INT7

    EINT;                           // Enable ST1.INTM
    ERTM;                           // Enable Global real time interrupt DBG

    // Enable interrupt on PIE level
    PieCtrlRegs.PIEIER1.bit.INTx1 = 0;          // ADCA
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;          // DMA

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;      // Sync TBCLK with CPU clock
    EDIS;

    StartDMACH1();

    while(1)
        {
        }

}

void Init_VARS(void)
{
    K1 = alpha*L/TS;                        // Constant for IREF
    K2 = exp(-TS*INV_TAU);                  // Parameter that describes system dynamics exp(-R*Ts/L) - constant used for IREG

    // Set initial values for IREG's integrator
    //Ud[1] = -16.4304f;
    //Uq[1] = 193.5485f;

    //dtheta = 2*PI*f_ref*TS;

}


void Configure_GPIO(void)
{

    EALLOW;

    // GPIOs for EPWM

    // Configure as EPWM1A output (phase A high side)
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;         // Mux to ePWM1A
    GpioDataRegs.GPASET.bit.GPIO0 = 1;          // Set high, until otherwise specified by control algorithm

    // Configure as EPWM1B output (phase A low side)
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;         // Mux to ePWM1B
    GpioDataRegs.GPASET.bit.GPIO1 = 1;          // Set high, until otherwise specified by control algorithm

    // Configure as EPWM2A output (phase B high side)
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;         // Mux to ePWM2A
    GpioDataRegs.GPASET.bit.GPIO2 = 1;          // Set high, until otherwise specified by control algorithm

    // Configure as EPWM2B output (phase B low side)
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;         // Mux to ePWM2B
    GpioDataRegs.GPASET.bit.GPIO3 = 1;          // Set high, until otherwise specified by control algorithm

    // Configure as EPWM3A output (phase C high side)
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;         // Mux to ePWM3A
    GpioDataRegs.GPASET.bit.GPIO4 = 1;          // Set high, until otherwise specified by control algorithm

    // Configure as EPWM3B output (phase C low side)
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;         // Mux to ePWM3B
    GpioDataRegs.GPASET.bit.GPIO5 = 1;          // Set high, until otherwise specified by control algorithm

    // GPIOs for debugging/measurements

    // Configure as EPWM4A output to notify MS_TBCTR=0 (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;         // Mux to ePWM3A

    // Configure as EPWM5A output to notify ADC_TBCTR=0 & ADC_TBCTR=TBPRD (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;         // Mux to ePWM5A

    // Configure as ordinary GPIO to measure isr duration (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;         // Configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // Mux as ordinary GPIO
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;       // ensure 0 before first interrupt occurs

    // Configure as ordinary GPIO to notify setting of the reference voltage (FOR OSCILOSCOPE MEASUREMENTS)
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;         // Configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // Mux as ordinary GPIO
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;       // ensure 0 before setting Vref

    //GPIOs for QEP

    // Configure as EQEP2A input
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;          // Configure as input
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;         // Mux to eQEP2A

    // Configure as EQEP2B input
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;          // Configure as input
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;         // Mux to eQEP2B

    // Configure as EQEP2I input
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;          // Configure as input
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;         // Mux to eQEP2I

    EDIS;

}

void Configure_ePWM(void)
{

    // EPWM1 as a multisampling carrier for phase A

    EPwm1Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;           // Up mode
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm1Regs.TBPRD = MS_TBPRD;                 // Counter period

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;         // Shadow mode active for CMPA
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;         // Shadow mode active for CMPB
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;         // Load on TBCTR=0
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 0;         // Load on TBCTR=0

    EPwm1Regs.CMPA.bit.CMPA = MS_TBPRD + 1;     // Value of the CMPA at the beginning (action never happens)
    EPwm1Regs.CMPB.bit.CMPB = MS_TBPRD + 1;     // Value of the CMPB at the beginning (action never happens)

    EPwm1Regs.AQCTLA.bit.CAU = 1;               // Set EPWMA low on TBCTR=CMPA during up count
    EPwm1Regs.AQCTLA.bit.CBU = 2;               // Set EPWMA high on TBCTR=CMPB during up count

    EPwm1Regs.DBCTL.bit.POLSEL = 1;             // Active low complementary - EPWMxB is inverted.
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;           // DBM is fully enabled
    EPwm1Regs.DBFED.bit.DBFED = DEADTIME;       // Set delay for rising edge (DBRED)
    EPwm1Regs.DBRED.bit.DBRED = DEADTIME;       // Set delay for falling edge (DBFED)

    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = 1;                // Set EPWMA high when the trip-zone is triggered
    EPwm1Regs.TZCTL.bit.TZB = 1;                // Set EPWMB high when the trip-zone is triggered
    EDIS;

    // EPWM2 as a multisampling carrier for phase B

    EPwm2Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1     TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;           // Up mode
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm2Regs.TBPRD = MS_TBPRD;                 // Counter period

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;         // Shadow mode active for CMPA
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;         // Shadow mode active for CMPB
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;         // Load on TBCTR=0
    EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;         // Load on TBCTR=0

    EPwm2Regs.CMPA.bit.CMPA = MS_TBPRD + 1;     // Value of the CMPA at the beginning (action never happens)
    EPwm2Regs.CMPB.bit.CMPB = MS_TBPRD + 1;     // Value of the CMPB at the beginning (action never happens)

    EPwm2Regs.AQCTLA.bit.CAU = 1;               // Set EPWMA low on TBCTR=CMPA during up count
    EPwm2Regs.AQCTLA.bit.CBU = 2;               // Set EPWMA high on TBCTR=CMPB during up count

    EPwm2Regs.DBCTL.bit.POLSEL = 1;             // Active low complementary - EPWMxB is inverted.
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;           // DBM is fully enabled
    EPwm2Regs.DBFED.bit.DBFED = DEADTIME;       // Set delay for rising edge (DBRED)
    EPwm2Regs.DBRED.bit.DBRED = DEADTIME;       // Set delay for falling edge (DBFED)

    EALLOW;
    EPwm2Regs.TZCTL.bit.TZA = 1;                // Set EPWMA high when the trip-zone is triggered
    EPwm2Regs.TZCTL.bit.TZB = 1;                // Set EPWMB high when the trip-zone is triggered
    EDIS;

    // EPWM3 as a multisampling carrier for phase C

    EPwm3Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1     TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;           // Up mode
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm3Regs.TBPRD = MS_TBPRD;                 // Counter period

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;         // Shadow mode active for CMPA
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0;         // Shadow mode active for CMPB
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;         // Load on TBCTR=0
    EPwm3Regs.CMPCTL.bit.LOADBMODE = 0;         // Load on TBCTR=0

    EPwm3Regs.CMPA.bit.CMPA = MS_TBPRD + 1;      // Value of the CMPA at the beginning (action never happens)
    EPwm3Regs.CMPB.bit.CMPB = MS_TBPRD + 1;      // Value of the CMPB at the beginning (action never happens)

    EPwm3Regs.AQCTLA.bit.CAU = 1;               // Set EPWMA low on TBCTR=CMPA during up count
    EPwm3Regs.AQCTLA.bit.CBU = 2;               // Set EPWMA high on TBCTR=CMPB during up count

    EPwm3Regs.DBCTL.bit.POLSEL = 1;             // Active low complementary - EPWMxB is inverted.
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;           // DBM is fully enabled
    EPwm3Regs.DBFED.bit.DBFED = DEADTIME;       // Set delay for rising edge (DBRED)
    EPwm3Regs.DBRED.bit.DBRED = DEADTIME;       // Set delay for falling edge (DBFED)

    EALLOW;
    EPwm3Regs.TZCTL.bit.TZA = 1;                // Set EPWMA high when the trip-zone is triggered
    EPwm3Regs.TZCTL.bit.TZB = 1;                // Set EPWMB high when the trip-zone is triggered
    EDIS;

    // Clear trip zone flags for switching EPWMs

    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    // EPWM4 to notify multisampling carrier TBCTR=0 (JUST FOR DEBUGGING)

    EPwm4Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;           // Up mode
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm4Regs.TBPRD = MS_TBPRD;                // Counter period

    EPwm4Regs.AQCTLA.bit.ZRO = 3;              // Toggle EPWMA each time TBCTR = 0


    // EPWM5 for ADC triggering

    EPwm5Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm5Regs.TBCTL.bit.CTRMODE = 2;           // Up-down mode
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm5Regs.TBPRD = ADC_TBPRD;               // Counter period
    EPwm5Regs.ETSEL.bit.SOCASEL = 1;           // ADCSOCA on TBCTR=0
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;            // Generate SOCA on 1st event
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;            // Enable SOCA generation

    // Set PWM output to notify SOCA (JUST FOR DEBUGGING)
    EPwm5Regs.AQCTLA.bit.ZRO = 2;              // Set PWM A high on TBCTR=0
    EPwm5Regs.AQCTLA.bit.PRD = 1;              // Set PWM A low on TBCTR=TBPRD
}

void Configure_ADC(void)
{
    EALLOW;

    // configure ADCA

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider; ADCCLK=SYSCLK/4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Set the resolution and signal mode for a given ADC
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;          // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 30;         // Acquisition window is ACQPS+1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5 SOC0

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // EOC A0 will set ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // ADCINT1 on each EOC pulse regardless of the flag bit (to avoid adca1_isr)
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable ADCINT1
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Make sure INT1 flag is cleared


    // configure ADCB

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider; ADCCLK=SYSCLK/4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Set the resolution and signal mode for a given ADC
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;          // SOC0 will convert pin B2 (B2 is available on launch pad)
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 30;         // Acquisition window is ACQPS+1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5 SOC0

    // JUST FOR DEBUGGING
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2;      // EOC B2 will set ADCINT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;        // Enable ADCINT1
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Make sure INT1 flag is cleared

    EDIS;
}

void Configure_DMA(void)
{
    EALLOW;

    // Configure CH1

    // Set up MODE Register
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = 1;      // Select the Trigger and Sync Source of DMACH1 to ADCAINT1
    DmaRegs.CH1.MODE.bit.PERINTSEL = 1;             // ADCAINT1 as peripheral interrupt source
    DmaRegs.CH1.MODE.bit.PERINTE = 1;               // Peripheral interrupt enabled
    DmaRegs.CH1.MODE.bit.ONESHOT = 0;               // 1 burst per SW interrupt
    DmaRegs.CH1.MODE.bit.CONTINUOUS = 1;            // Do not stop after each transfer
    DmaRegs.CH1.MODE.bit.DATASIZE = 0;              // 16-bit data size transfers
    DmaRegs.CH1.MODE.bit.CHINTMODE = 1;             // Generate ePIE interrupt at the end of transfer
    DmaRegs.CH1.MODE.bit.CHINTE = 1;                // Channel Interrupt to CPU enabled

    // Set up BURST registers
    DmaRegs.CH1.BURST_SIZE.all = 1;                  // Number of 16-bit words per burst (N-1)
    DmaRegs.CH1.SRC_BURST_STEP = 32;                 // Increment source burst address by 32 (16-bit) (ADCB result address = ADCA result address + 32)
    DmaRegs.CH1.DST_BURST_STEP = NOS_UR;             // Increment destination burst address by NOS_UR (16-bit)

    // Set up TRANSFER registers
    DmaRegs.CH1.TRANSFER_SIZE = NOS_UR-1;             // Number of bursts per transfer (N-1)
    DmaRegs.CH1.SRC_TRANSFER_STEP = -32;              // Increment source transfer address by -32 (16-bit)
    DmaRegs.CH1.DST_TRANSFER_STEP = -(NOS_UR-1);      // Increment destination transfer address by -(NOS_UR-1) (16-bit)

    // Set up WRAP registers
    DmaRegs.CH1.SRC_WRAP_SIZE = 17;                   // Wrap after 17 burst (wrapping disabled because wrap_size > transfer_size)
    DmaRegs.CH1.DST_WRAP_SIZE = 17;                   // Wrap after 17 burst (wrapping disabled because wrap_size > transfer_size)
    DmaRegs.CH1.SRC_WRAP_STEP = 0;
    DmaRegs.CH1.DST_WRAP_STEP = 0;

    // Set up SOURCE address
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)&AdcaResultRegs.ADCRESULT0;   // Point to beginning of source buffer
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)&AdcaResultRegs.ADCRESULT0;   // Source address

    // Set up DESTINATION address
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)&DMAbuffer1[0];               // Point to beginning of destination buffer
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)&DMAbuffer1[0];               // Toggle destination address in dmach1_isr

    // Clear any spurious flags & errors
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;            // Clear any spurious interrupt flags
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;               // Clear any spurious sync error

    EDIS;
}

void Configure_eQEP(void)
{

    EQep2Regs.QDECCTL.bit.QSRC = 0;         // Quadrature count mode
    EQep2Regs.QEPCTL.bit.PCRM = 1;          // 0 --> QPOSCNT reset on index event; 1 --> reset on max CNT
    EQep2Regs.QEPCTL.bit.QPEN = 1;          // QEP enable
    EQep2Regs.QCAPCTL.bit.CEN = 1;          // QEP capture Enable
    EQep2Regs.QPOSMAX = 4*ENC_LINE-1;       // QPOSCNT max (without this QEP does not count)

}

void Clear_DMAbuffer(void)
{
    int i;
    for(i=0; i<2*NOS_UR; i++)
       {
          DMAbuffer1[i] = 0;
          DMAbuffer2[i] = 0;
       }

    Ia_sum_Ts=0;
    Ib_sum_Ts=0;

    Sum_Id_avg_Ts = 0.0f;
    Sum_Iq_avg_Ts = 0.0f;

}

void PrintData()
{
    if(canPrint)
    {
        dataOut_1[data_count] =  Id; //AdcaResultRegs.ADCRESULT0;
        dataOut_2[data_count] =  Iq; //AdcbResultRegs.ADCRESULT0;
        //dataOut_3[data_count] =  Id;
        //dataOut_4[data_count] =  Iq;
        //dataOut_5[data_count] =  Id;
        //dataOut_6[data_count] =  Ud[0];

        data_count++;

        if(data_count == DATACNT_REF)
        {
            Iq_ref = 0.0f;
        }

        if (data_count >= MAX_data_count)
        {
            data_count = 0;
            Iq_ref = 0.0f;
            canPrint = 0;
        }

    }

}

__interrupt void adca1_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;             // Notify adca1_isr start

    //adc_count_a++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Clear acknowledge register

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;           // Notify adca1_isr end

}

__interrupt void dmach1_isr(void)
{

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                       // Notify dmach1_isr start

    theta_enc = ((float32)EQep2Regs.QPOSCNT)*ANG_CNV;        // Capture position

    int i_for = 0;

    // Slip calculation
    omega_k = INV_TAUR_ID*Iq_ref;
    theta_k+= omega_k*TS;
    if(theta_k >= 2*PI) theta_k-= 2*PI;

    theta_s[0] = theta_enc + theta_k;

    theta[0] += theta_s[0] - theta_s[1];
    if(theta_s[0] <= 2*PI && theta_s[0] >= 0)
        theta[0] = theta_s[0];
    if(theta[0]>=2*PI)
        theta[0]-=2*PI;
    if(theta[1]-theta[0]>= PI)
        theta[1]-=2*PI;
    theta_s[1] = theta_s[0];

    dma_count++;

/*
 // Set your own theta based on the predefined frequency
    theta[0]+= TWOPI_TS*f_ref;                        // Capture position

    if(theta[0]>= 2*PI)
        {
        for (i_for=UR;i_for>=0;i_for--)
            {
                theta[i_for]-= 2*PI;
            }
        }
*/

    static int dma_sgn = 1;     // Logic variable to indicate state of the ping pong algorithm

    Ia_sum_Ts = 0;
    Ib_sum_Ts = 0;

    Sum_Id_avg_Ts = 0.0f;
    Sum_Iq_avg_Ts = 0.0f;

    // change dma_sgn to indicate state of the ping pong algorithm
    if (dma_sgn==1)
    {
        EALLOW;
        // DST ADDRESS FOR NEXT DMA CYCLE (currently reading from previously active buffer)
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)&DMAbuffer2[0];  //PING
        EDIS;
        dma_sgn = -1;

        #if (OVERSAMPLING)
            // Collect data to be averaged
            for (i_for=0;i_for<NOS_UR;i_for++)
            {
                Ia_sum_Ts+=DMAbuffer1[i_for];
                Ib_sum_Ts+=DMAbuffer1[i_for+NOS_UR];
            }
        #else
            // Take last measurement
            Ia_sum_Ts = DMAbuffer1[NOS_UR-1];
            Ib_sum_Ts = DMAbuffer1[NOS_UR-1+NOS_UR];
        #endif
    }
    else if (dma_sgn==-1)
    {
        EALLOW;
        // DST ADDRESS FOR NEXT DMA CYCLE (currently reading from previously active buffer)
        DmaRegs.CH1.DST_ADDR_SHADOW =  (Uint32)&DMAbuffer1[0];  //PONG
        EDIS;
        dma_sgn=1;

        #if (OVERSAMPLING)
            // Collect data to be averaged
            for (i_for=0;i_for<NOS_UR;i_for++)
            {
                Ia_sum_Ts+=DMAbuffer2[i_for];
                Ib_sum_Ts+=DMAbuffer2[i_for+NOS_UR];
            }
        #else
            // Take last measurement
            Ia_sum_Ts = DMAbuffer2[NOS_UR-1];
            Ib_sum_Ts = DMAbuffer2[NOS_UR-1+NOS_UR];
        #endif
    }

    #if (OVERSAMPLING)
        // Averaging on regulation period & scaling ([dig] --> [A])
        Ia_avg_Ts = ((float32)(Ia_sum_Ts>>((int)LOG2_NOS_UR))*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
        Ib_avg_Ts = ((float32)(Ib_sum_Ts>>((int)LOG2_NOS_UR))*ADC_SCALE - ISENSE_OFFSET_B)*ISENSE_SCALE;
    #else
        // Take last measurement stored in DMA buffer
        Ia_avg_Ts = ((float32)(Ia_sum_Ts)*ADC_SCALE - ISENSE_OFFSET_A)*ISENSE_SCALE;
        Ib_avg_Ts = ((float32)(Ib_sum_Ts)*ADC_SCALE - ISENSE_OFFSET_B)*ISENSE_SCALE;
    #endif

    Ic_avg_Ts = - Ia_avg_Ts - Ib_avg_Ts;        // Calculate current in phase C

    error_flag = abs(Ia_avg_Ts) > IMAX || abs(Ib_avg_Ts) > IMAX || abs(Ic_avg_Ts) > IMAX;       // Check for overcurrent protection

    if(error_flag)
        {
            // Set TZ flags
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST = 1;
            EPwm2Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZFRC.bit.OST = 1;
            EDIS;
            // Disable regulation
            reg_enabled = 0;
        }
    else
        {
            if(reg_enabled)
                {
                    // Clear TZ flags
                    EALLOW;
                    EPwm1Regs.TZCLR.bit.OST = 1;
                    EPwm2Regs.TZCLR.bit.OST = 1;
                    EPwm3Regs.TZCLR.bit.OST = 1;
                    EDIS;

                    // Direct Clarke transform  (abc -> alpha/beta)
                    Ialpha_avg_Ts = Ia_avg_Ts;                                          // Ialpha=Ia
                    Ibeta_avg_Ts =  0.57735f * Ia_avg_Ts + 1.1547f * Ib_avg_Ts;         // Ibeta=1/sqrt(3)*Ia+2/sqrt(3)*Ib

                    // Angles used for dq transform & IREG
                    dtheta = theta[0] - theta[1];
                    theta_avg_Ts = (theta[0] + theta[1])*0.5f;     // Average measured angle on regulation period

                    // Trigonometry
                    sincos(theta_avg_Ts, &_sin[0], &_cos[0]);
                    sincos(theta[0], &_sin[1], &_cos[1]);
                    sincos(dtheta, &_sin[2], &_cos[2]);
                    sincos(2*dtheta, &_sin[3], &_cos[3]);

                    #if (OVERSAMPLING)

                        // Direct Park transform using theta_avg_Ts
                        Id_avg_Ts[0] = Ialpha_avg_Ts * _cos[0] + Ibeta_avg_Ts * _sin[0];
                        Iq_avg_Ts[0] = Ibeta_avg_Ts * _cos[0] - Ialpha_avg_Ts * _sin[0];

                        for (i_for=0;i_for<UR;i_for++)
                            {
                                Sum_Id_avg_Ts+= Id_avg_Ts[i_for];
                                Sum_Iq_avg_Ts+= Iq_avg_Ts[i_for];
                            }

                        for (i_for=UR-1;i_for>0;i_for--)
                            {
                                Id_avg_Ts[i_for]= Id_avg_Ts[i_for-1];
                                Iq_avg_Ts[i_for]= Iq_avg_Ts[i_for-1];
                            }

                        // Averaging on switching period
                        Id = Sum_Id_avg_Ts*INV_UR;
                        Iq = Sum_Iq_avg_Ts*INV_UR;
                    #else

                        // Direct Park transform using theta[0]
                        Id = Ialpha_avg_Ts * _cos[1] + Ibeta_avg_Ts * _sin[1];
                        Iq = Ibeta_avg_Ts * _cos[1] - Ialpha_avg_Ts * _sin[1];

                    #endif

                    // Current error (- added for active low logic)
                    dId[0] = - (Id_ref - Id);
                    dIq[0] = - (Iq_ref - Iq);

                    // IMC based IREG
                    Ud[0] = Ud[1] + K1*(dId[0]*_cos[3]-dIq[0]*_sin[3]-K2*(dId[1]*_cos[2]-dIq[1]*_sin[2]));
                    Uq[0] = Uq[1] + K1*(dIq[0]*_cos[3]+dId[0]*_sin[3]-K2*(dIq[1]*_cos[2]+dId[1]*_sin[2]));

                    // Saturate if necessary (based on the DC link voltage capabilities)
                    if(Ud[0] > Udq_max) Ud[0] = Udq_max;
                    else if(Ud[0] < -Udq_max) Ud[0] = Udq_max;

                    if(Uq[0] > Udq_max) Uq[0] = Udq_max;
                    else if(Uq[0] < -Udq_max) Uq[0] = Udq_max;

                    // Remember values for the next dma_isr (store previous)
                    Ud[1] = Ud[0];
                    Uq[1] = Uq[0];
                    dId[1] = dId[0];
                    dIq[1] = dIq[0];
                    theta[1] = theta[0];

                    // Open loop testing
                    //Ud[0] = 0.0f;
                    //Uq[0] = 0.0f;

                    // Inverse Park transform
                    Ualpha = Ud[0] * _cos[1] - Uq[0] * _sin[1];
                    Ubeta = Ud[0] * _sin[1] + Uq[0] * _cos[1];

                    // Inverse Clarke transform
                    Ua = Ualpha;
                    Ub = (1.73205081f * Ubeta - Ualpha) * 0.5f;
                    Uc = -(1.73205081f * Ubeta + Ualpha) * 0.5f;

                    // Dead time compensation
                /*
                    if(Ia_avg_Ts >= 0.1f) Ua+=UDT;
                    else if(Ia_avg_Ts <= -0.1f) Ua-=UDT;

                    if(Ib_avg_Ts >= 0.1f) Ub+=UDT;
                    else if(Ib_avg_Ts <= -0.1f) Ub-=UDT;

                    if(Ic_avg_Ts >= 0.1f) Uc+=UDT;
                    else if(Ic_avg_Ts <= -0.1f) Uc-=UDT;
                */

                    // Limit modulation signals
                /*
                    if(PWM_CMP_a>D_MAX) PWM_CMP_a=D_MAX;
                    else if (PWM_CMP_a<D_MIN) PWM_CMP_a=D_MIN;

                    if(PWM_CMP_b>D_MAX)PWM_CMP_b=D_MAX;
                    else if (PWM_CMP_b<D_MIN)PWM_CMP_b=D_MIN;

                    if(PWM_CMP_c>D_MAX)PWM_CMP_c=D_MAX;
                    else if (PWM_CMP_c<D_MIN)PWM_CMP_c=D_MIN;
                */

                    // Modulation signals (1-d added for active low logic)
                    PWM_CMP_a = (Uint16)(PWM_TBPRD*(1.0f - (0.5f + Ua*EINVERSE)));
                    PWM_CMP_b = (Uint16)(PWM_TBPRD*(1.0f - (0.5f + Ub*EINVERSE)));
                    PWM_CMP_c = (Uint16)(PWM_TBPRD*(1.0f - (0.5f + Uc*EINVERSE)));
                }

            else
                {

                    EALLOW;
                    EPwm1Regs.TZFRC.bit.OST = 1;
                    EPwm2Regs.TZFRC.bit.OST = 1;
                    EPwm3Regs.TZFRC.bit.OST = 1;
                    EDIS;
                    /*
                    // Set duty cycle to 0.5 if regulation is disabled and there is no error flag
                    PWM_CMP_a = (Uint16)(PWM_TBPRD*0.5f);
                    PWM_CMP_b = (Uint16)(PWM_TBPRD*0.5f);
                    PWM_CMP_c = (Uint16)(PWM_TBPRD*0.5f);
                    */
                }


            //GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
            //GpioDataRegs.GPCSET.bit.GPIO66 = 1;

            #if (UR!=1)

                 n_seg++; // Indicates a current segment of the virtual carrier

                 if(n_seg>UR)
                     n_seg=1;

                 if(n_seg>UR/2)
                     PWM_dir=2;
                 else
                     PWM_dir=1;

                 if(PWM_dir==1) // Virtual carrier up count (during this time set EPWM low)
                     {
                         // Never set EPWM high on up count (except for the last interrupt during virtual carrier up count);
                         MS_CMPB_a=MS_TBPRD+1; // Phase A
                         MS_CMPB_b=MS_TBPRD+1; // Phase B
                         MS_CMPB_c=MS_TBPRD+1; // Phase C

                         if (n_seg!=UR/2)  // Not the last interrupt during virtual carrier up count
                         {
                             // Maximum value of the virtual carrier in the following control period (n_seg + 1)
                             PWM_nextSeg_max=(n_seg)*(MS_TBPRD+1)+MS_TBPRD;
                             // Minimum value of the virtual carrier in the following control period (n_seg + 1)
                             PWM_nextSeg_min=(n_seg)*(MS_TBPRD+1);

                             // Phase A
                             if((PWM_CMP_a>=PWM_nextSeg_min+cross_margin))
                                 // Horizontal crossing (set EPWM low) or crossing occurs later (no action because MS_CMPA_a>MS_TBPRD)
                                 MS_CMPA_a=PWM_CMP_a-PWM_nextSeg_min;
                             else
                                 // Vertical crossing (set EPWM low immediately) or crossing already happened (no action)
                                 MS_CMPA_a=cross_margin; // Cross margin to ensure CMP loading before compare event

                             // Phase B
                             if((PWM_CMP_b>=PWM_nextSeg_min+cross_margin))
                                 // Horizontal crossing (set EPWM low) or crossing occurs later (no action because MS_CMPA_a>MS_TBPRD)
                                 MS_CMPA_b=PWM_CMP_b-PWM_nextSeg_min;
                             else
                                 // Vertical crossing (set EPWM low immediately) or crossing already happened (no action)
                                 MS_CMPA_b=cross_margin; // Cross margin to ensure CMP loading before compare event

                             // Phase C
                             if((PWM_CMP_c>=PWM_nextSeg_min+cross_margin))
                                 // Horizontal crossing (set EPWM low) or crossing occurs later (no action because MS_CMPA_a>MS_TBPRD)
                                 MS_CMPA_c=PWM_CMP_c-PWM_nextSeg_min;
                             else
                                 // Vertical crossing (set EPWM low immediately) or crossing already happened (no action)
                                 MS_CMPA_c=cross_margin; // Cross margin to ensure CMP loading before compare event
                         }
                         else // The last interrupt during virtual carrier up count
                         {
                              // Phase A
                              MS_CMPA_a=MS_TBPRD+1; // Do not set EPWM low (in the next interrupt virtual carrier will be in down count mode)
                              MS_CMPB_a=PWM_TBPRD-PWM_CMP_a; // There can be no vertical crossing here, due to the duty cycle upper limitation

                              // Phase B
                              MS_CMPA_b=MS_TBPRD+1; // Do not set EPWM low (in the next interrupt virtual carrier will be in down count mode)
                              MS_CMPB_b=PWM_TBPRD-PWM_CMP_b; // There can be no vertical crossing here, due to the duty cycle upper limitation

                              // Phase C
                              MS_CMPA_c=MS_TBPRD+1; // Do not set EPWM low (in the next interrupt virtual carrier will be in down count mode)
                              MS_CMPB_c=PWM_TBPRD-PWM_CMP_c; // There can be no vertical crossing here, due to the duty cycle upper limitation
                         }
                     }
                else // Virtual carrier down count (during this time set EPWM high)
                 {
                    // Never set EPWM low on down count (except for the last interrupt during virtual carrier down count);
                     MS_CMPA_a=MS_TBPRD+1; // Phase A
                     MS_CMPA_b=MS_TBPRD+1; // Phase B
                     MS_CMPA_c=MS_TBPRD+1; // Phase C

                     if (n_seg!=UR) // Not the last interrupt during virtual carrier down count
                         {
                             // Maximum value of the virtual carrier in the following control period (n_seg + 1)
                             PWM_nextSeg_max=PWM_TBPRD-(n_seg-UR/2)*(MS_TBPRD+1);
                             // Minimum value of the virtual carrier in the following control period (n_seg + 1)
                             PWM_nextSeg_min=PWM_TBPRD-(n_seg-UR/2)*(MS_TBPRD+1)-MS_TBPRD;

                             // Phase A
                             if((PWM_CMP_a<=PWM_nextSeg_max-cross_margin))
                                 // Horizontal crossing (set EPWM high) or crossing occurs later (no action because MS_CMPB_a>MS_TBPRD)
                                 MS_CMPB_a=PWM_nextSeg_max-PWM_CMP_a;
                             else
                                 // Vertical crossing (set EPWM high immediately) or crossing already happened (no action)
                                 MS_CMPB_a=cross_margin;

                             // Phase B
                             if((PWM_CMP_b<=PWM_nextSeg_max-cross_margin))
                                 // Horizontal crossing (set EPWM high) or crossing occurs later (no action because MS_CMPB_a>MS_TBPRD)
                                 MS_CMPB_b=PWM_nextSeg_max-PWM_CMP_b;
                             else
                                 // Vertical crossing (set EPWM high immediately) or crossing already happened (no action)
                                 MS_CMPB_b=cross_margin;

                             // Phase C
                             if((PWM_CMP_c<=PWM_nextSeg_max-cross_margin))
                                 // Horizontal crossing (set EPWM high) or crossing occurs later (no action because MS_CMPB_a>MS_TBPRD)
                                 MS_CMPB_c=PWM_nextSeg_max-PWM_CMP_c;
                             else
                                 // Vertical crossing (set EPWM high immediately) or crossing already happened (no action)
                                 MS_CMPB_c=cross_margin;
                         }
                     else // The last interrupt during virtual carrier down count
                         {
                             // Phase A
                             MS_CMPB_a=MS_TBPRD+1; // Do not set EPWM high (in the next interrupt virtual carrier will be in up count mode)
                             MS_CMPA_a=PWM_CMP_a; // There can be no vertical crossing here, due to the duty cycle lower limitation

                             // Phase B
                             MS_CMPB_b=MS_TBPRD+1; // Do not set EPWM high (in the next interrupt virtual carrier will be in up count mode)
                             MS_CMPA_b=PWM_CMP_b; // There can be no vertical crossing here, due to the duty cycle lower limitation

                             // Phase C
                             MS_CMPB_c=MS_TBPRD+1; // Do not set EPWM high (in the next interrupt virtual carrier will be in up count mode)
                             MS_CMPA_c=PWM_CMP_c; // There can be no vertical crossing here, due to the duty cycle lower limitation
                         }
                 }
            #else

                // Phase A
                MS_CMPA_a = PWM_CMP_a;
                MS_CMPB_a = MS_TBPRD + 1 - PWM_CMP_a;

                // Phase B
                MS_CMPA_b = PWM_CMP_b;
                MS_CMPB_b = MS_TBPRD + 1 - PWM_CMP_b;

                // Phase C
                MS_CMPA_c = PWM_CMP_c;
                MS_CMPB_c = MS_TBPRD + 1 - PWM_CMP_c;

            #endif


            // Phase A
            EPwm1Regs.CMPA.bit.CMPA = MS_CMPA_a;    // Set CMPA
            EPwm1Regs.CMPB.bit.CMPB = MS_CMPB_a;    // Set CMPB

            // Phase B
            EPwm2Regs.CMPA.bit.CMPA = MS_CMPA_b;    // Set CMPA
            EPwm2Regs.CMPB.bit.CMPB = MS_CMPB_b;    // Set CMPB

            // Phase C
            EPwm3Regs.CMPA.bit.CMPA = MS_CMPA_c;    // Set CMPA
            EPwm3Regs.CMPB.bit.CMPB = MS_CMPB_c;    // Set CMPB
        }

    PrintData();                                      // Data storage (JUST FOR DEBUGGING)

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;             // Notify dmach1_isr end

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;           // Clear acknowledge register

}


