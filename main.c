#include "F28x_Project.h"
#include "math.h"
#include "C28x_FPU_FastRTS.h"
#include <string.h>

#define UR 2                                        // Update rate (double or single)
#define OVERSAMPLING 1                              // Logic variable to differentiate between case with and without oversampling
#define NOS 16                                      // Number of samples to be measured on PWM period (if oversampling==1 NOS is oversampling factor)
#define NOS_UR (NOS/UR)                             // Ratio between NOS and UR
#define LOG2_NOS_UR (log2(NOS_UR))                  // Used for averaging if oversampling==1
#define FTB 100e6                                   // FTB=EPWMCLK=SYSCLKOUT/2 (time base clock ratio to EPWM clock = 1 assumed)
#define FPWM 10e3                                   // Switching frequency
// Counter period for ePWM used for switching, up-down mode assumed; closest to (Uint16)(FTB/(2*FPWM)-1) so that PWM_TBPRD%16=0
#define PWM_TBPRD 4992
#define TPWM (2*PWM_TBPRD/FTB)                      // Switching period
#define ADC_TBPRD  (PWM_TBPRD/NOS)                  // Counter period for ePWM used for ADC triggering, up-down mode assumed
#define TS (TPWM/UR)                                // Regulation period
#define LOAD_CMPA (UR==1 ? 0 : 2)                   // Load CMPA on TBCTR=0 if UR==1, otherwise on both TBCTR=0 and TBCTR=TBPRD
#define LOAD_CMPB (UR==1 ? 0 : 2)                   // Load CMPB on TBCTR=0 if UR==1, otherwise on both TBCTR=0 and TBCTR=TBPRD

#define E 3.3f                                      // Available DC voltage
#define EINVERSE (1 / E)                            // Inverse of E
#define DEADTIME 0                                  // Dead time in number of counts (see EPwmXRegs.TBPRD)
#define DEADTIME_HALF (DEADTIME / 2)                // Half of the dead time (see dmach1_isr)
#define MAX_data_count 180                          // Size of an array used for data storage

// Defines for VREG ADCINA0
#define inv_tau_a 916.4223f                         // 1/(R*C)
#define alfa_a 0.2f                                 // gain for IMC based voltage regulator
#define beta_a (exp(-TS*inv_tau_a))                 // parameter that describes system dynamics beta=exp(-Ts/(R*C))
#define alfa_1beta_a (alfa_a/(1-beta_a))            // alfa/(1-beta)

// Defines for VREG ADCINA0
#define inv_tau_b 916.4223f                         // 1/(R*C)
#define alfa_b 0.2f                                 // gain for IMC based voltage regulator
#define beta_b (exp(-TS*inv_tau_b))                 // parameter that describes system dynamics beta=exp(-Ts/(R*C))
#define alfa_1beta_b (alfa_b/(1-beta_b))            // alfa/(1-beta)


#pragma CODE_SECTION(dmach1_isr, ".TI.ramfunc");    // Allocate code (dmach1_isr) in RAM
#pragma CODE_SECTION(adca1_isr, ".TI.ramfunc");    // Allocate code (adca1_isr) in RAM

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
void PrintData(void);                          // Data storage used to export monitored variables to .dat file

__interrupt void adca1_isr(void);              // JUST FOR DEBIGGING to check ADCA
//__interrupt void adcb1_isr(void);              // JUST FOR DEBIGGING to check ADCB
__interrupt void dmach1_isr(void);             // Regulation takes place in dmach1_isr

// First RC filter (connected to ADCINA0)
volatile Uint16 Measurement_a;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_a[2] = {0.0f,0.0f};             // Averaged data from DMAbuffer; AvgMeas_a[1]=previous, AvgMeas_a[0]=current
float32 Vmeas_a = 0.0f;                        // Measured voltage
float32 Vref_a = 0.0f;                         // Reference voltage
float32 u_a[2] = {0.0f,0.0f};                  // Control signal
float32 err_a[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_a = 0;                                // Duty cycle
//float32 kp_a = 1.8f, ki_a = 0.2f/(float32)UR;  // PI voltage regulator

// Second RC filter (connected to ADCINB2)
volatile Uint16 Measurement_b;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_b[2] = {0.0f,0.0f};             // Averaged data from DMAbuffer; AvgMeas_a[1]=previous, AvgMeas_a[0]=current
float32 Vmeas_b = 0.0f;                        // Measured voltage
float32 Vref_b = 0.0f;                         // Reference voltage
float32 u_b[2] = {0.0f,0.0f};                  // Control signal
float32 err_b[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_b = 0;                                // Duty cycle
//float32 kp_b = 1.8f, ki_b = 0.2f/(float32)UR;  // PI voltage regulator

float32 dataOut_1[MAX_data_count] = {};        // Data storage
float32 dataOut_2[MAX_data_count] = {};        // Data storage
Uint16 canPrint = 1;                           // Logic signal used to trigger data storage
long int data_count = 0;                       // Counter for data storage

long int dma_count = 0;                        // Counter to check dmach1_isr
long int adc_count_a = 0;                      // Counter to check adca1_isr
long int adc_count_b = 0;                      // Counter to check adcb1_isr

/*
#define MAX_buf_count 8
Uint16 dataOut_buf[MAX_buf_count*NOS_UR] = {};        // Data storage
long int buf_count = 0;
Uint16 canBuf = 0;
*/

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

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Stop the TBCLK clock
    EDIS;

    Configure_ePWM();
    Configure_ADC();
    Clear_DMAbuffer();
    DMAInitialize();
    Configure_DMA();

    //StartDMA() moved to adca1_isr so that DMA starts after NOS_UR-th adca1 count in order to achieve synchronization
    // (for some reason SOC is not sent on the very first TBCTR=0 event (after turning on TB clock))

    // Write the ISR vector for each interrupt to the appropriate location in the PIE vector table
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    //PieVectTable.ADCB1_INT = &adcb1_isr;
    PieVectTable.DMA_CH1_INT = &dmach1_isr;
    EDIS;

    // Enable interrupts on CPU level
    IER |= M_INT1;                  // PIE group1 --> CPU INT1
    IER |= M_INT7;                  // PIE group7 --> CPU INT7

    EINT;                           // Enable ST1.INTM
    ERTM;                           // Enable Global real time interrupt DBG

    // Enable interrupt on PIE level
    PieCtrlRegs.PIEIER1.bit.INTx1 = 0;          // ADCA
    //PieCtrlRegs.PIEIER1.bit.INTx2 = 1;        // ADCB
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;          // DMA

    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;          // Enable SYSCLK to ADCA
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;          // Enable SYSCLK to ADCB
    CpuSysRegs.PCLKCR0.bit.DMA = 1;             // Enable SYSCLK to DMA
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;      // Sync TBCLK with CPU clock
    EDIS;

    GpioDataRegs.GPCSET.bit.GPIO67 = 1;         // Indicate setting reference (used for osciloscope measurements)
    Vref_b = 0.5f;

    StartDMACH1();

    while(1)
        {
        }

}

void Configure_GPIO(void)
{

    EALLOW;

    // Configure as EPWM1A output (to drive first RC filter)
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;         // Mux to ePWM1A

    // Configure as EPWM1B output (to drive second RC filter)
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;         // Mux to ePWM1B

    // Configure as EPWM2A output (to notify EPWM1_TBCTR=0 & EPWM1_TBCTR=TBPRD)
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;         // Mux to ePWM2A

    // Configure as EPWM5A output (JUST FOR DEBUGGING)
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
    EDIS;

}

void Configure_ePWM(void)
{

    // EPWM1 for switching

    EPwm1Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;           // Up-down mode
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm1Regs.TBPRD = PWM_TBPRD;                        // Counter period

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;                 // Shadow mode active for CMPA
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;                 // Shadow mode active for CMPB
    EPwm1Regs.CMPCTL.bit.LOADAMODE = LOAD_CMPA;         // Load on TBCTR=0 if UR==1, otherwise on either TBCTR=0 or TBCTR=TBPRD
    EPwm1Regs.CMPCTL.bit.LOADBMODE = LOAD_CMPB;         // Load on TBCTR=0 if UR==1, otherwise on either TBCTR=0 or TBCTR=TBPRD

    EPwm1Regs.CMPA.bit.CMPA = 0;               // Value of the CMPA at the beginning
    EPwm1Regs.AQCTLA.bit.CAU = 1;              // Set EPWMA low on TBCTR=CMPA during up count
    EPwm1Regs.AQCTLA.bit.CAD = 2;              // Set EPWMA high on TBCTR=CMPA during down count

    EPwm1Regs.CMPB.bit.CMPB = 0;               // Value of the CMPB at the beginning
    EPwm1Regs.AQCTLB.bit.CBU = 1;              // Set EPWMB low on TBCTR=CMPB during up count
    EPwm1Regs.AQCTLB.bit.CBD = 2;              // Set EPWMB high on TBCTR=CMPB during down count

    // EPWM2 for notifying EPWM1_TBCTR=0 & EPWM1_TBCTR=TBPRD

    EPwm2Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1     TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;           // Up-down mode
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.TBCTL.bit.PHSEN = 0;             // Phasing disabled

    EPwm2Regs.TBPRD = PWM_TBPRD;               // Counter period

    EPwm2Regs.AQCTLA.bit.ZRO = 2;              // Set PWM A high on TBCTR=0
    EPwm2Regs.AQCTLA.bit.PRD = 1;              // Set PWM A low on TBCTR=TBPRD

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

    // set PWM output to notify SOCA (JUST FOR DEBUGGING)
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

void Clear_DMAbuffer(void)
{
    int i;
    for(i=0; i<2*NOS_UR; i++)
       {
          DMAbuffer1[i] = 0;
          DMAbuffer2[i] = 0;
       }

    Measurement_a=0;
    Measurement_b=0;

}

void PrintData()
{
    if(canPrint)
    {
        dataOut_1[data_count] =  Vmeas_b;
        dataOut_2[data_count] =  u_b[0];

        data_count++;

        if (data_count >= MAX_data_count)
        {
            data_count = 0;
            Vref_b = 0.0f;
            canPrint = 0;
        }

    }

}

__interrupt void adca1_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;             // Notify adca1_isr start

    adc_count_a++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear interrupt flag
    /*
    if(adc_count_a==NOS)
        {
            StartDMACH1();                          // Run DMA (here instead of in main to achieve synchronization)
            PieCtrlRegs.PIEIER1.bit.INTx1 = 0;      // Disable this interrupt from happening again
        }
     */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Clear acknowledge register

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;           // notify adca1_isr end



}

/*
__interrupt void adcb1_isr(void)
{
    adc_count_b++;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;         // Clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;        // Clear acknowledge register

}
    */

__interrupt void dmach1_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;             // Notify dmach1_isr start

    #if (OVERSAMPLING)
        int i_for = 0;
    #endif

    static int dma_sgn = 1;     // Logic variable to indicate state of the ping pong algorithm

    Measurement_a = 0;
    Measurement_b = 0;

    dma_count++;

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
                Measurement_a+=DMAbuffer1[i_for];
                Measurement_b+=DMAbuffer1[i_for+NOS_UR];
            }
        #else
            // Take last measurement
            Measurement_a = DMAbuffer1[NOS_UR-1];
            Measurement_b = DMAbuffer1[NOS_UR-1+NOS_UR];
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
                Measurement_a+=DMAbuffer2[i_for];
                Measurement_b+=DMAbuffer2[i_for+NOS_UR];
            }
        #else
            // Take last measurement
            Measurement_a = DMAbuffer2[NOS_UR-1];
            Measurement_b = DMAbuffer2[NOS_UR-1+NOS_UR];
        #endif
    }


    #if (OVERSAMPLING)
        // Averaging on regulation period
        AvgMeas_a[0] = Measurement_a>>((int)LOG2_NOS_UR);
        AvgMeas_b[0] = Measurement_b>>((int)LOG2_NOS_UR);
        #if (UR==2)
            // Additional averaging if UR==2 to achieve given averaging on switching period
            Vmeas_a = (float32)((AvgMeas_a[0] + AvgMeas_a[1])>>1);
            Vmeas_b = (float32)((AvgMeas_b[0] + AvgMeas_b[1])>>1);
            // Save averaged measurements for next additional averaging
            AvgMeas_a[1] = AvgMeas_a[0];
            AvgMeas_b[1] = AvgMeas_b[0];
        #else
            Vmeas_a = (float32)(AvgMeas_a[0]);
            Vmeas_b = (float32)(AvgMeas_b[0]);
        #endif
    #else
        // Take last measurement stored in DMA buffer
        Vmeas_a = (float32)Measurement_a;
        Vmeas_b = (float32)Measurement_b;
    #endif

    // ADC scaling 3.0 --> 4095 (zero offset assumed)
    Vmeas_a = Vmeas_a*0.0007326f;
    Vmeas_b = Vmeas_b*0.0007326f;

    /*
    #if (!OVERSAMPLING)
        int i_for = 0;
    #endif

    if(canBuf)
    {
        if(dma_sgn==-1) //vec si obrnula dma_sgn gore
            for (i_for=0;i_for<NOS_UR;i_for++)
               {
                   dataOut_buf[i_for+NOS_UR*buf_count] = DMAbuffer1[i_for+NOS_UR];
               }
        else if (dma_sgn==1)
            for (i_for=0;i_for<NOS_UR;i_for++)
               {
                   dataOut_buf[i_for+NOS_UR*buf_count] = DMAbuffer2[i_for+NOS_UR];
               }

        buf_count++;

        if(buf_count>=MAX_buf_count)
            {
            canBuf = 0;
            buf_count = 0;
            }
    }
*/

    // Regulation

    // First RC filter
    err_a[0] = Vref_a - Vmeas_a;
    u_a[0] = u_a[1] + alfa_1beta_a*(err_a[0] - beta_a*err_a[1]);                // IMC based regulator
    //u_a[0] = u_a[1] + (kp_a + ki_a)*err_a[0] - kp_a*err_a[1];                 // Incremental PI regulator
    if(u_a[0] > E) u_a[0] = E;                                                  // Saturation to prevent wind-up
    if(u_a[0] < 0) u_a[0] = 0;

    err_a[1] = err_a[0];
    u_a[1] = u_a[0];

    d_a = (Uint16)(PWM_TBPRD*u_a[0]*EINVERSE);                                  // Calculate duty cycle

    // Second RC filter
    err_b[0] = Vref_b - Vmeas_b;
    u_b[0] = u_b[1] + alfa_1beta_b*(err_b[0] - beta_b*err_b[1]);                // IMC based regulator
    //u_b[0] = u_b[1] + (kp_b + ki_b)*err_b[0] - kp_b*err_b[1];                 // Incremental PI regulator
    if(u_b[0] > E) u_b[0] = E;                                                  // Saturation to prevent wind-up
    if(u_b[0] < 0) u_b[0] = 0;

    err_b[1] = err_b[0];
    u_b[1] = u_b[0];

    d_b = (Uint16)(PWM_TBPRD*u_b[0]*EINVERSE);                                  // Calculate duty cycle

    EPwm1Regs.CMPA.bit.CMPA = d_a + DEADTIME_HALF;    // Set CMPA
    EPwm1Regs.CMPB.bit.CMPB = d_b + DEADTIME_HALF;    // Set CMPB

    PrintData();                                      // Data storage (JUST FOR DEBUGGING)

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;             // Notify dmach1_isr end

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;           // Clear acknowledge register

}

