#include "F28x_Project.h"
#include "math.h"
#include "C28x_FPU_FastRTS.h"
#include <string.h>

#define NOS 16                                       // Number of samples to be averaged on PWM period (over-sampling factor)
#define UR 2                                        // Update rate (double or single)
#define NOS_UR (NOS!=1 ? NOS/UR : 1)                // Ratio between NOS and UR (adjusted to include case without oversampling (NOS=1))
#define LOG2_NOS_UR (log2(NOS_UR))                  // Used for averaging
#define FTB 100e6                                   // FTB=EPWMCLK=SYSCLKOUT/2 (time base clock ratio to EPWM clock = 1 assumed)
#define FPWM 10e3                                   // Switching frequency
// Counter period for ePWM used for ADC triggering, up-down mode assumed (adjusted to include case without oversampling (NOS=1))
#define ADC_TBPRD ((Uint16)(NOS!=1 ? FTB/(2*FPWM*NOS)-1 : FTB/(2*FPWM*UR)-1))
// Counter period for ePWM used for switching, up-down mode assumed (adjusted to include case without oversampling (NOS=1))
#define PWM_TBPRD (NOS!=1 ? ADC_TBPRD*NOS : ADC_TBPRD*UR)
#define E 3.3f                                      // Available DC voltage
#define EINVERSE (1 / E)                            // Inverse of E
#define DEADTIME 0                                  // Dead time in number of counts (see EPwmXRegs.TBPRD)
#define DEADTIME_HALF (DEADTIME / 2)                // Half of the dead time (see dmach1_isr)
#define MAX_data_count 256                          // Size of an array used for data storage

#pragma CODE_SECTION(dmach1_isr, ".TI.ramfunc");    // Allocate code (dmach1_isr) in RAM

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
__interrupt void adcb1_isr(void);              // JUST FOR DEBIGGING to check ADCB
__interrupt void dmach1_isr(void);             // Regulation takes place in dmach1_isr

volatile Uint16 Measurement_a;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_a[2] = {0.0f,0.0f};             // Averaged data from DMAbuffer; AvgMeas_a[1]=previous, AvgMeas_a[0]=current
float32 Vmeas_a = 0.0f;                        // Measured voltage
float32 Vref_a = 0.0f;                         // Reference voltage
float32 u_a[2] = {0.0f,0.0f};                  // Control signal
float32 err_a[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_a = 0;                                // Duty cycle
float32 kp_a = 1.8f, ki_a = 0.2f/(float32)UR;  // PI voltage regulator

volatile Uint16 Measurement_b;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_b[2] = {0.0f,0.0f};             // Averaged data from DMAbuffer; AvgMeas_a[1]=previous, AvgMeas_a[0]=current
float32 Vmeas_b = 0.0f;                        // Measured voltage
float32 Vref_b = 0.0f;                         // Reference voltage
float32 u_b[2] = {0.0f,0.0f};                  // Control signal
float32 err_b[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_b = 0;                                // Duty cycle
float32 kp_b = 1.8f, ki_b = 0.2f/(float32)UR;  // PI voltage regulator

float32 dataOut_a[MAX_data_count] = {};        // Data storage
float32 dataOut_b[MAX_data_count] = {};        // Data storage
Uint16 canPrint = 0;                           // Logic signal used to trigger data storage
long int data_count = 0;                       // Counter for data storage

long int dma_count = 0;                        // Counter to check dmach1_isr
long int adc_count_a = 0;                      // Counter to check adca1_isr
long int adc_count_b = 0;                      // Counter to check adcb1_isr

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

    // Write the ISR vector for each interrupt to the appropriate location in the PIE vector table
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    PieVectTable.ADCB1_INT = &adcb1_isr;
    PieVectTable.DMA_CH1_INT = &dmach1_isr;
    EDIS;

    // Enable interrupt on PIE level
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;          // ADCA
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;          // ADCB
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;          // DMA

    // Enable interrupts on CPU level
    IER |= M_INT1;                  // PIE group1 --> CPU INT1
    IER |= M_INT7;                  // PIE group7 --> CPU INT7

    EINT;                           // Enable ST1.INTM
    ERTM;                           // Enable Global real time interrupt DBG

    Configure_ePWM();
    Configure_ADC();
    Clear_DMAbuffer();

    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;          // Enable SYSCLK to ADCA
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;          // Enable SYSCLK to ADCB
    CpuSysRegs.PCLKCR0.bit.DMA = 1;             // Enable SYSCLK to DMA
    EDIS;

    DMAInitialize();
    Configure_DMA();
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

    // Configure as EPWM5A output (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;          // Configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;         // Mux to ePWM5A

    // Configure as ordinary GPIO to notify ADCA EOC (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;         // Configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // Mux as ordinary GPIO

    // Configure as ordinary GPIO to notify ADCB EOC (JUST FOR DEBUGGING)
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;         // Configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // Mux as ordinary GPIO
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;         // ensure 0 before setting Vref in PrintData()
    EDIS;

}

void Configure_ePWM(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Stop the TBCLK clock
    EDIS;

    // EPWM1 for switching

    EPwm1Regs.TBCTL.bit.CLKDIV =  0;            // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;          // HSPCLKDIV=1
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;            // Up-down mode

    EPwm1Regs.TBPRD = PWM_TBPRD;                // Counter period

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;         // Shadow mode active for CMPA
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;         // Shadow mode active for CMPB
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;         // Load on either CTR = Zero or CTR=PRD for CMPA
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 2;         // Load on either CTR = Zero or CTR=PRD for CMPB


    EPwm1Regs.CMPA.bit.CMPA = 0;                // Value of the CMPA at the beginning
    EPwm1Regs.AQCTLA.bit.CAU = 1;               // Set EPWMA low on TBCTR=CMPA during up count
    EPwm1Regs.AQCTLA.bit.CAD = 2;               // Set EPWMA high on TBCTR=CMPA during down count

    EPwm1Regs.CMPB.bit.CMPB = 0;                // Value of the CMPB at the beginning
    EPwm1Regs.AQCTLB.bit.CBU = 1;               // Set EPWMB low on TBCTR=CMPB during up count
    EPwm1Regs.AQCTLB.bit.CBD = 2;               // Set EPWMB high on TBCTR=CMPB during down count

    // EPWM5 for ADC triggering

    EPwm5Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV=1 TBCLK=EPWMCLK/(HSPCLKDIV*CLKDIV)
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV=1
    EPwm5Regs.TBCTL.bit.CTRMODE = 2;           // Up-down mode

    EPwm5Regs.TBPRD = ADC_TBPRD;               // Counter period
    EPwm5Regs.ETSEL.bit.SOCASEL = 1;           // ADCSOCA on TBCTR=0
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;            // Generate SOCA on 1st event
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;            // Enable SOCA generation

    // set PWM output to notify SOCA (JUST FOR DEBUGGING)
    EPwm5Regs.AQCTLA.bit.ZRO = 2;              // Set PWM A high on TBCTR=0
    EPwm5Regs.AQCTLA.bit.PRD = 1;              // Set PWM A low on TBCTR=TBPRD

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;      // Sync TBCLK with CPU clock
    EDIS;
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
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable ADCINT1
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
    DmaRegs.CH1.MODE.bit.CHINTMODE = 0;             // Generate ePIE interrupt at the beginning of transfer
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
        if(data_count == 0)
        {
            GpioDataRegs.GPCSET.bit.GPIO67 = 1;
            Vref_a = 1.0f;
            Vref_b = 1.0f;
        }

        dataOut_a[data_count] = Vmeas_a;
        dataOut_b[data_count] = Vmeas_b;
        data_count++;

        if (data_count >= MAX_data_count)
        {
            data_count = 0;
            canPrint=0;
            GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
            Vref_a = 0.0f;
            Vref_b = 0.0f;
        }
    }

}

__interrupt void adca1_isr(void)
{
    adc_count_a++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    // Generate short pulse to notify EOC (JUST FOR DEBUGGING)
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    DELAY_US(0.01);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Clear acknowledge register

}

__interrupt void adcb1_isr(void)
{
    adc_count_b++;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    // Generate short pulse to notify EOC (JUST FOR DEBUGGING)
    /*
    GpioDataRegs.GPCSET.bit.GPIO67 = 1;
    DELAY_US(0.01);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
    */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;        // Clear acknowledge register

}


__interrupt void dmach1_isr(void)
{
    int i_for;
    static int dma_sgn = 1;

    Measurement_a = 0;
    Measurement_b = 0;

    dma_count++;

    // change sign of the pointer to indicate state of the ping pong algorithm
    if (dma_sgn==1)
    {
        EALLOW;
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)&DMAbuffer2[0];  //PING
        EDIS;
        dma_sgn=-1;

        for (i_for=0;i_for<NOS_UR;i_for++)
        {
            Measurement_a+=DMAbuffer2[i_for];
            Measurement_b+=DMAbuffer2[i_for+NOS_UR];
        }
    }
    else if (dma_sgn==-1)
    {
        EALLOW;
        DmaRegs.CH1.DST_ADDR_SHADOW =  (Uint32)&DMAbuffer1[0];  //PONG
        EDIS;

        dma_sgn=1;

        for (i_for=0;i_for<NOS_UR;i_for++)
        {
            Measurement_a+=DMAbuffer1[i_for];
            Measurement_b+=DMAbuffer1[i_for+NOS_UR];
        }

    }

    AvgMeas_a[0] = Measurement_a>>((int)LOG2_NOS_UR);                           // Averaging on regulation period
#if (UR==2 && NOS!=1)
    Vmeas_a = (float32)((AvgMeas_a[0] + AvgMeas_a[1])>>1);                      // Additional averaging if UR==2
#else
    Vmeas_a = (float32)(AvgMeas_a[0]);
#endif
    Vmeas_a = Vmeas_a*0.0007326f;                                                // ADC scaling 3.1 --> 4095 (zero offset assumed)
    AvgMeas_a[1] = AvgMeas_a[0];

    AvgMeas_b[0] = Measurement_b>>((int)LOG2_NOS_UR);                           // Averaging on regulation period
#if (UR==2 && NOS!=1)
    Vmeas_b = (float32)((AvgMeas_b[0] + AvgMeas_b[1])>>1);                      // Additional averaging if UR==2
#else
    Vmeas_b = (float32)(AvgMeas_b[0]);
#endif
    Vmeas_b = Vmeas_b*0.0007326f;                                                // ADC scaling 3.1 --> 4095 (zero offset assumed)
    AvgMeas_b[1] = AvgMeas_b[0];

    // Regulation

    // First RC filter
    err_a[0] = Vref_a - Vmeas_a;
    u_a[0] = u_a[1] + (kp_a + ki_a)*err_a[0] - kp_a*err_a[1];                   // Incremental PI regulator
    if(u_a[0] > E) u_a[0] = E;                                                  // Saturation to prevent wind-up
    if(u_a[0] < 0) u_a[0] = 0;

    err_a[1] = err_a[0];
    u_a[1] = u_a[0];

    d_a = (Uint16)(PWM_TBPRD*u_a[0]*EINVERSE);                                  // Calculate duty cycle

    // Second RC filter
    err_b[0] = Vref_b - Vmeas_b;
    u_b[0] = u_b[1] + (kp_b + ki_b)*err_b[0] - kp_b*err_b[1];                   // Incremental PI regulator
    if(u_b[0] > E) u_b[0] = E;                                                  // Saturation to prevent wind-up
    if(u_b[0] < 0) u_b[0] = 0;

    err_b[1] = err_b[0];
    u_b[1] = u_b[0];

    d_b = (Uint16)(PWM_TBPRD*u_b[0]*EINVERSE);                                  // Calculate duty cycle

    EALLOW;
    EPwm1Regs.CMPA.bit.CMPA = d_a + DEADTIME_HALF;    // Set CMPA
    EPwm1Regs.CMPB.bit.CMPB = d_b + DEADTIME_HALF;    // Set CMPB
    EDIS;

    PrintData();                                      // Data storage (JUST FOR DEBUGGING)

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;           // Clear acknowledge register

}

