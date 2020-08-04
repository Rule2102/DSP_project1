#include "F28x_Project.h"
#include "C28x_FPU_FastRTS.h"
#include <string.h>

#define E 3.3f
#define EINVERSE (1 / E)
#define DEADTIME 0                              // number of counts (see EPwmXRegs.TBPRD)
#define DEADTIME_HALF (DEADTIME / 2)

#pragma CODE_SECTION(dmach1_isr, ".TI.ramfunc"); // allocate code (dma_isr) in RAM "ramfunc"

// First array for the ping pong algorithm
#pragma DATA_SECTION(DMAbuffer1,"ramgs0"); // allocate data (DMAbuffer1) in RAM "DMARAML4"
volatile Uint16 DMAbuffer1[32];

// Second array for the ping pong algorithm
#pragma DATA_SECTION(DMAbuffer2,"ramgs0"); // allocate data (DMAbuffer2) in RAM "DMARAML4"
volatile Uint16 DMAbuffer2[32];

void Configure_ePWM(void);
void Configure_ADC(void);
void Configure_DMA(void);
void Clear_DMAbuffer(void);
void Configure_GPIO(void);

__interrupt void adca1_isr(void);
__interrupt void adcb1_isr(void);
__interrupt void dmach1_isr(void);

volatile Uint16 Measurement_a;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_a[2] = {0.0f,0.0f};            // Averaging on regulation period (relevant for double update rate); AvgMeas_a[1]=previous, AvgMeas_a[0]=current
float32 Vmeas_a = 0.0f;                        // Measured voltage
float32 Vref_a = 0.1f;                         // Reference voltage
float32 u_a[2] = {0.0f,0.0f};                  // Control signal
float32 err_a[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_a = 0;                                // Duty cycle
float32 kp_a = 1.0f, ki_a = 0.2f;              // PI voltage regulator

volatile Uint16 Measurement_b;                 // Measurements (take data from DMAbuffer)
Uint16 AvgMeas_b[2] = {0.0f,0.0f};            // Averaging on regulation period (relevant for double update rate); AvgMeas_b[1]=previous, AvgMeas_b[0]=current
float32 Vmeas_b = 0.0f;                        // Measured voltage
float32 Vref_b = 0.1f;                         // Reference voltage
float32 u_b[2] = {0.0f,0.0f};                  // Control signal
float32 err_b[2] = {0.0f,0.0f};                // Error err[1]=previous, err[0]=current
Uint16 d_b = 0;                                // Duty cycle
float32 kp_b = 1.0f, ki_b = 0.2f;              // PI voltage regulator

long int dma_count = 0;
long int adc_count_a = 0;
long int adc_count_b = 0;

void main(void)
{
    // to run from FLASH uncomment the following and include in Linker ...FLASH...cmd instead of ...RAM...cmd

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

    InitPieCtrl();                  //DINT, PIECTRL, PIEIER.x, .PIEIFR.x (disable all, clear all)
    InitPieVectTable();             //PIECTRL (enable) & what else ("setup to a known state")?

    //Write the ISR vector for each interrupt to the appropriate location in the PIE vector table
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

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;           // enable ePWM5 clock - is this really necessary?
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;           // enable ePWM1 clock - is this really necessary?
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // stop the ePWM5 TBCTR clock
    EDIS;

    Configure_ePWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;       // sync ePWM TBCTR with CPU clock
    //EPwm1Regs.TBCTL.bit.SWFSYNC = 1;
    EDIS;

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

    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;          // configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;         // mux to ePWM5A

    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;         // configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // mux as ordinary GPIO

    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;          // configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;         // mux to ePWM1A

    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // configure as output
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;         // mux to ePWM1B

    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;         // configure as output
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // mux as ordinary GPIO

    EDIS;

}

void Configure_ePWM(void)
{
    EALLOW;

    // ePWM5 for ADC triggering

    EPwm5Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV = 1
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV = 1
    EPwm5Regs.TBCTL.bit.CTRMODE = 2;           // up - down mode

    EPwm5Regs.TBPRD = 156;                     // fadc=Nos*fpwm & dma_transferSize=Nos/2 (for double update rate)100MHz/(2*16*f_pwm); f_pwm=10kHz; EPWMCLKDIV = 1 -> ePWM runs with PLLSYSCLK/2

    EPwm5Regs.ETSEL.bit.SOCASEL = 2;           // ADCSOCA on TBCTR=TBPRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;            // Generate SOCA on 1st event
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;            // Enable SOCA generation

    // set PWM output to notify SOCA
    EPwm5Regs.AQCTLA.bit.ZRO = 2;              // set PWM A high on TBCTR=0
    EPwm5Regs.AQCTLA.bit.PRD = 1;              // set PWM A low on TBCTR=TBPRD

    // ePWM1 for switching

    EPwm1Regs.TBCTL.bit.CLKDIV =  0;           // CLKDIV = 1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;         // HSPCLKDIV = 1
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;           // up - down mode

    EPwm1Regs.TBPRD = 4992;                     // 100MHz/(2*f_pwm); f_pwm=10kHz; EPWMCLKDIV = 1 -> ePWM runs with PLLSYSCLK/2


    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;         // Shadow mode active for CMPA
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;         // Shadow mode active for CMPB
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;         // Load on either CTR = Zero or CTR = PRD for CMPA
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 2;         // Load on either CTR = Zero or CTR = PRD for CMPB


    EPwm1Regs.CMPA.bit.CMPA = 0;                // value of the CMPA at the beginning

    EPwm1Regs.AQCTLA.bit.CAU = 1;              // set EPWMA low on TBCTR = CMPA during up count
    EPwm1Regs.AQCTLA.bit.CAD = 2;              // set EPWMA high on TBCTR = CMPA during down count

    EPwm1Regs.CMPB.bit.CMPB = 0;                // value of the CMPB at the beginning

    EPwm1Regs.AQCTLB.bit.CBU = 1;              // set EPWMA low on TBCTR = CMPB during up count
    EPwm1Regs.AQCTLB.bit.CBD = 2;              // set EPWMB high on TBCTR = CMPB during down count

    // JUST FOR DEBUGGINNG - set PWM output to check period
    /*
    EPwm1Regs.AQCTLA.bit.ZRO = 2;              // set PWM A high on TBCTR=0
    EPwm1Regs.AQCTLA.bit.PRD = 1;              // set PWM A low on TBCTR=TBPRD
    */

    EDIS;
}

void Configure_ADC(void)
{
    EALLOW;

    // configure ADCA

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider; SYSCLK/4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Set the resolution and signal mode for a given ADC
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;          // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 30;         // acquisition window is ACQPS+1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // trigger on ePWM5 SOC0

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // EOC A0 will set ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable ADCINT1
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

    // configure ADCB

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider; SYSCLK/4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Set the resolution and signal mode for a given ADC
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;          // SOC0 will convert pin B2 (B0 is not available on launch pad)
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 30;         // acquisition window is ACQPS+1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // trigger on ePWM5 SOC0

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2;      // EOC B2 will set ADCINT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable ADCINT1
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared


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
    DmaRegs.CH1.MODE.bit.CHINTMODE = 0;             // generate ePIE interrupt at the beginning of transfer
    DmaRegs.CH1.MODE.bit.CHINTE = 1;                // Channel Interrupt to CPU enabled

    // Set up BURST registers
    DmaRegs.CH1.BURST_SIZE.all = 1; //0;                 // Number of 16-bit words per burst (N-1)
    DmaRegs.CH1.SRC_BURST_STEP = 32; //0;                 // Increment source burst address by 32 (16-bit) (ADCB result address = ADCA result adress+32)
    DmaRegs.CH1.DST_BURST_STEP = 16; //0;                 // Increment destination burst address by 16 (16-bit) DMAbuffer

    // Set up TRANSFER registers
    DmaRegs.CH1.TRANSFER_SIZE = 15;                 // Number of bursts per transfer (N-1)
    DmaRegs.CH1.SRC_TRANSFER_STEP = -32; //0;              // Increment source transfer address by -32
    DmaRegs.CH1.DST_TRANSFER_STEP = -15; //1;              // Increment destination transfer address by -16

    // Set up WRAP registers
    DmaRegs.CH1.SRC_WRAP_SIZE = 17;                 // wrap after 17 burst (wrapping disabled because wrap_size > transfer_size)
    DmaRegs.CH1.DST_WRAP_SIZE = 17;                 // wrap after 17 burst (wrapping disabled because wrap_size > transfer_size)
    DmaRegs.CH1.SRC_WRAP_STEP = 0;
    DmaRegs.CH1.DST_WRAP_STEP = 0;

    // Set up SOURCE address
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)&AdcaResultRegs.ADCRESULT0;   // Point to beginning of source buffer
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)&AdcaResultRegs.ADCRESULT0;

    // Set up DESTINATION address
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)&DMAbuffer1[0];               // Point to beginning of destination buffer
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)&DMAbuffer1[0];               // Toggle destination address in dmach1_isr

    // Clear any spurious flags & errors
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;          // Clear any spurious interrupt flags
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;             // Clear any spurious sync error

    EDIS;
}

void Clear_DMAbuffer(void)
{
    int i;
    for(i=0; i<32; i++)
       {
          DMAbuffer1[i] = 0;
          DMAbuffer2[i] = 0;
       }

    Measurement_a=0;
    Measurement_b=0;

}

__interrupt void adca1_isr(void)
{

    adc_count_a++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    /*

    // generate short pulse to notify EOC (rising edge of GPIO66)
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    DELAY_US(0.01);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;

    */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //clear acknowledge register


}

__interrupt void adcb1_isr(void)
{

    adc_count_b++;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    /*

    // generate short pulse to notify EOC (rising edge of GPIO66)
    GpioDataRegs.GPCSET.bit.GPIO67 = 1;
    DELAY_US(0.01);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //clear acknowledge register


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

        for (i_for=0;i_for<16;i_for++)
        {
            Measurement_a+=DMAbuffer2[i_for];
            Measurement_b+=DMAbuffer2[i_for+16];
        }
    }
    else if (dma_sgn==-1)
    {
        EALLOW;
        DmaRegs.CH1.DST_ADDR_SHADOW =  (Uint32)&DMAbuffer1[0];  //PONG
        EDIS;

        dma_sgn=1;

        for (i_for=0;i_for<16;i_for++)
        {
            Measurement_a+=DMAbuffer1[i_for];
            Measurement_b+=DMAbuffer1[i_for+16];
        }

    }

    AvgMeas_a[0] = Measurement_a>>4;                                            // averaging on regulation period
    Vmeas_a = (float32)((AvgMeas_a[0] + AvgMeas_a[1])>>1);                      // Averaging on PWM period
    Vmeas_a = Vmeas_a*0.000757f;                                                // ADC scaling 3.1 --> 4095 (zero offset assumed)
    AvgMeas_a[1] = AvgMeas_a[0];

    AvgMeas_b[0] = Measurement_b>>4;                                           // averaging on regulation period)
    Vmeas_b = (float32)((AvgMeas_b[0] + AvgMeas_b[1])>>1);                     // Averaging on PWM period
    Vmeas_b = Vmeas_b*0.000757f;                                               // ADC scaling 3.1 --> 4095 (zero offset assumed)
    AvgMeas_b[1] = AvgMeas_b[0];

    // regulation
    err_a[0] = Vref_a - Vmeas_a;
    u_a[0] = u_a[1] + (kp_a + ki_a)*err_a[0] - kp_a*err_a[1];                  // incremental PI regulator
    if(u_a[0] > E) u_a[0] = E;                                                 // saturation to prevent wind-up

    err_a[1] = err_a[0];
    u_a[1] = u_a[0];

    d_a = (Uint16)(EPwm1Regs.TBPRD*u_a[0]*EINVERSE); // calculate duty cycle

    err_b[0] = Vref_b - Vmeas_b;
    u_b[0] = u_b[1] + (kp_b + ki_b)*err_b[0] - kp_b*err_b[1];                // incremental PI regulator
    if(u_b[0] > E) u_b[0] = E;                                               // saturation to prevent wind-up

    err_b[1] = err_b[0];
    u_b[1] = u_b[0];

    d_b = (Uint16)(EPwm1Regs.TBPRD*u_b[0]*EINVERSE); // calculate duty cycle

    EALLOW;
    EPwm1Regs.CMPA.bit.CMPA = d_a + DEADTIME_HALF;    // set CMPA
    EPwm1Regs.CMPB.bit.CMPB = d_b + DEADTIME_HALF;    // set CMPB
    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;                    //clear acknowledge register
}

