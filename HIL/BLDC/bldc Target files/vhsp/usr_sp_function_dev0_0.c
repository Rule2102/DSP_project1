// generated using template: cop_main.template---------------------------------------------
/******************************************************************************************
**
**  Module Name: cop_main.c
**  NOTE: Automatically generated file. DO NOT MODIFY!
**  Description:
**            Main file
**
******************************************************************************************/
// generated using template: arm/custom_include.template-----------------------------------

#include "math.h"
#include <stdint.h>

// x86 libraries:
#include "../include/sp_functions_dev0.h"

// H files from Advanced C Function components

// Header files from additional sources (Advanced C Function)

// ----------------------------------------------------------------------------------------
// generated using template: VirtualHIL/custom_defines.template----------------------------

typedef unsigned char X_UnInt8;
typedef char X_Int8;
typedef signed short X_Int16;
typedef unsigned short X_UnInt16;
typedef int X_Int32;
typedef unsigned int X_UnInt32;
typedef unsigned int uint;
typedef double real;

// ----------------------------------------------------------------------------------------
// generated using template: custom_consts.template----------------------------------------

// arithmetic constants
#define C_SQRT_2                    1.4142135623730950488016887242097f
#define C_SQRT_3                    1.7320508075688772935274463415059f
#define C_PI                        3.1415926535897932384626433832795f
#define C_E                         2.7182818284590452353602874713527f
#define C_2PI                       6.283185307179586476925286766559f

//@cmp.def.start
//component defines






//@cmp.def.end


//-----------------------------------------------------------------------------------------
// generated using template: common_variables.template-------------------------------------
// true global variables


//@cmp.var.start
// variables
double _bldc_machine_wrapper1__out[0];

X_UnInt32 _constant1__out = 706;
X_UnInt32 _scada_input1__out;
float _digital_probe1__tmp;
double _integrator1__out;
X_UnInt32 _signal_switch1__out;
//@cmp.var.end

//@cmp.svar.start
// state variables
double _bldc_machine_wrapper1__model_load;
double _integrator1__state;
X_UnInt32 _integrator1__reset_state;//@cmp.svar.end

//
// Tunable parameters
//
static struct Tunable_params {
} tunable_params;

void *tunable_params_dev0_cpu0_ptr = &tunable_params;





// generated using template: virtual_hil/custom_functions.template---------------------------------
void ReInit_user_sp_cpu0_dev0() {
#if DEBUG_MODE
    printf("\n\rReInitTimer");
#endif
    //@cmp.init.block.start
    _bldc_machine_wrapper1__model_load = 0.0;
    _integrator1__state = 0.0;
    _integrator1__reset_state = 2;
    //@cmp.init.block.end
}

void ReInit_sp_scope_user_sp_cpu0_dev0() {
    // initialise SP Scope buffer pointer
}

void load_fmi_libraries_user_sp_cpu0_dev0(void) {
#if defined(_WIN64)
#else
#endif
}
// generated using template:generic_macros.template-----------------------------------------
/*********************** Macros (Inline Functions) Definitions ***************************/

// ----------------------------------------------------------------------------------------

#ifndef MAX
#define MAX(value, limit) (((value) > (limit)) ? (value) : (limit))
#endif
#ifndef MIN
#define MIN(value, limit) (((value) < (limit)) ? (value) : (limit))
#endif

// generated using template: common_timer_counter_handler.template-------------------------

/*****************************************************************************************/
/**
* This function is the handler which performs processing for the timer counter.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized.  It is called when the timer counter expires
* if interrupts are enabled.
*
*
* @param    None
*
* @return   None
*
* @note     None
*
*****************************************************************************************/

void TimerCounterHandler_0_user_sp_cpu0_dev0() {
#if DEBUG_MODE
    printf("\n\rTimerCounterHandler_0");
#endif
    //////////////////////////////////////////////////////////////////////////
    // Set tunable parameters
    //////////////////////////////////////////////////////////////////////////
    // Generated from the component: Constant1
    //////////////////////////////////////////////////////////////////////////
    // Output block
    //////////////////////////////////////////////////////////////////////////
    //@cmp.out.block.start
    // Generated from the component: BLDC.Machine Wrapper1
    HIL_OutFloat((0x800000 + 0x40000 * 0x0 + 0x30),  _bldc_machine_wrapper1__model_load * 3);
    // Generated from the component: Constant1
    // Generated from the component: SCADA Input1
    _scada_input1__out = XIo_InInt32(0x55000100);
    // Generated from the component: Digital Probe1
    HIL_OutInt32(0xf00400, _scada_input1__out != 0x0);
    // Generated from the component: Integrator1
    if ((_scada_input1__out <= 0.0) && (_integrator1__reset_state == 1)) {
        _integrator1__state = 0.0;
    }
    _integrator1__out = _integrator1__state;
    // Generated from the component: Signal switch1
    _signal_switch1__out = (_scada_input1__out != 0.0) ? _constant1__out : _scada_input1__out;
    //@cmp.out.block.end
    //////////////////////////////////////////////////////////////////////////
    // Update block
    //////////////////////////////////////////////////////////////////////////
    //@cmp.update.block.start
    // Generated from the component: BLDC.Machine Wrapper1
    _bldc_machine_wrapper1__model_load = _integrator1__out;
    // Generated from the component: Integrator1
    _integrator1__state += _signal_switch1__out * 2e-06;
    if (_scada_input1__out > 0)
        _integrator1__reset_state = 1;
    else if (_scada_input1__out < 0)
        _integrator1__reset_state = -1;
    else
        _integrator1__reset_state = 0;
    _integrator1__state = MIN(_integrator1__state, 565.4866776461628);
    _integrator1__state = MAX(_integrator1__state, 0.0);
    //@cmp.update.block.end
}
// ----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------