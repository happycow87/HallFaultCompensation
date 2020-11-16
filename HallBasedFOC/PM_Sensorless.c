/* ==============================================================================
System Name:  	PM_Sensorless

File Name:	  	PM_Sensorless.C

Description:	Primary system file for the Real Implementation of Sensorless  
          		Field Orientation Control for Three Phase Permanent-Magnet
          		Synchronous Motor(s) (PMSM) 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8412-EVM kit. 
=====================================================================================
 History: 04-9-2010	Version 1.1: Support F2803x 
=================================================================================  */

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless-Settings.h"
#include "IQmathLib.h"
#include "PM_Sensorless.h"
#include <math.h>
#include "filter.h"
#include "dec_lut.h"
#include "F28069_sci_monitor.h"
#include "spi_dac.h"

Uint16 FaultEmulationValue = 0;
Uint16 FaultEmulationMask = 0;
Uint16 FaultEmulationSignal = 0;
Uint16 Fault_Buffer2 = 0;
Uint16 FaultEmulationBuffer = 0;
Uint16 faultFlag = 0;
Uint16 FaultStep = 0;

#ifdef DRV8301
union DRV8301_STATUS_REG_1 DRV8301_stat_reg1;
union DRV8301_STATUS_REG_2 DRV8301_stat_reg2;
union DRV8301_CONTROL_REG_1 DRV8301_cntrl_reg1;
union DRV8301_CONTROL_REG_2 DRV8301_cntrl_reg2;
Uint16 read_drv_status = 0;
#endif

// Prototype statements for functions found within this file.
interrupt void MainISR(void);
interrupt void OffsetISR(void);

void DeviceInit();
void MemCopy();
void InitFlash();
// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;

// Global variables used in this system

_iq VdTesting = _IQ(0.0);			// Vd reference (pu)
_iq VqTesting = _IQ(0.1);			// Vq reference (pu)

_iq cal_offset_A = _IQ15(0.5108);       //F28069
_iq cal_offset_B = _IQ15(0.5077);       //F28069

_iq IdRef = _IQ(0.0);				// Id reference (pu) 
_iq IqRef = _IQ(0.0);				// Iq reference (pu)
_iq SpeedRef = _IQ(0.0);	    // Speed reference (pu)

_iq offsetA = 0;
_iq offsetB = 0;

#define  OFFSET_FILTER_CORNER_FREQ 60
#define  OFFSET_LPF_K  (OFFSET_FILTER_CORNER_FREQ * 0.001 / ISR_FREQUENCY)
_iq K1 = _IQ(1.0 - OFFSET_LPF_K);   //Offset filter coefficient K1
_iq K2 = _IQ(OFFSET_LPF_K);         //Offset filter coefficient K2

_iq cal_filt_gain;	 

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h 

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint16 lsw = 0;

Uint16 SpiDacCh1=0;
Uint16 SpiDacCh2=0;
Uint16 SpiDacCh3=0;
Uint16 SpiDacCh4=0;

#if (BUILDLEVEL==LEVEL1)	 
Uint16 DRV_RESET = 1;
#else
Uint16 DRV_RESET = 0;
#endif

volatile Uint16 EnableFlag = 0;
Uint16 LockRotorFlag = FALSE;
Uint16 RunMotor = FALSE;

Uint16 SpeedLoopPrescaler = 10;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter

//	Instance a position estimator
//SMOPOS smo1 = SMOPOS_DEFAULTS;

// Instance a sliding-mode position observer constant Module
//SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;

// Instance a QEP interface driver 
QEP qep1 = QEP_DEFAULTS; 

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

PARK parkV_es = PARK_DEFAULTS;
PARK parkI_es = PARK_DEFAULTS;

_iq Vd = _IQ(0);
_iq Vq = _IQ(0);

_iq Id = _IQ(0);
_iq Iq = _IQ(0);

_iq emf_d = _IQ(0);
_iq emf_q = _IQ(0);

_iq speed_hat_old = _IQ(0);
_iq speed_hat = _IQ(0);
_iq speed_hat_lpf = _IQ(0);

_iq EMF_theta_hat = 0;
_iq EMF_theta_err = _IQ(0);
_iq EMF_speed_hat = _IQ(0);
_iq EMF_speed_hat_lpf = _IQ(0);
_iq EMF_speed_hat_old = _IQ(0);

FILTER EMF_filter = FILTER_DEFAULTS;
FILTER Hall_filter = FILTER_DEFAULTS;

_iq T_PU = (BASE_FREQ/ISR_FREQUENCY/1000.0);

// Ticker
Uint32 SpeedTicker = 0;

Uint16 currState = 0;
Uint16 prevState = 0;

_iq HallEMFThetaErr = 0.;
_iq HallEMFTheta = 0.;

Uint16 VirtualTimer = 0;
Uint16 VirtualTimer2 = 0;
Uint16 zero_spd_flag = 0;

// Hall based FOC
Uint16   HallSum = 0;
Uint16   FaultEmulationSignal_Accepted_transformation2 = 0;
typedef struct _Hall_Obj_{
	int16   HallA;		     	// Variable : A-phase Hall effect sensor signal
	int16   HallB;		     	// Variable : B-phase Hall effect sensor signal
	int16   HallC;		     	// Variable : C-phase Hall effect sensor signal
} HALL;
HALL  	H;
HALL	Hall;
_iq     HallAlpha;   		// Variable : Hall effect sensor signal on alpha axis
_iq     HallBeta;		    // Variable : Hall effect sensor signal on beta axis
_iq     HallFilteredAlpha;  // Variable : Filtered Hall effect sensor signal on alpha axis
_iq     HallFilteredBeta;   // Variable : Filtered Hall effect sensor signal on beta axis
_iq     Hall_theta_err;
_iq     Hall_theta_hat = 0;
_iq     Hall_thata_hat_offset = 0;
_iq     Hall_theta_hat_cal = -0.25137;
_iq     Hall_speed_hat_old;
_iq     Hall_speed_hat = 0;
_iq     Hall_speed_integral = 0;
_iq     Hall_speed_hat_lpf = 0;
_iq     Hall_filter_control = 12;
_iq     EMF_theta_hat_minus_Hall_thata_hat_offset = 0;
Uint32   LUTIndex;          // Variable : Index of lookup table (Q0)

_iq     thetaInput = 0;

_iq SamplingPeriod = _IQ(0.0001);

// Instance PID regulators to regulate the d and q  axis currents, and speed
PID_GRANDO_CONTROLLER pid1_id = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_iq = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_spd = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_EMF_pll = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_Hall_pll = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
SPIDAC spidac1 = SPIDAC_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGENDQ svgen_dq1 = SVGENDQ_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

//	Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//	Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

SPEED_MEAS_CAP speed_hall = SPEED_MEAS_CAP_DEFAULTS;

HALL3 hall1 = HALL3_DEFAULTS;

FILTER filter1 = FILTER_DEFAULTS;

FLTDET fltdet1 = FLTDET_DEFAULTS;

void main(void){
	
	DeviceInit();	// Device Life support & GPIO
	SPIDAC_setup();

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler
// (see TwoChannelBuck.pjt file)
#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
//    EnableFlag=TRUE;
#else
   // Waiting for enable flag set
   while (EnableFlag==FALSE)
    { 
      BackTicker++;
    }
#endif //(FLASH)

// Timing sync for slow background tasks 
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec1;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec5;		// B tasks
	CpuTimer2Regs.PRD.all =  mSec50;	// C tasks

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

// Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1 
	PWM_INIT_MACRO(pwm1)

// Initialize SPIDAC module
//    spidac1.DataMax = 0x400;   // @60Mhz: 1500->20kHz, 1000-> 30kHz, 500->60kHz
//    spidac1.SpiDacInPointer0 = &SpiDacCh1;
//    spidac1.SpiDacInPointer1 = &SpiDacCh2;
//    spidac1.SpiDacInPointer2 = &SpiDacCh3;
//    spidac1.SpiDacInPointer3 = &SpiDacCh4;

//    SPIDAC_INIT_MACRO(spidac1)

    hall1.DebounceAmount = 0;
    hall1.Revolutions = -3;
    HALL3_INIT_MACRO(hall1)

// Initialize ADC module
    ADC_MACRO()

// Initialize QEP module
    qep1.LineEncoder = 250; //RLS encoder : 250
    qep1.MechScaler = _IQ30(0.25 / qep1.LineEncoder);
    qep1.PolePairs = POLES / 2;
    qep1.CalibratedAngle = -337;
    QEP_INIT_MACRO(qep1)

// Initialize the Speed module for QEP based speed calculation
    speed1.K1 = _IQ21(1 / (BASE_FREQ * T));
    speed1.K2 = _IQ(1 / (1 + T * 2 * PI * 5));  // Low-pass cut-off frequency
    speed1.K3 = _IQ(1) - speed1.K2;
    speed1.BaseRpm = 120 * (BASE_FREQ / POLES);

// Initialize the Speed module for Hall based speed calculation
    speed_hall.InputSelect = 0;
    speed_hall.BaseRpm = 120 * (BASE_FREQ / POLES);
    speed_hall.SpeedScaler = (Uint32)(ISR_FREQUENCY / (1 *BASE_FREQ * 0.001));

// Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ * T);

// Initialize the SMOPOS constant module
//	smo1_const.Rs = RS;
//	smo1_const.Ls = LS;
//	smo1_const.Ib = BASE_CURRENT;
//	smo1_const.Vb = BASE_VOLTAGE;
//	smo1_const.Ts = T;
//	SMO_CONST_MACRO(smo1_const)

// Initialize the SMOPOS module
// 	smo1.Fsmopos = _IQ(smo1_const.Fsmopos);
// 	smo1.Gsmopos = _IQ(smo1_const.Gsmopos);
// 	smo1.Kslide = _IQ(1.5);
//  smo1.Kslf = _IQ(0.3);

// Initialize the PID_GRANDO_CONTROLLER module for Id
//  pid1_id.param.Kp = _IQ(2.5);
//  pid1_id.param.Ki = _IQ(0.001);

    pid1_id.param.Kp = _IQ(0.5);//(0.01);//0.5);
    pid1_id.param.Ki = _IQ(0.25);//(0.03);
	pid1_id.param.Kd = _IQ(0/T);
    pid1_id.param.Kr = _IQ(1.0);
	pid1_id.param.Km = _IQ(1.0);
	pid1_id.param.Umax = _IQ(0.90);
	pid1_id.param.Umin = _IQ(-0.90);

// Initialize the PID_GRANDO_CONTROLLER module for Iq
//	pid1_iq.param.Kp = _IQ(2.5);
//  pid1_iq.param.Ki = _IQ(0.001);
	pid1_iq.param.Kp = _IQ(0.5);//(0.01);//0.5);
    pid1_iq.param.Ki = _IQ(0.25);//(0.03);
	pid1_iq.param.Kd = _IQ(0/T);
    pid1_iq.param.Kr = _IQ(1.0);
	pid1_iq.param.Km = _IQ(1.0);
	pid1_iq.param.Umax = _IQ(0.95);
	pid1_iq.param.Umin = _IQ(-0.95);

// Initialize the PID_GRANDO_CONTROLLER module for Speed
//	pid1_spd.param.Kp = _IQ(3); //1
//	pid1_spd.param.Ki = _IQ(0.01);//0.001
	pid1_spd.param.Kp = _IQ(2.0);//(1.25);     //0.6     //2
    pid1_spd.param.Ki = _IQ(0.0025); //0.0015//0.005 //0.01
	pid1_spd.param.Kd = _IQ(0/(T*SpeedLoopPrescaler));
    pid1_spd.param.Kr = _IQ(1.0);
	pid1_spd.param.Km = _IQ(1.0);
	pid1_spd.param.Umax = _IQ(0.95);
	pid1_spd.param.Umin = _IQ(-0.95);

// Initialize the PID_GRANDO_CONTROLLER module for PLL
	pid1_EMF_pll.param.Kp = _IQ(0.65);
	pid1_EMF_pll.param.Ki = _IQ(0.01);
	pid1_EMF_pll.param.Kd = _IQ(0/T);
	pid1_EMF_pll.param.Km = _IQ(1.0);
	pid1_EMF_pll.param.Umax = _IQ(1);
	pid1_EMF_pll.param.Umin = _IQ(-1);

// Initialize the PID_GRANDO_CONTROLLER module for Hall
    pid1_Hall_pll.param.Kp = _IQ(0.085);//1.25
    pid1_Hall_pll.param.Ki = _IQ(0.005);
    pid1_Hall_pll.param.Kd = _IQ(0/T);
    pid1_Hall_pll.param.Kr = _IQ(1.0);
    pid1_Hall_pll.param.Km = _IQ(1.0);
    pid1_Hall_pll.param.Umax = _IQ(1);
    pid1_Hall_pll.param.Umin = _IQ(-1);

// Initialize the phase current offset calibration filter
	cal_filt_gain = _IQ15(T/(T+TC_CAL));
	
#ifdef DRV8301
// Initialize SPI for communication to the DRV8301
	DRV8301_SPI_Init(&SpibRegs);
#endif
	
// Initialize SCI for Monitor
    SCI_init();

// Reassign ISRs. 

	EALLOW;	// This is needed to write to EALLOW protected registers
//	PieVectTable.EPWM1_INT = &MainISR;
	PieVectTable.EPWM1_INT = &OffsetISR;
	PieVectTable.SCIRXINTB = &scibRxFifoIsr;
	EDIS;

// Enable PIE group 3 interrupt 1 for EPWM1_INT
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE bloc
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;     // PIE Group 9, INT3 SCIB-RX
//    PieCtrlRegs.PIEIER9.bit.INTx4=1;     // PIE Group 9, INT4 SCIB-TX


// Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation 
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

// Enable CPU INT3 for EPWM1_INT:
	IER |= M_INT3;
	IER |= M_INT9;
// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

// IDLE loop. Just sit and loop forever:	
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

//		// SCI Monitor
//		EnableFlag = Cmd_Index_Var[EN_FLAG];
//		if(rxb_finish){
//          //INDEX
//          lsw = (Uint16)Para_Index_Var[LSW].ufloat;
//          SpeedRef = _IQ(Para_Index_Var[SPEED_REF].ufloat);
//          IqRef = _IQ(Para_Index_Var[IQ_REF].ufloat);
//          IdRef = _IQ(Para_Index_Var[ID_REF].ufloat);
//          VqTesting = _IQ(Para_Index_Var[VQ_REF].ufloat);
//          VdTesting = _IQ(Para_Index_Var[VD_REF].ufloat);
//          pid1_iq.param.Kp = _IQ(Para_Index_Var[IQ_KI].ufloat);
//          pid1_iq.param.Ki = _IQ(Para_Index_Var[IQ_KI].ufloat);
//          pid1_id.param.Kp = _IQ(Para_Index_Var[ID_KI].ufloat);
//          pid1_id.param.Ki = _IQ(Para_Index_Var[ID_KI].ufloat);
//          pid1_spd.param.Kp = _IQ(Para_Index_Var[SPD_KP].ufloat);
//          pid1_spd.param.Ki = _IQ(Para_Index_Var[SPD_KI].ufloat);
//          qep1.CalibratedAngle = (Uint32)Para_Index_Var[CAL_ANG].ufloat;
//          cal_offset_A = _IQ(Para_Index_Var[CAL_A].ufloat);
//          cal_offset_B = _IQ(Para_Index_Var[CAL_B].ufloat);
//
//          //READ
//          Page0_Var[M_LSW].ufloat = (float)lsw;
//          Page0_Var[M_IQFBK].ufloat = _IQtoF(pid1_iq.term.Fbk);
//          Page0_Var[M_IDFBK].ufloat = _IQtoF(pid1_id.term.Fbk);
//          //Page0_Var[M_SPEED3].ufloat = _IQtoF(speed3.EstimatedSpeed);
//          Page0_Var[M_SPEED1].ufloat = _IQtoF(speed1.Speed);
//          Page0_Var[M_ELECTHETA].ufloat = _IQtoF(qep1.ElecTheta);
//          //Page0_Var[M_SMOTHETA].ufloat = _IQtoF(smo1.Theta);
//          Page0_Var[M_RGOUT].ufloat = _IQtoF(rg1.Out);
//          Page0_Var[M_CALANG].ufloat = _IQtoF(qep1.CalibratedAngle);
//          Page0_Var[M_CLARKEA].ufloat = _IQtoF(clarke1.As);
//          Page0_Var[M_CLARKEB].ufloat = _IQtoF(clarke1.Bs);
//          Page0_Var[M_IQI1].ufloat = _IQtoF(pid1_iq.data.i1);
//          Page0_Var[M_IDI1].ufloat = _IQtoF(pid1_id.data.i1);
//
//          rxb_finish = 0;
//      }

#ifdef DRV8301
		//read the status registers from the DRV8301
		if(read_drv_status)
		{
			if(GpioDataRegs.GPADAT.bit.GPIO14 == 0)
			{
				DRV8301_stat_reg1.all = DRV8301_SPI_Read(&SpibRegs,STAT_REG_1_ADDR);
				DRV8301_stat_reg2.all = DRV8301_SPI_Read(&SpibRegs,STAT_REG_2_ADDR); 
				read_drv_status = 0;
			}
		}
#endif		
	}
}

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;	// Back to State A0
}


//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
	if (EnableFlag == FALSE)
	{
		//de-assert the DRV830x EN_GATE pin
		#ifdef DSP2803x_DEVICE_H
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
		#endif
		#ifdef F2806x_DEVICE_H
		GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;
		#endif

		RunMotor = FALSE;
		
		EALLOW;
	 	EPwm1Regs.TZFRC.bit.OST=1;
		EPwm2Regs.TZFRC.bit.OST=1;
		EPwm3Regs.TZFRC.bit.OST=1;
	 	EDIS;
	}
	else if((EnableFlag == TRUE) && (RunMotor == FALSE))
	{

#ifdef DRV8302
#if DRV_GAIN == 10
		GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;	// GAIN = 10
#elif DRV_GAIN == 40
		GpioDataRegs.GPASET.bit.GPIO25 = 1;		// GAIN = 40
#else
#error  Invalid GAIN setting for DRV8302!!
#endif
		//GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;	// M_OC - cycle by cycle current limit
		GpioDataRegs.GPASET.bit.GPIO24 = 1;		// M_OC - fault on OC
#endif

		//if we want the power stage active we need to enable the DRV830x
		//and configure it.
		if(DRV_RESET == 0)
		{
			//assert the DRV830x EN_GATE pin
			#ifdef DSP2803x_DEVICE_H
			GpioDataRegs.GPBSET.bit.GPIO39 = 1;
			#endif
			#ifdef F2806x_DEVICE_H
			GpioDataRegs.GPBSET.bit.GPIO51 = 1;
			#endif

			DELAY_US(50000);		//delay to allow DRV830x supplies to ramp up
			
#ifdef DRV8301
			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 0;		// full current 1.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 1;		// med current 0.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 2;		// min current 0.25A
			DRV8301_cntrl_reg1.bit.GATE_RESET = 0;			// Normal Mode
			DRV8301_cntrl_reg1.bit.PWM_MODE = 0;			// six independant PWMs
//			DRV8301_cntrl_reg1.bit.OC_MODE = 0;				// current limiting when OC detected
			DRV8301_cntrl_reg1.bit.OC_MODE = 1;				// latched OC shutdown
//			DRV8301_cntrl_reg1.bit.OC_MODE = 2;				// Report on OCTWn pin and SPI reg only, no shut-down
//			DRV8301_cntrl_reg1.bit.OC_MODE = 3;				// OC protection disabled
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 0;			// OC @ Vds=0.060V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 4;			// OC @ Vds=0.097V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 6;			// OC @ Vds=0.123V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 9;			// OC @ Vds=0.175V
			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 15;			// OC @ Vds=0.358V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 16;			// OC @ Vds=0.403V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 17;			// OC @ Vds=0.454V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 18;			// OC @ Vds=0.511V
			DRV8301_cntrl_reg1.bit.Reserved = 0;
			
//			DRV8301_cntrl_reg2.bit.OCTW_SET = 0;			// report OT and OC
			DRV8301_cntrl_reg2.bit.OCTW_SET = 1;			// report OT only
#if DRV_GAIN == 10
			DRV8301_cntrl_reg2.bit.GAIN = 0;				// CS amplifier gain = 10
#elif DRV_GAIN == 20
			DRV8301_cntrl_reg2.bit.GAIN = 1;				// CS amplifier gain = 20
#elif DRV_GAIN == 40
			DRV8301_cntrl_reg2.bit.GAIN = 2;				// CS amplifier gain = 40
#elif DRV_GAIN == 80
			DRV8301_cntrl_reg2.bit.GAIN = 3;				// CS amplifier gain = 80
#endif
			DRV8301_cntrl_reg2.bit.DC_CAL_CH1 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.DC_CAL_CH2 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.OC_TOFF = 0;				// normal mode
			DRV8301_cntrl_reg2.bit.Reserved = 0;
			
			//write to DRV8301 control register 1, returns status register 1 
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpibRegs,CNTRL_REG_1_ADDR,DRV8301_cntrl_reg1.all);
			//write to DRV8301 control register 2, returns status register 1
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpibRegs,CNTRL_REG_2_ADDR,DRV8301_cntrl_reg2.all);

#endif
		}

		rg1.Freq=0;
		rg1.Out=0;
		rg1.Angle=0;
		rc1.TargetValue=0;
		rc1.SetpointValue=0;
		rc1.RampDelayMax=3;

//		smo1.Theta=0;
//		smo1.Ealpha=0;
//		smo1.Ebeta=0;

		pid1_id.data.d1 = 0;
		pid1_id.data.d2 = 0;
		pid1_id.data.i1 = 0;
		pid1_id.data.ud = 0;
		pid1_id.data.ui = 0;
		pid1_id.data.up = 0;
		pid1_id.data.v1 = 0;
		pid1_id.data.w1 = 0;
		pid1_id.term.Out = 0;

		pid1_iq.data.d1 = 0;
		pid1_iq.data.d2 = 0;
		pid1_iq.data.i1 = 0;
		pid1_iq.data.ud = 0;
		pid1_iq.data.ui = 0;
		pid1_iq.data.up = 0;
		pid1_iq.data.v1 = 0;
		pid1_iq.data.w1 = 0;
		pid1_iq.term.Out = 0;

		pid1_spd.data.d1 = 0;
		pid1_spd.data.d2 = 0;
		pid1_spd.data.i1 = 0;
		pid1_spd.data.ud = 0;
		pid1_spd.data.ui = 0;
		pid1_spd.data.up = 0;
		pid1_spd.data.v1 = 0;
		pid1_spd.data.w1 = 0;
		pid1_spd.term.Out = 0;

		pid1_EMF_pll.data.d1 = 0;
		pid1_EMF_pll.data.d2 = 0;
		pid1_EMF_pll.data.i1 = 0;
		pid1_EMF_pll.data.ud = 0;
		pid1_EMF_pll.data.ui = 0;
		pid1_EMF_pll.data.up = 0;
		pid1_EMF_pll.data.v1 = 0;
		pid1_EMF_pll.data.w1 = 0;
		pid1_EMF_pll.term.Out = 0;

		pid1_Hall_pll.data.d1 = 0;
        pid1_Hall_pll.data.d2 = 0;
        pid1_Hall_pll.data.i1 = 0;
        pid1_Hall_pll.data.ud = 0;
        pid1_Hall_pll.data.ui = 0;
        pid1_Hall_pll.data.up = 0;
        pid1_Hall_pll.data.v1 = 0;
        pid1_Hall_pll.data.w1 = 1;
        pid1_Hall_pll.term.Out = 0;

        hall1.CmtnTrigHall = 0;
        hall1.CapCounter = 0;
        hall1.DebounceCount = 0;
        hall1.DebounceAmount = 0;
        hall1.HallGpio = 0;
        hall1.HallGpioBuffer = 0;
        hall1.HallGpioAccepted = 0;
        hall1.EdgeDebounced = 0;
        hall1.HallMap[0] = 0;
        hall1.HallMap[1] = 0;
        hall1.HallMap[2] = 0;
        hall1.HallMap[3] = 0;
        hall1.HallMap[4] = 0;
        hall1.HallMap[5] = 0;
        hall1.CapFlag = 0;
        hall1.StallCount = 0xFFFF;
        hall1.HallMapPointer = 0;
        hall1.Revolutions = -3;

		lsw=0; 

		RunMotor = TRUE;
			
		EALLOW;
		EPwm1Regs.TZCLR.bit.OST=1;
		EPwm2Regs.TZCLR.bit.OST=1;
		EPwm3Regs.TZCLR.bit.OST=1;
		EDIS;

		
	}
	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{	

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A3
	A_Task_Ptr = &A3;
	//-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}



//=================================================================================
//	B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;	
	//-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;	
	//-----------------
}


//=================================================================================
//	C - TASKS (executed in every 50 msec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void) 	// Toggle GPIO-34 
//----------------------------------------
{

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	// Blink LED
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------

}

//----------------------------------------
void C2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C3
	C_Task_Ptr = &C3;	
	//-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C1
	C_Task_Ptr = &C1;	
	//-----------------
}

// MainISR 
interrupt void MainISR(void)
{

// Verifying the ISR
    IsrTicker++;

if(RunMotor){

    Node_ActiveCount[RUNTICKER].uword++;
// =============================== LEVEL 11 ======================================
//    Checks target independent modules, duty cycle waveforms and PWM update
//    Keep the motors disconnected at this level
// ==============================================================================

#if (BUILDLEVEL==LEVEL11)
      //  lsw=0: lock the rotor of the motor
      //  lsw=1: open loop
      //  lsw=2: closed the current loop
      //  lsw=4: fault detection
      //  lsw=5: closed the speed loop using EMF  Based FOC

      // ------------------------------------------------------------------------------
      //  Connect inputs of the RMP module and call the ramp control macro
      // ------------------------------------------------------------------------------
          if(lsw==0)rc1.TargetValue = 0;
          else rc1.TargetValue = SpeedRef;
          RC_MACRO(rc1)

      // ------------------------------------------------------------------------------
      //  Connect inputs of the RAMP GEN module and call the ramp generator macro
      // ------------------------------------------------------------------------------
          rg1.Freq = rc1.SetpointValue;
          RG_MACRO(rg1)

      // ------------------------------------------------------------------------------
      //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
      //  Connect inputs of the CLARKE module and call the clarke transformation macro
      // ------------------------------------------------------------------------------

          #ifdef F2806x_DEVICE_H
          clarke1.As = ((AdcResult.ADCRESULT0 - offsetA) * 0.00024414) * 2; // Phase A curr.
          clarke1.Bs = ((AdcResult.ADCRESULT1 - offsetB) * 0.00024414) * 2; // Phase B curr.
          #endif                                                         // ((ADCmeas(q12)/2^12)-0.5)*2

          #ifdef DSP2803x_DEVICE_H
      //    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
      //    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
          clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
          clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
          #endif

          CLARKE_MACRO(clarke1)

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PARK module and call the park trans. macro
      // ------------------------------------------------------------------------------
          park1.Alpha = clarke1.Alpha;
          park1.Beta = clarke1.Beta;

          if(lsw==0) park1.Angle = 0;
          else if(lsw==1 || lsw==2) park1.Angle = rg1.Out;//Hall_thata_hat_offset;//for torque control
          else if(lsw==3 || lsw==4) park1.Angle = Hall_thata_hat_offset;
          else if(lsw == 5) park1.Angle = EMF_theta_hat;

          park1.Sine = _IQsinPU(park1.Angle);
          park1.Cosine = _IQcosPU(park1.Angle);

          PARK_MACRO(park1)

      // ------------------------------------------------------------------------------
      //    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID speed controller macro
      // ------------------------------------------------------------------------------
         if (SpeedLoopCount==SpeedLoopPrescaler){
             pid1_spd.term.Ref = rc1.SetpointValue;
             if(lsw == 5) pid1_spd.term.Fbk = EMF_speed_hat_lpf;
             else pid1_spd.term.Fbk = Hall_speed_hat_lpf;
             PID_GR_MACRO(pid1_spd)
             SpeedLoopCount=1;
         }
         else SpeedLoopCount++;

         if(lsw==0 || lsw==1 || lsw==2){
              pid1_spd.data.ui=0;
              pid1_spd.data.i1=0;
         }

      // ------------------------------------------------------------------------------
      //    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
      // ------------------------------------------------------------------------------
          if(lsw==0) pid1_iq.term.Ref = 0;
          if(lsw==2) pid1_iq.term.Ref = IqRef;
          else pid1_iq.term.Ref =  pid1_spd.term.Out;
          pid1_iq.term.Fbk = park1.Qs;
          PID_GR_MACRO(pid1_iq)

      // ------------------------------------------------------------------------------
      //    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
      // ------------------------------------------------------------------------------
          if(lsw==0) pid1_id.term.Ref = _IQ(0.0);
          pid1_id.term.Ref = IdRef;
          pid1_id.term.Fbk = park1.Ds;
          PID_GR_MACRO(pid1_id)

      // ------------------------------------------------------------------------------
      //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
      // ------------------------------------------------------------------------------

          if(lsw==0){
              ipark1.Ds = 0;
              ipark1.Qs = 0;
          }
          else if(lsw==1){
              ipark1.Ds = VdTesting;
              ipark1.Qs = VqTesting;

          }
          else{
              ipark1.Ds = pid1_id.term.Out;
              ipark1.Qs = pid1_iq.term.Out;
          }

          ipark1.Sine=park1.Sine;
          ipark1.Cosine=park1.Cosine;
          IPARK_MACRO(ipark1)

      // ------------------------------------------------------------------------------
      //    Connect inputs of the VOLT_CALC module and call the phase voltage macro
      // ------------------------------------------------------------------------------
          #ifdef F2806x_DEVICE_H
          volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
          #endif                                                       // (ADCmeas(q12)/2^12)

          #ifdef DSP2803x_DEVICE_H
          volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));      // DC Bus voltage meas.
          #endif

          volt1.MfuncV1 = svgen_dq1.Ta;
          volt1.MfuncV2 = svgen_dq1.Tb;
          volt1.MfuncV3 = svgen_dq1.Tc;
          VOLT_MACRO(volt1)

      // ------------------------------------------------------------------------------
      //    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
      // ------------------------------------------------------------------------------
  //        smo1.Ialpha = clarke1.Alpha;
  //        smo1.Ibeta  = clarke1.Beta;
  //        smo1.Valpha = volt1.Valpha;
  //        smo1.Vbeta  = volt1.Vbeta;
  //        SMO_MACRO(smo1)

      // ------------------------------------------------------------------------------
      //    Connect inputs of the SPEED_EST module and call the estimated speed macro
      // ------------------------------------------------------------------------------
  //        speed3.EstimatedTheta = smo1.Theta;
  //        SE_MACRO(speed3)

      // ------------------------------------------------------------------------------
      //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
      // ------------------------------------------------------------------------------
          svgen_dq1.Ualpha = ipark1.Alpha;
          svgen_dq1.Ubeta = ipark1.Beta;
          SVGEN_MACRO(svgen_dq1)

      // ------------------------------------------------------------------------------
      //  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
      // ------------------------------------------------------------------------------
          pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
          pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);
          pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
          PWM_MACRO(pwm1)                                // Calculate the new PWM compare values

          EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
          EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
          EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC


      // ------------------------------------------------------------------------------
      //  EMF Based FOC
      // ------------------------------------------------------------------------------
          parkV_es.Alpha = volt1.Valpha;
          parkV_es.Beta = volt1.Vbeta;
          parkV_es.Angle = EMF_theta_hat;
          parkV_es.Sine = _IQsinPU(parkV_es.Angle);
          parkV_es.Cosine = _IQcosPU(parkV_es.Angle);
          PARK_MACRO(parkV_es)
          Vd = parkV_es.Ds * BASE_VOLTAGE;
          Vq = parkV_es.Qs * BASE_VOLTAGE;

          parkI_es.Alpha = clarke1.Alpha;
          parkI_es.Beta = clarke1.Beta;
          parkI_es.Angle = EMF_theta_hat;
          parkI_es.Sine = _IQsinPU(parkI_es.Angle);
          parkI_es.Cosine = _IQcosPU(parkI_es.Angle);
          PARK_MACRO(parkI_es)
          Id = parkI_es.Ds * BASE_CURRENT;
          Iq = parkI_es.Qs * BASE_CURRENT;

          emf_d = (Vd - (Id * RS) + (EMF_speed_hat * 2 * PI * BASE_FREQ * Iq * LS))/BASE_VOLTAGE;
          emf_q = (Vq - (Iq * RS) - (EMF_speed_hat * 2 * PI * BASE_FREQ * Id * LS))/BASE_VOLTAGE;

          EMF_theta_err = atan2(-emf_d, emf_q) / 2 / PI;

          pid1_EMF_pll.term.Ref = EMF_theta_err;
          pid1_EMF_pll.term.Fbk = 0;
          PID_GR_MACRO(pid1_EMF_pll);
          EMF_speed_hat = pid1_EMF_pll.term.Out;

          EMF_theta_hat += _IQmpy(_IQmpy((EMF_speed_hat_old),BASE_FREQ),T);

          if (EMF_theta_hat < 0.0) EMF_theta_hat += 1;
          if (EMF_theta_hat > 1.0) EMF_theta_hat -= 1;

          EMF_speed_hat_old = EMF_speed_hat;

          EMF_filter.RawSignal = EMF_speed_hat;
          EMF_filter.Bandwidth = 60;
          FILTER_MACRO(EMF_filter)
          EMF_speed_hat_lpf = EMF_filter.FilteredSignal;

      // ------------------------------------------------------------------------------
      //  hall sensor signal to estimate speed
      // ------------------------------------------------------------------------------

          currState = hall1.counterHall;

          if(currState == 5 && prevState == 4){
              speed_hall.TimeStamp = VirtualTimer;
              SPEED_PR_MACRO(speed_hall)
          }
          else if(currState == 0 && prevState == 1){
              speed_hall.TimeStamp = VirtualTimer;
              SPEED_PR_MACRO(speed_hall);
              speed_hall.Speed = -speed_hall.Speed;
              speed_hall.SpeedRpm = -speed_hall.SpeedRpm;
          }

          if(currState == prevState) VirtualTimer2++;
          else VirtualTimer2 = 0;

          if(SpeedRef == 0 && VirtualTimer2 > 100){
              pid1_spd.term.Out = 0;
              pid1_iq.term.Ref = 0;
              zero_spd_flag = 1;
          }
          if(zero_spd_flag == 1){
              pid1_spd.term.Out = 0;
              pid1_iq.term.Ref = 0;
              if(SpeedRef != 0) zero_spd_flag = 0;
          }
      // ------------------------------------------------------------------------------
      //  Hall Based FOC
      // ------------------------------------------------------------------------------

          HALL3_READ_MACRO(hall1)

          switch(hall1.HallGpio){//if there is deviation, can change to correct state
              case 1: HallSum = 1; break;
              case 2: HallSum = 2; break;
              case 3: HallSum = 3; break;
              case 4: HallSum = 4; break;
              case 5: HallSum = 5; break;
              case 6: HallSum = 6; break;
          }

          if(lsw==2 || lsw==4){
              //the whole if section is using programming method to achieving broken hall effect
              //could be fault at specified step
              if(FaultEmulationMask != 0 && faultFlag == 0){
                    switch(FaultStep){
                        case 1: if(hall1.HallGpio == 1){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 1;}break;
                        case 2: if(hall1.HallGpio == 2){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 2;}break;
                        case 3: if(hall1.HallGpio == 3){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 3;}break;
                        case 4: if(hall1.HallGpio == 4){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 4;}break;
                        case 5: if(hall1.HallGpio == 5){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 5;}break;
                        case 6: if(hall1.HallGpio == 6){faultFlag = 1;fltdet1.HallGpioAcceptedLog[0] = 6;}break;
                    }
                }
                else if(FaultEmulationMask == 0)faultFlag = 0;

                if(faultFlag != 0){
                    //change healthy hall signal to fault signal we want
                    //FaultEmulationMask is to assigning witch hall to be fault
                    //FaultEmulationValue is to let the fault hall to be always 1 or 0
                    FaultEmulationSignal = (HallSum & (~FaultEmulationMask));
                    FaultEmulationBuffer = (HallSum & (~FaultEmulationMask));
                    FaultEmulationSignal = (FaultEmulationSignal | (FaultEmulationValue & FaultEmulationMask));
                }
                else{
                    FaultEmulationSignal = HallSum;
                }

              // Fault detection
              fltdet1.HallGpioAccepted = FaultEmulationSignal;
              fltdet1.deHealth = FaultEmulationMask;
              FLTDET_MACRO(fltdet1)

              if(fltdet1.reverse_flag != 0){
                  FaultEmulationSignal = fltdet1.HallGpioAcceptedLog[1];
              }
              if(fltdet1.zero_vector_flag != 0){
                  FaultEmulationSignal = fltdet1.zero_vector_flag;
              }

              Fault_Buffer2 = FaultEmulationSignal;

              /* Preprocessing */ //let dummy bit become 0
              FaultEmulationSignal &= ~fltdet1.FaultType;

              //==========================================================================
              //=======   change discrete hall signal to continuous angle signal   =======
              //==========================================================================
              Hall.HallA = FaultEmulationSignal & 1 ? 1 : -1;                                                                                                      \
              Hall.HallB = FaultEmulationSignal >> 1 & 1 ? 1 : -1;                                                                                                 \
              Hall.HallC = FaultEmulationSignal >> 2 & 1 ? 1 : -1;

              HallAlpha = _IQmpy(_IQ(Hall.HallA) - _IQdiv(_IQ(Hall.HallB), _IQ(2)) - _IQdiv(_IQ(Hall.HallC), _IQ(2)), _IQdiv(_IQ(PI), _IQ(6)));
              HallBeta = _IQmpy(_IQ(Hall.HallB) - _IQ(Hall.HallC), _IQmpy(_IQsqrt(_IQ(3)), _IQdiv(_IQ(PI), _IQ(12))));

              HallFilteredAlpha = HallAlpha + _IQcosPU(Hall_theta_hat);
              HallFilteredBeta = HallBeta + _IQsinPU(Hall_theta_hat);

              if(!(fltdet1.FaultType & 1)){
                  if(Hall_theta_hat < _IQ(0.25) || Hall_theta_hat > _IQ(0.75)){
                      HallFilteredAlpha -= _IQdiv(_IQ(PI), _IQ(3));
                  }
              }
              if(!(fltdet1.FaultType & 2)){
                  if(Hall_theta_hat > _IQ(1./12) && Hall_theta_hat < _IQ(7./12)){
                      HallFilteredAlpha -= _IQmpy(_IQ(PI/3), _IQcosPU(_IQ(1./3)));
                      HallFilteredBeta -= _IQmpy(_IQ(PI/3), _IQsinPU(_IQ(1./3)));
                  }
              }
              if(!(fltdet1.FaultType & 4)){
                  if(Hall_theta_hat > _IQ(5./12) && Hall_theta_hat < _IQ(11./12)){
                      HallFilteredAlpha -= _IQmpy(_IQ(PI/3), _IQcosPU(_IQ(2./3)));
                      HallFilteredBeta -= _IQmpy(_IQ(PI/3), _IQsinPU(_IQ(2./3)));
                  }
              }
              //=================================================================================
              //========   end of change discrete hall signal to continuous angle signal   ======
              //=================================================================================
              switch(Fault_Buffer2){// change hall signal to step signal
                    case 1: FaultEmulationSignal_Accepted_transformation2 = 2; break;
                    case 2: FaultEmulationSignal_Accepted_transformation2 = 4; break;
                    case 3: FaultEmulationSignal_Accepted_transformation2 = 3; break;
                    case 4: FaultEmulationSignal_Accepted_transformation2 = 6; break;
                    case 5: FaultEmulationSignal_Accepted_transformation2 = 1; break;
                    case 6: FaultEmulationSignal_Accepted_transformation2 = 5; break;
                    case 0: FaultEmulationSignal_Accepted_transformation2 = -1; break;
                    case 7: FaultEmulationSignal_Accepted_transformation2 = -1; break;
              }

          }
          else{
              //the whole else section is real situation to check if there is broken hall and compensating the fault signal
              FaultEmulationSignal = HallSum;
              // Fault detection
              fltdet1.HallGpioAccepted = FaultEmulationSignal;
              FLTDET_MACRO(fltdet1)

              if(fltdet1.reverse_flag != 0){
                  FaultEmulationSignal = fltdet1.HallGpioAcceptedLog[1];
              }
              if(fltdet1.zero_vector_flag != 0){
                  FaultEmulationSignal = fltdet1.zero_vector_flag;
              }

              Fault_Buffer2 = FaultEmulationSignal;

              /* Preprocessing */ //let dummy bit become 0
              FaultEmulationSignal &= ~fltdet1.FaultType;

              //==========================================================================
              //=======   change discrete hall signal to continuous angle signal   =======
              //==========================================================================
              Hall.HallA = FaultEmulationSignal & 1 ? 1 : -1;                                                                                                      \
              Hall.HallB = FaultEmulationSignal >> 1 & 1 ? 1 : -1;                                                                                                 \
              Hall.HallC = FaultEmulationSignal >> 2 & 1 ? 1 : -1;

              HallAlpha = _IQmpy(_IQ(Hall.HallA) - _IQdiv(_IQ(Hall.HallB), _IQ(2)) - _IQdiv(_IQ(Hall.HallC), _IQ(2)), _IQdiv(_IQ(PI), _IQ(6)));
              HallBeta = _IQmpy(_IQ(Hall.HallB) - _IQ(Hall.HallC), _IQmpy(_IQsqrt(_IQ(3)), _IQdiv(_IQ(PI), _IQ(12))));

              HallFilteredAlpha = HallAlpha + _IQcosPU(Hall_theta_hat);
              HallFilteredBeta = HallBeta + _IQsinPU(Hall_theta_hat);

              if(!(fltdet1.FaultType & 1)){
                  if(Hall_theta_hat < _IQ(0.25) || Hall_theta_hat > _IQ(0.75)){
                      HallFilteredAlpha -= _IQdiv(_IQ(PI), _IQ(3));
                  }
              }
              if(!(fltdet1.FaultType & 2)){
                  if(Hall_theta_hat > _IQ(1./12) && Hall_theta_hat < _IQ(7./12)){
                      HallFilteredAlpha -= _IQmpy(_IQ(PI/3), _IQcosPU(_IQ(1./3)));
                      HallFilteredBeta -= _IQmpy(_IQ(PI/3), _IQsinPU(_IQ(1./3)));
                  }
              }
              if(!(fltdet1.FaultType & 4)){
                  if(Hall_theta_hat > _IQ(5./12) && Hall_theta_hat < _IQ(11./12)){
                      HallFilteredAlpha -= _IQmpy(_IQ(PI/3), _IQcosPU(_IQ(2./3)));
                      HallFilteredBeta -= _IQmpy(_IQ(PI/3), _IQsinPU(_IQ(2./3)));
                  }
              }
              //=================================================================================
              //========   end of change discrete hall signal to continuous angle signal   ======
              //=================================================================================
              switch(Fault_Buffer2){// change hall signal to step signal
                    case 1: FaultEmulationSignal_Accepted_transformation2 = 2; break;
                    case 2: FaultEmulationSignal_Accepted_transformation2 = 4; break;
                    case 3: FaultEmulationSignal_Accepted_transformation2 = 3; break;
                    case 4: FaultEmulationSignal_Accepted_transformation2 = 6; break;
                    case 5: FaultEmulationSignal_Accepted_transformation2 = 1; break;
                    case 6: FaultEmulationSignal_Accepted_transformation2 = 5; break;
                    case 0: FaultEmulationSignal_Accepted_transformation2 = -1; break;
                    case 7: FaultEmulationSignal_Accepted_transformation2 = -1; break;
              }
          }

          Hall_theta_err = HallFilteredAlpha * _IQsinPU(Hall_theta_hat) - HallFilteredBeta * _IQcosPU(Hall_theta_hat);

          pid1_Hall_pll.term.Ref = -Hall_theta_err;
          pid1_Hall_pll.term.Fbk = 0;
          PID_GR_MACRO(pid1_Hall_pll);
          Hall_speed_hat = pid1_Hall_pll.term.Out;

          Hall_speed_integral += _IQmpy(Hall_speed_hat, SamplingPeriod);

          Hall_theta_hat = _IQfrac(_IQmpy(Hall_speed_integral, _IQ(BASE_FREQ)));

          if(Hall_theta_hat < 0) Hall_theta_hat += _IQ(1);
          Hall_thata_hat_offset = Hall_theta_hat + Hall_theta_hat_cal;
          if(Hall_thata_hat_offset < 0.0) Hall_thata_hat_offset += _IQ(1);
          if(Hall_thata_hat_offset > 1.0) Hall_thata_hat_offset -= _IQ(1);

          if(Hall_thata_hat_offset > _IQ(1))        { Hall_thata_hat_offset -= _IQ(1);}
          if(Hall_thata_hat_offset < _IQ(0.0))      { Hall_thata_hat_offset += _IQ(1);}

          Hall_filter.RawSignal = Hall_speed_hat;
          Hall_filter.Bandwidth = Hall_filter_control;
          FILTER_MACRO(Hall_filter)
          Hall_speed_hat_lpf = Hall_filter.FilteredSignal;

          fltdet1.observer_spd = Hall_speed_hat_lpf;

      VirtualTimer++;
      VirtualTimer &= 0x00007FFF;

      //to check hall position accuracy
      EMF_theta_hat_minus_Hall_thata_hat_offset = EMF_theta_hat - Hall_thata_hat_offset;

      if(EMF_theta_hat_minus_Hall_thata_hat_offset > 0.5) EMF_theta_hat_minus_Hall_thata_hat_offset = -(EMF_theta_hat_minus_Hall_thata_hat_offset - 1);
      if(EMF_theta_hat_minus_Hall_thata_hat_offset < -0.5) EMF_theta_hat_minus_Hall_thata_hat_offset = -(EMF_theta_hat_minus_Hall_thata_hat_offset + 1);

      SPIDAC_write_dac_channel(0,Hall_thata_hat_offset*4095);
      SPIDAC_write_dac_channel(1,faultFlag*4095);
      SPIDAC_write_dac_channel(2,(park1.Qs)*2048+2048);//(2,(park1.Qs*7)*2048+2048);//(2,Hall_speed_hat_lpf*4095);//
      SPIDAC_write_dac_channel(3,_IQtoQ15((float)FaultEmulationSignal_Accepted_transformation2/7));//(3,(pid1_iq.term.Ref)*2048+2048);//

      SPIDAC_update_all();
      prevState = currState;
#endif

}//end if(RunMotor)

#if (DSP2803x_DEVICE_H==1)||(DSP280x_DEVICE_H==1)||(F2806x_DEVICE_H==1)
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif

}

interrupt void OffsetISR(void)
{
// Verifying the ISR
    IsrTicker++;

// DC offset measurement for ADC

    if (IsrTicker >= 5000){
        offsetA= 2052;//_IQmpy(K1,offsetA)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT0)); 		//Phase A offset
        offsetB= 2024;//_IQmpy(K1,offsetB)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT1));			//Phase B offset
    }

	if (IsrTicker > 20000){
		EALLOW;
		PieVectTable.EPWM1_INT =&MainISR;
//		PieVectTable.ADCINT1=&MainISR;
		EDIS;
	}

// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;
	//AdcRegs.ADCINTFLG.bit.ADCINT1=1;

// Acknowledge interrupt to recieve more interrupts from PIE group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	//PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
