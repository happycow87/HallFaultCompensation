#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "IQmathLib.h"

typedef struct{
	_iq RawSignal;			//input
	_iq FilteredSignal;		//output
	_iq Bandwidth;			//parameter unit : Hz
	_iq SmoothingFactor;	//variable
} FILTER;

typedef FILTER *FILTER_handle;

#define FILTER_DEFAULTS {	\
	0,						\
	0,						\
	60,						\
	0						\
}

#define FILTER_MACRO(v)																								\
	v.SmoothingFactor = _IQmpy(v.Bandwidth, M_PI * 2 * 0.001 / ISR_FREQUENCY);										\
	v.FilteredSignal = _IQmpy(v.FilteredSignal, 1 - v.SmoothingFactor) + _IQmpy(v.RawSignal, v.SmoothingFactor);

#ifdef __cplusplus
}
#endif

#endif
