#ifndef __FLT_DET_H__
#define __FLT_DET_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "IQmathLib.h"

enum FaultTypes{Normal, H1, H2, H1H2, H3, H1H3, H2H3, All, Other};

typedef struct{
    Uint16 HallGpioAccepted;            // Input : Debounced logic level on ECAP/GPIO (Q0)
    Uint16 HallGpioAcceptedLog[4];      // Variable : Debounced logic level log on ECAP/GPIO (Q0)
    enum FaultTypes FaultType;          // Output : Hall effect sensor fault type
    enum FaultTypes FaultTypePrev;      // Output : Previous Hall effect sensor fault type
    _iq observer_spd;                   // speed observed by hall signal
    Uint16 reverse_flag;                // reverse flag
    Uint16 zero_vector_flag;            // zero vector flag
    Uint16 deHealth;                    // deHealth == 0, then check whether hall become healthy
} FLTDET;

typedef FLTDET *FLTDET_handle;

#define FLTDET_DEFAULTS {   \
    0,                      \
    {8, 8, 8, 8},           \
    Normal,                 \
    Normal,                 \
    0,                      \
    0,                      \
    0,                      \
    0                       \
}

#define FLTDET_MACRO(v)                                                                                                             \
        /*check reverse*/                                                                                                           \
        if((v.observer_spd > 0.05) && (v.FaultType==0)){                                                                            \
            switch(v.HallGpioAcceptedLog[0]){                                                                                       \
                case 4: if(v.HallGpioAcceptedLog[1] == 5) {v.reverse_flag = 1;} break;                                              \
                case 3: if(v.HallGpioAcceptedLog[1] == 2) {v.reverse_flag = 1;} break;                                              \
                case 1: if(v.HallGpioAcceptedLog[1] == 3) {v.reverse_flag = 2;} break;                                              \
                case 6: if(v.HallGpioAcceptedLog[1] == 4) {v.reverse_flag = 2;} break;                                              \
                case 2: if(v.HallGpioAcceptedLog[1] == 6) {v.reverse_flag = 3;} break;                                              \
                case 5: if(v.HallGpioAcceptedLog[1] == 1) {v.reverse_flag = 3;} break;                                              \
            }                                                                                                                       \
        }                                                                                                                           \
        if((v.reverse_flag != 0) && (v.HallGpioAcceptedLog[0] != v.HallGpioAccepted)){                                              \
            switch(v.reverse_flag){                                                                                                 \
                case 1: v.FaultType = H1; break;                                                                                    \
                case 2: v.FaultType = H2; break;                                                                                    \
                case 3: v.FaultType = H3; break;                                                                                    \
            }                                                                                                                       \
            v.reverse_flag = 0;                                                                                                     \
        }                                                                                                                           \
    /*use zero vector to check other single fault*/                                                                                 \
        if((v.FaultType == 0) && (v.HallGpioAcceptedLog[0] == 0 || v.HallGpioAcceptedLog[0] == 7) && v.reverse_flag == 0){          \
            switch(v.HallGpioAcceptedLog[1]){                                                                                       \
                case 1: v.zero_vector_flag = 3; break;                                                                              \
                case 4: v.zero_vector_flag = 5; break;                                                                              \
                case 3: v.zero_vector_flag = 2; break;                                                                              \
                case 6: v.zero_vector_flag = 4; break;                                                                              \
                case 2: v.zero_vector_flag = 6; break;                                                                              \
                case 5: v.zero_vector_flag = 1; break;                                                                              \
            }                                                                                                                       \
        }                                                                                                                           \
        if((v.zero_vector_flag != 0) && (v.HallGpioAcceptedLog[0] != v.HallGpioAccepted)){                                          \
            v.zero_vector_flag = 0;                                                                                                 \
        }                                                                                                                           \
        /*the next step of zero vector could know the exact fault type*/                                                            \
        if(v.HallGpioAcceptedLog[1] == 0){                                                                                          \
            switch(v.HallGpioAcceptedLog[0]){                                                                                       \
                case 1: v.FaultType = H3; break;                                                                                    \
                case 2: v.FaultType = H1; break;                                                                                    \
                case 4: v.FaultType = H2; break;                                                                                    \
            }                                                                                                                       \
        }                                                                                                                           \
        else if(v.HallGpioAcceptedLog[1] == 7){                                                                                     \
            switch(v.HallGpioAcceptedLog[0]){                                                                                       \
                case 6: v.FaultType = H3; break;                                                                                    \
                case 5: v.FaultType = H1; break;                                                                                    \
                case 3: v.FaultType = H2; break;                                                                                    \
            }                                                                                                                       \
        }                                                                                                                           \
        /*end of the next step of zero vector could know the exact fault type*/                                                     \
        /*if deHealth != don't use the method to say hall become health*/                                                           \
        /*if there is signal changing in broken hall bit, then the hall become health */                                            \
        if(v.deHealth == 0) v.FaultType &= ~(v.HallGpioAcceptedLog[0] ^ v.HallGpioAcceptedLog[1]);                                  \
        /*record the hall step*/                                                                                                    \
        if(v.HallGpioAccepted != v.HallGpioAcceptedLog[0]){                                                                         \
            v.HallGpioAcceptedLog[3] = v.HallGpioAcceptedLog[2];                                                                    \
            v.HallGpioAcceptedLog[2] = v.HallGpioAcceptedLog[1];                                                                    \
            v.HallGpioAcceptedLog[1] = v.HallGpioAcceptedLog[0];                                                                    \
            v.HallGpioAcceptedLog[0] = v.HallGpioAccepted;                                                                          \
        }


#ifdef __cplusplus
}
#endif

#endif

