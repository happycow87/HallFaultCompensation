//###########################################################################
//
// FILE:   F2837xD_sci_io.h
//
// TITLE:  Prototypes for SCI redirection to STDIO
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef F28069_SCI_MONITOR_H
#define F28069_SCI_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif



//
// Defines
//
#define DSP28_SCIA 0    // 1: Enable; 0: Disable
#define DSP28_SCIB 1    // 1: Enable; 0: Disable

#define CPU_FREQ         80E6
#define LSPCLK_FREQ      CPU_FREQ/4
#define SCIA_FREQ        38400  //default: 100E3
#define SCIA_PRD         (LSPCLK_FREQ/(SCIA_FREQ*8))-1
#define SCIB_FREQ        115200  //default: 100E3
#define SCIB_PRD         (LSPCLK_FREQ/(SCIB_FREQ*8))-1

//
// SCI Protocol Defines
//
/* SCI Prolog Head Table */
#define SCI_HEAD_TABLE \
    X( Scope_0,   0x53, 3, "Request scope command")\
    X( Scope_1,   0x54, 4, "Set scope channel index")\
    X( Scope_2,   0x56, 2, "Trigger DMA scope")\
    X( Scope_3,   0x51, 2, "Read scope strings and channel index in present")\
    X( Scope_4,   0x52, 2, "Request rolling scope")\
    X( Scope_5,   0x57, 2, "Request to stop rolling scope")\
    X( Scope_6,   0x50, 3, "Request page strings")\
    X( Scope_7,   0x59, 3, "Request page variable state")\
    X( Scope_8,   0x27, 9, "Request write parameter")\
    X( Scope_9,   0x25, 7, "Request read block parameter")\
    X(Scope_10,   0x17, 3, "Request command inverter")\
    X(Scope_11,   0x19, 2, "Request strings")\
    X(Scope_12,   0x61, 2, "Request node bit test")\
    X(Scope_13,   0x63, 2, "Request node bit state")\
    X(  Read_0,   0x52, 7, "Read command") // X(Prolog, Variable, TotalBytes, Comment)
//#define X(a, b, c, d) a,
//    enum Sci_Evalue { SCI_HEAD_TABLE };
//#undef X
#define X(a, b, c, d) b,
    static char Sci_Headtable[] = { SCI_HEAD_TABLE };
#undef X
#define X(a, b, c, d) c,
    static Uint16 Sci_Bytes[] = { SCI_HEAD_TABLE };
#undef X

/* Epilog Defines */
#define EPILOG 0x0D

/* Scope Defines */
#define CH1 0x00
#define CH2 0x01
#define CH3 0x02
#define CH4 0x03

/* Data Type Defines */
#define UINT8  0x06
#define UINT16 0x04
#define UINT32 0x03
#define INT8   0x05
#define INT16  0x02
#define INT32  0x01
#define FLOAT  0x00

typedef union
{
    Uint16  uword;
    struct
    {
        char  lo      :8;
        char  hi      :8;
    } ubyte;

} uUint16;

typedef union
{
    float   ufloat;
    Uint32  ulong;
    struct
    {
        Uint16  lo      :16;
        Uint16  hi      :16;
    } uword;
    struct
    {
        unsigned char  b0 :8;
        unsigned char  b1 :8;
        unsigned char  b2 :8;
        unsigned char  b3 :8;
    } ubyte;

} uUint32;


/* Page Defines */
#define PAGE0_SIZE 25
#define PAGE0_TABLE \
        X(ApeedLoop, {"ApeedLoop       "},  0x00 , 0.0)\
        X(M_LSW, {"lsw             "},  FLOAT, 0.0)\
        X(M_IQFBK, {"iq_fbk          "},  FLOAT, 0.0)\
        X(M_IDFBK, {"id_fbk          "},  FLOAT, 0.0)\
        X(M_SPEED3, {"speed3          "},  FLOAT, 0.0)\
        X(M_SPEED1, {"speed1          "},  FLOAT, 0.0)\
        X(M_ELECTHETA, {"electheta     "},  FLOAT, 0.0)\
        X(M_SMOTHETA, {"smotheta       "},  FLOAT, 0.0)\
        X(M_RGOUT, {"rgout           "},  FLOAT, 0.0)\
        X(M_CALANG, {"cal_ang         "},  FLOAT, 0.0)\
        X(M_CLARKEA, {"clarkea         "},  FLOAT, 0.0)\
        X(M_CLARKEB, {"clarkeb         "},  FLOAT, 0.0)\
        X(M_IQI1, {"iqi1            "},  FLOAT, 0.0)\
        X(M_IDI1, {"idi1            "},  FLOAT, 0.0)\
        X(ApeedLo13, {"ApeedLo13       "},  FLOAT, 0.0)\
        X(ApeedLo14, {"ApeedLo14       "},  FLOAT, 0.0)\
        X(ApeedLo15, {"ApeedLo15       "},  FLOAT, 0.0)\
        X(ApeedLo16, {"ApeedLo16       "},  FLOAT, 0.0)\
        X(ApeedLo17, {"ApeedLo17       "},  FLOAT, 0.0)\
        X(ApeedLo18, {"ApeedLo18       "},  FLOAT, 0.0)\
        X(ApeedLo19, {"ApeedLo19       "},  FLOAT, 0.0)\
        X(ApeedLo20, {"ApeedLo20       "},  FLOAT, 0.0)\
        X(ApeedLo21, {"ApeedLo21       "},  FLOAT, 0.0)\
        X(ApeedLo22, {"ApeedLo22       "},  FLOAT, 0.0)\
        X(ApeedLend, {"ApeedLend       "},  FLOAT, 0.0) //X(String, Variable)
#define X(a, b, c, d) a,
    enum Page0_Index { PAGE0_TABLE };
#undef X
#define X(a, b, c, d) b,
    static char* Page0_Str[] = { PAGE0_TABLE };
#undef X
#define X(a, b, c, d) c,
    static char Page0_VarType[] = { PAGE0_TABLE };
#undef X
extern uUint32 Page0_Var[];

#define PAGE1_SIZE 25
#define PAGE1_TABLE \
        X(BpeedLoop, {"BpeedLoop       "},  0x00 , 0.0)\
        X(BpeedLo00, {"BpeedLo00       "},  FLOAT, 0.0)\
        X(BpeedLo01, {"BpeedLo01       "},  FLOAT, 0.0)\
        X(BpeedLo02, {"BpeedLo02       "},  FLOAT, 0.0)\
        X(BpeedLo03, {"BpeedLo03       "},  FLOAT, 0.0)\
        X(BpeedLo04, {"BpeedLo04       "},  FLOAT, 0.0)\
        X(BpeedLo05, {"BpeedLo05       "},  FLOAT, 0.0)\
        X(BpeedLo06, {"BpeedLo06       "},  FLOAT, 0.0)\
        X(BpeedLo07, {"BpeedLo07       "},  FLOAT, 0.0)\
        X(BpeedLo08, {"BpeedLo08       "},  FLOAT, 0.0)\
        X(BpeedLo09, {"BpeedLo09       "},  FLOAT, 0.0)\
        X(BpeedLo10, {"BpeedLo10       "},  FLOAT, 0.0)\
        X(BpeedLo11, {"BpeedLo11       "},  FLOAT, 0.0)\
        X(BpeedLo12, {"BpeedLo12       "},  FLOAT, 0.0)\
        X(BpeedLo13, {"BpeedLo13       "},  FLOAT, 0.0)\
        X(BpeedLo14, {"BpeedLo14       "},  FLOAT, 0.0)\
        X(BpeedLo15, {"BpeedLo15       "},  FLOAT, 0.0)\
        X(BpeedLo16, {"BpeedLo16       "},  FLOAT, 0.0)\
        X(BpeedLo17, {"BpeedLo17       "},  FLOAT, 0.0)\
        X(BpeedLo18, {"BpeedLo18       "},  FLOAT, 0.0)\
        X(BpeedLo19, {"BpeedLo19       "},  FLOAT, 0.0)\
        X(BpeedLo20, {"BpeedLo20       "},  FLOAT, 0.0)\
        X(BpeedLo21, {"BpeedLo21       "},  FLOAT, 0.0)\
        X(BpeedLo22, {"BpeedLo22       "},  FLOAT, 0.0)\
        X(BpeedLend, {"BpeedLend       "},  FLOAT, 0.0) //X(String, Variable)
#define X(a, b, c, d) a,
    enum Page1_Index { PAGE1_TABLE };
#undef X
#define X(a, b, c, d) b,
    static char* Page1_Str[] = { PAGE1_TABLE };
#undef X
#define X(a, b, c, d) c,
    static char Page1_VarType[] = { PAGE1_TABLE };
#undef X
extern uUint32 Page1_Var[];

#define PAGE2_SIZE 25
#define PAGE2_TABLE \
        X(CpeedLoop, {"CpeedLoop       "},  0x00 , 0.0)\
        X(CpeedLo00, {"CpeedLo00       "},  FLOAT, 0.0)\
        X(CpeedLo01, {"CpeedLo01       "},  FLOAT, 0.0)\
        X(CpeedLo02, {"CpeedLo02       "},  FLOAT, 0.0)\
        X(CpeedLo03, {"CpeedLo03       "},  FLOAT, 0.0)\
        X(CpeedLo04, {"CpeedLo04       "},  FLOAT, 0.0)\
        X(CpeedLo05, {"CpeedLo05       "},  FLOAT, 0.0)\
        X(CpeedLo06, {"CpeedLo06       "},  FLOAT, 0.0)\
        X(CpeedLo07, {"CpeedLo07       "},  FLOAT, 0.0)\
        X(CpeedLo08, {"CpeedLo08       "},  FLOAT, 0.0)\
        X(CpeedLo09, {"CpeedLo09       "},  FLOAT, 0.0)\
        X(CpeedLo10, {"CpeedLo10       "},  FLOAT, 0.0)\
        X(CpeedLo11, {"CpeedLo11       "},  FLOAT, 0.0)\
        X(CpeedLo12, {"CpeedLo12       "},  FLOAT, 0.0)\
        X(CpeedLo13, {"CpeedLo13       "},  FLOAT, 0.0)\
        X(CpeedLo14, {"CpeedLo14       "},  FLOAT, 0.0)\
        X(CpeedLo15, {"CpeedLo15       "},  FLOAT, 0.0)\
        X(CpeedLo16, {"CpeedLo16       "},  FLOAT, 0.0)\
        X(CpeedLo17, {"CpeedLo17       "},  FLOAT, 0.0)\
        X(CpeedLo18, {"CpeedLo18       "},  FLOAT, 0.0)\
        X(CpeedLo19, {"CpeedLo19       "},  FLOAT, 0.0)\
        X(CpeedLo20, {"CpeedLo20       "},  FLOAT, 0.0)\
        X(CpeedLo21, {"CpeedLo21       "},  FLOAT, 0.0)\
        X(CpeedLo22, {"CpeedLo22       "},  FLOAT, 0.0)\
        X(CpeedLend, {"CpeedLend       "},  FLOAT, 0.0) //X(String, Variable)
#define X(a, b, c, d) a,
    enum Page2_Index { PAGE2_TABLE };
#undef X
#define X(a, b, c, d) b,
    static char* Page2_Str[] = { PAGE2_TABLE };
#undef X
#define X(a, b, c, d) c,
    static char Page2_VarType[] = { PAGE2_TABLE };
#undef X
extern uUint32 Page2_Var[];

static char** Page_Str[] = { Page0_Str,Page1_Str,Page2_Str};
static char* Page_VarType[] = { Page0_VarType,Page1_VarType,Page2_VarType};
static Uint16 Page_Size[] = {PAGE0_SIZE,PAGE1_SIZE,PAGE2_SIZE};
extern uUint32* Page_Var[];

/* Channel_Index Defines */
static Uint16 Chx_Index[4] = {0,1,2,3};
#define Channel_Index_TABLE \
        X(Cndex000, {"Cndex000"}, 0.0)\
        X(Cndex001, {"Cndex001"}, 0.0)\
        X(Cndex002, {"Cndex002"}, 0.0)\
        X(Cndex003, {"Cndex003"}, 0.0)\
        X(Cndex004, {"Cndex004"}, 0.0)\
        X(Cndex005, {"Cndex005"}, 0.0)\
        X(Cndex006, {"Cndex006"}, 0.0)\
        X(Cndex007, {"Cndex007"}, 0.0)\
        X(Cndex008, {"Cndex008"}, 0.0)\
        X(Cndex009, {"Cndex009"}, 0.0)\
        X(Cndex00A, {"Cndex00A"}, 0.0)\
        X(Cndex00B, {"Cndex00B"}, 0.0)\
        X(Cndex00C, {"Cndex00C"}, 0.0)\
        X(Cndex00D, {"Cndex00D"}, 0.0)\
        X(Cndex00E, {"Cndex00E"}, 0.0)\
        X(Cndex00F, {"Cndex00F"}, 0.0)\
        X(Cndex010, {"Cndex010"}, 0.0)\
        X(Cndex011, {"Cndex011"}, 0.0)\
        X(Cndex012, {"Cndex012"}, 0.0)\
        X(Cndex013, {"Cndex013"}, 0.0)\
        X(Cndex014, {"Cndex014"}, 0.0)\
        X(Cndex015, {"Cndex015"}, 0.0)\
        X(Cndex016, {"Cndex016"}, 0.0)\
        X(Cndex017, {"Cndex017"}, 0.0)\
        X(Cndex018, {"Cndex018"}, 0.0)\
        X(Cndex019, {"Cndex019"}, 0.0)\
        X(Cndex01A, {"Cndex01A"}, 0.0)\
        X(Cndex01B, {"Cndex01B"}, 0.0)\
        X(Cndex01C, {"Cndex01C"}, 0.0)\
        X(Cndex01D, {"Cndex01D"}, 0.0)\
        X(Cndex01E, {"Cndex01E"}, 0.0)\
        X(Cndex01F, {"Cndex01F"}, 0.0)\
        X(Cndex020, {"Cndex020"}, 0.0)\
        X(Cndex021, {"Cndex021"}, 0.0) //X(Index, Variable)
#define X(a, b, c) a,
    enum Ch_Index { Channel_Index_TABLE };
#undef X
#define X(a, b, c) b,
    static char* Ch_Index_Str[] = { Channel_Index_TABLE };
#undef X
extern uUint32 Ch_Index_Var[];

//#define X(a, b, c) c,
//    extern uUint32 Ch_Index_Var[] = { Channel_Index_TABLE };
//#undef X

/* Parameter Index Defines */
#define Parameter_Index_TABLE \
        X(LSW, {"lsw             "}, 2)\
        X(SPEED_REF, {"speedref        "}, 0.03)\
        X(IQ_REF, {"iqref           "}, 0.2)\
        X(ID_REF, {"idref           "}, 0.0)\
        X(VQ_REF, {"vqref           "}, 0.0)\
        X(VD_REF, {"vdref           "}, 0.0)\
        X(IQ_KP, {"iqkp            "}, 0.0)\
        X(IQ_KI, {"iqki            "}, 0.0)\
        X(ID_KP, {"idkp            "}, 0.0)\
        X(ID_KI, {"idki            "}, 0.0)\
        X(SPD_KP, {"spdkp           "}, 0.0)\
        X(SPD_KI, {"spdki           "}, 0.0)\
        X(CAL_ANG, {"CalibratedAngle "}, 0.0)\
        X(CAL_A, {"cala            "}, 0.0)\
        X(CAL_B, {"calb            "}, 0.0)\
        X(Pndex009, {"Pndex009        "}, 0.0)\
        X(Pndex010, {"Pndex010        "}, 0.0)\
        X(Pndex011, {"Pndex011        "}, 0.0)\
        X(Pndex012, {"Pndex012        "}, 0.0)\
        X(Pndex013, {"Pndex013        "}, 0.0)\
        X(Pndex014, {"Pndex014        "}, 0.0)\
        X(Pndex015, {"Pndex015        "}, 0.0)\
        X(Pndex016, {"Pndex016        "}, 0.0)\
        X(Pndex017, {"Pndex017        "}, 0.0)\
        X(Pndex018, {"Pndex018        "}, 0.0)\
        X(Pndex019, {"Pndex019        "}, 0.0)\
        X(Pndex01A, {"Pndex01A        "}, 0.0)\
        X(Pndex01B, {"Pndex01B        "}, 0.0)\
        X(Pndex01C, {"Pndex01C        "}, 0.0)\
        X(Pndex01D, {"Pndex01D        "}, 0.0)\
        X(Pndex01E, {"Pndex01E        "}, 0.0)\
        X(Pndex01F, {"Pndex01F        "}, 0.0)\
        X(Pndex020, {"Pndex020        "}, 0.0)\
        X(Pndex021, {"Pndex021        "}, 0.0)\
        X(Pndex022, {"Pndex022        "}, 0.0) //X(Index, Variable)
#define X(a, b, c) a,
    enum Para_Index { Parameter_Index_TABLE };
#undef X
#define X(a, b, c) b,
    static char* Para_Index_Str[] = { Parameter_Index_TABLE };
#undef X
extern uUint32 Para_Index_Var[];

/* Request Command Index Defines */
#define Command_Index_TABLE \
        X(EN_FLAG, {"EnableFlag      "}, 0)\
        X(Cmdex001, {"Cmdex001        "}, 0)\
        X(Cmdex002, {"Cmdex002        "}, 0)\
        X(Cmdex003, {"Cmdex003        "}, 0)\
        X(Cmdex004, {"Cmdex004        "}, 0)\
        X(Cmdex005, {"Cmdex005        "}, 0)\
        X(Cmdex006, {"Cmdex006        "}, 0)\
        X(Cmdex007, {"Cmdex007        "}, 0)\
        X(Cmdex008, {"Cmdex008        "}, 0)\
        X(Cmdex009, {"Cmdex009        "}, 0)\
        X(Cmdex00A, {"Cmdex00A        "}, 0)\
        X(Cmdex00B, {"Cmdex00B        "}, 0)\
        X(Cmdex00C, {"Cmdex00C        "}, 0)\
        X(Cmdex00D, {"Cmdex00D        "}, 0)\
        X(Cmdex00E, {"Cmdex00E        "}, 0)\
        X(Cmdex00F, {"Cmdex00F        "}, 0)\
        X(Cmdex010, {"Cmdex010        "}, 0)\
        X(Cmdex011, {"Cmdex011        "}, 0)\
        X(Cmdex012, {"Cmdex012        "}, 0)\
        X(Cmdex013, {"Cmdex013        "}, 0)\
        X(Cmdex014, {"Cmdex014        "}, 0)\
        X(Cmdex015, {"Cmdex015        "}, 0)\
        X(Cmdex016, {"Cmdex016        "}, 0)\
        X(Cmdex017, {"Cmdex017        "}, 0)\
        X(Cmdex018, {"Cmdex018        "}, 0)\
        X(Cmdex019, {"Cmdex019        "}, 0)\
        X(Cmdex01A, {"Cmdex01A        "}, 0)\
        X(Cmdex01B, {"Cmdex01B        "}, 0)\
        X(Cmdex01C, {"Cmdex01C        "}, 0)\
        X(Cmdex01D, {"Cmdex01D        "}, 0)\
        X(Cmdex01E, {"Cmdex01E        "}, 0)\
        X(Cmdex01F, {"Cmdex01F        "}, 0)\
        X(Cmdex020, {"Cmdex020        "}, 0)\
        X(Cmdex021, {"Cmdex021        "}, 0) //X(Index, Variable)
#define X(a, b, c) a,
    enum Cmd_Index { Command_Index_TABLE };
#undef X
#define X(a, b, c) b,
    static char* Cmd_Index_Str[] = { Command_Index_TABLE };
#undef X
extern Uint32 Cmd_Index_Var[];

/* Node Index Defines */
#define Node_Index_TABLE \
        X(RUNTICKER, {"RunTicker       "}, 0x0000, 0x0000, 0x0000)\
        X(ISRTICKER, {"IsrTicker       "}, 0x0000, 0x0000, 0x0000)\
        X(SPEED_REF_T, {"SpeedTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(IQ_REF_T, {"IqrefTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(ID_REF_T, {"IdrefTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(VQ_REF_T, {"VqrefTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(VD_REF_T, {"VdrefTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(IQ_KP_T, {"IqkpTicker      "}, 0x0000, 0x0000, 0x0000)\
        X(IQ_KI_T, {"IqkiTicker      "}, 0x0000, 0x0000, 0x0000)\
        X(ID_KP_T, {"IdkpTicker      "}, 0x0000, 0x0000, 0x0000)\
        X(ID_KI_T, {"IdkiTicker      "}, 0x0000, 0x0000, 0x0000)\
        X(SPD_KP_T, {"SpdkpTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(SPD_KI_T, {"SpdkiTicker     "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex00D, {"Nndex00D        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex00E, {"Nndex00E        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex00F, {"Nndex00F        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex010, {"Nndex010        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex011, {"Nndex011        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex012, {"Nndex012        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex013, {"Nndex013        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex014, {"Nndex014        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex015, {"Nndex015        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex016, {"Nndex016        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex017, {"Nndex017        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex018, {"Nndex018        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex019, {"Nndex019        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01A, {"Nndex01A        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01B, {"Nndex01B        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01C, {"Nndex01C        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01D, {"Nndex01D        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01E, {"Nndex01E        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex01F, {"Nndex01F        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex020, {"Nndex020        "}, 0x0000, 0x0000, 0x0000)\
        X(Nndex021, {"Nndex021        "}, 0x0000, 0x0000, 0x0000) //X(Index, Status, ActiveCount, DeactiveCount)
#define X(a, b, c, d, e) a,
    enum Node_Index { Node_Index_TABLE };
#undef X
#define X(a, b, c, d, e) b,
    static char* Node_Index_Str[] = { Node_Index_TABLE };
#undef X
extern uUint16 Node_Status[];
extern uUint16 Node_ActiveCount[];
extern uUint16 Node_DeactiveCount[];

/* Protocol Struct Defines */
typedef struct
{
    char Prolog;
    char Sort;
    char String[8];
    char ByteLen[2];
    char Messages[4];
    char Epilog;
}RSC_data;//Response Scope Command, Total Byte Len: 2061

typedef struct
{
    char Prolog;
    char Epilog;
}AD_data;//Ack Done, Response to Stop Rolling Data, Ack Write Parameter, Ack Request
//Total Byte Len: 2

typedef struct
{
    char Prolog;
    char InitCh01;
    char InitCh02;
    char InitCh03;
    char InitCh04;
    char IndexSize;
    char Strings[8];
    char Epilog;
}RSS_data;//Response Scope Strings, Total Byte Len:

typedef struct
{
    char Prolog;
    char StreamIndex;
    char Ch0xData[4][4];
    char Epilog;
}RRS_data;//Response Rolling Scope, Total Byte Len: 17

typedef struct
{
    char Prolog;
    char Page;
    char Strings[16];
    char DataType;
    char Epilog;
}APS_data;//Ack Page Strings, Total Byte Len:

typedef struct
{
    char Prolog;
    char Page;
    char Variable[25][4];
    char Epilog;
}APVS_data;//Ack Page Variable State, Total Byte Len:

typedef struct
{
    char Prolog;
    char Null;
    char Data[4];
    char Epilog;
}ARBP_data;//Ack Read Block Parameter, Total Byte Len:

typedef struct
{
    char Prolog;
    char IndexSize;
    char Name[16];
    char Epilog;
}RPS_data;//Requset Page Strings, Total Byte Len:

typedef struct
{
    char Prolog;
    char Null;
    char BitNum[2];
    char Strings[16];
    char Epilog;
}ANBT_data;//Ack Node Bit Text, Total Byte Len:

typedef struct
{
    char Prolog;
    char Null;
    char Status[2];
    char ActiveCounts[2];
    char DeactiveCounts[2];
    char Epilog;
}ANBS_data;//Ack Node Bit State, Total Byte Len:

//
// Function Prototypes
//
extern void SCI_init(void);
extern void SCIA_read(Uint16 *buf, Uint16 count);
extern void SCIB_read(Uint16 *buf, Uint16 count);
extern int SCI_write(char * buf, unsigned count);

extern interrupt void sciaTxFifoIsr(void);
extern interrupt void scibTxFifoIsr(void);
extern interrupt void sciaRxFifoIsr(void);
extern interrupt void scibRxFifoIsr(void);



extern void ResponseScopeCommand(Uint16 *buf);
extern void AckIndexDone(Uint16 *buf);
extern void AckDmaDone(Uint16 *buf);
extern void ResponseScopeStrings(Uint16 *buf);
extern void ResponseRollingScope(Uint16 *buf);
extern void ResponsetoStopRollingData(Uint16 *buf);
extern void AckPageStrings(Uint16 *buf);
extern void AckPageVariableState(Uint16 *buf);
extern void AckWriteParameter(Uint16 *buf);
extern void AckReadBlockParameter(Uint16 *buf);
extern void AckRequest(Uint16 *buf);
extern void RequsetPageStrings(Uint16 *buf);
extern void AckNodeBitText(Uint16 *buf);
extern void AckNodeBitState(Uint16 *buf);
extern void (* switch_ackfunction[])(Uint16 *buf);

extern Uint16 rxb_finish;


#ifdef __cplusplus
}
#endif /* extern "C" */


#endif

//
// End of file
//
