//#############################################################################
//
// File:   F2837xD_sci_monitor.c
//
// Description:  Contains the various functions related to the serial
//               communications interface (SCI) object
//
//#############################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

//
// Included Files
//
#include "PeripheralHeaderIncludes.h"
#include "F28069_sci_monitor.h"
#include "string.h"

//
// Globals
//
Uint16 deviceOpen = 0;
#if DSP28_SCIA
Uint16 sdataA[20];    // Send data for SCI-A
Uint16 rdataA[20];    // Received data for SCI-A
Uint16 rxa_finish = 0;
Uint16 rxa_count = 0;
Uint16 rxa_length = 0;
Uint16 rxa_index = 0;
#endif
#if DSP28_SCIB
Uint16 sdataB[20];    // Send data for SCI-B
Uint16 rdataB[20];    // Received data for SCI-B
Uint16 rxb_finish = 0;
Uint16 rxb_count = 0;
Uint16 rxb_length = 0;
Uint16 rxb_index = 255;
#endif


#define X(a, b, c, d) d,
    uUint32 Page0_Var[] = { PAGE0_TABLE };
    uUint32 Page1_Var[] = { PAGE1_TABLE };
    uUint32 Page2_Var[] = { PAGE2_TABLE };
#undef X
uUint32* Page_Var[] = { Page0_Var,Page1_Var,Page2_Var};
#define X(a, b, c) c,
    uUint32 Ch_Index_Var[] = { Channel_Index_TABLE };
#undef X
#define X(a, b, c) c,
    uUint32 Para_Index_Var[] = { Parameter_Index_TABLE };
#undef X
#define X(a, b, c) c,
    Uint32 Cmd_Index_Var[] = { Command_Index_TABLE };
#undef X
#define X(a, b, c, d, e) c,
    uUint16 Node_Status[] = { Node_Index_TABLE };
#undef X
#define X(a, b, c, d, e) d,
    uUint16 Node_ActiveCount[] = { Node_Index_TABLE };
#undef X
#define X(a, b, c, d, e) e,
    uUint16 Node_DeactiveCount[] = { Node_Index_TABLE };
#undef X

// for test
Uint32 testsize=0;
Uint32 testsize1=0;
char testdata[4]={0,0,0,0};
uUint32 test={0};

//
// Function Prototypes
//
void Scia_GpioInit(void);
void Scib_GpioInit(void);
void Scia_Setting(void);
void Scib_Setting(void);

void ClearSciaData(void);
void ClearScibData(void);
void error(void);


//
// error - Function to halt debugger on error
//
void error(void)
{
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

#if DSP28_SCIA
//
// ClearSciaData - Function to clear rx data
//
void ClearSciaData(void)
{
    Uint16 i=0;
    for(i=0;i<sizeof(rdataA);i++)
        rdataA[i] = 0;
    rxa_count = 0;
    rxa_index = 255;
}

//
// sciaTxFifoIsr - SCIA Transmit FIFO ISR
// SPARE (not used)
//
interrupt void sciaTxFifoIsr(void)
{
    /*
    Uint16 i;
    if(rx_finish)
    {
        for(i=0; i< 2; i++)
        {
            sdata[i]=rdata[i];
            SciaRegs.SCITXBUF.all=sdata[i];  // Send data    sdata[i]
            SciaRegs.SCITXBUF.all=0x55;  // Send data    sdata[i]
            ScibRegs.SCITXBUF.all=0x55;  // Send data    sdataB[i]
        }
        rx_finish = 0;
    }
     */

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void)
{
    int i;
    rdataA[rxa_count] = SciaRegs.SCIRXBUF.all & 0x00FF; // read SCI data

    rxa_count++;

    if(rxa_count==1){
        rxa_finish = 0;

        for(i=0;i<sizeof(Sci_Headtable);i++)
            {
                if(rdataA[0] == Sci_Headtable[i])
                {
                    rxa_index = i;
                    rxa_length = Sci_Bytes[i];
                    break;
                }
            }
        if(rxa_index==255)
        {
            ClearSciaData();
        }

    }else
    {
        if(rdataA[rxa_length-1]==EPILOG)
        {
            switch_ackfunction[rxa_index](rdataA);
            ClearSciaData();
            rxa_finish = 1;
        }
        else if(rdataA[rxa_length-1]!=EPILOG && rxa_count>=rxa_length)
        {
            ClearSciaData();
            rxa_finish = 0;
        }
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}
#endif

#if DSP28_SCIB
//
// ClearScibData - Function to clear rx data
//
void ClearScibData(void)
{
    Uint16 i=0;
    for(i=0;i<sizeof(rdataB);i++)
        rdataB[i] = 0;
    rxb_count = 0;
    rxb_index = 255;
}

//
// scibTxFifoIsr - SCIB Transmit FIFO ISR
// SPARE (not used)
//
interrupt void scibTxFifoIsr(void)
{
    /*Add code here*/

    /*end*/

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// scibRxFifoIsr - SCIB Receive FIFO ISR
//
interrupt void scibRxFifoIsr(void)
{
    // Enable nested interrupts
    Uint16 TempPIEIER;
    TempPIEIER = PieCtrlRegs.PIEIER9.all; // Save PIEIER register for later
    IER |= M_INT3; // Set global priority by adjusting IER
    IER &= M_INT3;
    PieCtrlRegs.PIEACK.all = 0xFFFF;      // Enable PIE interrupts
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    //
    // Insert ISR Code here.......
    //
    int i;
    rdataB[rxb_count] = ScibRegs.SCIRXBUF.all & 0x00FF; // read SCI data
    rxb_count++;

    if(rxb_count==1){
        rxb_finish = 0;
        for(i=0;i<sizeof(Sci_Headtable);i++)
            {
                if(rdataB[0] == Sci_Headtable[i])
                {
                    rxb_index = i;
                    rxb_length = Sci_Bytes[i];
                    break;
                }
            }
        if(rxb_index==255)
        {
            ClearScibData();
        }
    }else
    {
        if(rdataB[rxb_length-1]==EPILOG)
        {
            switch_ackfunction[rxb_index](rdataB);
            ClearScibData();
            rxb_finish = 1;
        }
        else if(rdataB[rxb_length-1]!=EPILOG && rxb_count>=rxb_length)
        {
            ClearScibData();
            rxb_finish = 0;
        }
    }

        ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
        ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack

    //
    // ISR Code end
    //
    DINT;
    PieCtrlRegs.PIEIER9.all = TempPIEIER;
}
#endif


//
// SCIA_read - Read from the SCI RX buffer
//
void SCIA_read(Uint16 *buf, Uint16 count)
{
    Uint16 readCount = 0;
    Uint16 *bufPtr = buf;

    while((readCount < count) && SciaRegs.SCIRXST.bit.RXRDY)
    {
        *bufPtr = SciaRegs.SCIRXBUF.bit.RXDT;
        readCount++;
        bufPtr++;
    }
}

//
// SCIB_read - Read from the SCI RX buffer
//
void SCIB_read(Uint16 *buf, Uint16 count)
{
    Uint16 readCount = 0;
    Uint16 *bufPtr = buf;

    while((readCount < count) && ScibRegs.SCIRXST.bit.RXRDY)
    {
        *bufPtr = ScibRegs.SCIRXBUF.bit.RXDT;
        readCount++;
        bufPtr++;
    }
}

//
// SCI_write - Write to the SCI TX buffer
//
int SCI_write(char *buf, unsigned count)
{
    Uint16 writeCount = 0;
    Uint16* bufPtr = (Uint16*) buf;

    if(count == 0)
    {
        return (0);
    }

    while(writeCount < count)
    {
#if DSP28_SCIA
        while(SciaRegs.SCICTL2.bit.TXRDY != 1)
        {
        }
        SciaRegs.SCITXBUF = *bufPtr;
#endif
#if DSP28_SCIB
        while(ScibRegs.SCICTL2.bit.TXRDY != 1)
        {
        }
        ScibRegs.SCITXBUF = *bufPtr;
#endif
        writeCount++;
        bufPtr++;
    }
    return (writeCount);
}


//
// Scope - Response Scope Command
// buf[0]: Prolog, buf[1]: Channel, buf[2]: Epilog
//
void ResponseScopeCommand(Uint16 *buf)
{
    Uint16 i = 0;
    RSC_data tx_buf;
    uUint16 Varlen = {1};     //ByteLen

    tx_buf.Prolog=buf[0];   // 0x53
    tx_buf.Sort=buf[1];
    strncpy(tx_buf.String,Ch_Index_Str[Chx_Index[buf[1]]],8);
    tx_buf.ByteLen[0]=Varlen.ubyte.lo;
    tx_buf.ByteLen[1]=Varlen.ubyte.hi;
    SCI_write(&tx_buf.Prolog,12);

    for(i=0;i<Varlen.uword;i++){ //i<ByteLen
        tx_buf.Messages[0]=Ch_Index_Var[Chx_Index[buf[1]]].ubyte.b0;
        tx_buf.Messages[1]=Ch_Index_Var[Chx_Index[buf[1]]].ubyte.b1;
        tx_buf.Messages[2]=Ch_Index_Var[Chx_Index[buf[1]]].ubyte.b2;
        tx_buf.Messages[3]=Ch_Index_Var[Chx_Index[buf[1]]].ubyte.b3;
        SCI_write(tx_buf.Messages,4);
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// Scope - Ack Index Done
// buf[0]: Prolog, buf[1]: Channel, buf[2]: Channel Index, buf[3]: Epilog
//
void AckIndexDone(Uint16 *buf)
{

    AD_data tx_buf={0x54,EPILOG};

    Chx_Index[buf[1]] = buf[2];
    SCI_write(&tx_buf.Prolog,2);
}

//
// Scope - Ack Dma Done
// buf[0]: Prolog, buf[1]: Epilog
//
void AckDmaDone(Uint16 *buf)
{
    AD_data tx_buf = {0x56,EPILOG};
    SCI_write(&tx_buf.Prolog,2);
}

//
// Scope - Response Scope Strings
// buf[0]: Prolog, buf[1]: Epilog
//
void ResponseScopeStrings(Uint16 *buf)
{
    int i = 0;
    RSS_data tx_buf;

    tx_buf.Prolog=buf[0]; // 0x51
    tx_buf.InitCh01=0x00;
    tx_buf.InitCh02=0x01;
    tx_buf.InitCh03=0x02;
    tx_buf.InitCh04=0x03;
    tx_buf.IndexSize=sizeof(Ch_Index_Str)/sizeof(Ch_Index_Str[0]);
    SCI_write(&tx_buf.Prolog,6);

    for(i=0;i<tx_buf.IndexSize;i++)
    {
        strncpy(tx_buf.Strings,Ch_Index_Str[i],8);
        SCI_write(tx_buf.Strings,8);
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// Scope - Response Rolling Scope
// buf[0]: Prolog, buf[1]: Epilog
//
void ResponseRollingScope(Uint16 *buf)
{
    int i = 0;
    RRS_data tx_buf;

    tx_buf.Prolog=buf[0]; // 0x52
    tx_buf.StreamIndex=0xFF;
    for(i=0;i<4;i++){ //i<ByteLen
        tx_buf.Ch0xData[i][0]=Ch_Index_Var[Chx_Index[i]].ubyte.b0;
        tx_buf.Ch0xData[i][1]=Ch_Index_Var[Chx_Index[i]].ubyte.b1;
        tx_buf.Ch0xData[i][2]=Ch_Index_Var[Chx_Index[i]].ubyte.b2;
        tx_buf.Ch0xData[i][3]=Ch_Index_Var[Chx_Index[i]].ubyte.b3;
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Prolog,19);
}

//
// Scope - Response to Stop Rolling Data
// buf[0]: Prolog, buf[1]: Epilog
//
void ResponsetoStopRollingData(Uint16 *buf)
{
    AD_data tx_buf = {0x57,EPILOG};
    SCI_write(&tx_buf.Prolog,2);
}

//
// MonitorBlock - Ack Page Strings
// buf[0]: Prolog, buf[1]: Page, buf[2]: Epilog
//
void AckPageStrings(Uint16 *buf)
{
    Uint16 i = 0;
    APS_data tx_buf;

    tx_buf.Prolog=buf[0]; // 0x50
    tx_buf.Page=buf[1];
    SCI_write(&tx_buf.Prolog,2);


    for(i=0;i<Page_Size[tx_buf.Page];i++)
    {
        strncpy(tx_buf.Strings,Page_Str[tx_buf.Page][i],16);
        SCI_write(tx_buf.Strings,16);
    }
    for(i=1;i<Page_Size[tx_buf.Page];i++)
    {
        tx_buf.DataType=Page_VarType[tx_buf.Page][i];
        SCI_write(&tx_buf.DataType,1);
    }
    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);

}

//
// MonitorBlock - Ack Page Variable State
// buf[0]: Prolog, buf[1]: Page, buf[2]: Epilog
//
void AckPageVariableState(Uint16 *buf)
{
    Uint16 i = 0;
    APVS_data tx_buf;

    tx_buf.Prolog=buf[0]; // 0x59
    tx_buf.Page=buf[1];
    SCI_write(&tx_buf.Prolog,2);

    for(i=0;i<Page_Size[tx_buf.Page];i++)
    {
        tx_buf.Variable[i][0]=Page_Var[tx_buf.Page][i+1].ubyte.b0;
        tx_buf.Variable[i][1]=Page_Var[tx_buf.Page][i+1].ubyte.b1;
        tx_buf.Variable[i][2]=Page_Var[tx_buf.Page][i+1].ubyte.b2;
        tx_buf.Variable[i][3]=Page_Var[tx_buf.Page][i+1].ubyte.b3;
        SCI_write(tx_buf.Variable[i],4);
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// ReadWrite Command - Ack Write Parameter
// buf[ 0 ]: Prolog, buf[ 1 ]: Null,
// buf[2-3]: Index,  buf[3-6]: Data(float),
// buf[ 7 ]: Epilog
//
void AckWriteParameter(Uint16 *buf)
{
    AD_data tx_buf = {0x27,EPILOG};
    uUint16 Index = {0};

    Index.ubyte.lo=buf[2];
    Index.ubyte.hi=buf[3];

    Para_Index_Var[Index.uword].ubyte.b0=buf[4];
    Para_Index_Var[Index.uword].ubyte.b1=buf[5];
    Para_Index_Var[Index.uword].ubyte.b2=buf[6];
    Para_Index_Var[Index.uword].ubyte.b3=buf[7];

    SCI_write(&tx_buf.Prolog,2);
}

//
// ReadWrite Command - Ack Read Block Parameter
// buf[ 0 ]: Prolog,     buf[ 1 ]: Null,
// buf[2-3]: IndexStart, buf[3-4]: IndexStop,
// buf[ 5 ]: Epilog
//
void AckReadBlockParameter(Uint16 *buf)
{

    uUint16 IndexStart = {0};
    uUint16 IndexStop = {0};
    Uint16 i = 0;

    ARBP_data tx_buf;
    tx_buf.Prolog=buf[0]; // 0x25
    tx_buf.Null=buf[1]; // 0x00
    SCI_write(&tx_buf.Prolog,2);

    IndexStart.ubyte.lo=buf[2];
    IndexStart.ubyte.hi=buf[3];
    IndexStop.ubyte.lo=buf[4];
    IndexStop.ubyte.hi=buf[5];


    for(i=IndexStart.uword;i<=IndexStop.uword;i++)
    {
        tx_buf.Data[0]=Para_Index_Var[i].ubyte.b0;
        tx_buf.Data[1]=Para_Index_Var[i].ubyte.b1;
        tx_buf.Data[2]=Para_Index_Var[i].ubyte.b2;
        tx_buf.Data[3]=Para_Index_Var[i].ubyte.b3;
        SCI_write(tx_buf.Data,4);
    }
    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// Request Command - Ack Request
// buf[0]: Prolog, buf[1]: Request command, buf[2]: Epilog
//
void AckRequest(Uint16 *buf)
{
    Cmd_Index_Var[buf[1]] = (Cmd_Index_Var[buf[1]]==1)? 0 : 1;

    AD_data tx_buf = {0x17,EPILOG};
    SCI_write(&tx_buf.Prolog,2);
}

//
// Request Command - Ack Page Strings
// buf[0]: Prolog, buf[1]: Epilog
//
void RequsetPageStrings(Uint16 *buf)
{
    Uint16 i = 0;
    RPS_data tx_buf;

    tx_buf.Prolog=buf[0]; // 0x19
    tx_buf.IndexSize=sizeof(Cmd_Index_Str)/sizeof(Cmd_Index_Str[0]);
    SCI_write(&tx_buf.Prolog,2);

    for(i=0;i<tx_buf.IndexSize;i++)
    {
        strncpy(tx_buf.Name,Cmd_Index_Str[i],16);
        SCI_write(tx_buf.Name,16);
    }
    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// NodeBit - Ack Node Bit Text
// buf[0]: Prolog, buf[1]: Epilog
//
void AckNodeBitText(Uint16 *buf)
{
    Uint16 i = 0;
    ANBT_data tx_buf;
    uUint16 BitNumber = {sizeof(Node_Index_Str)/sizeof(Node_Index_Str[0])};

    tx_buf.Prolog=0x62; // 0x62
    tx_buf.Null=0x00;
    tx_buf.BitNum[0]=BitNumber.ubyte.lo;
    tx_buf.BitNum[1]=BitNumber.ubyte.hi;
    SCI_write(&tx_buf.Prolog,4);

    for(i=0;i<BitNumber.uword;i++)
    {
        strncpy(tx_buf.Strings,Node_Index_Str[i],16);
        SCI_write(tx_buf.Strings,16);
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// NodeBit - Ack Node Bit State
// buf[0]: Prolog, buf[1]: Epilog
//
void AckNodeBitState(Uint16 *buf)
{
    Uint16 i = 0;
    ANBS_data tx_buf;
    uUint16 BitNumber = {sizeof(Node_Index_Str)/sizeof(Node_Index_Str[0])};

    tx_buf.Prolog=0x64; // 0x64
    tx_buf.Null=0x00;
    SCI_write(&tx_buf.Prolog,2);

    for(i=0;i<BitNumber.uword;i++)
    {
        tx_buf.Status[0] = Node_Status[i].ubyte.lo;
        tx_buf.Status[1] = Node_Status[i].ubyte.hi;
        tx_buf.ActiveCounts[0] = Node_ActiveCount[i].ubyte.lo;
        tx_buf.ActiveCounts[1] = Node_ActiveCount[i].ubyte.hi;
        tx_buf.DeactiveCounts[0] = Node_DeactiveCount[i].ubyte.lo;
        tx_buf.DeactiveCounts[1] = Node_DeactiveCount[i].ubyte.hi;
        SCI_write(tx_buf.Status,6);
    }

    tx_buf.Epilog=EPILOG;
    SCI_write(&tx_buf.Epilog,1);
}

//
// Function Switch Case for Ack
//
void (* switch_ackfunction[])(Uint16 *buf) = {
                                           ResponseScopeCommand,
                                           AckIndexDone,
                                           AckDmaDone,
                                           ResponseScopeStrings,
                                           ResponseRollingScope,
                                           ResponsetoStopRollingData,
                                           AckPageStrings,
                                           AckPageVariableState,
                                           AckWriteParameter,
                                           AckReadBlockParameter,
                                           AckRequest,
                                           RequsetPageStrings,
                                           AckNodeBitText,
                                           AckNodeBitState
    };

//
// SCI_init - Initialize and setup SCI
//
void SCI_init()
{


    int i;
//    SysCtrlRegs.LOSPCP.bit.LSPCLK = 1; // LSPCLK = CPU_FREQ/2


#if DSP28_SCIA
    for(i=0;i<sizeof(rdataA);i++)
    {
        rdataA[i]=0;
        sdataA[i]=0;
    }
    Scia_GpioInit();
    Scia_Setting();
#endif // endif DSP28_SCIA

#if DSP28_SCIB
    for(i=0;i<sizeof(rdataB);i++)
    {
        rdataB[i]=0;
        sdataB[i]=0;
    }
    Scib_GpioInit();
    Scib_Setting();
#endif // endif DSP28_SCIB


}

#if DSP28_SCIA
void Scia_GpioInit()
{
    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled disabled by the user.
 // This will enable the pullups for the specified pins.

     GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up for GPIO7  (SCIRXDA)

     GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCITXDA)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up for GPIO12 (SCITXDA)

 /* Set qualification for selected pins to asynch only */
 // Inputs are synchronized to SYSCLKOUT by default.
 // This will select asynch (no qualification) for the selected pins.

     GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
 //  GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;   // Asynch input GPIO7 (SCIRXDA)

 /* Configure SCI-A pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SCI functional pins.

     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
 //  GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;    // Configure GPIO7  for SCIRXDA operation

     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
 //  GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;   // Configure GPIO12 for SCITXDA operation

     EDIS;
}
void Scia_Setting()
{
    SciaRegs.SCICCR.all = 0x0067;              // 1 stop bit,  No loopback
                                               // Even, 8 char bits,
                                               // async mode, idle-line protocol

    SciaRegs.SCICTL1.all = 0x0003;             // enable TX, RX, internal SCICLK,
                                               // Disable RX ERR, SLEEP, TXWAKE
//    SciaRegs.SCICTL2.bit.TXINTENA = 1;         // TXRDY interrupt
//    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;       // RXRDY/BRKDT interrupt
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = SCIA_PRD;
//    SciaRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back
    SciaRegs.SCIFFTX.all = 0xC004;
    SciaRegs.SCIFFRX.all = 0x0021;
    SciaRegs.SCIFFCT.all = 0x00;

    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

#endif // endif DSP28_SCIA

#if DSP28_SCIB
void Scib_GpioInit()
{
    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled disabled by the user.
 // This will enable the pullups for the specified pins.

     GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
//    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
//    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
  // GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;    // Enable pull-up for GPIO19 (SCIRXDB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)
 //  GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;    // Enable pull-up for GPIO41 (SCIRXDB)
 //  GpioCtrlRegs.GPBPUD.bit.GPIO44 = 0;    // Enable pull-up for GPIO44 (SCIRXDB)

     GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;     // Enable pull-up for GPIO9 (SCITXDB)
//    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;     // Enable pull-up for GPIO9 (SCITXDB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up for GPIO14 (SCITXDB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;    // Enable pull-up for GPIO18 (SCITXDB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)
 //  GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;    // Enable pull-up for GPIO40 (SCITXDB)
 //  GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;    // Enable pull-up for GPIO58 (SCITXDB)

 /* Set qualification for selected pins to asynch only */
 // Inputs are synchronized to SYSCLKOUT by default.
 // This will select asynch (no qualification) for the selected pins.

     GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
//     GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO11 (SCIRXDB)
 //  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
  // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
 //  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)
 //  GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;  // Asynch input GPIO41 (SCIRXDB)
 //  GpioCtrlRegs.GPBQSEL1.bit.GPIO44 = 3;  // Asynch input GPIO44 (SCIRXDB)


 /* Configure SCI-B pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SCI functional pins.

     GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
//     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO11 for SCIRXDB operation
 //  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation
 //  GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;   // Configure GPIO41 for SCIRXDB operation
 //  GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 2;   // Configure GPIO44 for SCIRXDB operation


     GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;   // Configure GPIO9 for SCITXDB operation
//     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO9 for SCITXDB operation
 //  GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation
 //  GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;   // Configure GPIO40 for SCITXDB operation
 //  GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 2;   // Configure GPIO58 for SCITXDB operation


     EDIS;
}
void Scib_Setting()
{
    ScibRegs.SCICCR.all = 0x0067;              // 1 stop bit,  No loopback
                                               // Even, 8 char bits,
                                               // async mode, idle-line protocol

    ScibRegs.SCICTL1.all = 0x0003;             // enable TX, RX, internal SCICLK,
                                               // Disable RX ERR, SLEEP, TXWAKE
//    ScibRegs.SCICTL2.bit.TXINTENA = 1;         // TXRDY interrupt
//    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;       // RXRDY/BRKDT interrupt
    ScibRegs.SCIHBAUD = 0x0000;    //0x0000
    ScibRegs.SCILBAUD = SCIB_PRD;
//    ScibRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back
    ScibRegs.SCIFFTX.all = 0xC004;
    ScibRegs.SCIFFRX.all = 0x0021;
    ScibRegs.SCIFFCT.all = 0x00;

    ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}
#endif // endif DSP28_SCIB
//
// End of file
//
