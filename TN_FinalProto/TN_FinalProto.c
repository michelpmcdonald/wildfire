// Final Proto of VTD on USIV2 (Universal Sensor Interface Version 2) Board

// Changes - 8/26/2007 - ARI
//       * MCLR fuse enabled
//       * Turned OFF VHF Radio Power (XMIT) Before main while loop
// 8/31/2007
//       * Status message & configuration  is displayed on startup
//       * Added define for DEFAULT_ID
//       * LED toggle added in main loop to provide visual feedback of pkts rcvd

#include <18F6722.h>

#device  adc=10

// External Crystal Oscillator
#use delay(clock=7372800,restart_wdt)

#fuses NOWDT, HS, NOPROTECT, NOIESO, NOBROWNOUT, BORV25, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOCPB, NOEBTRB, NOEBTR, NOWRTD, NOWRTC, NOWRTB, NOFCMEN, WDT_32768, LPT1OSC, MCLR, NOXINST, CCP2B3

// Serial Port Parameters
#USE RS232(BAUD=9600, XMIT=PIN_C6, RCV=PIN_C7, ERRORS, RESTART_WDT, STREAM=COM_VHF)      //Hardware UART#1 connected to Ext VHF Radio
#USE RS232(BAUD=9600, XMIT=PIN_G1, RCV=PIN_G2, ERRORS, RESTART_WDT, STREAM=COM_XBEE)      //Hardware UART#2 connected to XBEE


//Indicates that this is the CN\VTD "Big Board" hardware
#define CN_VTD_HARDWARE

//Zigbee Interrupt for use in common FR_Zigbee.h
#define ZIGBEE_INTERRUPT #int_rda2

//GPS Interrupt for use in Gps.h
#define GPS_INTERRUPT    #int_ext1

//Debug Output to PC Port
//#define FR_DEBUG

//Define causes unit test functions to compile
//#define ENABLE_UNIT_TEST

//Flag to pause output
char wait_flag;

//******************************************************************************
//Globals - Config globals, assigned default values, overridden from
//          EEPROM if saved or FRSBConfigCommand if received
//******************************************************************************

   //Holds the TagID of THIS unit
   //Defined in FR_Zigbee.h
   //Included here as a comment
   //only for documentation
   //static int16 g_tagID;

   //Battery threshold Cutout
   int16 g_batteryCutout = 903;

   //Zigbee Destination Address - TNs always TX to a specific CN, not broadcast
   int16 g_zibeeDestination = 21007;

   #define DEFAULT_ID 22010

//******************************************************************************
//******************************************************************************

#include <STDDEF.H>
#include <USIV2A.h>
#include "FR_Queue.h"
#include "FR_Zigbee.h"


//******************************************************************************
//Function Decs
//******************************************************************************

   //Incoming Data
   void ProcessIncomingXbeePacket(char zigbeeMessage[], int8 zigbeeMessageLen );
   void ProcessIncomingXbeeQueue();

   void StartUpLedFlash();

   //Calcs time change accounting for rollover
   int16 CalcTimeDiff( int16 oldTime, int16 newTime );

   //Reads battery level
   int16 GetBatteryReading();

   //Save global settings to EEProm
   void SaveConfigsToEEProm();

   //Take values out of config message and assign
   //to config vars, save to EEProm
   void ProcessConfigMessage( char *configMessage  );

   //Atempt to read config from EEProm
   int LoadConfigsFromEEProm();

   void WriteConfig();
   void PCPort_Printf(char *buffer);

   #ifdef ENABLE_UNIT_TEST
      void WipeEEProm();
   #endif

//******************************************************************************
//******************************************************************************


//******************************************************************************
//Globals( what da ya gona do )
//******************************************************************************

   //Keeps track of LED status
   char bLedOn = 1;
   int32 ledLoopCount = 0;

   //Allow Config messages to be Processed
   int g_processConfigMessage = 1;

//******************************************************************************
//******************************************************************************




//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//Main
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
void main()
{
   StartUpLedFlash();

   //Attempt to read config options
   //from EEProm
   if( LoadConfigsFromEEProm() != 1 )
   {
      //No Config Info Stored
      //Need to set a default TagID
      //FR Format of TagID SNXXX
      //S= Site (0 = Unknown, 1=Oke, 2=BW)
      //N=Node Type (1-5)
      //XXX=Node ID (000-999)
      g_tagID = DEFAULT_ID;
   }

   //Set Zigbee Buffers( Software Only )
   InitZbRx();

   // Perform various Board initialization tasks
   InitializeBoard();

   // Initialize XBEE
   ResetXBEE();

   // Initialize VHF-B Port (PIC Serial Port #1)
   VHFB_init();

   // Initialize MAX3110E Port A (Laptop)
   MAXA_init('x');

   // Initialize MAX3110E Port B (GPS)
   MAXB_init('i');

   WriteConfig();

   //Assign zigbee a tagID
   InitZigbee( g_tagID );

   // Enable MAX3110E (A) Interrupts - Laptop
   //ext_int_edge(0,H_TO_L);
   //enable_interrupts(int_ext);

   // VHF-B Rx data Interrupts
   enable_interrupts(int_rda);

   // XBEE Rx data Interrupts
   enable_interrupts(int_rda2);

   // 30 Hz Clock
   onesec = 0;
   enable_interrupts(int_timer2);

   //All
   enable_interrupts(global);

   //Take a little breather
   delay_ms(500);
   StartUpLedFlash();

   fprintf(COM_VHF,"\n\r TN Firmware Build: ");
   fprintf(COM_VHF,__DATE__);
   fprintf(COM_VHF," ");
   fprintf(COM_VHF,__TIME__);
   fprintf(COM_VHF,"\n\r Settings: ID = %LU  Zigbee Dest ID = %LU  \n\r",g_tagID,g_zibeeDestination);

   output_low(XMIT);       //Turn off VHF Radio Power

   while (1)
   {
      //Force the watch dog timer to reset
      delay_ms(1);
      output_high(LED);
      if(IsZigbeeQueueEmpty() != 1)
      {
         //fprintf(COM_VHF, "%S", "TN Got Zigbee\n\r" );
         ProcessIncomingXbeeQueue();
         output_toggle(LED);
      }
   }
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//End Main
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************


//******************************************************************************
//Main loop detected incoming data in Queue
//******************************************************************************
void ProcessIncomingXbeeQueue()
{
   //int x;
   char dataBuffer[50];
   int8 reportLen;
   reportLen = 0;

   Dequeue( dataBuffer, &reportLen );

   if( reportLen != 0 )
   {
      ProcessIncomingXbeePacket( dataBuffer, reportLen );
   }
}
//******************************************************************************
//******************************************************************************


//******************************************************************************
//Got A Zigbee API message
//******************************************************************************
void ProcessIncomingXbeePacket(char zigbeeMessage[], int8 zigbeeMessageLen )
{
   char configMessage[15];

   //fprintf(COM_VHF, "%S", "TN Start Process In Zigbee\n\r" );

   switch( zigbeeMessage[3] )
   {
      //RX Packet
      case 0x81:
      {
         //TN Config Message
         if( zigbeeMessage[10] == 0x08 )
         {
            //Got A config Message, check to see if we
            //should process
            if( (g_processConfigMessage == 1) && (onesec < 60) )
            {
               memcpy(configMessage, &zigbeeMessage[8], 15);
               ProcessConfigMessage( configMessage );
               g_processConfigMessage = 0;
            }
         }

         //Location report from another unit, foward it
         else if( zigbeeMessage[10] == 0x01 )
         {
            //Got a TX from another unit
            //TxZigbeeToSerial( zigbeeMessage, zigbeeMessageLen );
            ZigbeeTx( &zigbeeMessage[8], 21, g_zibeeDestination );
            //fprintf(COM_VHF, "%S", "TN Zigbee Resend\n\r" );
         }

         zbRxMessageFound = 0;
      }
      break;

      //AT command Response
      case 0x88:
      {
         ZigbeeAtQueryResp( zigbeeMessage, zigbeeMessageLen );
         zbRxMessageFound = 0;
      }
      break;

      default:
      {
      }
      break;
   }
}


//******************************************************************************
//Flash LED
//******************************************************************************
void StartUpLedFlash()
{
   //Diagnostic Flash
   output_low(LED);
   output_toggle(LED);
   delay_ms(200);
   output_toggle(LED);
   delay_ms(200);
   output_toggle(LED);
   delay_ms(200);
   output_toggle(LED);
   delay_ms(200);
   output_toggle(LED);

}
//******************************************************************************
//End Flash LED
//******************************************************************************



//******************************************************************************
// Calc Time diff accounting for rollover
//******************************************************************************
int16 CalcTimeDiff( int16 oldTime, int16 newTime )
{
   signed int16 diff;

   diff = newTime - oldTime;

   return abs(diff);
}


//******************************************************************************
// Get Battery reading
//******************************************************************************
int16 GetBatteryReading()
{
   int16 batteryReading;

   //Turn on Voltage Divider (Power Monitor)
   output_high(BPWR);

   //Battery Voltage
   batteryReading = ReadAnalogChannelAvg(BCH);

   //Turn off Voltage Divider (Power Monitor)
   output_low(BPWR);

   return batteryReading;
}
//******************************************************************************
//******************************************************************************

//******************************************************************************
//Pull config values out of message, assign to config vars
//******************************************************************************
void ProcessConfigMessage(char *configMessage  )
{
   int16 prevXbeeTagID;
   prevXbeeTagID = g_tagID;

   //THIS zigbee tagID
   g_tagID = make16(configMessage[8],configMessage[7]);

   //Battery threshold Cutout
   g_batteryCutout = make16( configMessage[10],configMessage[9]);

   //TN Zigbee TX Destination
   g_zibeeDestination = make16( configMessage[12],configMessage[11]);

   if( g_tagID != prevXbeeTagID )
   {
      //The tag ID changed, update xbee device
      InitZigbee( g_tagID );
   }

   SaveConfigsToEEProm();

   StartUpLedFlash();



}
//******************************************************************************
//******************************************************************************


//******************************************************************************
//Save Config values to Eprom, called after a config RF Message is Received
//******************************************************************************
void SaveConfigsToEEProm()
{
   //Config Marker - Indicates that configs have been save to EProm
   write_EEPROM(0, 0x01);
   write_EEPROM(1, 0x02);
   write_EEPROM(2, 0x03);
   write_EEPROM(3, 0x04);

   //Zigbee TagID of THIS unit
   write_EEPROM( 4, make8(g_tagID,1) );
   write_EEPROM( 5, make8(g_tagID,0) );

   //Battery threshold Cutout
   write_EEPROM( 6, make8(g_batteryCutout,1) );
   write_EEPROM( 7, make8(g_batteryCutout,0) );

   //TN Zigbee TX Destination
   write_EEPROM( 8, make8(g_zibeeDestination,1) );
   write_EEPROM( 9, make8(g_zibeeDestination,0) );
}
//******************************************************************************
//SaveConfigsToEEProm END
//******************************************************************************

//******************************************************************************
//Check EEprom to see if config values have been saved, if so override defaults
//******************************************************************************
int LoadConfigsFromEEProm()
{
   int32 configEEPromMarker;

   //Pull Config Marker from EEProm
   configEEPromMarker = make32( read_EEPROM(0),read_EEPROM(1),
                                read_EEPROM(2), read_EEPROM(3) );

   //If first 4 bytes of EEProm == our marker
   //Config has been stored, pull the info
   if( configEEPromMarker == 0x01020304)
   {
      //Zigbee TagID of THIS unit
      g_tagID= make16( read_EEPROM(4),read_EEPROM(5) );

      //Battery threshold Cutout
      g_batteryCutout = make16( read_EEPROM(6),read_EEPROM(7) );

      //Praxis Sensor Report Interval
      g_zibeeDestination = make16( read_EEPROM(8),read_EEPROM(9) );

      return 1;


   }

   return 0;
}
//******************************************************************************
//End Of LoadConfigsFromEEProm
//******************************************************************************

//Call this to print out debug output to the
//PC Serial Port.  When the ENABLE_UNIT_TEST define
//is removed, any debug output will cause a compile
//error
void PCPort_Printf(char *buffer)
{
   //fprintf(COM_VHF,buffer );
}

void WriteConfig()
{
   char cOutputMess[50];

   sprintf(cOutputMess, "g_TagID = %05lu\n\r", g_tagID);
   PCPort_Printf( cOutputMess );

   sprintf(cOutputMess, "g_batteryCutOut = %05lu\n\r", g_batteryCutout);
   PCPort_Printf( cOutputMess );

   sprintf(cOutputMess, "g_zibeeDestination = %05lu\n\r",g_zibeeDestination);
   PCPort_Printf( cOutputMess );
}

#ifdef ENABLE_UNIT_TEST

void WipeEEProm()
{
   //Overwrite Marker so it does not look like configs are set
   write_EEPROM(0, 0xAA);
   write_EEPROM(1, 0xBB);
   write_EEPROM(2, 0xCC);
   write_EEPROM(3, 0xDD);
}

#endif


