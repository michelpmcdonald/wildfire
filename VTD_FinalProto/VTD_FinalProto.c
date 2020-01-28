// Final Proto of VTD on USIV2 (Universal Sensor Interface Version 2) Board
//
// Changes - 8/26/2007 - ARI
//       * MCLR fuse enabled
// Changes - 8/31/2007
//       * GPS Interrupts are now enabled after GPS is initialized
//       * Outer IF statement commented out in ProcessGGA()
//       * TX LED is turn OFF on startup
//       * Added define for DEFAULT_ID

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

//Define causes unit test functions to compile
//#define ENABLE_UNIT_TEST

//Enable Debugging output
//#define FR_DEBUG



char wait_flag;            //Flag to pause output

//******************************************************************************
//Equipment Tag Scan Report - Report VTD sends when it detects an ET broadcast
//******************************************************************************
struct etScanReport
{
   int16        tagID;
   int8         reportType;
   int16        battery;
   int16        scannerTagID;
};
//******************************************************************************
//******************************************************************************

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

   //Report interval for GPS Location Report (seconds)
   int16 g_locationReportInterval = 10;

   //1 - Check the VHF buffer line full before TXing
   //0 - Ignore the VHF buffer line full before Txing
   int8 g_radioBufferCheck = 1;

   #define DEFAULT_ID 13007

//******************************************************************************
//******************************************************************************

#include <USIV2A.h>
#include "TdLocReport.h"
#include "FR_Queue.h"
#include "FR_Zigbee.h"
#include "SerialServer.h"
#include "Gps.h"



//******************************************************************************
//Function Decs
//******************************************************************************

   //Incoming Data
   void ProcessGGA( struct ptdLocReport *locRep );
   void ProcessIncomingXbeeQueue();
   void ProcessIncomingXbeePacket(char zigbeeMessage[], int8 zigbeeMessageLen );

   void ProcessConfigMessage(char *configMessage  );
   void SaveConfigsToEEProm();
   int LoadConfigsFromEEProm();

   //Hardware Reads( ToDo: Move to USIV2A.h ?? )
   void GetLocRepAccelerometer (struct ptdLocReport *locRep);
   void GetLocRepTemp( struct ptdLocReport *locRep );
   void GetLocRepBattery( struct ptdLocReport *locRep );

   void StartUpLedFlash();

   #ifdef ENABLE_UNIT_TEST
      void WriteConfig();
      void PCPort_Printf(char *buffer);
   #endif


//******************************************************************************
//******************************************************************************


// VHF-B Receive Data Interrupts
#int_rda
// Just read the character and save it in "wait_flag" (used to pause output)
void vhf_isr(void)
{
   char inChar;
   inChar = fgetc(COM_VHF);
   MAXA_putc(inChar);
}

// INT0 - MAX3110E PORT A Interrupt (Laptop) - Not Used
// Just set the wait flag so user can pause XBEE output
#int_ext
void laptopRx_isr ( void ) {
   wait_flag = MAXA_igetc();       // just get char from UART for now
}






//******************************************************************************
//Globals( what da yu gona do )
//******************************************************************************

   //Keeps track of LED status
   char bLedOn = 1;
   int32 ledLoopCount = 0;

   //Allow Config messages to be Processed
   int g_processConfigMessage = 1;

   //Keep track of the last location report
   //Deals with the GPS not prograbable bug
   int16 g_lastLocReportSendTime;

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
   //Create a report to hold
   //local and incoming
   struct ptdLocReport locRep;
   locRep.reportType = (int8)0x01;

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

   //Init GPS RX status( Software only )
   InitGpsRxBuffer();

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

   //Turn on GPS Module
   GPS_ON();



   fprintf(COM_VHF,"\n\r VTD Firmware Build: ");
   fprintf(COM_VHF,__DATE__);
   fprintf(COM_VHF," ");
   fprintf(COM_VHF,__TIME__);
   fprintf(COM_VHF,"\n\r Settings: ID = %LU  Report Rate = %LU  BufferCheck = %U  \n\r",g_tagID,g_locationReportInterval,g_radioBufferCheck);


   //Assign zigbee a tagID
   InitZigbee( g_tagID );

   //GGAOn
   InitGPS( g_locationReportInterval );


   //GPS RX Error Flags
   cRxErrorFlag=0;
   cError=0;

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

   //Enable GPS interrupts
   ext_int_edge(1,H_TO_L);
   enable_interrupts (int_ext1);

   delay_ms(500);


   //Get the current tag from the Zigbee
   QueryZigbeeTagID();

   g_lastLocReportSendTime = onesec;

   output_low(XMIT);             //Turn off 12v Power supply

   while (1)
   {
      //Force the watch dog timer to reset
      delay_ms(1);


      //Check For Zigbee
      if(IsZigbeeQueueEmpty() != 1)
      {
         ProcessIncomingXbeeQueue();
      }

      //check GPS for Location
      if(cRxGPSMsgReady == 1)
      {
         ProcessGGA(&locRep);
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
//Got A GGA message from GPS, build location report and send
//******************************************************************************
void ProcessGGA( struct ptdLocReport *locRep )
{
   signed int32 timeDiff;

   //timeDiff = onesec - g_lastLocReportSendTime;

   //if( abs(timeDiff) >= g_locationReportInterval )
   //{
      //set the last location report time to current
      g_lastLocReportSendTime = onesec;

      //Make sure we have a valid tagID
      if( g_tagID != 0 )
      {

         //Found GPS sentence
         GetLocRepGGA( locRep, cRxBuffer );
         GetLocRepAccelerometer( locRep );
         GetLocRepTemp( locRep );
         GetLocRepBattery( locRep );
         (*locRep).tagID = g_tagID;

         WriteFRReportToServer( locRep, sizeof(*locRep) );
      }
      else
      {
         //Dont have a valid tagID, Request it
         QueryZigbeeTagID();
      }
   //}

   //Reset GPS buffer
   cRxErrorFlag=0;
   cError=0;
   InitGpsRxBuffer();
}
//******************************************************************************
//End ProcessGGA
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
   char configMessage[14];

   switch( zigbeeMessage[3] )
   {
      //RX Packet
      case 0x81:
      {
         if( zigbeeMessage[10] == 0x07 )
         {
            //Got A config Message, check to see if we
            //should process
            if( (g_processConfigMessage == 1) && (onesec < 60) )
            {
               memcpy(configMessage, &zigbeeMessage[8], 14);
               ProcessConfigMessage( configMessage );
               g_processConfigMessage = 0;
            }
         }

         else
         {
            if( zigbeeMessage[10] == 0x01 )
            {
               //Got a Location Report from a PTD
               TxZigbeeToSerial( zigbeeMessage, zigbeeMessageLen );
            }
            else if( zigbeeMessage[10] == 0x03 )
            {
               //Got a ET broadscast, add our tagID, send to server
               struct etScanReport scanRpt;
               memcpy( &scanRpt, &zigbeeMessage[8], 5 );
               scanRpt.scannerTagID = g_tagID;
               WriteFRReportToServer( &scanRpt, sizeof(scanRpt) );
            }
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
//End of ProcessIncomingXbeePacket()
//******************************************************************************

//******************************************************************************
//Pull config values out of message, assign to config vars
//******************************************************************************
void ProcessConfigMessage(char *configMessage  )
{
   int16 prevLocRepInterval;
   int16 prevXbeeTagID;

   prevLocRepInterval = g_locationReportInterval;
   prevXbeeTagID = g_tagID;

   //THIS zigbee tagID
   g_tagID = make16(configMessage[8],configMessage[7]);

   //Battery threshold Cutout
   g_batteryCutout = make16( configMessage[10],configMessage[9]);

   //Sensor Report Interval
   g_locationReportInterval = make16( configMessage[12],configMessage[11]);

   //VHF Buffer Check
   g_radioBufferCheck = configMessage[13];


   if( g_locationReportInterval != prevLocRepInterval )
   {
         //Loc Report Interval changed, update
         InitGPS( g_locationReportInterval );

   }

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

   //Sensor Report Interval
   write_EEPROM( 8, make8(g_locationReportInterval,1) );
   write_EEPROM( 9, make8(g_locationReportInterval,0) );

   //VHF Buffer Check
   write_EEPROM( 10, g_radioBufferCheck );
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

      //Location Report Interval
      g_locationReportInterval = make16( read_EEPROM(8),read_EEPROM(9) );

      //VHF Buffer Check
      g_radioBufferCheck = read_EEPROM(10);

      return 1;
   }

   return 0;
}
//******************************************************************************
//End Of LoadConfigsFromEEProm
//******************************************************************************

void GetLocRepAccelerometer (struct ptdLocReport *locRep)
{
   (*locRep).xPos = ReadAnalogChannelAvg (XCH);
   (*locRep).yPos = ReadAnalogChannelAvg (YCH);

   //fprintf(COM_PC, "Accl xPos: %lu yPos: %lu \n\r", (*locRep).xPos, (*locRep).yPos );
}

//Get the current temp from the ptd
void GetLocRepTemp( struct ptdLocReport *locRep )
{
   //Just hard code, temp sensor info not avail
   (*locRep).temp = ReadAnalogChannelAvg(TCH);
}


void GetLocRepBattery( struct ptdLocReport *locRep )
{
   //Turn on Voltage Divider (Power Monitor)
   //output_high(BPWR);

   //Battery Voltage
   (*locRep).battery = ReadAnalogChannelAvg(BCH);

    //Turn off Voltage Divider (Power Monitor)
   //output_low(BPWR);
}

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

#ifdef ENABLE_UNIT_TEST

//Call this to print out debug output to the
//PC Serial Port.  When the ENABLE_UNIT_TEST define
//is removed, any debug output will cause a compile
//error
void PCPort_Printf(char *buffer)
{
   MAXA_printf( buffer );
}

void WriteConfig()
{
   char cOutputMess[50];

   sprintf(cOutputMess, "g_TagID = %05lu\n\r", g_tagID);
   MAXA_printf( cOutputMess );

   sprintf(cOutputMess, "g_batteryCutOut = %05lu\n\r", g_batteryCutout);
   MAXA_printf( cOutputMess );

   sprintf(cOutputMess, "g_locationReportInterval = %05lu\n\r",g_locationReportInterval);
   MAXA_printf( cOutputMess );

}


void WipeEEProm()
{
   //Overwrite Marker so it does not look like configs are set
   write_EEPROM(0, 0xAA);
   write_EEPROM(1, 0xBB);
   write_EEPROM(2, 0xCC);
   write_EEPROM(3, 0xDD);
}

#endif


