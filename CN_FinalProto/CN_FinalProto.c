// Final Proto of VTD on USIV2 (Universal Sensor Interface Version 2) Board
//
// Changes - 8/26/2007 - ARI
//       * MCLR fuse enabled
//       * Disabled Battery check in GetSensors() since it interferes with VHF power saving code
//         and also the Call to GetBatteryReading() removed from ProcessFCSData() function. Battery
//         voltage is now read in VHFOn() function.
//       * Radio Power save mode is now ON by default
//       * Battery Voltages are now read in VHFOn() function
//       * Added Accelerometer Z-channel definition (ZCH) in usiv2a.h
// 8/31/2007
//       * Status message & configuration  is displayed on startup
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

   //Report interval for Sensor
   int16 g_sensorReportInterval = 900;

   //Report Interval for FCS Sensor
   int16 g_FCSReportInterval = 900;

   //Turn off radio if not in use to save power( CN are Solar\Battery )
   //0 = false = do not turn radio off
   int8 g_radioPowerSave = 1;

   //Buffer full check 1 - Detect Radio buffer full, loop until radio buffer
   //has room to accept data.  0 - do not check buffer
   int8 g_radioBufferCheck = 1;

   #define DEFAULT_ID 11002


//******************************************************************************
//******************************************************************************



//******************************************************************************
//Cn Sensor Report
//******************************************************************************
struct cnSensorReport
{
   int16        tagID;

   int8         reportType;
   int16        battery;

   //Wind Speed
   unsigned int16 speed;

   //Wind Direction
   unsigned int16 direction;

   //External Temperature
   unsigned int16 temperature;

   //On-board Temperature
   unsigned int16 btemperature;

   //Humidity
   unsigned int16 humidity;

   //Barometric Pressure
   unsigned int16 pressure;

   //Rain Fall
   unsigned int16 rain;

   //Solar Radiation
   unsigned int16 solar;

   //Soil Moisture
   unsigned int16 moisture;
};
//******************************************************************************
//End Cn Sensor Report
//******************************************************************************


//******************************************************************************
//AeGIS FCS Sensor Report
//******************************************************************************
struct csFCSReport
{
   int16        tagID;

   int8         reportType;

   int16        battery;

   //Figaro TGS5042 CO
   unsigned int16 fcs0;

   //A3CO EnviroceL CO
   unsigned int16 fcs1;

   //Figaro TGS4161 CO2
   unsigned int16 fcs2;

   //Synkera 707 VOC
   unsigned int16 fcs3;

   //Precon HS-2000V Temp
   unsigned int16 fcs4;

   //Precon HS-2000V RH
   unsigned int16 fcs5;

   //Airware Hi Res CO2
   unsigned int16 fcs6;

   //VESDA Smoke
   unsigned int16 fcs7;

} FCSRep;

//******************************************************************************
//End AeGIS FCS Sensor Report
//******************************************************************************

#include <STDDEF.H>
#include <USIV2A.h>
#include "TdLocReport.h"
#include "FR_Queue.h"
#include "FR_Zigbee.h"
#include "SerialServer.h"


//******************************************************************************
//Function Decs
//******************************************************************************

   //Incoming Data
   void ProcessGGA( struct ptdLocReport *locRep );
   void ProcessIncomingXbeePacket(char zigbeeMessage[], int8 zigbeeMessageLen );
   void ProcessIncomingXbeeQueue();

   //Hardware Reads( ToDo: Move to USIV2A.h ?? )
   void ProcessSensorData( struct cnSensorReport *sensorRpt );

   void StartUpLedFlash();

   //Send a Poll Command to the FCS
   void PollFCS (void);

   // Calculate FCS Packet Checksum
   unsigned int16 FCSChecksum (void);

   //Polls FCS, wait for result
   unsigned int8 ProcessFCSData(void);

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

   //Radio on\off
   void VHFOn();
   void VHFOff();

   #ifdef ENABLE_UNIT_TEST
      void WriteConfig();
      void PCPort_Printf(char *buffer);
   #endif

//******************************************************************************
//******************************************************************************


//******************************************************************************
// Globals for FCS Sensor RX
//******************************************************************************
   #define RX_BUFFER_SIZE  150

   // Number of bytes in the recv fifo
   static char cRxByteCnt;

   //Hold incoming FCS Report Serial Data
   static char cRxBuffer[RX_BUFFER_SIZE];

   //Flag to signal end of FCS packet
   int8  fcs_flag;

//******************************************************************************
//******************************************************************************


//******************************************************************************
// INT0 - MAX3110E PORT A Interrupt (Laptop/FCS)
//******************************************************************************
#int_ext
void FCSRx_isr ( void )
{
   static unsigned int16 pcnt;
   char cChar;

   if (cRxByteCnt==0)
      pcnt=0;
   cChar = MAXA_igetc();       // get char from FCS UART
   cRxBuffer[cRxByteCnt]=cChar;
   if (cRxByteCnt==2)
      pcnt=cRxBuffer[cRxByteCnt];
   if (cRxByteCnt==pcnt+5)
      fcs_flag=1;
   cRxByteCnt++;
   if (cRxByteCnt >= RX_BUFFER_SIZE)
      cRxByteCnt=0;
}
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

   //Current State of VHF Radio 0 = OFF,  1 = ON
   int g_radioPowerState = 1;

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
   //Hold last "Time"onesec sensor data was reported
   int16 lastSensorReportTime;

   //Hold Last FCS Report time
   int16 lastFCSReportTime;

   //Holds sensor report
   struct cnSensorReport sensorRpt;
   sensorRpt.reportType = 0x02;

   FCSRep.reportType = 0x05;

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

   output_low(XMIT);                         //make sure radio is off

   fprintf(COM_VHF,"\n\r CN/TNW Firmware Build: ");
   fprintf(COM_VHF,__DATE__);
   fprintf(COM_VHF," ");
   fprintf(COM_VHF,__TIME__);
   fprintf(COM_VHF,"\n\r Settings: ID = %LU  Sensor Report Rate = %LU  BufferCheck = %U \n\r",g_tagID,g_sensorReportInterval,g_radioBufferCheck);
   fprintf(COM_VHF,"  PowerSave = %U  FCS Report Rate = %LU \n\r",g_radioPowerSave,g_FCSReportInterval);

   //Assign zigbee a tagID
   InitZigbee( g_tagID );

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

   //If radio Power save is enabled, turn off the radio
   if( g_radioPowerSave == 1 )
   {
      VHFOff();
   }
   else
   {
      //make sure the vhf is On
      VHFOn();
   }

   //fprintf(COM_VHF, "%S", "Start Test, vhf\n\r" );

   lastSensorReportTime = onesec;
   lastFCSReportTime = onesec;

   while (1)
   {
      //Force the watch dog timer to reset
      delay_ms(1);

      if(IsZigbeeQueueEmpty() != 1)
      {
         //Check to see if the Radio
         //needs to be turned on
         if( g_radioPowerState == 0 )
         {
            VHFOn();
         }

         ProcessIncomingXbeeQueue();

         //If the queue is empty AND Radio Power Save is enabled, shut the
         //Radio off
         if( (IsZigbeeQueueEmpty() == 1) && (g_radioPowerSave == 1) )
         {
            VHFOff();
         }
      }

      //Time to send Sensor data ??
      if( CalcTimeDiff(lastSensorReportTime, onesec) >= g_sensorReportInterval )
      {
         lastSensorReportTime = onesec;

         //Check to see if the Radio
         //needs to be turned on
         if( g_radioPowerState == 0 )
         {
            VHFOn();
         }

         ProcessSensorData( &sensorRpt);

         //If the queue is empty AND Radio Power Save is enabled, shut the
         //Radio off
         if( (IsZigbeeQueueEmpty() == 1) && (g_radioPowerSave == 1) )
         {
            VHFOff();
         }
      }

      //if g_FCSReportInterval > 0 Is it time to poll for FCS Data, check time
      if( ( g_FCSReportInterval > 0 ) &&
          ( CalcTimeDiff(lastFCSReportTime, onesec) >= g_FCSReportInterval ) )
      {
         lastFCSReportTime = onesec;
         PollFCS();
      }

      if( fcs_flag == 1 )
      {
         fcs_flag = 0;

         //Check to see if the Radio
         //needs to be turned on
         if( g_radioPowerState == 0 )
         {
            VHFOn();
         }

         ProcessFCSData();

         //If the queue is empty AND Radio Power Save is enabled, shut the
         //Radio off
         if( (IsZigbeeQueueEmpty() == 1) && (g_radioPowerSave == 1) )
         {
            VHFOff();
         }
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
   char configMessage[17];

   switch( zigbeeMessage[3] )
   {
      //RX Packet
      case 0x81:
      {
         if( zigbeeMessage[10] == 0x06 )
         {
            //Got A config Message, check to see if we
            //should process
            if( (g_processConfigMessage == 1) && (onesec < 60) )
            {
               memcpy(configMessage, &zigbeeMessage[8], 17);
               ProcessConfigMessage( configMessage );
               g_processConfigMessage = 0;
            }
         }

         else
         {
            if( zigbeeMessage[10] == 0x01 )
            {
               //Got a Location Report TX from another unit, send
               //in to server
               TxZigbeeToSerial( zigbeeMessage, zigbeeMessageLen );
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

void ProcessSensorData( struct cnSensorReport *sensorRpt )
{

   //fprintf(COM_VHF, "%S", "PS1" );
   //Make sure we have a valid tagID
   if( g_tagID != 0 )
   {

      //fprintf(COM_VHF, "%S", "PS2" );
      //Read all sensor values into global SensorData struct
      GetSensors();

      //Get TagID for report
      (*sensorRpt).tagID = g_tagID;

      //Get battery data
      (*sensorRpt).battery = SensorData.battery;

      //Copy Sensor values to report struct
      //Wind Speed
      (*sensorRpt).speed = SensorData.speed;

      //Wind Direction
      (*sensorRpt).direction= SensorData.direction;

      //External Temperature
      (*sensorRpt).temperature= SensorData.temperature;

      //On-board Temperature
      (*sensorRpt).btemperature= SensorData.btemperature;

      //Humidity
      (*sensorRpt).humidity= SensorData.humidity;

      //Barometric Pressure
      (*sensorRpt).pressure= SensorData.pressure;

      //Rain Fall
      (*sensorRpt).rain= SensorData.rain;

      //Solar Radiation
      (*sensorRpt).solar= SensorData.solar;

      //Soil Moisture
      (*sensorRpt).moisture= SensorData.moisture;

      //Send it in
      WriteFRReportToServer( sensorRpt, sizeof(*sensorRpt) );
   }
   else
   {
      //Dont have a valid tagID, Request it
      QueryZigbeeTagID();
      //fprintf(COM_VHF, "%S", "PS3" );
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
// Send a Poll Command to the FCS
//******************************************************************************
void PollFCS (void)
{
   cRxByteCnt=0;
   fcs_flag=0;
   disable_interrupts(int_ext);
   delay_ms(10);
   MAXA_putc(0xFA);
   MAXA_putc(0x51);
   ext_int_edge(0,H_TO_L);
   enable_interrupts(int_ext);
}
//******************************************************************************
//******************************************************************************

//******************************************************************************
// Calculate FCS Packet Checksum
//******************************************************************************
unsigned int16 FCSChecksum (void)
{
   unsigned int16 i, csum;
      csum=0;

      for (i=3; i<cRxBuffer[2]+3; i++) {
         csum=csum+cRxBuffer[i];
      }
      csum=0xFFFF-csum;
      return(csum);
}
//******************************************************************************
//******************************************************************************


//******************************************************************************
// Poll the FCS, Read the FCS packet, parse it and update FCSData struct
// Return a 1 indicating success or a 0 indicating failure.
//******************************************************************************
unsigned int8 ProcessFCSData(void)
{
   /*FCSRep.tagID   = g_tagID;
   FCSRep.battery = GetBatteryReading();
   FCSRep.fcs0    = 123;
   FCSRep.fcs1    = 456;
   FCSRep.fcs2    = 789;
   FCSRep.fcs3    = 246;
   FCSRep.fcs4    = 369;
   FCSRep.fcs5    = 478;
   FCSRep.fcs6    = 2468;
   FCSRep.fcs7    = 5789;*/

   if (FCSChecksum() !=  Make16(cRxBuffer[cRxByteCnt-3],cRxBuffer[cRxByteCnt-2]))
      return(0);        //Bad packet

   FCSRep.tagID   = g_tagID;
   //FCSRep.battery = GetBatteryReading();
   FCSRep.fcs0    = Make16(cRxBuffer[4],cRxBuffer[5]);
   FCSRep.fcs1    = Make16(cRxBuffer[7],cRxBuffer[8]);
   FCSRep.fcs2    = Make16(cRxBuffer[10],cRxBuffer[11]);
   FCSRep.fcs3    = Make16(cRxBuffer[13],cRxBuffer[14]);
   FCSRep.fcs4    = Make16(cRxBuffer[16],cRxBuffer[17]);
   FCSRep.fcs5    = Make16(cRxBuffer[19],cRxBuffer[20]);
   FCSRep.fcs6    = Make16(cRxBuffer[22],cRxBuffer[23]);
   FCSRep.fcs7    = Make16(cRxBuffer[25],cRxBuffer[26]);

   //Send it in
   WriteFRReportToServer( &FCSRep, sizeof(FCSRep) );

   return(1);

}
//******************************************************************************
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
// Get Battery reading - This function affects the XMIT (power line) of the VHF Radio
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

   //Sensor Report Interval
   g_sensorReportInterval = make16( configMessage[12],configMessage[11]);

   //Report Interval for FCS Sensor
   g_FCSReportInterval = make16( configMessage[14],configMessage[13]);

   //Turn radio off when not in use to save power
   g_radioPowerSave = configMessage[15];

   g_radioBufferCheck = configMessage[16];

   if( g_tagID != prevXbeeTagID )
   {
      //The tag ID changed, update xbee device
      InitZigbee( g_tagID );
   }

   SaveConfigsToEEProm();

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
   write_EEPROM( 8, make8(g_sensorReportInterval,1) );
   write_EEPROM( 9, make8(g_sensorReportInterval,0) );

   //FCS Report Interval
   write_EEPROM( 10, make8(g_FCSReportInterval,1) );
   write_EEPROM( 11, make8(g_FCSReportInterval,0) );

   //Radio Power Save
   write_EEPROM( 12, g_radioPowerSave );

   //Radio Buffer Check
   write_EEPROM( 13, g_radioBufferCheck );
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
      g_sensorReportInterval = make16( read_EEPROM(8),read_EEPROM(9) );

      //FCS Sensor Report interval
      g_FCSReportInterval   = make16( read_EEPROM(10),read_EEPROM(11) );

      //Radio Power Save
      g_radioPowerSave = read_EEPROM(12);

      //Radio Buffer Check
      g_radioBufferCheck = read_EEPROM(13);

      return 1;
   }

   return 0;
}
//******************************************************************************
//End Of LoadConfigsFromEEProm
//******************************************************************************

//******************************************************************************
// Turn VHF Radio on ( power up )
//******************************************************************************
void VHFOn()
{
   //Turn ON Tx Power
   output_high(XMIT);

   //Warm up time
   delay_ms(600);

   SensorData.battery = ReadAnalogChannelAvg(BCH);           //Also Read Battery Voltage
   FCSRep.battery = SensorData.battery;

   g_radioPowerState = 0;
}
//******************************************************************************
//******************************************************************************

//******************************************************************************
// Turn VHF Radio OFF ( power off )
//******************************************************************************
void VHFOff()
{
   int8 x = 0;
   //Only turn off radio if Radio Power Save is enabled
   if( g_radioPowerSave == 1 )
   {
       //Give the radio 15 seconds to clear the Radios Internal Hardware send buffer
       //In between waits, check to make sure no outgoing VHF traffic appears
       //in the software buffer
       for( x = 0; x < 15; x++ )
       {
         delay_ms(1000);
         if( IsZigbeeQueueEmpty() != 1 )
         {
            return;
         }
       }

      //Turn OFF Tx Power
      output_low(XMIT);

      g_radioPowerState = 0;
   }
}
//******************************************************************************
//******************************************************************************





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

   sprintf(cOutputMess, "g_sensorReportInterval = %05lu\n\r",g_sensorReportInterval);
   MAXA_printf( cOutputMess );

   sprintf(cOutputMess, "g_FCSReportInterval = %05lu\n\r",g_FCSReportInterval);
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


