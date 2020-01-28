// Personnel Tracker, final Prototype Board
// ARI - 08-24-2007 - Changed definitions for Accelerometer XCH, YCH, and added ZCH
//                    to conform to WSN2 board
//                    Un commented "pos = ReadAnalogChannelAvg (ZCH);" in StartupOutputTest()
//                    Changed FUSES from NOMCLR to MCLR
//       * Added "ATSM=1" command for Pin Hibernate mode (Sleep) in
//         InitZigbee() function
//       * Status message & configuration  is displayed on startup in StartupOutputTest() function
//       * Added define for DEFAULT_ID

#include <18F6722.h>
#device  adc=10
#include <stdlib.h>

// External Crystal Oscillator
#use delay(clock=7372800,restart_wdt)

//#fuses NOWDT, HS, NOPROTECT, NOIESO, NOBROWNOUT, BORV25, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOCPB, NOEBTRB, NOEBTR, NOWRTD, NOWRTC, NOWRTB, NOFCMEN, WDT_32768, LPT1OSC, NOMCLR, NOXINST, CCP2B3
#fuses WDT, WDT_256, HS, NOPROTECT, NOIESO, NOBROWNOUT, BORV25, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOCPB, NOEBTRB, NOEBTR, NOWRTD, NOWRTC, NOWRTB, NOFCMEN, LPT1OSC, NOMCLR, NOXINST, CCP2B3

// Hardware UART#1 connected to XBEE
#USE RS232(BAUD=9600, XMIT=PIN_C6, RCV=PIN_C7, ERRORS, RESTART_WDT, STREAM=COM_XBEE)

//Hardware UART#2 connected to GPS
#USE RS232(BAUD=4800, XMIT=PIN_G1, RCV=PIN_G2, ERRORS, RESTART_WDT, STREAM=COM_GPS)

//Zigbee Interrupt for use in common FR_Zigbee.h
#define ZIGBEE_INTERRUPT #int_rda

//GPS Interrupt for use in Gps.h
#define GPS_INTERRUPT    #int_rda2

//Define causes unit test functions to compile
//#define ENABLE_UNIT_TEST



#include "FR_Zigbee.h"
#include "TdLocReport.h"
#include "Gps.h"

//LED OUTPUT
#define LED    PIN_E3

// XBEE Pin Definitions
#define SLP    PIN_D1         //XBEE SLEEP PIN
#define CTS    PIN_D3         //XBEE CTS PIN
#define RTS    PIN_D4         //XBEE RTS PIN
#define DTR    PIN_D2         //XBEE DTR PIN
#define RESET  PIN_D0         //XBEE RESET PIN

//EM-408 GPS Module Pin Definitions
#define GPSEN  PIN_E1         //EM-408 ENABLE PIN

// LM-60 Temperature Sensor Pin Definitions
#define  TCH   6              //Analog Channel - Temperature
#define  TPWR  PIN_C0         //Power Pin

// MCP1525 Voltage Reference Pin Definitions
#define  VPWR  PIN_C2         //Power Pin

// ADXL322/330 Accelerometer Pin Definitions
#define  XCH    9               //Analog Channel - Xpos
#define  YCH    10              //Analog Channel - Ypos
#define  ZCH    11              //Analog Channel - Zpos
#define  APWR   PIN_D7         //Power Pin

// Battery Monitor Pin Definitions
#define  BCH    7             //Analog Channel - Battery Voltage
#define  BPWR   PIN_E0

//******************************************************************************
//Globals
//******************************************************************************
   //gets incremented every one second
   int16 g_oneSec = 0;

   //Allow Config messages to be Processed
   int g_processConfigMessage = 1;


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

   //When awake, check for motion ~X seconds, should
   //never == 0( would get divide by 0 below )
   int g_awakeMotionCheckTime = 2;

   //If no motion after ~X seconds, go into sleep mode
   //0 means no sleep
   int16 g_noMotionCutoutTime = 300;

   //Accelometer change threshold.  Any change beyond
   //X is considered movement, not device drift
   int g_posChangeThreshold = 5;

   //Outgoing Zigbee Destination Address,
   //defaults to broadcast( All Zigbees )
   int16 g_XbeeDestinAddress = 0xFFFF;

   //Interval( seconds) to send in Device Report( LocRep, ETRep )
   int16 g_reportInterval = 1;

   //This software supports 2 devices 3 = PTD, 4 = ET
   int g_deviceType = 3;

   //Output Startup Message 1 = yes, 0 = no
   int g_StartupMessage = 1;

   //Seconds till the unit is moved from sleep
   //to deep sleep, 0 disable
   int16 g_deepSleep = 500;

   //Sleep report interval secondswdt
   //Even if the unit is asleep, or deep sleep
   //have the device report in every x( 30 to 60 minutes suggested )
   int16 g_sleepReportInterval = 1000;

   #define DEFAULT_ID 24001


//******************************************************************************
//Function Decs
//******************************************************************************

   //Convert GGA from GPS and Send it out via Zigbee
   void ProcessGGA( struct ptdLocReport *locRep );

   //Get Incoming Zigbee data and process it
   void ProcessIncomingXbeePacket();

   //Check voltage, shutdown if below threshold to avoid battery damage
   void HardShutDownCheck(int16 *pLastBattCutoffCheckSeconds);

   //Chec EEProm to see if Config have beens saved
   //if so set them in code
   int LoadConfigsFromEEProm();

   //Transparent Mode startup Output message
   void StartupOutputTest();

   //Save global settings to EEProm
   void SaveConfigsToEEProm();

   //Take values out of config message and assign
   //to config vars, save to EEProm
   void ProcessConfigMessage( char *configMessage  );

   //Main Processing
   void ProcessSleepMode(int16 *pXpos, int16 *pYpos, int *pProcessingState,
                       int16 *pStartSleepSeconds, int *pDeepSleepOn,
                       struct ptdLocReport *pLocRep );

   void ProcessFullPowerMode( int16 *pXpos, int16 *pYpos, int *pProcessingState,
                           int16 *pAccelSeconds, int *pNoMotionCount,
                           struct ptdLocReport *pLocRep,
                           int16 *pStartSleepSeconds );

   #ifdef ENABLE_UNIT_TEST
      //Unit test for eeprom write\read
      //WARNING will Overwrite EEProm and will NOT restore
      //Pre function EEprom config values
      void TestEEpromConfig();

      void ZigbeeWriteConfig( int16 zigbeeDestin  );

      void WipeEEProm();

      void DebugZigbeeMessage( char* cOutputMess,int16 len,int16 zigbeeDestin );
   #endif


//******************************************************************************
//******************************************************************************


// Function to read a specified Analog Channel from the 10-bit ADC
// Takes 50 readings, eliminates glitches and returns the average reading.
unsigned int16 ReadAnalogChannelAvg (int8 chn) {
   unsigned int8 i;
   unsigned int16 j;
   unsigned int16 a;
   unsigned int16 e;
   unsigned int16 k[51];

   set_adc_channel(chn);
   delay_us(10);
   a=0;
   for (i=0;i<50;i++) {
      k[i] = Read_ADC();
      a=a+k[i];
   }
   a=a/50;

   e=0;
   j=0;
   // scan for glitches
   for (i=0;i<50;i++) {
      if (k[i] < a+6 && k[i] > a-6) {
         j=j+k[i];
      }
      else {
         e++;
      }
   }
   j=j/(50-e);

   if (a > 5)
      return(j);
   else
      return(a);
}

#int_timer2
second_handler() {
   static unsigned int8 ic=0,s=0;
   ic++;
   if (ic>29)
   {
      ic=0;
      g_oneSec++;
   }
}



//Get the current temp from the ptd
void GetLocRepTemp( struct ptdLocReport *locRep )
{
   (*locRep).temp = ReadAnalogChannelAvg(TCH);
}

// Read Accelerometer X and Y position into Global variables x and y
//
void GetLocRepAccelerometer (struct ptdLocReport *locRep)
{
   (*locRep).xPos = ReadAnalogChannelAvg (XCH);
   (*locRep).yPos = ReadAnalogChannelAvg (YCH);


}

void GetLocRepBattery( struct ptdLocReport *locRep )
{
   (*locRep).battery = ReadAnalogChannelAvg(BCH);
}


void LocRepZigbeeTx( struct ptdLocReport *locRep, int16 destinAddress )
{
   ZigbeeTx( locRep, sizeof(*locRep), destinAddress );
}

// Reset XBEE Module (assume that RESET line has been initialized
// at the HIGH state.  Call Once during start only
void ResetXBEE (void) {
   output_high(RESET);                   //XBEE RESET=1
   output_low(DTR);                   //XBEE DTR/SLEEP=0
   output_low(RTS);                    //XBEE RTS=0
   input(SLP);                         //XBEE Sleep Status Output
   input(CTS);                         //XBEE CTS line Output
   delay_ms(100);
   output_low(RESET);                   //XBEE RESET=1
   delay_us(1);
   output_high(RESET);                   //XBEE RESET=1
   delay_ms(100);
}

void StartUpLedFlash(int flashes)
{
   int x;

   //Turn it off
   output_low(LED);

   //Flash it
   for( x = 0; x < flashes; x++ )
   {
      output_toggle(LED);
      delay_ms(120);
   }

   //Be sure to leav it ON
   output_high(LED);

}

//returns 1 for motion, 0 for no change
int CheckForMotion( int16 *pXpos, int16 *pYpos )
{
   int16 curXpos;
   int16 curYpos;

   signed int16 xDiff;
   signed int16 yDiff;

   //char test[40];

   //Don't reassign unless threshold is exceeded or we might drift
   curXpos = ReadAnalogChannelAvg (XCH);
   curYpos = ReadAnalogChannelAvg (YCH);

   xDiff = *pXpos-curXpos;
   yDiff = *pYpos-curYpos;

   //sprintf(test, "xd: %03ld yd: %03ld \n\r", abs(xDiff), abs(yDiff) );
   //ZigbeeTx( test, 19, 0xFFFF );

   if( (abs(xDiff) > g_posChangeThreshold ) ||
       (abs(yDiff) > g_posChangeThreshold ) )
   {
      *pXpos = curXpos;
      *pYpos = curYpos;
      return 1;
   }

   return 0;
}

void DevicePowerDown()
{
   //Shutdown devices to save power

   //Turn OFF Temp Sensor
   output_low(TPWR);

   //Turn OFF Accelerometer
   output_low(APWR);

   //Turn Off Zigbee
   output_high(DTR);

   //Turn OFF Voltage Divider (Power Monitor)
   output_low(BPWR);
}

void DevicePowerUp()
{
   //Shutdown devices to save power

   //Turn ON Temp Sensor
   output_high(TPWR);

   //Turn ON Accelerometer
   output_high(APWR);

   //Turn ON Zigbee
   output_low(DTR);

   //Turn ON Voltage Divider (Power Monitor)
   output_high(BPWR);

   delay_ms(100);


}


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
   //Last Accel Check time in seconds
   int16 lastAccelCheckSeconds;
   int noMotionCount;

   //Last BatteryCutOffCheck
   int16 lastBattCutoffCheckSeconds = 0;

   //Keeps count of time unit is in sleep mode
   //used to go into deep sleep report send
   int16 startSleepSeconds;

   //Indicates if unit is in
   //deep sleep mode
   int deepSleepOn = 0;


   //keep last accel readings
   int16 xPos;
   int16 yPos;

   //Awake(1), asleep(0), Command(2) status
   int processingState;

   int16 startProcessingSeconds;

   int16 lastETReport;
   signed int16 etTimeCheck = 0;
   int etSleepLoop  = 0;


   //Et Report
   struct etReport etRep;

   //Create a report to hold
   //local and incoming
   struct ptdLocReport locRep;
   locRep.reportType = (int8)0x0E;
   
   //Test code only
   //hard code new fields for test
   locRep.speed = 357;
   locRep.heading = 46;
   locRep.zPos = 274;
   locRep.spare = 14789;


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

   //Default for INT OSC is 1MHz
   //setup_oscillator(OSC_8MHZ);

   //External VREF+
   setup_adc_ports(ALL_ANALOG|VSS_VREF);

   setup_adc(ADC_CLOCK_INTERNAL);
   setup_psp(PSP_DISABLED);
   setup_spi(FALSE);
   setup_timer_0(RTCC_OFF);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DIV_BY_16,240,16);
   setup_timer_3(T3_DISABLED|T3_DIV_BY_1);
   setup_comparator(NC_NC_NC_NC);
   setup_vref(FALSE);

   output_high(LED);

   //Init GPS RX status
   InitGpsRxBuffer();

   //Set Zigbee Buffers
   InitZbRx();

   //Turn On, Reset the XBEE Module
   ResetXBEE();

   //Enable GPS Module
   output_high(GPSEN);

   //Turn ON Voltage Reference
   output_high(VPWR);

   //Turn ON Temp Sensor
   output_high(TPWR);

   //Turn ON Accelerometer
   output_high(APWR);

   //Turn on Voltage Divider (Power Monitor)
   output_high(BPWR);

   delay_ms(200);

   //GGAOn
   InitGPS( g_reportInterval );

   //Zigbee transparent startup message
   if( g_StartupMessage == 1 )
   {
      StartupOutputTest();
   }
   else
   {
      output_high(LED);
   }



   //Assign TagID, API Mode, Power Level
   InitZigbee(g_tagID);

   //ZigbeeWriteConfig( 0x5555  );

   // XBEE Rx data Interrupts
   enable_interrupts(int_rda);

   //GPS Rx Data Interrupts
   enable_interrupts(int_rda2);

   // 30 Hz Clock
   enable_interrupts(int_timer2);

   enable_interrupts(global);

   //GPS RX Error Flags
   cRxErrorFlag=0;
   cError=0;

   //Get the current tag from the Zigbee
   QueryZigbeeTagID();

   QueryGpsForLocation();

   lastAccelCheckSeconds = g_oneSec;

   noMotionCount = 0;

   processingState = 1;

   StartUpLedFlash(10);

   startProcessingSeconds = g_oneSec;

   lastETReport = 0;


   //WipeEEProm();

   if( g_deviceType == 4 )
   {
      //turn Everything off
      DevicePowerDown();

      //Turn ON Zigbee
      output_low(DTR);

      while( (g_oneSec-startProcessingSeconds) < 60 )
      {
         delay_ms(20);

         //Check For zigbee
         if( zbRxMessageFound == 1 )
         {
            //Received something from the Xbee
            ProcessIncomingXbeePacket();
         }
      }
      g_processConfigMessage = 0;
   }



   while (1)
   {
      if( g_deviceType == 3 )
      {
         //Force the watch dog timer to reset
         delay_ms(1);

         //Check for low battery\shutdown to avoid battery damage
         HardShutDownCheck(&lastBattCutoffCheckSeconds);

         //If we have been looping for more than 60,
         //Disable Config MEssage Prcessing
         if( (g_processConfigMessage == 1) && ((g_oneSec-startProcessingSeconds) > 60) )
         {
            g_processConfigMessage = 0;
         }

         switch( processingState )
         {
            //Asleep, check accel to see if we need to wake
            case 0:
               ProcessSleepMode( &xPos, &yPos, &processingState,
                                 &startSleepSeconds, &deepSleepOn, &locRep );
               break;


            //full power state
            case 1:
               //Normal Ops, check for\Send in GPS Message, check for no motion
               ProcessFullPowerMode( &xPos, &yPos, &processingState,
                                  &lastAccelCheckSeconds,
                                  &noMotionCount, &locRep, &startSleepSeconds );
               break;

            default:
               //Never should hit this
               break;
         }
      }
      else
      {

         etRep.reportType = 0x03;

         //Force the watch dog timer to reset
         delay_ms(1);

         //Check for low battery\shutdown to avoid battery damage
         //Turn ON Voltage Divider (Power Monitor)
         output_high(BPWR);
         delay_ms(100);
         HardShutDownCheck(&lastBattCutoffCheckSeconds);
         output_low(BPWR);
         delay_ms(100);

         //Turn ON Zigbee
         output_low(DTR);

         //Turn ON Voltage Divider (Power Monitor)
         output_high(BPWR);

         delay_ms(200);

         //Get the TAG, might of changed
         etRep.tagID = g_tagID;

         //Get the battery Reading
         etRep.battery = ReadAnalogChannelAvg(BCH);

         ZigbeeTx( &etRep, sizeof(etRep), g_XbeeDestinAddress );

         //Flash Led
         output_low(LED);
         delay_ms(100);
         output_high(LED);

         //Turn Off Zigbee
         output_high(DTR);

         //Turn OFF Voltage Divider (Power Monitor)
         output_low(BPWR);
         delay_ms(100);

         for( etSleepLoop = 0; etSleepLoop < g_reportInterval; etSleepLoop++ )
         {
            sleep();
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
//Main Process - called when in sleep mode, turns on Accel, checks for motion
//******************************************************************************
void ProcessSleepMode(int16 *pXpos, int16 *pYpos, int *pProcessingState,
                       int16 *pStartSleepSeconds, int *pDeepSleepOn,
                       struct ptdLocReport *pLocRep )
{
   //If gpswarmup is greater than 0 and less than 30
   //unit is in a gps warmup mode preparing for a deep sleep
   //report send.   This is done so that while the
   //gps is warming up, the normal sleep mode
   //processing is still done.  If the unit is moved(woken up)
   //during the warmup stage, don't want to miss that movement so
   //normal processing must continue during the gps warmup.
   //Also, if unit is moved during gps warm up stage, no need
   //to worry about sending out location report since the reports
   //will go out once the unit is awake
   static int16 gps_warmup = 0;
   static int16 gps_warmup_time_len = 30;

   int loop;
   //Devices are off, give up the CPU
   sleep();

   //One More WDT cycle
   (*pStartSleepSeconds)++;

   //if the GPS is greater than 0 but less than 30, we
   //are in the middle of a warmup and have just completed
   //another WDT 1 second cycle
   if( (gps_warmup > 0) && (gps_warmup <= gps_warmup_time_len) )
   {
      gps_warmup++;
   }

   //If it been past the warmup time send a position report
   if( gps_warmup >= gps_warmup_time_len )
   {
      //Enable the GPS Rx Data Interrupts
      enable_interrupts(int_rda2);

      //Turn on the hardware, need this for location report
      DevicePowerUp();
      delay_ms(200);

      //Check GPS for Location
      while(cRxGPSMsgReady != 1)
      {
         delay_ms(100);
      }

      output_high(LED);

      ProcessGGA(pLocRep);

      //Process GGA leaves the LED on, turn it off
      output_low(LED);

      //Shut the devices back down
      DevicePowerDown();

      //If we are in deep sleep mode, shutdown the gps
      if( *pDeepSleepOn == 1 )
      {
         output_low(GPSEN);
      }

      gps_warmup = 0;

   }

   //Check to see if it is time to send out a report,
   if( g_sleepReportInterval > 0 )
   {
      if( ((*pStartSleepSeconds % g_sleepReportInterval) == 0) &&
          ( gps_warmup == 0 ) )
      {
         //If the unit is in a deep sleep, gps if off and must be
         //warmed up before sendig report
         if( *pDeepSleepOn == 1 )
         {

            gps_warmup = 1;

            //Disable the GPS Rx Data Interrupts while the unit warms up
            disable_interrupts(int_rda2);

            //Clear any GGA message
            InitGpsRxBuffer();

            //turn on the GPS
            output_high(GPSEN);
         }
         //Not in deep sleep, gps on no need to warm up
         else
         {
            //Turn on the hardware, need this for location report
            DevicePowerUp();
            delay_ms(200);

            //Reset the buffer to get rid of any old GGA
            InitGpsRxBuffer();

            //Check GPS for Location
            while(cRxGPSMsgReady != 1)
            {
               delay_ms(100);
            }

            output_high(LED);

            ProcessGGA(pLocRep);

            //Process GGA leaves the LED on, turn it off
            output_low(LED);

            //Shut the devices back down
            DevicePowerDown();
         }

      }
   }



   //If not already in deep sleep AND not in the middle of a gps warmup\sleep
   //report send, check to see if its time to go into a deep sleep
   if( (*pDeepSleepOn == 0) && (gps_warmup == 0) )
   {
      //only attempt deep sleep if enabled
      if( g_deepSleep > 0 )
      {
         //Check to see if it has been more than X cycles
         //since we slept
         if( *pStartSleepSeconds >= g_deepSleep )
         {
            //At this point, only the GPS
            //is up, Shut it down
            output_low(GPSEN);
            *pDeepSleepOn = 1;

         }
      }
   }

   //Turn ON Accelerometer
   output_high(APWR);
   delay_ms(50);

   //If movment, wake up
   if( CheckForMotion( pXpos,pYpos ) == 1 )
   {
      //If moved, wake up
      DevicePowerUp();
      StartUpLedFlash(10);

      //Must of been woken up in the
      //middle of a GPS Warmup, let the warmup finish
      if(gps_warmup != 0)
      {
         //Let if finish warming up
         //Flash while the GPS warms up
         for( loop = 0; loop < (30 - gps_warmup); loop++ )
         {
            output_toggle(LED);
            delay_ms(1000);
         }

         //If it was in the middle of a warmup, the
         //GPS interupt is disabled, reeanble it
         enable_interrupts(int_rda2);

         gps_warmup = 0;
      }

      //If in deep sleep, start the
      //GPS and let it warm up
      else if( *pDeepSleepOn == 1 )
      {
         //turn on the GPS
         output_high(GPSEN);

         //Flash while the GPS warms up
         for( loop = 0; loop < 30; loop++ )
         {
            output_toggle(LED);
            delay_ms(1000);

         }
      }

      //Now unit is awake
      *pDeepSleepOn = 0;

      //Make sure to leave led on
      output_high(LED);

      *pProcessingState = 1;
   }
   else
   {
      //Turn OFF Accelerometer
      output_low(APWR);
   }

}
//******************************************************************************
//End ProcessSleepMode
//******************************************************************************

//******************************************************************************
//Main Process - called when in normal Mode- check for GPS data, no motion, XBee
//******************************************************************************
void ProcessFullPowerMode( int16 *pXpos, int16 *pYpos, int *pProcessingState,
                           int16 *pAccelSeconds, int *pNoMotionCount,
                           struct ptdLocReport *pLocRep,
                           int16 *pStartSleepSeconds )
{
   signed int16 accelTimeDiff;

   //Dont attempt powerdown modes unless
   //g_noMotionCutoutTime is greater than 0
   if(g_noMotionCutoutTime > 0 )
   {
      //don't check, divide by 0 possible below
      if(g_awakeMotionCheckTime > 0 )
      {
         //Check the accel for motion every two seconds
         accelTimeDiff = g_oneSec - *pAccelSeconds;
         if( abs(accelTimeDiff)  >= g_awakeMotionCheckTime )
         {
            //Check for motion, xPos and yPos hold last check
            if( CheckForMotion( pXpos,pYpos ) == 1 )
            {
               output_high(LED);
               *pNoMotionCount = 0;
            }
            else
            {
               (*pNoMotionCount)++;
            }

            //No motion for X seconds( noMotionCount*seconds)
            if( *pNoMotionCount > (g_noMotionCutoutTime / g_awakeMotionCheckTime) )
            {
               output_low(LED);
               *pNoMotionCount = 0;

               //Set Processing state to asleep
               *pProcessingState = 0;

               //Reset the Start sleep cycle
               *pStartSleepSeconds = 0;

               DevicePowerDown();
            }

            //reset seconds marker for next check
            *pAccelSeconds = g_oneSec;
         }
      }
   }

   //Check For zigbee
   if( zbRxMessageFound == 1 )
   {
      //Received something from the Xbee
      ProcessIncomingXbeePacket();
   }

   //Check GPS for Location
   if(cRxGPSMsgReady == 1)
   {
      ProcessGGA(pLocRep);
   }
}
//******************************************************************************
//End ProcessFullPowerMode
//******************************************************************************

//******************************************************************************
//Checks for power level below low battery threshold, shuts it all down
//******************************************************************************
void HardShutDownCheck(int16 *pLastBattCutoffCheckSeconds)
{
   static signed int16 timeCheck = 0;

   //Calc seconds since last check
   timeCheck = g_oneSec - *pLastBattCutoffCheckSeconds;

   if( abs(timeCheck) > 5 )
   {
      if(  ReadAnalogChannelAvg(BCH) < g_batteryCutout )
      {
         setup_wdt(WDT_OFF);
         output_low(LED);
         setup_adc(ADC_OFF);
         output_low(GPSEN);   //Disable GPS Module
         output_low(VPWR);    //Turn OFF Voltage Reference
         output_low(TPWR);    //Turn OFF Temp Sensor
         output_low(APWR);    //Turn OFF Accelerometer
         output_low(BPWR);    //Turn OFF Voltage Divider (Power Monitor)
         output_high(DTR);    //XBEE DTR/SLEEP=1

         disable_interrupts(int_rda);          // XBEE Rx data Interrupts
         disable_interrupts(int_rda2);          // GPS data Interrupts
         disable_interrupts(global);
         sleep();
      }
   }

   //Reset last check time to now
   *pLastBattCutoffCheckSeconds = g_oneSec;
}
//******************************************************************************
//End HardShutDownCheck()
//******************************************************************************


//******************************************************************************
//Got A GGA message from GPS, build location report and send
//******************************************************************************
void ProcessGGA( struct ptdLocReport *locRep )
{
   int x;

   //Make sure we have a valid tagID
   if( g_tagID != 0 )
   {
      //ZigbeeTx(test2, 4, 0xFFFF );
      //Found GPS GGA sentence
      GetLocRepGGA( locRep, cRxBuffer );
      GetLocRepAccelerometer( locRep );
      GetLocRepTemp( locRep );
      GetLocRepBattery( locRep );

      (*locRep).tagID = g_tagID;

      //Send it
      LocRepZigbeeTx( locRep, g_XbeeDestinAddress );

     //Flash Led
     output_low(LED);
     delay_ms(20);
     output_high(LED);

   }
   else
   {
      //Dont have a valid tagID, Request it
      QueryZigbeeTagID();

   }

   //Reset GPS buffer
   cRxErrorFlag=0;
   cError=0;
   InitGpsRxBuffer();
}
//******************************************************************************
//End ProcessGGA
//******************************************************************************




//******************************************************************************
//Got A Zigbee API message
//******************************************************************************
void ProcessIncomingXbeePacket()
{
   char configMessage[25];

   switch( zbRxMessage[3] )
   {
      //RX Packet
      case 0x81:
      {
         if( zbRxMessage[10] == 0x04 )
         {
            //Got A config Message, check to see if we
            //should process
            if( g_processConfigMessage == 1 )
            {
               memcpy(configMessage, &zbRxMessage[8], 25);
               StartUpLedFlash(20);
               ProcessConfigMessage( configMessage );
               g_processConfigMessage = 0;
            }
         }
         zbRxMessageFound = 0;

      }
      break;

      //AT command Response
      case 0x88:
      {
         ZigbeeAtQueryResp( zbRxMessage, zbRxMessageLen );
         zbRxMessageFound = 0;
      }
      break;

      default:
      {
         //unknown
         zbRxMessageFound = 0;
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
   //Holds the prev report Rate
   int16 oldReportInterval;

   //THIS zigbee tagID
   g_tagID = make16(configMessage[8],configMessage[7]);

   //Battery threshold Cutout
   g_batteryCutout = make16( configMessage[10],configMessage[9]);

   //When awake, check for motion
   g_awakeMotionCheckTime = configMessage[11];

   //If no motion after ~X seconds, go into sleep mode
   g_noMotionCutoutTime = make16( configMessage[13],configMessage[12]);

   //Accelometer change threshold.
   g_posChangeThreshold = configMessage[14];

   //Outgoing Zigbee Destination Address,
   g_XbeeDestinAddress = make16( configMessage[16],configMessage[15]);

   //Interval( seconds) to send in Device Report( LocRep, ETRep )
   oldReportInterval = g_reportInterval;
   g_reportInterval = make16( configMessage[18],configMessage[17]);

   //This software supports 2 devices 3 = PTD, 4 = ET
   g_deviceType = configMessage[19];

   //Output Startup Message 1 = yes, 0 = no
   g_StartupMessage = configMessage[20];

   //cycles( 1.0xx) until unit goes from sleep( gps on) to Deep Sleep( GPS off )
   g_deepSleep = make16( configMessage[22],configMessage[21] );

   g_sleepReportInterval = make16( configMessage[24],configMessage[23] );

   SaveConfigsToEEProm();

   //IF the report rate changed, reset the GPS
   if( oldReportInterval != g_reportInterval )
   {
      InitGPS(g_reportInterval);
   }



}
//******************************************************************************
//End of ProcessConfigMessage
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

      //When awake, check for motion ~X seconds, should
      //never == 0( would get divide by 0 )
      g_awakeMotionCheckTime = read_EEPROM(8);
      if( g_awakeMotionCheckTime < 1 )
      {
         g_awakeMotionCheckTime = 2;
      }

      //If no motion after ~X seconds, go into sleep mode
      //0 means no sleep
      g_noMotionCutoutTime = make16( read_EEPROM(9),read_EEPROM(10) );

      //Accelometer change threshold.  Any change beyond
      //X is considered movement, not device drift
      g_posChangeThreshold = read_EEPROM(11);

      //Outgoing Zigbee Destination Address,
      //defaults to broadcast( All Zigbees )
      g_XbeeDestinAddress = make16( read_EEPROM(12),read_EEPROM(13) );

      //Interval( seconds) to send in Device Report( LocRep, ETRep )
      g_reportInterval = make16( read_EEPROM(14),read_EEPROM(15) );

      //Device type 3 = PTD, 4 = Et
      g_deviceType = read_EEPROM(16);

      g_StartupMessage = read_EEPROM(17);

      g_deepSleep = make16( read_EEPROM(18),read_EEPROM(19) );

      g_sleepReportInterval = make16( read_EEPROM(20),read_EEPROM(21) );

      return 1;
   }

   return 0;
}
//******************************************************************************
//End Of LoadConfigsFromEEProm
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

   //When awake, check for motion ~X seconds, should
   //never == 0( would get divide by 0 )
   if( g_awakeMotionCheckTime < 1 )
   {
      write_EEPROM( 8, 2 );
   }
   else
   {
      write_EEPROM( 8, g_awakeMotionCheckTime );
   }

   //If no motion after ~X seconds, go into sleep mode
   write_EEPROM( 9,  make8(g_noMotionCutoutTime,1) );
   write_EEPROM( 10, make8(g_noMotionCutoutTime,0) );

   //Accelometer change threshold.  Any change beyond
   write_EEPROM(11,g_posChangeThreshold);

   //Outgoing Zigbee Destination Address,
   write_EEPROM( 12, make8(g_XbeeDestinAddress,1) );
   write_EEPROM( 13, make8(g_XbeeDestinAddress,0) );

   //Interval( seconds) to send in Device Report( LocRep, ETRep )
   write_EEPROM( 14, make8(g_reportInterval,1) );
   write_EEPROM( 15, make8(g_reportInterval,0) );

   //Device type 3 = PTD, 4 = Et
   write_EEPROM( 16, g_deviceType );

   //Enable \ disable startup message
   write_EEPROM( 17, g_StartupMessage );

   //Deep sleep
   write_EEPROM( 18, make8(g_deepSleep,1) );
   write_EEPROM( 19, make8(g_deepSleep,0) );

   //Sleep Report interval g_sleepReportInterval
   write_EEPROM( 20, make8(g_sleepReportInterval,1) );
   write_EEPROM( 21, make8(g_sleepReportInterval,0) );

}
//******************************************************************************
//SaveConfigsToEEProm END
//******************************************************************************

//******************************************************************************
//Broadcast a  FR Startup Message Zigbee Transparent
//******************************************************************************
void StartupOutputTest()
{
   int16 pos;
   int16 temp;
   int16 batt;


   SetZigbeeTransparentMode();

   //Flash Twice
   output_low(LED);
   delay_ms(500);
   output_High(LED);
   delay_ms(500);
   output_low(LED);
   delay_ms(500);
   output_high(LED);


   //Test Header
   putc( 0x11, COM_XBEE );
   putc( 0x22, COM_XBEE );
   putc( 0x33, COM_XBEE );
   putc( 0x44, COM_XBEE );
   putc( 0x55, COM_XBEE );
   putc( 0x66, COM_XBEE );

   //Read and out put Accel X
   pos = ReadAnalogChannelAvg (XCH);
   putc( make8(pos, 1), COM_XBEE );
   putc( make8(pos, 0), COM_XBEE );

   //Read and out put Accel Y
   pos = ReadAnalogChannelAvg (YCH);
   putc( make8(pos, 1), COM_XBEE );
   putc( make8(pos, 0), COM_XBEE );

   //Read and out put Accel TODO add Z PIN
   pos = ReadAnalogChannelAvg (ZCH);
   putc( make8(pos, 1), COM_XBEE );
   putc( make8(pos, 0), COM_XBEE );

   //Read and output Temp
   temp = ReadAnalogChannelAvg(TCH);
   putc( make8(temp, 1), COM_XBEE );
   putc( make8(temp, 0), COM_XBEE );

   //Read and output Battery
   batt = ReadAnalogChannelAvg(BCH);
   putc( make8(batt, 1), COM_XBEE );
   putc( make8(batt, 0), COM_XBEE );

   //Test Trailer
   putc( 0x00, COM_XBEE );
   putc( 0x00, COM_XBEE );
   putc( 0xAA, COM_XBEE );
   putc( 0xBB, COM_XBEE );
   putc( 0xCC, COM_XBEE );
   putc( 0xDD, COM_XBEE );
   putc( 0xEE, COM_XBEE );
   putc( 0xFF, COM_XBEE );

   fprintf(COM_XBEE,"\n\r PTD/DET Firmware Build(2.0): ");
   fprintf(COM_XBEE,__DATE__);
   fprintf(COM_XBEE," ");
   fprintf(COM_XBEE,__TIME__);
   fprintf(COM_XBEE,"\n\r Settings: ID = %lu  Report Interval = %lu  DeviceType = %U \n\r",g_tagID,g_reportInterval,g_deviceType);
   fprintf(COM_XBEE,"  NoMotionTime = %lu  BatteryCutout = %lu \n\r",g_noMotionCutoutTime,g_batteryCutout);

      fprintf(COM_XBEE,"\n\r PTD/DET Firmware Build: ");
   fprintf(COM_XBEE,__DATE__);
   fprintf(COM_XBEE," ");
   fprintf(COM_XBEE,__TIME__);
   fprintf(COM_XBEE,"\n\r Settings: ID = %lu  Report Interval = %lu  DeviceType = %U \n\r",g_tagID,g_reportInterval,g_deviceType);
   fprintf(COM_XBEE,"  NoMotionTime = %lu  BatteryCutout = %lu \n\r",g_noMotionCutoutTime,g_batteryCutout);

      fprintf(COM_XBEE,"\n\r PTD/DET Firmware Build: ");
   fprintf(COM_XBEE,__DATE__);
   fprintf(COM_XBEE," ");
   fprintf(COM_XBEE,__TIME__);
   fprintf(COM_XBEE,"\n\r Settings: ID = %lu  Report Interval = %lu  DeviceType = %U \n\r",g_tagID,g_reportInterval,g_deviceType);
   fprintf(COM_XBEE,"  NoMotionTime = %lu  BatteryCutout = %lu \n\r",g_noMotionCutoutTime,g_batteryCutout);

      fprintf(COM_XBEE,"\n\r PTD/DET Firmware Build: ");
   fprintf(COM_XBEE,__DATE__);
   fprintf(COM_XBEE," ");
   fprintf(COM_XBEE,__TIME__);
   fprintf(COM_XBEE,"\n\r Settings: ID = %lu  Report Interval = %lu  DeviceType = %U \n\r",g_tagID,g_reportInterval,g_deviceType);
   fprintf(COM_XBEE,"  NoMotionTime = %lu  BatteryCutout = %lu \n\r",g_noMotionCutoutTime,g_batteryCutout);

      fprintf(COM_XBEE,"\n\r PTD/DET Firmware Build: ");
   fprintf(COM_XBEE,__DATE__);
   fprintf(COM_XBEE," ");
   fprintf(COM_XBEE,__TIME__);
   fprintf(COM_XBEE,"\n\r Settings: ID = %lu  Report Interval = %lu  DeviceType = %U \n\r",g_tagID,g_reportInterval,g_deviceType);
   fprintf(COM_XBEE,"  NoMotionTime = %lu  BatteryCutout = %lu \n\r",g_noMotionCutoutTime,g_batteryCutout);

   delay_ms(5000);
}




#ifdef ENABLE_UNIT_TEST

//Use this for Debug\Unit Testing output
void DebugZigbeeMessage( char* cOutputMess, int16 len, int16 zigbeeDestin )
{
   ZigbeeTx( cOutputMess, len, zigbeeDestin );
}

//******************************************************************************
//Save\Read Config values to\from Eprom Unit TEST code
//Unit test for eeprom write\read
//WARNING will Overwrite EEProm and will NOT restore
//Pre function EEprom config values
//******************************************************************************
void TestEEpromConfig()
{
   //Output message
   char cOutputMess[50];

   //Set them to test values
      g_tagID = 4567;

      //Battery threshold Cutout
      g_batteryCutout = 567;

      g_awakeMotionCheckTime = 123;

      //If no motion after ~X seconds, go into sleep mode
      //0 means no sleep
      g_noMotionCutoutTime = 23456;

      //Accelometer change threshold.  Any change beyond
      //X is considered movement, not device drift
      g_posChangeThreshold = 145;

      //Outgoing Zigbee Destination Address,
      //defaults to broadcast( All Zigbees )
      g_XbeeDestinAddress = 6789;

      //Interval( seconds) to send in Device Report( LocRep, ETRep )
      g_reportInterval = 134;

      //This software supports 2 devices 3 = PTD, 4 = ET
      g_deviceType = 167;

      //Enable \ Disable startup message
      g_StartupMessage = 234;

      //Save em
      SaveConfigsToEEProm();

      //Zero em out
      g_tagID = 0;

      //Battery threshold Cutout
      g_batteryCutout = 0;

      g_awakeMotionCheckTime = 0;

      //If no motion after ~X seconds, go into sleep mode
      //0 means no sleep
      g_noMotionCutoutTime = 0;

      //Accelometer change threshold.  Any change beyond
      //X is considered movement, not device drift
      g_posChangeThreshold = 0;

      //Outgoing Zigbee Destination Address,
      //defaults to broadcast( All Zigbees )
      g_XbeeDestinAddress = 0;

      //Interval( seconds) to send in Device Report( LocRep, ETRep )
      g_reportInterval = 0;

      //This software supports 2 devices 3 = PTD, 4 = ET
      g_deviceType = 0;

      //Enable disable startup
      g_StartupMessage = 0;

   //Read them
      LoadConfigsFromEEProm();

   //Test values read should = values set
      if( g_tagID != 4567 )
      {
         sprintf(cOutputMess, "g_TagID FAILED s=04567 r=%05lu\n\r", g_tagID);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_TagID SUCCESS s=04567 r=%05lu\n\r", g_tagID);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }

      if( g_batteryCutout != 567 )
      {
         sprintf(cOutputMess, "g_batteryCutOut FAILED s=00567 r=%05lu\n\r",
            g_batteryCutout);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_batteryCutOut SUCCESS s=00567 r=%05lu\n\r",
            g_batteryCutout);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }

      if( g_awakeMotionCheckTime != 123 )
      {
         sprintf(cOutputMess, "g_awakeMotionCheckTime FAILED s=123 r=%03U\n\r",
            g_awakeMotionCheckTime);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_awakeMotionCheckTime SUCCESS s=123 r=%03U\n\r",
            g_awakeMotionCheckTime);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      if( g_noMotionCutoutTime != 23456 )
      {
         sprintf(cOutputMess, "g_noMotionCutoutTime FAILED s=23456 r=%05lu\n\r",
            g_noMotionCutoutTime);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_noMotionCutoutTime SUCCESS s=23456 r=%05lu\n\r",
            g_noMotionCutoutTime);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      //Accelometer change threshold.  Any change beyond
      //X is considered movement, not device drift
      if( g_posChangeThreshold != 145 )
      {
         sprintf(cOutputMess, "g_posChangeThreshold FAILED s=145 r=%03U\n\r",
            g_posChangeThreshold);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_posChangeThreshold SUCCESS s=145 r=%03U\n\r",
            g_posChangeThreshold);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      //Outgoing Zigbee Destination Address,
      //defaults to broadcast( All Zigbees )
      if( g_XbeeDestinAddress != 6789 )
      {
         sprintf(cOutputMess, "g_XbeeDestinAddress FAILED s=06789 r=%05lu\n\r",
            g_XbeeDestinAddress);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_XbeeDestinAddress SUCCESS s=06789 r=%05lu\n\r",
            g_XbeeDestinAddress);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      //Interval( seconds) to send in Device Report( LocRep, ETRep )
      if( g_reportInterval != 134 )
            {
         sprintf(cOutputMess, "g_reportInterval FAILED s=00134 r=%05lu\n\r",
            g_reportInterval);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_reportInterval SUCCESS s=00134 r=%05lu\n\r",
            g_reportInterval);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      //This software supports 2 devices 3 = PTD, 4 = ET
      if( g_deviceType != 167 )
      {
         sprintf(cOutputMess, "g_deviceType FAILED s=167 r=%03U\n\r",
            g_deviceType);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_deviceType SUCCESS s=167 r=%03U\n\r",
            g_deviceType);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }

      if( g_StartupMessage != 234 )
      {
         sprintf(cOutputMess, "g_StartupMessage FAILED s=167 r=%03U\n\r",
            g_StartupMessage);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }
      else
      {
         sprintf(cOutputMess, "g_StartupMessage SUCCESS s=167 r=%03U\n\r",
            g_StartupMessage);
         ZigbeeTx( cOutputMess, strlen(cOutputMess), 0xFFFF );
      }


      //Overwrite Marker so it does not look like configs are set
      write_EEPROM(0, 0xAA);
      write_EEPROM(1, 0xBB);
      write_EEPROM(2, 0xCC);
      write_EEPROM(3, 0xDD);


}
//******************************************************************************
//TestEEpromConfig() END
//******************************************************************************

void ZigbeeWriteConfig( int16 zigbeeDestin  )
{
   char cOutputMess[50];

   sprintf(cOutputMess, "g_TagID = %05lu\n\r", g_tagID);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_batteryCutOut = %05lu\n\r", g_batteryCutout);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_awakeMotionCheckTime = %03U\n\r",g_awakeMotionCheckTime);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_noMotionCutoutTime = %05lu\n\r",g_noMotionCutoutTime);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_posChangeThreshold = %03U\n\r",g_posChangeThreshold);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_XbeeDestinAddress = %05lu\n\r", g_XbeeDestinAddress);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_reportInterval = %05lu\n\r", g_reportInterval);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_deviceType = %03U\n\r", g_deviceType);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_StartupMessage = %03U\n\r", g_StartupMessage);
   DebugZigbeeMessage( cOutputMess, strlen(cOutputMess), zigbeeDestin );


   sprintf(cOutputMess, "g_deepSleep = %05lu\n\r", g_deepSleep);
   ZigbeeTx( cOutputMess, strlen(cOutputMess), zigbeeDestin );

   sprintf(cOutputMess, "g_sleepReportInterval = %05lu\n\r", g_sleepReportInterval);
   ZigbeeTx( cOutputMess, strlen(cOutputMess), zigbeeDestin );


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



















