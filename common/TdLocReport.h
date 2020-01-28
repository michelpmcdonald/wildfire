//Location report that gets
//sent to server
struct ptdLocReport
{
   int16        tagID;

   int8         reportType;
   int16        battery;

   signed int32 lat;
   signed int32 lon;
   int16        height;
   
   int16        speed;
   int16        heading;
   
   int16        temp;

   int16        xPos;
   int16        yPos;
   int16        zPos;
   
   int16        spare;

};

struct ptdConfigCommand
{
   //Not used for incoming
   int16        tagID;

   //
   int8         reportType;
   int16        battery;

   //Not used on Zigbee side
   int16 targetTagID;

   //Holds the Zigbee TagID of THIS unit
   int16 newTagID;

   //Battery threshold Cutout
   int16 batteryCutout;

   //When awake, check for motion ~X seconds, should
   //never == 0( would get divide by 0 below )
   int awakeMotionCheckTime;

   //If no motion after ~X seconds, go into sleep mode
   //0 means no sleep
   int16 noMotionCutoutTime;

   //Accelometer change threshold.  Any change beyond
   //X is considered movement, not device drift
   int posChangeThreshold;

   //Outgoing Zigbee Destination Address,
   //defaults to broadcast( All Zigbees )
   int16 xbeeDestinAddress;

   //Interval( seconds) to send in Device Report( LocRep, ETRep )
   int16 reportInterval;

   //This software supports 2 devices 3 = PTD, 4 = ET
   int deviceType;

   //Output Startup Message 1 = yes, 0 = no
   int startupMessage;

   //seconds to go from sleep mode
   //to deep sleep(everything powered off)
   int16 deepSleep;

   //send out loc report even if unit is sleeping(not moving)
   int16 g_sleepReportInterval;
};

//Equipment Tag Report
struct etReport
{
   int16        tagID;
   int8         reportType;
   int16        battery;
};
