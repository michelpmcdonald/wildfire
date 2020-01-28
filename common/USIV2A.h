// Header file for USIV2 Board Hardware Drivers and Pin Definitions
// 07-03-2007:  Added GPS_Off() function
//              Added port selction parameter to MAXB_init() and MAXA_init() functions
//              Removed LED Toggle from int_timer2() function
// 07-08-2007: Added defines for VHF-B Port Control
//
// 08-26-2007: Battery Check eliminated in GetSensors()
//
// 08-31-2007: Added "vonesec" timer variable for VHF radio timeout.

// Must use this for GetSensors() function's input_d() statement.
#use fast_io (D)

//Function Prototypes
void ResetXBEE (void);
unsigned int16 ReadAnalogChannelAvg (int8 chn);
void SetADCVoltageReference (int v);
void InitializeRainGauge (void);
void GetSensors (void);
void MAXB_init (char port);
void MAXB_putc(char tx);
char MAXB_getc (void);
char MAXB_igetc (void);
void MAXB_printf(char *buffer);
void GPS_ON (void);
void GPS_OFF (void);
void InitializeBoard (void);
void MAXA_init (char port);
void MAXA_putc(char tx);
char MAXA_getc (void);
char MAXA_igetc (void);
void MAXA_printf(char *buffer);

// Pin and ADC Channel Definitions

#define LED    PIN_G4         //LED OUTPUT

// XBEE Pin Definitions
#define SLEEP  PIN_G3         //XBEE SLEEP PIN
#define CTS    PIN_B2         //XBEE CTS PIN
//#define RTS    PIN_         //XBEE RTS PIN
//#define DTR    PIN_         //XBEE DTR PIN
//#define RESET  PIN_         //XBEE RESET PIN

//EM-408 GPS Module Pin Definitions
#define GPSEN  PIN_G0         //EM-408 ENABLE PIN

// LM-60 Temperature Sensor Pin Definitions
#define  TCH   2              //Analog Channel - Temperature
//#define  TPWR  PIN_C0         //Power Pin

// AD592 Temperature Sensor Pin Definitions
#define  TOCH  1              //Analog Channel - Temperature Outside

// HIH-3610 Humidity Sensor Pin Definitions
#define  HOCH  0              //Analog Channel - Humidity Outside

//Davis Solar Radiation Sensor
#define  SCH   8              //Analog Channel - Solar Radiation

// Decagon ECH2O EC-5 Soil Moisture Sensor
#define  MCH   7              //Analog Channel - Soil Moisture

// MCP1541 Voltage Reference Pin Definitions
#define  VPWR  PIN_C1         //Power Pin

// ADXL322 Accelerometer Pin Definitions
#define  XCH    4              //Analog Channel - Xpos
#define  YCH    5              //Analog Channel - Ypos
#define  ZCH    9              //Analog Channel - Zpos
//#define  APWR   PIN_D7         //Power Pin

// Battery Monitor Pin Definitions
#define  BCH    10             //Analog Channel - Battery Voltage
#define  BPWR   PIN_D7

// MPX4115A Barometric Pressure Sensor Pin Definitions
#define  PCH   6              //ANalog Channel - Pressure

// Voltage Regulator Enables
#define  EN5V  PIN_E3         //Enable 5v Regulator
#define  EN3V  PIN_E4         //Enable 3.3v Regulator
#define  EN8V  PIN_E2         //Enable 8v Regulator

// RS232 LINE DRIVER CONTROL
#define  SHDN1    PIN_E1      //Drive LOW for on board XBEE
#define  EN1      PIN_E0      //Drive HIGH for on board XBEE
#define  SHDN2    PIN_B5      //Drive HIGH to enable VHF-B Port
#define  EN2      PIN_B4      //Drive LOW to enable VHF-B Port
#define  EN3      PIN_E5      //Drive HIGH to enable GPS and DC-ME

//MAX 3110E UART Control Lines
#define  MAX_CSA     PIN_D5   //Chip Select - A
#define  MAX_SHDNA   PIN_E6   //Shutdown - A

#define  MAX_CSB     PIN_D4   //Chip Select - B
#define  MAX_SHDNB   PIN_E7   //Shutdown - B

//Midland SD-125 VHF Radio Interface Pins - J2 VHF-A Connector
#define AUDIO PIN_D6             //Audio Tone output to Maxon Radio
#define PTT   PIN_B7             //PTT-Push to Talk output to Maxon
#define XMIT  PIN_D7             //12v Power to Maxon turned on before transmit

//Midland SD-171 VHF Radio Interface Pins - J8 VHF-B Connector
#define PTTB  PIN_C2             //PTT-Push to Talk output to Maxon
#define BUSY  PIN_B3             //BUSY input from Maxon

// Gray Code Conversion Table for TWI Wind Direction
const char GrayTable[16] = {0,1,3,2,7,6,4,5,15,14,12,13,8,9,11,10};

unsigned int16 wcount;     //wind speed counter
unsigned int16 onesec;     //1-second counter
unsigned int16 vonesec;     //1-second counter for VHF radio timerout
unsigned int16 onemin;     //1-minute counter

// Holds all the sensor data
struct {
   unsigned int16 speed;               //Wind Speed
   unsigned int16 direction;           //Wind Direction
   unsigned int16 temperature;         //External Temperature
   unsigned int16 btemperature;        //On-board Temperature
   unsigned int16 humidity;            //Humidity
   unsigned int16 pressure;            //Barometric Pressure
   unsigned int16 rain;                //Rain Fall
   unsigned int16 solar;                //Solar Radiation
   unsigned int16 moisture;             //Soil Moisture
   unsigned int16 xpos;                //Accelerometer X position
   unsigned int16 ypos;                //Accelerometer Y position
   unsigned int16 zpos;                //Accelerometer Z position
   unsigned int16 battery;             //Battery Voltage
} SensorData;




// Count number of Wind Speed pulses per second
// FOSC/4:      7.3728MHz/4 = 1.8432MHz
// Prescaler:   1.8432MHz/16 = 115200 Hz
// Postscaler:   115200Hz/16 = 7200 Hz
// 7200Hz/240 = 30Hz
// Timer 2 Period Register = 240

#int_timer2
windspeed_handler() {
   static unsigned int8 ic=0,s=0;
   ic++;
   if (ic>29) {
      ic=0;
      wcount=get_timer0();
      set_timer0(0);
      onesec++;
      vonesec++;
      s++;
      if (s>59) {
         s=0;
         onemin++;
      }
   }
}


// Perform Misc Board Initialization Chores
void InitializeBoard (void) {

   set_tris_d (0x0F);                   // 0-3 (inputs), 4-7 (outputs)
   SetADCVoltageReference(5);          // Internal Reference
   output_high(VPWR);                  //Turn ON External Voltage Reference

   setup_psp(PSP_DISABLED);
   setup_spi(FALSE);
   setup_timer_0(RTCC_EXT_H_TO_L | RTCC_DIV_1);          //Wind Speed Pulse counter
   setup_timer_1(T1_DISABLED);                           // Rain Gauge Counter (enabled Later)
   setup_timer_2(T2_DIV_BY_16,240,16);                   //30Hz timer for wind speed RTC)
   setup_timer_3(T3_DISABLED|T3_DIV_BY_1);               // This timer is Available
   setup_comparator(NC_NC_NC_NC);
   setup_vref(FALSE);

   output_high(EN3V);         //Turn on 3.3v
   output_high(EN5V);         //Turn on 5v
   output_high(EN8V);         //Turn on 8v

}

// Reset XBEE Module (does not have RESET control in USIV2 board)
// Enable on-board XBEE Module
void ResetXBEE (void) {
   input(SLEEP);                         //XBEE Sleep Status Output
   input(CTS);                         //XBEE CTS line Output
   delay_ms(100);
   output_low(SHDN1);         //Enable on board XBEE
   output_high(EN1);         //Enable on board XBEE level shifter
}


// Read a specified Analog Channel and eliminate glitches.
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
      else
      {
         e++;
      }

   }
   j=j/(50-e);

   if (a > 5)
      return(j);
   else
      return(a);
}


// Initialize ADC & Select Internal (5v) or External (4.096v) Voltage Reference
void SetADCVoltageReference (int v) {
   if (v==5) {
      setup_adc_ports(ALL_ANALOG|VSS_VDD);           //Internal 5 volt VREF+
      setup_adc(ADC_CLOCK_INTERNAL);
   }
   if (v==4) {
      setup_adc_ports(ALL_ANALOG|VSS_VREF);           //External 4.096 volt VREF+
      setup_adc(ADC_CLOCK_INTERNAL);
   }
   delay_ms(100);
}


// Start Rain Gauge Timer and Reset Rain COunter
void InitializeRainGauge (void) {

   setup_timer_1(T1_EXTERNAL | T1_DIV_BY_1);             //Enable Rain Gauge
   set_timer1(0);                                        //Reset Rain Gauge

}



// Read all sensor data into struct "sensordata"
void GetSensors (void) {
   unsigned int8 dir;
   signed int16 deg;
   float soil;
   float vo;
   unsigned int16 count;

   SetADCVoltageReference(4);
   SensorData.xpos = ReadAnalogChannelAvg (XCH);              //Accelerometer Xpos
   SensorData.ypos = ReadAnalogChannelAvg (YCH);              //Accelerometer Ypos
   SensorData.btemperature = ReadAnalogChannelAvg(TCH);           //on-board Temperature
   SensorData.temperature = ReadAnalogChannelAvg(TOCH);          //External Temperature
   SensorData.humidity = ReadAnalogChannelAvg(HOCH);          //Humidity
   SensorData.solar = ReadAnalogChannelAvg(SCH);           //Solar Radiation

   count = ReadAnalogChannelAvg(MCH);                    //Soil Moisture
   //vo = (float) count * 5000 / 1024;                   // voltage out (mV) - based on 5v Reference
   vo = (float) count * 4096 / 1024;                     // voltage out (mV) - based on 4.096v Reference
   soil = 1.16E-3*vo - 0.612 - 3.14E-7*(vo*vo);          // Soil Moisture (VWC) in m3/m3
   if (soil < 0)
      soil=0;
   SensorData.moisture = soil * 10000;                        // scale by 10000 to make integer

   SetADCVoltageReference(5);
   SensorData.pressure = ReadAnalogChannelAvg(PCH);           //Barometric Pressure
   //output_high(BPWR);                                       //Turn on Voltage Divider (Power Monitor)
   //SensorData.battery = ReadAnalogChannelAvg(BCH);           //Battery Voltage
   //output_low(BPWR);                                        //Turn off Voltage Divider (Power Monitor)
   SensorData.rain = get_timer1();                          //Rain fall

   dir = input_d();              //Wind Direction (MUST #use fast_io for this to work)
   dir = dir & 0x0F;
   dir = GrayTable[dir];         // Convert to Binary
   deg = dir * 22.5;             // Convert to Degrees
   if (deg != 0)                 // Reverse direction
      deg = 360 - deg;
   SensorData.direction = deg;   //Wind Direction
   SensorData.speed = wcount;    //Wind Speed

}

// Initialize MAX3110E Port B and Select internal or xternal port
//  char='x' (FCS port-9600 baud) 'i' (GPS Port-4800 baud)
void MAXB_init (char port) {
   int8 temp;
   long WriteConfig;

   output_high(EN3);             //Enable Level Shifter for GPS/DC-ME
   output_high(MAX_CSB);            //Initial State of Port B CS line
   if (port=='x')
      output_high(MAX_SHDNB);         //Enable RS232 Drivers (Connector J6) for FCS
   else
      output_low(MAX_SHDNB);         //Disable RS232 Drivers (for GPS)
   delay_us(100);

   setup_spi(SPI_MASTER | SPI_XMIT_L_TO_H | SPI_CLK_DIV_16);
   delay_us(100);

   output_low(MAX_CSB);              //Enable Chip
   //delay_us(1);

   //WriteConfig = 0xC40F;         // 600 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40D;         // 2400 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40C;         // 4800 baud, no parity, 1 stop, INT on Receive, Write Configuration

   if (port=='x')
      WriteConfig = 0xC40B;         // 9600 baud, no parity, 1 stop, INT on Receive, Write Configuration
   else
      WriteConfig = 0xC40C;         // 4800 baud, no parity, 1 stop, INT on Receive, Write Configuration

   //WriteConfig = 0xC40B;         // 9600 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40A;         // 19200 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC409;         // 38400 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC401;         // 115200 baud, no parity, 1 stop, INT on Receive, Write Configuration

   temp=spi_read(make8(WriteConfig,1));      //can also use spi_write()
   temp=spi_read(make8(WriteConfig,0));      // but spi_read() also does a write
   //delay_us(10);
   output_high(MAX_CSB);
   delay_ms(100);                            //This delay will eliminate power-up problems
}


// Output a character to the MAX3110E UART (B)
void MAXB_putc(char tx) {
   unsigned int8 tempH, tempL;
   long WriteData;
   do {
      output_low(MAX_CSB);
      //delay_us(1);
      tempH=spi_read(0x40);          //Read Configuration
      tempL=spi_read(0x00);
      tempH = tempH & 0x40;         //Check Transmitter Ready bit
      output_high(MAX_CSB);
   } while (tempH == 0);

   output_low(MAX_CSB);
   //delay_us(1);
   WriteData=0x8000 | tx;          //Transmit
   tempH=spi_read(make8(WriteData,1));      //Write Data
   tempL=spi_read(make8(WriteData,0));
   //delay_us(10);
   output_high(MAX_CSB);
}


// Wait for a character to arrive from the MAX3110E UART (B) and return it
char MAXB_getc (void) {
   unsigned int8 tempH, tempL;

   do {
      output_low(MAX_CSB);
      //delay_us(1);
      tempH=spi_read(0x40);          //Read Configuration
      tempL=spi_read(0x00);
      tempH = tempH & 0x80;         //Check Receiver Ready bit
      output_high(MAX_CSB);
   } while (tempH == 0);

   output_low(MAX_CSB);
   //delay_us(1);
   tempH=spi_read(0x00);               //Read Data
   tempL=spi_read(0x00);
   output_high(MAX_CSB);
   return (tempL);

}


// Get a character from the MAX3110E UART (B) Rx buffer and return it (does NOT wait)
char MAXB_igetc (void) {
   unsigned int8 tempH, tempL;
   output_low(MAX_CSB);
   //delay_us(1);
   tempH=spi_read(0x00);
   tempL=spi_read(0x00);
   output_high(MAX_CSB);
   return (tempL);
}


// Send a string to MAX3110 UART (B) - a VERY limited printf()
void MAXB_printf(char *buffer)
{
   int8 i;
   for (i=0; buffer[i]!=0; i++)
   {
     MAXB_putc(buffer[i]);
   }
}


// Turn on GPS Module
void GPS_ON (void) {
   output_high(GPSEN);
   delay_ms(100);
}

// Turn on GPS Module
void GPS_OFF (void) {
   output_low(GPSEN);
   delay_ms(100);
}

// Initialize MAX3110E Port A and select External (J4) or internal (DC-ME) Port
//  char='x' (External port) 'i' (DC-ME Internal Port)
void MAXA_init (char port) {
   int8 temp;
   long WriteConfig;

   output_high(MAX_CSA);            //Initial State of Port A CS line
   if (port=='x')
      output_high(MAX_SHDNA);         //Enable RS232 Drivers (Connector J4)
   else
      output_low(MAX_SHDNA);         //Disable RS232 Drivers (for DC-ME)
   delay_us(100);

   setup_spi(SPI_MASTER | SPI_XMIT_L_TO_H | SPI_CLK_DIV_16);
   delay_us(100);

   output_low(MAX_CSA);              //Enable Chip
   //delay_us(1);

   //WriteConfig = 0xC40F;         // 600 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40D;         // 2400 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40C;         // 4800 baud, no parity, 1 stop, INT on Receive, Write Configuration
   WriteConfig = 0xC40B;         // 9600 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC40A;         // 19200 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC409;         // 38400 baud, no parity, 1 stop, INT on Receive, Write Configuration
   //WriteConfig = 0xC401;         // 115200 baud, no parity, 1 stop, INT on Receive, Write Configuration

   temp=spi_read(make8(WriteConfig,1));      //can also use spi_write()
   temp=spi_read(make8(WriteConfig,0));      // but spi_read() also does a write
   //delay_us(10);
   output_high(MAX_CSA);
   delay_ms(100);                            //This delay will eliminate power-up problems
}

// Output a character to the MAX3110E UART (A)
void MAXA_putc(char tx) {
   unsigned int8 tempH, tempL;
   long WriteData;
   do {
      output_low(MAX_CSA);
      //delay_us(1);
      tempH=spi_read(0x40);          //Read Configuration
      tempL=spi_read(0x00);
      tempH = tempH & 0x40;         //Check Transmitter Ready bit
      output_high(MAX_CSA);
   } while (tempH == 0);

   output_low(MAX_CSA);
   //delay_us(1);
   WriteData=0x8000 | tx;          //Transmit
   tempH=spi_read(make8(WriteData,1));      //Write Data
   tempL=spi_read(make8(WriteData,0));
   //delay_us(10);
   output_high(MAX_CSA);
}


// Wait for a character to arrive from the MAX3110E UART (A) and return it
char MAXA_getc (void) {
   unsigned int8 tempH, tempL;

   do {
      output_low(MAX_CSA);
      //delay_us(1);
      tempH=spi_read(0x40);          //Read Configuration
      tempL=spi_read(0x00);
      tempH = tempH & 0x80;         //Check Receiver Ready bit
      output_high(MAX_CSA);
   } while (tempH == 0);

   output_low(MAX_CSA);
   //delay_us(1);
   tempH=spi_read(0x00);               //Read Data
   tempL=spi_read(0x00);
   output_high(MAX_CSA);
   return (tempL);

}


// Get a character from the MAX3110E UART (A) Rx buffer and return it (does NOT wait)
char MAXA_igetc (void) {
   unsigned int8 tempH, tempL;
   output_low(MAX_CSA);
   //delay_us(1);
   tempH=spi_read(0x00);
   tempL=spi_read(0x00);
   output_high(MAX_CSA);
   return (tempL);
}

// Send a string to MAX3110 UART (A) - a VERY limited printf()
void MAXA_printf(char *buffer)
{
   int8 i;
   for (i=0; buffer[i]!=0; i++)
   {
     MAXA_putc(buffer[i]);
   }
}


// Initialize VHF-B RS232 Port (PIC Serial Port #1) on J8
void VHFB_init (void) {
   output_high(SHDN2);         //Enable VHF Port ST3222EBP chip
   output_low(EN2);           //Enable VHF Port ST3222EBP chip
}
