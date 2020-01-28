//GPS Processing
// Changes - 8/31/2007 - ARI
//    * GPS Initialize sequence changed to compensate for oddity in GPS module
//              All output is first turned off. GGA is turned on last
//              Inter-command delay increased to 500msec.
//    * LED is toggled in GPS ISR
//    * GPSSend() now does not enable/disable interrupts
//

#define LF              10
#define CR              13
#define RX_BUFFER_SIZE  150




//******************************************************************************
//Function Decs
//******************************************************************************

void InitGpsRxBuffer();

//******************************************************************************
//End Function Decs
//******************************************************************************


static char cRxByteCnt;                // Number of bytes in the recv fifo
static char *cRxBufferWritePtr;        // Pointers for the Rx buffer
static char *cRxBufferReadPtr;
static char cRxIsrState, cRxMsgTypeReceived;
static char cRxGPSMsgReady, cReceiveFlag;
static char cRxBuffer[RX_BUFFER_SIZE];    // Fifo
static char cRxErrorFlag, cError;



char NEMAChecksumCalc( char *buffer, int iBufferlen )
{
   char i,j;
   j=0;
   for (i=0; i<iBufferlen; i++)
   {
     j=j^buffer[i];
   }
   return(j);
}

//******************************************************************************
//* Name: GPS_Send
//*
//* In Params: cCommand - a command string to send TO the GPS
//*            bDisableInter - Only used for CN\VPD Extern UART hardware
//*                            0 - no enable\disable.Usually used during device
//*                            init when interrupts are off and need to stay off
//*                            1 - Disable\Enable gps interrupts to prevent
//*                            timing problems during send
//* Use: Used to Deal with different hardware UARTS that the GPS chip uses.
//*      On the CN\VTD an external UART is used that requires a custom printf.
//*      On the PTD, the normal csc fprintf to a serial port is used
//******************************************************************************
void GPS_Send( char *cCommand, char bDisableInter )
{
   #ifdef CN_VTD_HARDWARE
   {
      if( bDisableInter == 1 )
      {
         //disable_interrupts(int_ext1);
      }

      //This is the Hardware that uses an Extern UART
      MAXB_printf(cCommand);

      if( bDisableInter == 1 )
      {
         //enable_interrupts(int_ext1);
      }
   }
   #else
   {
      //This must be a internal UART
      fprintf(COM_GPS, cCommand );
   }
   #endif

}

//******************************************************************************
//* Name: GPS_getc
//*
//* In Params: None
//*
//* Out Params - Returns char from GPS UART
//* Use: Used to Deal with different hardware UARTS that the GPS chip uses.
//*      On the CN\VTD an external UART is used that requires a custom getc
//*      On the PTD, the normal csc getc to a serial port is used
//******************************************************************************
char GPS_getc()
{
   #ifdef CN_VTD_HARDWARE
   {
      //This is the Hardware that uses an Extern UART
      return MAXB_igetc();
   }
   #else
   {
      //This must be a internal UART
      return fgetc(COM_GPS);
   }
   #endif

}

void InitGPS(int16 rptRate )
{
   char tempCommand[25];
   char temp[5];
   char cGGAOn[20];

   //char cGGAOn[20]="PSRF103,00,00,05,01";
   //char cRMCOn[20]="PSRF103,04,00,02,01";
   //char cGGLOn[20]="PSRF103,01,00,02,01";
   //char cGSAOn[20]="PSRF103,02,00,02,01";
   //char cGSVOn[20]="PSRF103,03,00,02,01";
   //char cVTGOn[20]="PSRF103,05,00,02,01";

   char cGGAOff[20]="PSRF103,00,00,00,01";
   char cRMCOff[20]="PSRF103,04,00,00,01";
   char cGGLOff[20]="PSRF103,01,00,00,01";
   char cGSAOff[20]="PSRF103,02,00,00,01";
   char cGSVOff[20]="PSRF103,03,00,00,01";
   char cVTGOff[20]="PSRF103,05,00,00,01";

   delay_ms(500);


   //GGA Off
   sprintf(tempCommand,"$%S*%X%C%C",cGGAOff,NEMAChecksumCalc( cGGAOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   //RMC Off
   sprintf(tempCommand,"$%S*%X%C%C",cRMCOff,NEMAChecksumCalc( cRMCOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   //GGL off
   sprintf(tempCommand,"$%S*%X%C%C",cGGLOff,NEMAChecksumCalc( cGGLOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   //GSA Off
   sprintf(tempCommand,"$%S*%X%C%C",cGSAOff,NEMAChecksumCalc( cGSAOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   //GSV off
   sprintf(tempCommand,"$%S*%X%C%C",cGSVOff,NEMAChecksumCalc( cGSVOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   //VTG Off
   sprintf(tempCommand,"$%S*%X%C%C",cVTGOff,NEMAChecksumCalc( cVTGOff, 19 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);

   sprintf(cGGAOn, "PSRF103,00,00,%03lu,01", rptRate );
   //GGA On
   sprintf(tempCommand,"$%S*%X%C%C",cGGAOn,NEMAChecksumCalc( cGGAOn, 20 ),CR,LF);
   GPS_Send( tempCommand, 1 );
   delay_ms(500);



   /*//old way
   //GGA On
   putc('$',GPS_PORT);
   fprintf(COM_GPS,cGGAOn);
   putc('*',COM_GPS);
   fprintf(COM_GPS,"%X",NEMAChecksumCalc( cGGAOn, 19 ));
   putc(CR,COM_GPS);
   putc(LF,COM_GPS);
   delay_ms(1000);
   */
}

// GPS Receive Data Interrupt
GPS_INTERRUPT
void gpsRx_isr2 (void) {
    //Reads incoming data from the USART and puts in in a rolling buffer
    //( but in this application, it should never roll.)
    //If the buffer is full, this routine just discards the received byte.
    //Not checking the LRC byte at the end of the NMEA-0183 sentence.
    char cChar;
    //fprintf( COM_VHF, "%S\n", "G" );
    //if (rs232_errors & 0x04) {  // get framing error bit from Rx status reg
    //    cRxErrorFlag = ON;
    //}

    cChar = GPS_getc();       // get char from GPS UART, clear any errors
    //fputc(cChar,COM_VHF);     // echo to VHF - for debugging
   #ifdef CN_VTD_HARDWARE
      output_toggle(LED);
   #endif
    // is recv fifo full ???
    if (cRxByteCnt == RX_BUFFER_SIZE)
    {
        //Reset GPS buffer
         cRxErrorFlag=0;
         cError=0;
         InitGpsRxBuffer();
         goto done;   //  ... so must use the "goto"
    }
    switch (cRxIsrState) {
        case 0:
        {
            if (cChar == '$') {  // if start of NMEA0183 message
                cRxByteCnt = 0;     // reset byte count
                cReceiveFlag = 0;     // default to off
                cRxMsgTypeReceived = 0;  // set hashed value to null
                cRxIsrState++;                 // next state
                cRxBufferWritePtr = cRxBuffer; //set ptr to start
                *cRxBufferWritePtr = cChar;  //Write the start byte
                cRxBufferWritePtr++;
                cRxByteCnt++;


            }
            break;
        }
        case 1:                           // five type characters to obtain
        case 2:
        case 3:
        case 4:
        case 5:
        {
            cRxMsgTypeReceived ^= cChar;      // hash in msg type
            *cRxBufferWritePtr = cChar;  //Write the start byte
            cRxBufferWritePtr++;
            cRxByteCnt++;
            if (cRxIsrState++ == 5)
            {
                // if time to check message type
                if (1)//cRxMsgTypeReceived == cRxMsgTypeDesired)
                {  // if good
                    cReceiveFlag = 1;            // enable receiving
                }
                else
                {                    // don't want this message
                    cRxIsrState = 0;    // reset to look for next msg
                }
            }
            break;
        }
        //case 6:
        //{
            // Case 6 skips the comma character following msg type
        //    cRxIsrState++;
        //    break;
        //}
        default:                          // remainder of characters
        {
            if (cReceiveFlag == 1) {        // if this message is wanted
                *cRxBufferWritePtr = cChar;     // put char in fifo
                cRxBufferWritePtr++;            // increment pointer
                if (cRxBufferWritePtr == (cRxBuffer + RX_BUFFER_SIZE)) {// pointer past end ?
                    cRxBufferWritePtr = cRxBuffer;      // set pointer to start of fifo
                }
                cRxByteCnt++;              // Increment byte count
                if (cChar == CR) {
                    *cRxBufferWritePtr = 10;     // put char in fifo
                    cRxBufferWritePtr++;
                    cRxByteCnt++;
                    cRxGPSMsgReady = 1;         // signal that message is ready
                     #ifdef CN_VTD_HARDWARE
                        output_low(LED);           //Turn off LED
                     #endif
                    cReceiveFlag = 0;      // no more receive
                }
             }
        }
    }
    done:
      ;               // label
}

void InitGpsRxBuffer()
{
    cRxBufferWritePtr = cRxBuffer;      // point to beginning of buffer
    cRxBufferReadPtr = cRxBuffer;
    cRxByteCnt = 0;
    cRxIsrState = 0;
    cRxGPSMsgReady = 0;
}


char NEMAChecksum (char cCode) {
   char i,j;
   j=cCode^',';
   for (i=0; i<cRxByteCnt; i++) {
      if (cRxBuffer[i]=='*')
         break;
      j=j^cRxBuffer[i];
   }
   return(j);

}



unsigned int16 SirfChecksumCalc( char *buffer, int iBufferLen )
{
   char i;
   int16 checkSum=0;
   for(i=0; i < iBufferLen; i++ )
   {
      checkSum = checkSum + buffer[i];
   }

   checkSum = checkSum & 0x7FFF;




   return checkSum;
}

//Reads Lat,lon and elevation from GGA string into ptdLocReport struct
//true, pos avail, false pos not ready
int8 GetLocRepGGA(struct ptdLocReport *locRep, char strGGA[])
{
    char cX, cIndex;
    char *GGAIndex;
    char field[20];
    char *fieldIndex;
    int  fieldLen;
    int32 tempLoc;
    char cDegree[4];
    char cMinute[7];
    int32 degree, minute;
    int x;
    signed int32 height;

    //fprintf(COM_PC,strGGA);

    cX = 0;
    GGAIndex = strGGA;

    for(x = 0; x < 10; x++)
    {
      //Reset field buffer
      memset(field, 'a', 20);
      fieldIndex = field;
      fieldLen = 0;

      while (1)
      {
         //Get the next char from the GGA input string

         cX = *GGAIndex++;
         //fputc(cX, COM_PC );


         // ',' marks end of field, CR marks end of string
         if ((cX == ',') || (cX == CR))
            break;

         if( cX != '.')
         {
            //Write GGA char to field buffer
            *fieldIndex = cX;
            fieldIndex++;
            fieldLen++;
            //sanitity check
            if(fieldLen > 19 )
            {
               return 0;
            }
         }

      }

      //add null term to field buffer
      *fieldIndex = '\0';
      fieldIndex++;

      switch(x)
      {
         case 0:
            //Nmea Message type, GGA in this case
            break;

         case 1:
            //GPS time
            break;

         case 2:
            if( fieldLen == 0 )
               return 0;
            //Latitude
            //Get degree part
            memcpy(cDegree, field, 2 );
            memset(cDegree+2,'\0',1);
            degree = atoi(cDegree);
            degree = degree * 1000000;

            //Get minute part
            //No need to round, well below
            //GPS accuracy threshold
            memcpy(cMinute, field + 2, 6 );
            memset(cMinute+6,'\0',1);
            minute = atoi32(cMinute);
            minute = minute / .6;

            tempLoc = degree + minute;

            break;

         case 3:
            //Latitude hemi
            if( field[0] == 'S')
            {
               (*locRep).lat = tempLoc * -1;
            }
            else
            {
               (*locRep).lat = tempLoc;
            }

            break;

         case 4:
            //longitude
            if( fieldLen == 0 )
               return 0;
            //Get degree part
            memcpy(cDegree, field, 3 );
            memset(cDegree+3,'\0',1);
            degree = atoi(cDegree);
            degree = degree * 1000000;

            //Get minute part
            //No need to round, well below
            //GPS accuracy threshold
            memcpy(cMinute, field + 3, 6 );
            memset(cMinute+6,'\0',1);
            minute = atoi32(cMinute);
            minute = minute / .6;

            tempLoc = degree + minute;

            break;

         case 5:
            //longitude hemi
            if( field[0] = 'W')
            {
               (*locRep).lon = tempLoc * -1;
            }
            else
            {
               (*locRep).lon = tempLoc;
            }
            break;

        case 9:
            //Height, meters
            //No need to round, well below
            //GPS accuracy threshold, is that cheap
            height = atoi32(field);
            if(height < 0)
            {
               (*locRep).height = 0;
            }
            else if( height > 65535 )
            {
               (*locRep).height = 65535;
            }
            else
            {
               (*locRep).height = height;
            }
            break;


        default:
            break;
      }

    }


    //fprintf(COM_PC, "decimal x: %8ld y: %8ld z:%ld\n\r", (*locRep).lat, (*locRep).lon, (*locRep).height );

    return 1;
}


void QueryGpsForLocation()
{
   //This Code is not used and has big changes but was not tested
   char tempCommand[25];
   char cGGAQuery[20] =  "PSRF103,00,01,00,01";

   //GGA On
   sprintf(tempCommand,"$%S*%X%C%C",cGGAQuery,NEMAChecksumCalc( cGGAQuery, 19 ),CR,LF);
   GPS_Send( tempCommand, 0 );


}



void SwitchTOSirfMode()
{
   //Not funtioning
   //Switch ti Sirf Bin mode
   //char cSirfMode[21]="PSRF100,0,9600,8,1,0";
   //char cSirfNmea[21]="PSRF100,1,9600,8,1,0";

   //Sirf Nmea Mode
   //char cSirfNmeaMode[32];

   //cSirfNmeaMode[0]=0xA0;
   //cSirfNmeaMode[1]=0xA2;
   //cSirfNmeaMode[2]=0x00;
   //cSirfNmeaMode[3]=0x18;

   /*
   //Text example
   cSirfNmeaMode[4]=0x81;
   cSirfNmeaMode[5]=0x02;
   cSirfNmeaMode[6]=0x01;
   cSirfNmeaMode[7]=0x01;
   cSirfNmeaMode[8]=0x00;
   cSirfNmeaMode[9]=0x01;
   cSirfNmeaMode[10]=0x01;
   cSirfNmeaMode[11]=0x01;
   cSirfNmeaMode[12]=0x05;
   cSirfNmeaMode[13]=0x01;
   cSirfNmeaMode[14]=0x01;
   cSirfNmeaMode[15]=0x01;
   cSirfNmeaMode[16]=0x00;
   cSirfNmeaMode[17]=0x01;
   cSirfNmeaMode[18]=0x00;
   cSirfNmeaMode[19]=0x01;
   cSirfNmeaMode[20]=0x00;
   cSirfNmeaMode[21]=0x01;
   cSirfNmeaMode[22]=0x00;
   cSirfNmeaMode[23]=0x01;
   cSirfNmeaMode[24]=0x00;
   cSirfNmeaMode[25]=0x01;
   cSirfNmeaMode[26]=0x25;
   cSirfNmeaMode[27]=0x80;
   */


   //Chart Examnple
   /*
   cSirfNmeaMode[4]=0x81;
   cSirfNmeaMode[5]=0x02;
   cSirfNmeaMode[6]=0x01;
   cSirfNmeaMode[7]=0x01;
   cSirfNmeaMode[8]=0x00;
   cSirfNmeaMode[9]=0x01;
   cSirfNmeaMode[10]=0x01;
   cSirfNmeaMode[11]=0x01;
   cSirfNmeaMode[12]=0x05;
   cSirfNmeaMode[13]=0x01;
   cSirfNmeaMode[14]=0x01;
   cSirfNmeaMode[15]=0x01;
   cSirfNmeaMode[16]=0x00;
   cSirfNmeaMode[17]=0x01;
   cSirfNmeaMode[18]=0x00;
   cSirfNmeaMode[19]=0x01;
   cSirfNmeaMode[20]=0x00;
   cSirfNmeaMode[21]=0x00;
   cSirfNmeaMode[22]=0x00;
   cSirfNmeaMode[23]=0x01;
   cSirfNmeaMode[24]=0x00;
   cSirfNmeaMode[25]=0x00;
   cSirfNmeaMode[26]=0x25;
   cSirfNmeaMode[27]=0x80;


   cSirfNmeaMode[28]=0x01;
   cSirfNmeaMode[29]=0x3A;
   cSirfNmeaMode[30]=0xB0;
   cSirfNmeaMode[31]=0xB3;
   */

     //Switch to SirF
   /*
   putc('$',COM_GPS);
   fprintf(COM_GPS,cSirfMode);
   putc('*',COM_GPS);
   fprintf(COM_GPS,"%X",NEMAChecksumCalc( cSirfMode, 20 ));
   putc(CR,COM_GPS);
   putc(LF,COM_GPS);

   delay_ms(2000);
   */

   //Use sirf binary to switch back
   //to nmea

   /*
   cs = SirfChecksumCalc( &cSirfNmeaMode[4],24 );
   fprintf(COM_XBEE,"SirfChecksum= ");

   putc(cs>>8, COM_XBEE );
   putc(cs, COM_XBEE );

   cSirfNmeaMode[28] = cs>>8;
   cSirfNmeaMode[29] = cs;
   */



   /*
   delay_ms(5000);

   //Nmea Stuff
   for( i = 0; i < 32; i++)
   {
      putc(cSirfNmeaMode[i],COM_GPS);
      delay_ms(5);
      putc(cSirfNmeaMode[i],COM_XBEE);
   }

   delay_ms(5000);
   */


   //Switch to SirF
   /*
   putc('$',COM_GPS);
   fprintf(COM_GPS,cSirfNmea);
   putc('*',COM_GPS);
   fprintf(COM_GPS,"%X",NEMAChecksumCalc( cSirfNmea, 20 ));
   putc(CR,COM_GPS);
   putc(LF,COM_GPS);*/
}
