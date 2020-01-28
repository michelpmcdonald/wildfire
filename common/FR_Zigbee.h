// Changes - 9/2/2007 - ARI
// * Added "ATSM=1" command for Pin Hibernate mode (Sleep) in
//   InitZigbee() function (only applies to small board)

#include <stdlib.h>

//Holds the TagID of THIS unit
static int16 g_tagID;

//Zigbee RX buffer
#define ZB_RX_BUFFER_SIZE    128

#define ZB_RX_STATE_HEADER   20 //Searching for Header indicator(7e)
#define ZB_RX_STATE_DATA     21 //Found Header, buffering data
#define ZB_RX_STATE_UNESCAPE 22 //Found 0x7D(Escape Marker), next byte unEscaped

static int8  zbRxState;

static char  *zbRxWritePtr;
static int8  zbRxBuffer[ZB_RX_BUFFER_SIZE];
static int8  zbRxByteCount;

static int8  zbRxMessageFound;
static int8  zbRxMessage[ZB_RX_BUFFER_SIZE];
static int8  zbRxMessageLen;
static int16 zbRxIndicatedLength;
static int8  zbRxEscapeBytesFound;

//Functions
void ZigbeeAtQueryCommand( char *cCommand );


void InitZigbee(int16 xbeeTag)
{

   //Get into Command Mode
   delay_ms(1000);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(1000);                  //Guard time

   //my address
   fprintf(COM_XBEE,"ATMY=%LX\r",xbeeTag);
   delay_ms(500);

   //API Mode
   fprintf(COM_XBEE,"ATAP=2\r");
   delay_ms(500);

   //Broadcast
   fprintf(COM_XBEE,"ATDH=0000\r");
   delay_ms(500);
   fprintf(COM_XBEE,"ATDL=FFFF\r");
   delay_ms(500);

   //Pan
   fprintf(COM_XBEE,"ATID=3332\r");
   delay_ms(500);

   //FULL Power
   fprintf(COM_XBEE,"ATPL=4\r");
   delay_ms(500);

   #ifndef CN_VTD_HARDWARE
      //Pin Hibernate Mode
      fprintf(COM_XBEE,"ATSM=1\r");
      delay_ms(500);
   #endif

   //Save
   fprintf(COM_XBEE,"ATWR\r");
   delay_ms(2000);

   //exit command mode
   fprintf(COM_XBEE,"ATCN\r");
   delay_ms(500);

}

//******************************************************************************
//******************************************************************************
//Process Incoming Zigbee Stream
//******************************************************************************
//******************************************************************************

//Use length to determine the end
ZIGBEE_INTERRUPT
void xbeeRx_isr2(void)
{
   char rxData;

   //Get the byte
   rxData =  fgetc(COM_XBEE);

   switch(zbRxState)
   {
      //Found Header
      case ZB_RX_STATE_HEADER:
      {
         //Look for header
         if( rxData == 0x7E )
         {
            //Found header
            *zbRxWritePtr = rxData;
            zbRxWritePtr++;
            zbRxByteCount++;
            zbRxState = ZB_RX_STATE_DATA;
         }
      }
      break;

      //Byte needs unescaping
      case ZB_RX_STATE_UNESCAPE:
      {
         //write Escaped Data
         *zbRxWritePtr = (rxData ^ 0x20);
         zbRxWritePtr++;
         zbRxByteCount++;
         zbRxState = ZB_RX_STATE_DATA;
      }
      break;

      case ZB_RX_STATE_DATA:
      {
         switch( rxData )
         {
            //Escape Byte check
            case 0x7D:
            {
               //Found escape byte, next byte needs to be escaped before
               //written
               zbRxState = ZB_RX_STATE_UNESCAPE;
            }
            break;

            //Zigbee Packet byte, just write it
            default:
            {
               //write Data
               *zbRxWritePtr = rxData;
               zbRxWritePtr++;
               zbRxByteCount++;
            }
            break;
         }
      }
      break;

   }

   //Check to see if we have the complete packet
   if( zbRxByteCount == (zbRxIndicatedLength + 4) )
   {
      //current packet complete
      //Check to see if Message buffer is avail
      if( zbRxMessageFound == 1 )
      {
         //Prev packet not sent yet
         //fprintf(COM_PC,"PMNS");
      }
      else
      {
         //copy buffer to message location
         #ifdef CN_VTD_HARDWARE
            //Only the VTD\CN use the queue
            EnqueueZigbeeMessage(  zbRxBuffer, zbRxByteCount );
         #else
            //No queue, just copy to buffer
            memcpy(zbRxMessage, zbRxBuffer, zbRxByteCount );
            zbRxMessageLen = zbRxByteCount;
            zbRxMessageFound = 1;
         #Endif


      }

      //reset buffers
      zbRxByteCount = 0;
      zbRxWritePtr = zbRxBuffer;
      zbRxIndicatedLength = 9999;

      zbRxState = ZB_RX_STATE_HEADER;


   }

   //Check for and set Indicated Length
   if( zbRxByteCount == 3 )
   {
      zbRxIndicatedLength = MAKE16(zbRxBuffer[1], zbRxBuffer[2]);
   }

   //if the rxBuffer is full, reset and start lookin for start byte again
   if( zbRxByteCount == ZB_RX_BUFFER_SIZE )
   {
      zbRxState = ZB_RX_STATE_HEADER;
      zbRxByteCount = 0;
      zbRxWritePtr = zbRxBuffer;
      zbRxIndicatedLength = 9999;
   }
}

//Use length to determine the end
//Keep bytes escaped
//ZIGBEE_INTERRUPT
void xbeeRx_isr_KeepEscape(void)
{
   char rxData;

   //Get the byte
   rxData =  fgetc(COM_XBEE);

   switch(zbRxState)
   {
      //Found Header
      case ZB_RX_STATE_HEADER:
      {
         //Look for header
         if( rxData == 0x7E )
         {
            //Found header
            *zbRxWritePtr = rxData;
            zbRxWritePtr++;
            zbRxByteCount++;
            zbRxState = ZB_RX_STATE_DATA;
         }
      }
      break;

      case ZB_RX_STATE_DATA:
      {
         //Escape Byte check
         if( rxData == 0x7D)
         {
            //Found escape byte, next byte needs to be escaped before
            //written
            zbRxEscapeBytesFound++;
          }

          //write Data
          *zbRxWritePtr = rxData;
          zbRxWritePtr++;
          zbRxByteCount++;
      }
      break;

   }

   //Check to see if we have the complete packet
   if( ( zbRxByteCount - zbRxEscapeBytesFound ) == (zbRxIndicatedLength + 4) )
   {
      //current packet complete
      //Check to see if Message buffer is avail
      if( zbRxMessageFound == 1 )
      {
         //Prev packet not sent yet
         //fprintf(COM_PC,"PMNS");
      }
      else
      {
         //copy buffer to message location
         memcpy(zbRxMessage, zbRxBuffer, zbRxByteCount );
         zbRxMessageLen = zbRxByteCount;
         zbRxMessageFound = 1;

      }

      //reset buffers
      zbRxByteCount = 0;
      zbRxWritePtr = zbRxBuffer;
      zbRxIndicatedLength = 9999;
      zbRxEscapeBytesFound = 0;
      zbRxState = ZB_RX_STATE_HEADER;


   }

   //Check for and set Indicated Length
   if( zbRxByteCount == 3 )
   {
      zbRxIndicatedLength = MAKE16(zbRxBuffer[1], zbRxBuffer[2]);
   }

   //if the rxBuffer is full, reset and start lookin for start byte again
   if( zbRxByteCount == ZB_RX_BUFFER_SIZE )
   {
      zbRxState = ZB_RX_STATE_HEADER;
      zbRxByteCount = 0;
      zbRxWritePtr = zbRxBuffer;
      zbRxIndicatedLength = 9999;
      zbRxEscapeBytesFound = 0;
   }
}

//******************************************************************************
//******************************************************************************
//End Incoming Zigbee Stream
//******************************************************************************
//******************************************************************************

void InitZbRx()
{
   zbRxState = ZB_RX_STATE_HEADER;
   zbRxByteCount = 0;
   zbRxWritePtr = zbRxBuffer;

   zbRxMessageFound = 0;
   zbRxIndicatedLength = 9999;

   zbRxEscapeBytesFound = 0;

}

//Sends a query to the Zigbee for the tag ID
void QueryZigbeeTagID()
{
   char tagIDQuery[2];

   //Query for the TagID
   tagIDQuery[0] = 'M';
   tagIDQuery[1] = 'Y';

   ZigbeeAtQueryCommand( tagIDQuery );
   ZigbeeAtQueryCommand( tagIDQuery );
}


//Takes a raw byte, checks if the byte needs escapin,
//and write to the zigbee port
void WriteZigbeeEscapeByte( char writeByte )
{
   if( writeByte == 0x7E ||
       writeByte == 0x7D ||
       writeByte == 0x11 ||
       writeByte == 0x13    )
   {
      //Have to escape data byte can't have raw it in Zigbee TX
      //Write Escape marker
      putc( 0x7D, COM_XBEE );

      //Write out escaped data
      putc( writeByte ^ 0x20, COM_XBEE );
   }

   else
   {
      //No escape required, just write raw data
      //putc( writeByte, COM_PC );
      putc( writeByte, COM_XBEE );
   }
}

void WriteZigbeeEscapeUInt16(int16 uInt16Data)
{
   int8 writeData;
   char x;
   const int8 z = 1;

   WriteZigbeeEscapeByte( make8(uInt16Data, 1) );
   WriteZigbeeEscapeByte( make8(uInt16Data, 0) );

}

void ZigbeeTx( char* dataBuffer, int bufferLen, int16 iDestinAddress )
{
   int8  x;
   int16 txLen;
   int32 checkSum;

   checkSum = 0;


   //***************************************************************************
   //Calc and write out API Header
   //***************************************************************************
      //Write out start delim
      putc( 0x7E, COM_XBEE );

      //Write out the length
      txLen = 5;           //API_ID(1) + Frame_ID(1) + dest address(2) + options(1)
      txLen += bufferLen;  //locRepData
      WriteZigbeeEscapeUInt16( txLen );

      //Write out TX API_ID
      checkSum += 0x01;
      putc( 0x01, COM_XBEE );

      //Write out Frame ID
      putc( 0x00, COM_XBEE );

      //Write out Broadcast destin address(0xFF)
      checkSum += make8(iDestinAddress, 1 );
      checkSum += make8(iDestinAddress, 0 );
      WriteZigbeeEscapeUInt16(iDestinAddress);

      //Write out Options byte
      putc( 0x00, COM_XBEE );



   //***************************************************************************
   //Write out data
   //***************************************************************************
      //loop through each byte in the buffer
      for( x = 0; x < bufferLen; x++)
      {
         //Write out each byte, escape if needed
         checkSum += *dataBuffer;
         WriteZigbeeEscapeByte( *dataBuffer );

         //advance to next byte
         dataBuffer++;
      }


   //***************************************************************************
   //Calc and write out checksum
   //***************************************************************************
      //keep only lowest byte of checkSum
      checkSum = make8(checkSum, 0 );

      //subtract from constant
      checkSum = 0xFF - checkSum;

      //Write it out
      WriteZigbeeEscapeByte( make8(checkSum, 0) );
}


void ZigbeeAtQueryCommand( char *cCommand )
{
   int8  x;
   int16 txLen;
   int32 checkSum;

   checkSum = 0;


   //***************************************************************************
   //Calc and write out API Header
   //***************************************************************************
      //Write out start delim
      putc( 0x7E, COM_XBEE );

      //Write out the length
      txLen = 4;
      WriteZigbeeEscapeUInt16( txLen );

      //Write out TX API_ID
      checkSum += 0x08;
      putc( 0x08, COM_XBEE );

      //Write out Frame ID
      checkSum += 0x52;
      putc( 0x52, COM_XBEE );


   //***************************************************************************
   //Write out Query
   //***************************************************************************
      checkSum += cCommand[0];
      WriteZigbeeEscapeByte( cCommand[0] );

      checkSum += cCommand[1];
      WriteZigbeeEscapeByte( cCommand[1] );

   //***************************************************************************
   //Calc and write out checksum
   //***************************************************************************
      //keep only lowest byte of checkSum
      checkSum = make8(checkSum, 0 );

      //subtract from constant
      checkSum = 0xFF - checkSum;

      //Write it out
      WriteZigbeeEscapeByte( make8(checkSum, 0) );

}

//Process the response that comes back for a query
void ZigbeeAtQueryResp( char* respBuffer, int8 length )
{
   //Make Sure Status is OK
   if( respBuffer[7] == 1 )
   {
      return;
   }

   //Look for the 'MY' command Resp to set the tagID
   //If the Command is MY, the Tag was set and we need
   //to update the global tagID variable to reflect
   //the current tag ID
   if( (respBuffer[5] == 'M') && (respBuffer[6] == 'Y') )
   {
      g_tagID = MAKE16(respBuffer[8], respBuffer[9] );
   }

}

void ZigbeeAtSetCommand( char* cCommand, char cCommandValue )
{
}



void EscapeTest()
{
   //Test bed for basic escaping, message from zigbee docs
   //Not in good shape, many changes since last run of this function
   //note should escape the x11, so the result zigbee raw data tx part  should look like:
   //                      0x00  0x02  0x23  0x7D 0x31 0xCB
   //char cEscapeTest[] = {0x00, 0x02, 0x23, 0x11,     0xCB}; //note should escape the x11

   //note the result zigbee raw data tx part  should look like:
   //Start delimiter:     7E
   //Length bytes:        00 0A
   //API identifier       01
   //API frame ID         01
   //Destination address: XX XX
   //Option byte:         00
   //Data Packet          48 65 6C 6C 6F
	//Checksum             B8
   //If Destin = 0x5001, FrameID = 0x01
   //char cEscapeTest[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F};
   //ZigbeeTx( cEscapeTest, 5, 0x5001 ); //65535 = 0xFFFF
}

//Forces Zigbee into transparent mode
void SetZigbeeTransparentMode()
{
   //Get into Command Mode
   delay_ms(1000);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(1000);

   //Restore Defaults Zigbee Special Command
   //Default mode is transparent
   fprintf(COM_XBEE,"ATRE\r");
   delay_ms(1000);

   //Get into Command Mode
   delay_ms(1000);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(10);
   fprintf(COM_XBEE,"+");
   delay_ms(1000);

   //Broadcast
   fprintf(COM_XBEE,"ATDH=0000\r");
   delay_ms(500);
   fprintf(COM_XBEE,"ATDL=FFFF\r");
   delay_ms(500);

   //Save
   fprintf(COM_XBEE,"ATWR\r");
   delay_ms(5000);

   //exit command mode
   fprintf(COM_XBEE,"ATCN\r");
   delay_ms(500);

}


