// Changes - 08-31-2007 - ARI
//    * Possible infinite loop in WriteCharToServer() eliminate by timing out after 15 seconds


void WriteCharToServer( char writeData )
{
   if( g_radioBufferCheck == 1 )
   {
      //Wait if the Serial line is busy
      vonesec=0;
      while (input(BUSY)==1) {
         if (vonesec==15) {
            g_radioBufferCheck = 0;
            break;     
         }      
      }
   }

   putc( writeData, COM_VHF );
}

//Takes a raw byte, checks if the byte needs escapin,
//and write to the zigbee port
void WriteSerialEscapeByte( char writeByte )
{
   if( writeByte == 0x7E ||
       writeByte == 0x7D ||
       writeByte == 0x11 ||
       writeByte == 0x13    )
   {
      //Write Escape marker
      WriteCharToServer( 0x7D );

      //Write out escaped data
      WriteCharToServer( writeByte ^ 0x20 );
   }

   else
   {
      //No escape required, just write raw data
      WriteCharToServer( writeByte );
   }
}


//Write a FR Report to the server
void WriteFRReportToServer(char *dataBuff, int buffLen )
{
   int8  x;
   int32 checkSum;
   checkSum = 0;

   //***************************************************************************
   // Activate Push To Talk on Radio Modem, do this per packet, not per char
   // per char( Rapid Activate\deactivate PTT ) stresses the radio circuts
   //***************************************************************************
   output_low(PTTB);

   //***************************************************************************
   // Write out Start Delim Header
   //***************************************************************************

      //Write out start delim
      WriteCharToServer( 0x7E );

   //***************************************************************************
   //Write out data
   //***************************************************************************

      //loop through each data byte in the buffer
      for( x = 0; x < buffLen; x++)
      {
         //Write out each byte, escape if needed
         checkSum += *dataBuff;
         WriteSerialEscapeByte( *dataBuff );

         //advance to next byte
         dataBuff++;
      }



   //***************************************************************************
   //Calc and write out checksum
   //***************************************************************************

      //keep only lowest byte of checkSum
      checkSum = make8(checkSum, 0 );

      //subtract from constant
      checkSum = 0xFF - checkSum;

      //Write it out
      WriteSerialEscapeByte( make8(checkSum, 0) );

   //***************************************************************************
   // Deactivate Push To Talk on Radio Modem, activated at start of function
   //***************************************************************************
   output_high(PTTB);
}


//Takes an UnEscaped zigbeeRX, strips out the data message,
//escapes it, checksums it, puts it on the Serial port
//to the server
void TxZigbeeToSerial( char* zigbeeRXBuffer, int bufferLen )
{
                         //Start of rf Data  //Don't count api header\checksum
   WriteFRReportToServer(zigbeeRXBuffer += 8,bufferLen - 9 );

}
