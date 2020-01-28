#define QUEUE_ITEMS 8

int16 head = 0;
int16 tail = 1;

struct queueItem
{
   int8   len;
   char   frReport[50];
};

struct queueItem zigbeeQueue[QUEUE_ITEMS];

void CopyReportToQueueItem( char data[], struct queueItem *pQueueItem, int8 len )
{
   #ifdef  FR_DEBUG
      char debug_message[10];
   #endif



   (*pQueueItem).len = len;
   memcpy( (*pQueueItem).frReport, data, len );

   #ifdef  FR_DEBUG
      sprintf(debug_message,"CMT=%lu\n\r\0",tail);
      MAXA_printf( debug_message );
   #endif
}


void EnqueueZigbeeMessage( char messageData[], int8 len )
{
   int16 newTail;



   #ifdef  FR_DEBUG
      char debug_message[10];
      sprintf(debug_message,"EMT=%lu\n\r\0",tail);
      MAXA_printf( debug_message );
   #endif

   if( len > 50 )
   {
      //Can't hold it, right now no valid reports are greather than 50
      return;
   }

   //Calc a new tail position
   newTail = ( tail + QUEUE_ITEMS - 1 ) % QUEUE_ITEMS;


   //If the new tail position will overflow the queue, just
   //add this message to the current tail( which is at the last
   //queue position.  In other words, if the queue is full, always
   //replace the last item in the queue with the latest message
   if( !((head + 1) == (newTail % QUEUE_ITEMS)) )
   {
      tail = newTail;
   }

   CopyReportToQueueItem( messageData, &zigbeeQueue[tail], len );
}



int8 IsZigbeeQueueEmpty()
{
   return ((head + 1) % QUEUE_ITEMS) == (tail % QUEUE_ITEMS);
}

//pData should be a buffer of at least 100 bytes( max zigbee packet size )
//provides a byte array of data and the length,
//should make a copy of head data, can't just point to head data because the head
//position could be ovewritten at any time, esp with small queues in a busy env
void Dequeue( char pDataBuffer[], int8 *reportLen )
{
   #ifdef  FR_DEBUG
      char debug_message[10];
   #endif


   if( IsZigbeeQueueEmpty() == 1 )
   {
      *reportLen = 0;
   }
   else
   {
      memcpy( pDataBuffer, zigbeeQueue[head].frReport, zigbeeQueue[head].len );
      *reportLen = zigbeeQueue[head].len;
      head = ( head + QUEUE_ITEMS - 1 ) % QUEUE_ITEMS;
      #ifdef  FR_DEBUG
      sprintf(debug_message,"DQ=%lu\n\r\0",head);
      MAXA_printf( debug_message );
      #endif
   }
}

/* Not used OR TESTED
void EnqueueLocReport( struct ptdLocReport *pFrLocRep )
{
   //Calc a new tail position
   int8 newTail;
   newTail = ( tail + QUEUE_ITEMS - 1 ) % QUEUE_ITEMS;

   //If the new tail position will overflow the queue, just
   //add this message to the current tail( which is at the last
   //queue position.  In other words, if the queue is full, always
   //replace the last item in the queue with the latest message
   if( !((head + 1) == (newTail % QUEUE_ITEMS)) )
   {
      tail = newTail;
   }

   CopyReportToQueueItem( pFrLocRep, (*pFrLocRep).reportType,
      &frReportQueue[tail], sizeof(struct ptdLocReport) );
}

void EnqueueSensorReport( struct cnSensorReport *pFrSensorReport )
{
   //Calc a new tail position
   int8 newTail;
   newTail = ( tail + QUEUE_ITEMS - 1 ) % QUEUE_ITEMS;

   //If the new tail position will overflow the queue, just
   //add this message to the current tail( which is at the last
   //queue position.  In other words, if the queue is full, always
   //replace the last item in the queue with the latest message
   if( !((head + 1) == (newTail % QUEUE_ITEMS)) )
   {
      tail = newTail;
   }

   CopyReportToQueueItem( pFrSensorReport, (*pFrSensorReport).reportType,
      &frReportQueue[tail], sizeof(struct cnSensorReport) );
}*/










