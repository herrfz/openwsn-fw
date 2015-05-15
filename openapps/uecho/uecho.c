#include "opendefs.h"
#include "uecho.h"
#include "openudp.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"

#include "riot.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

//=========================== variables =======================================
uint8_t expect_echo;
//=========================== prototypes ======================================

//=========================== public ==========================================

void uecho_init(void) {
}

void uecho_receive(OpenQueueEntry_t* request) {
   uint16_t          temp_l4_destination_port;
   OpenQueueEntry_t* reply;
   
   if (!expect_echo) {
      reply = openqueue_getFreePacketBuffer(COMPONENT_UECHO);
      if (reply==NULL) {
         openserial_printError(
            COMPONENT_UECHO,
            ERR_NO_FREE_PACKET_BUFFER,
            (errorparameter_t)0,
            (errorparameter_t)0
         );
         return;
      }

      reply->owner                         = COMPONENT_UECHO;

      // reply with the same OpenQueueEntry_t
      reply->creator                       = COMPONENT_UECHO;
      reply->l4_protocol                   = IANA_UDP;
      temp_l4_destination_port             = request->l4_destination_port;
      reply->l4_destination_port           = request->l4_sourcePortORicmpv6Type;
      reply->l4_sourcePortORicmpv6Type     = temp_l4_destination_port;
      reply->l3_destinationAdd.type        = ADDR_128B;

      // copy source to destination to echo.
      memcpy(&reply->l3_destinationAdd.addr_128b[0],&request->l3_sourceAdd.addr_128b[0],16);

      packetfunctions_reserveHeaderSize(reply,request->length);
      memcpy(&reply->payload[0],&request->payload[0],request->length);
      openqueue_freePacketBuffer(request);

      if ((openudp_send(reply))==E_FAIL) {
         openqueue_freePacketBuffer(reply);
      } else {
         openqueue_freePacketBuffer(request);
         expect_echo = FALSE;
      }
   }
}

void uecho_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

bool uecho_debugPrint(void) {
   return FALSE;
}

void uecho_send(uint8_t *dest_addr)
{
   OpenQueueEntry_t *request;
   request = openqueue_getFreePacketBuffer(COMPONENT_UECHO);

   if (request==NULL) {
      DEBUG("UECHO: ERROR, no free packet.\n");
      return;
   }

   request->owner = COMPONENT_UECHO;
   request->creator = COMPONENT_UECHO;
   request->l4_protocol = IANA_UDP;
   request->l4_destination_port = UDP_PORTS_16b_SRC_16b_DEST_INLINE;
   request->l4_sourcePortORicmpv6Type = UDP_PORTS_16b_SRC_16b_DEST_INLINE;
   request->l3_destinationAdd.type = ADDR_128B;
   memcpy(&(request->l3_destinationAdd.addr_128b[0]), dest_addr, 16);

   packetfunctions_reserveHeaderSize(request, 13);
   expect_echo = TRUE;
   if ((openudp_send(request))==E_FAIL) {
      openqueue_freePacketBuffer(request);
   }
}

//=========================== private =========================================
