/******************************************************************************
* File Name          : mc_msgs.c
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : CAN format msgs for master controller
*******************************************************************************/
#include "mc_msgs.h"
#include "PC_gateway_comm.h"
#include "USB_PC_gateway.h"
#include "CAN_gateway.h"
#include <fcntl.h>
#include <unistd.h>
#include "canwinch_ldr.h"
#include "CAN_test_msgs.h"
#include "CAN_error_msgs.h"


/* Advance circular pointer macro */
int incIdx(int x, int y){x += 1; if (x >= y) x = 0; return x;} 

/* Circular buffer for incoming CAN + USB -> MC msgs */
#define CANMCBUFSIZE	8			// Number of incoming CAN msgs to buffer
static struct CANRCVBUF canmcbuf[CANMCBUFSIZE];
static int canmcidxi = 0;			// Incoming index into canbuf
static int canmcidxm = 0;			// Outgoing index into canbuf

static struct PCTOGATEWAY pctogateway; // CAN->PC
static struct PCTOGATEWAY gatewayToPC; // PC->CAN

static struct CANRCVBUF* 	pfifo0;		// Pointer to CAN driver buffer for incoming CAN msgs, low priority
static struct CANRCVTIMBUF*	pfifo1;		// Pointer to CAN driver buffer for incoming CAN msgs, high priority

/* Put sequence number on incoming CAN messages that will be sent to the PC */
u8 canmsgctr = 0;	// Count incoming CAN msgs

/* **************************************************************************************
 * void mc_msg_init(void);
 * @brief	: Initialization for msg handling
 * ************************************************************************************** */
void mc_msg_init(void)
{
#ifdef GATEWAYLOCAL
		PC_msg_initg(&pctogateway);	// Initialize struct for CAN message from PC
		PC_msg_initg(&gatewayToPC);	// Initialize struct for CAN message from PC

		/* Set modes for USB-serial port routines that receive and send CAN msgs */
		pctogateway.mode_link = MODE_LINK;
		gatewayToPC.mode_link = MODE_LINK;
#endif
	return;
}

/* **************************************************************************************
 * static void msg_out_can(struct CANRCVBUF* p);
 * @brief	: Output msg from MC to CAN and USB
 * ************************************************************************************** */
static void msg_out_can(struct CANRCVBUF* p)
{
	CAN_gateway_send(p);	// Add to xmit buffer (if OK)
	return;
}
/* **************************************************************************************
 * static void canmcbuf_add(struct CANRCVBUF* p);
 * @brief	: Add msg to buffer holding structs of CAN format msg
 * @param	: p = Pointer to CAN msg
 * ************************************************************************************** */
static void canmcbuf_add(struct CANRCVBUF* p)
{
	int temp;
	canmcbuf[canmcidxi] = *p;		// Copy struct
	temp = incIdx(canmcidxi,CANMCBUFSIZE);	// Increment the index for incoming msgs.
	if (canmcidxm == temp)  		// Did this last fill the last one?
	{ // Yes, we have filled the buffer.  This CAN msg might be dropped (by not advancing the index)
		Errors_misc(-1);		// Add to buffer overrun counter
	}
	else
	{ // Here, there is room in the buffer and we are good to go.
		canmcidxi = temp;		// Update the index to next buffer position.
	}	
	return;
}
/* **************************************************************************************
 * static void msg_out_usb(struct CANRCVBUF* p);
 * @param	: p = pointer to struct with msg to be sent
 * @brief	: Output msg to USB-serial port
 * ************************************************************************************** */
static void msg_out_usb(struct CANRCVBUF* p)
{
	pctogateway.cmprs.seq = canmsgctr++;		// Add sequence number (for PC checking for missing msgs)
	USB_toPC_msg_mode(STDOUT_FILENO, &pctogateway, p); 	// Send to PC via STDOUT
	return;
}
/* **************************************************************************************
 * static int msg_get_can(void);
 * @brief	: Check and get incoming msg from CAN bus
 * @return	: Pointer to a struct CANRCVBUF if there is a msg, otherwise return NULL
 * ************************************************************************************** */
static struct CANRCVBUF* msg_get_can(void)
 {
	if ( (pfifo1 = canrcvtim_get_ldr()) != 0)	// Did we receive a HIGH PRIORITY CAN BUS msg?
	{ // Here yes.
		return &pfifo1->R;	// Return pointer to CANRCVBUF struct
	}
	if ( (pfifo0 = canrcv_get_ldr()) != 0)		// Did we receive a LESS-THAN-HIGH-PRIORITY CAN BUS msg?
	{ // Here yes.
		return pfifo0;	// Add msg to buffer
	}
	return NULL;
	}
/* **************************************************************************************
 * static struct CANRCVBUF* msg_get_usb(void);
 * @brief	: Check and get incoming msg from CAN bus
 * @return	: Pointer to a struct CANRCVBUF if there is a msg, otherwise return NULL
 * ************************************************************************************** */
u32 msg_get_usb_err1 = 0;	// Running error count
static struct CANRCVBUF canrcvbuf;

static struct CANRCVBUF* msg_get_usb(void)
{
	int tmp;
	int temp;
	struct CANRCVBUF* p = 0;  // default to NULL

	/* Each time 'USB_PC_get_msg_mode' is called it adds any buffered incoming ASCII chars */
	temp=USB_PC_get_msg_mode(STDIN_FILENO, &gatewayToPC, &canrcvbuf);	// Check if msg is ready
	if (temp != 0)	// Do we have completion of a msg?
	{ // Here, yes.  We have a msg, but it might not be valid.
		if ( temp == 1 ) // Was valid?
		{ // Here, yes.
			tmp = temp >> 16; // Isolate compression error codes
			if (tmp < 0)	// Did the compression detect some anomolies?
			{ // Here, something wrong with the msg--
				msg_get_usb_err1 += 1;	// Count errors
			}
			else
			{ // Here, msg is OK msg from the PC
				p = &canrcvbuf;	// Return pointer to struct with msg
			}
		}
		else
		{ // Something wrong with the msg.  Count the various types of error returns from 'USB_PC_msg_getASCII'
			Errors_USB_PC_get_msg_mode(temp);
		} // Note: 'pctogateway' gets re-intialized in 'PC_msg_initg' when there are errors.

		/* Initialize struct for next msg from PC to gateway */
		PC_msg_initg(&pctogateway);	
	}
	return p;
}
/* **************************************************************************************
 * static struct CANRCVBUF* msg_get(void);
 * @brief	: Check and add incoming msgs to MC circular buff
 * @return	: pointer to msg, or NULL if no msgs buffered.
 * ************************************************************************************** */
struct CANRCVBUF* msg_get(void)
{
	struct CANRCVBUF* p;

	/* Add all incoming msgs to MC circular buffer */

#ifdef GATEWAYLOCAL
	/* USB -> MC+CAN */
	while ( (p=msg_get_usb()) != 0)
	{
		canmcbuf_add(p); // Add to MC buffer
		CAN_gateway_send(&canrcvbuf); // Add to CAN xmit buffer
	}
#endif

	/* CAN -> MC+USB */
	while ( (p=msg_get_can()) != 0) // CAN msg ready?
	{ // Here, yes.
#ifdef GATEWAYLOCAL
		msg_out_usb(p);	// Send incoming CAN msg to USB
#endif	
		canmcbuf_add(p);
	}

	/* Get next buffered msg */
	if (canmcidxm == canmcidxi) return NULL; // Return NULL if MC buffer empty
	p = &canmcbuf[canmcidxm];		// Pointer to msg
	canmcidxm = incIdx(canmcidxm,CANMCBUFSIZE); // Advance index
	return p;
}

/* **************************************************************************************
 * void msg_out_mc(struct CANRCVBUF* p);
 * @brief	: Output msg from MC to CAN and USB
 * ************************************************************************************** */
void msg_out_mc(struct CANRCVBUF* p)
{
#ifdef GATEWAYLOCAL
	msg_out_usb(p);
#endif
	msg_out_can(p);
	return;
}

