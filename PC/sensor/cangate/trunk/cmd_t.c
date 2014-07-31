/******************************************************************************
* File Name          : cmd_t.c
* Date First Issued  : 12/10/2013
* Board              : PC
* Description        : Send message of choice to the F4 board
*******************************************************************************/
/*
*/

#include "cmd_t.h"
#include "gatecomm.h"
#include "PC_gateway_comm.h"	// Common to PC and STM32
#include "USB_PC_gateway.h"

static u8 canseqnumber = 0;

/******************************************************************************
 * int cmd_t_init(char* p);
 * @brief 	: Reset 
 * @param	: p = pointer to line entered on keyboard
 * @return	: -1 = too few chars.  0 = OK.
*******************************************************************************/
// static 	u32 keybrd_id;

int cmd_t_init(char* p)
{
	if (strlen(p) < 3){
		// Here too few chars
		printf("\n\nThe 't' command requires the name of a message, example\nt EXAMPLE_MESSAGE \n");
		return -1;
	} else if(p[2] == '-' && p[3] == '-') {
		if(p[4] == 'h' && p[5] == 'e' && p[6] == 'l' && p[7] == 'p') {
			cmd_t_help();
			return -1; // not really an error, but we dont need to do anything else
		} else {
			printf("sorry, unrecognized command\n");
			return -1;
		}
	}
	
	// sscanf( (p+1), "%x",&keybrd_id);
	// printf ("ID: %x\n",keybrd_id);
//	keybrd_id = keybrd_id << 16;
	return 0;
}

int cmd_t_help(void) {
	printf("\nHere is the list of messages that can be used with the 't' command:\n");
	printf("    EXAMPLE_MESSAGE - just an example since nothing is defined yet\n");
	return 1; // successful
}

int cmd_t_send(char* p, int fd) {
	struct CANRCVBUF can;
	struct PCTOGATEWAY pctogateway; 
	printf("the message we need to send is: %s", &p[2]);

	// Switch(message)
	if (strcmp(&p[2], "ledblue\n") == 0) { // case "LEDBLUE"
		can.id       = 0x44200000;
		can.dlc      = 0x00000002;
		can.cd.us[0] = 0x0000;
		pctogateway.mode_link = MODE_LINK;	// Set mode for routines that receive and send CAN msgs
		pctogateway.cmprs.seq = canseqnumber++;	// Add sequence number (for PC checking for missing msgs)
		USB_toPC_msg_mode(fd, &pctogateway, &can); 	// Send to file descriptor (e.g. serial port)

	} else if(strcmp(&p[2], "ledgreen\n") == 0) {
		can.id       = 0x44600000;
		can.dlc      = 0x00000002;
		can.cd.us[0] = 0x0000;
		pctogateway.mode_link = MODE_LINK;	// Set mode for routines that receive and send CAN msgs
		pctogateway.cmprs.seq = canseqnumber++;	// Add sequence number (for PC checking for missing msgs)
		USB_toPC_msg_mode(fd, &pctogateway, &can); 	// Send to file descriptor (e.g. serial port)
	}
	
	return 1;
}