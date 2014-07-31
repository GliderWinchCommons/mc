/******************************************************************************
* File Name          : cmd_t.h
* Date First Issued  : 12/10/2013
* Board              : PC
* Description        : Send message of choice to the F4 board
*******************************************************************************/

#ifndef __CMD_T_PC
#define __CMD_T_PC

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "common_can.h"	// Definitions common to CAN and this project.


// function defs
int cmd_t_init(char* p);
int cmd_t_help(void);
int cmd_t_send(char* p, int fd);


#endif

