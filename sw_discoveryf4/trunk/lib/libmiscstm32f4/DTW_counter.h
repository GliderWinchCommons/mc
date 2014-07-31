/******************************************************************************
* File Name          : DTW_counter.h
* Date First Issued  : 10/21/2013
* Board              : STM32
* Description        : Use of the DTW_CYCCNT counter
*******************************************************************************/
#ifndef __DTW_COUNTER
#define __DTW_COUNTER

#include "common.h"

/******************************************************************************/
void DTW_counter_init(void);
/* @brief 	: Setup the DTW counter so that it can be read
*******************************************************************************/
static inline u32 DTW(void);
/* @brief 	: Return DTW counter
*******************************************************************************/

#endif 

