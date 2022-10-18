/*
 * dma.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __DMA_H__
#define __DMA_H__

#include "types.h"

/*** DMA functions ***/

void DMA1_init_channel3(void);
void DMA1_start_channel3(void);
void DMA1_stop_channel3(void);
void DMA1_set_channel3_source_addr(uint32_t source_buf_addr, uint16_t source_buf_size);
uint8_t DMA1_get_channel3_status(void);

void DMA1_init_channel6(void);
void DMA1_start_channel6(void);
void DMA1_stop_channel6(void);
void DMA1_set_channel6_dest_addr(uint32_t dest_buf_addr, uint16_t dest_buf_size);

#endif /* __DMA_H__ */
