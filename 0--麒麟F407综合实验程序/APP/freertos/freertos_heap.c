/*
 * FreeRTOS ucHeap in CCM RAM (CPU only, no DMA).
 *
 * Layout @ 0x10000000 (64KB CCM):
 *   mem3base  [MEM3_MAX_SIZE]     16KB — arbiter 不用 MJPEG
 *   mem3map   [MEM3_ALLOC_TABLE_SIZE * 2]
 *   ucHeap    [FREERTOS_HEAP_BYTES] 32KB
 */
#include "malloc.h"
#include "FreeRTOS.h"

#if ( ( MEM3_MAX_SIZE + ( MEM3_ALLOC_TABLE_SIZE * 2 ) + FREERTOS_HEAP_BYTES ) > ( 64 * 1024 ) )
    #error "CCM layout overflow: reduce MEM3_MAX_SIZE or FREERTOS_HEAP_BYTES"
#endif

#define FREERTOS_UCHEAP_BASE   ( 0x10000000u + MEM3_MAX_SIZE + ( MEM3_ALLOC_TABLE_SIZE * 2u ) )

uint8_t ucHeap[ FREERTOS_HEAP_BYTES ] __attribute__( ( at( FREERTOS_UCHEAP_BASE ) ) );
