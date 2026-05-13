#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

static uint8_t block_map[BLOCK_COUNT];


void heap_init(void)
{
    // set all blocks to free 0
    memset(block_map, 0, sizeof(block_map));
}


void* heap_alloc(size_t size)
{
    // return NULL if size is 0
    if (size == 0) return NULL;

    //calculate how many blocks are needed
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    //scan for a contiguous run of free blocks
    size_t count = 0;
    size_t start = 0;

    for (size_t i = 0; i < BLOCK_COUNT; i++) {

        //when found a free block
        if (block_map[i] == 0) {
            // if this is the first free block , mark the start index
            if (count == 0) start = i;

            count++;

            if (count == blocks_needed) {
                // Mark blocks as used
                for (size_t j = start; j < start + blocks_needed; j++) {
                    block_map[j] = 1;
                }

                //return pointer to the start of the allocated memory
                return (void*)(HEAP_START_ADDR + start * BLOCK_SIZE);
            }

        } 

        // if not free, reset count
        else count = 0;
        
    }

    // no space found
    return NULL;
}


void heap_free(void* ptr)
{
    // if null pointer then do nothing
    if (ptr == NULL) return;

    //convert pointer to byte pointer 
    uint8_t* p = (uint8_t*)ptr;

    // Check pointer is within bounds
    if (p < HEAP_START_ADDR || p >= HEAP_START_ADDR + HEAP_SIZE) return;

    // Find which block this pointer starts at
    size_t start = (p - HEAP_START_ADDR) / BLOCK_SIZE;

    // Free blocks until we hit a free block
    for (size_t i = start; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 0) break;
        block_map[i] = 0;
    }
}