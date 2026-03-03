#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used 

// Add you code below

static uint8_t block_map[BLOCK_COUNT];  

void heap_init(void)
{
    for(int i = 0; i < BLOCK_COUNT; i++)
    {
        block_map[i] = 0;
    }
}

void* heap_alloc(size_t size)
{
    if (size == 0)
        return NULL;

    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    for (int i = 0; i <= BLOCK_COUNT - blocks_needed; i++)
    {
        int found = 1;

        for (int j = 0; j < blocks_needed; j++)
        {
            if (block_map[i + j] != 0)
            {
                found = 0;
                break;
            }
        }

        if (found)
        {
            for (int j = 0; j < blocks_needed; j++)
            {
                block_map[i + j] = 1;
            }

            return (void*)(HEAP_START_ADDR + (i * BLOCK_SIZE));
        }
    }

    return NULL;  
}

void heap_free(void* ptr)
{
    if (ptr == NULL)
        return;

    uintptr_t address = (uintptr_t)ptr;
    uintptr_t heap_start = (uintptr_t)HEAP_START_ADDR;
    uintptr_t heap_end = heap_start + HEAP_SIZE;

    if (address < heap_start || address >= heap_end)
        return;

    int index = (address - heap_start) / BLOCK_SIZE;

    while (index < BLOCK_COUNT && block_map[index] == 1)
    {
        block_map[index] = 0;
        index++;
    }
}