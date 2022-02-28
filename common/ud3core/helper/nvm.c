/*
 * UD3 - NVM
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "nvm.h"
#include "stdlib.h"

//-----------------------------------------------------------------------------
// The CY8C5888AXI-LP032 has 256K flash memory, 2K EEPROM, 64K SRam
//
// o flash memory is considered program space (where the firmware resides).  
//   The contents will NOT be lost when power is turned off
//   memory range = 0 - 0x3ffff
//   there are 4 arrays, each array is 64K in size
//   256 rows per array, one row = 256 bytes
// o sram is static ram (does not require constant refreshing, but is lost when 
//   power is turned off.
//   memory range = 0x20000000-0x3f000000
// o EEPROM is for storing long term data like configuration info.  It is not 
//   lost when power is turned off.
//
// This file implements a memory manager for flash memory.  Flash memory must 
// be written one "row" at a time, where each row is 256 bytes.  This hides
// all the details and allows the memory to be treated as a contiguous array
// of 32K individually addressable bytes.
//
// For now this is only used to store the VMS data structures for playing a 
// MIDI file.  These data structures are transferred one time only (after
// burning a new firmware).  The transfer is accomplished by dragging a MCF
// file containing the data to the Teslaterm window.
//-----------------------------------------------------------------------------

// Flash memory is divided into 4 "arrays".  Each array consists of 256 rows.  Each row consists of 256 bytes.
#define NVM_START_ADDR 0x38000      // Last byte of flash mem = 0x3ffff, so room for 0x8000 bytes here (32K)
#define NVM_ARRAY 3                 // The flash "array" containing NVM_START_ADDR
#define NVM_START_PAGE 128          // The row within NVM_ARRAY of NVM_START_ADDR.  In other words, NVM_START_ADDR = NVM_ARRAY * 65536 bytes per array + NVM_START_PAGE * NVM_PAGE_SIZE = 0x3800
#define NVM_PAGE_SIZE 256           // The number of bytes per row

#define NVM_DEBUG 0

#ifndef SIMULATOR
    // NVM_mapMem is the start of MAPTABLE_HEADER's.  A MAPTABLE_HEADER corresponds 
    // to a MIDI program.  Each MAPTABLE_HEADER is immediately followed by a variable 
    // number of MAPTABLE_ENTRY's.
    const volatile uint8_t* NVM_mapMem = (uint8_t*)NVM_START_ADDR; 
    
    // NVM_blockMem is the start of the VMS_BLOCK array.  The blocks are part of a database
    // that comes with the MidiStick software in a .mcf file.  The database is transferred
    // once when the user drags a .mcf file onto the Teslaterm window.
    const volatile uint8_t * NVM_blockMem = (uint8_t*)(NVM_START_ADDR + MAPMEM_SIZE);
    const VMS_BLOCK* VMS_BLKS = (VMS_BLOCK*)(NVM_START_ADDR + MAPMEM_SIZE);
#else
    
    uint8_t flash_pages[128][256];
    
    uint8_t * NVM_mapMem = (uint8_t*)flash_pages;
    uint8_t * NVM_blockMem = (uint8_t*)(((uint8_t*)flash_pages) + MAPMEM_SIZE);
    VMS_BLOCK* VMS_BLKS = (VMS_BLOCK*)(((uint8_t*)flash_pages) + MAPMEM_SIZE);
    
    uint8_t CyWriteRowData(uint8_t arr_n, uint16_t page, uint8_t* page_content){
        page -= NVM_START_PAGE;
        memcpy(&flash_pages[page][0], page_content, NVM_PAGE_SIZE);
        return 0;
    }
    void CyFlushCache(){
        
    }
    void CySetTemp(){
        
    }
#endif

static uint8_t *page_content=NULL;
static uint16_t last_page=0xFFFF;
static uint16_t page;

// Commits the cached page to flash (if any)
uint8_t nvm_flush(){
    // TODO: if(!page_content) return pdTRUE;
    
    CySetTemp();         // Must update the temperature before writing flash
    uint32_t rc = CyWriteRowData((uint8)NVM_ARRAY, page, page_content);
    CyFlushCache();     // Required to invalidate the CPU cache
    
    vPortFree(page_content);
    page_content = NULL;
    last_page=0xFFFF;
    
    return rc ? pdFAIL : pdTRUE;
}

// Writes len bytes from buffer to flash memory starting at NVM_START_ADDR + index.
// The bytes are written to a cache and may not be committed to flash until nvm_flush() is called.
// This is called by the min task to store VMS data received on the wire to store both VMS BLOCKS and MAPS.
uint8_t nvm_write_buffer(uint16_t index, uint8_t* buffer, int32_t len){

    uint16_t n_pages = 0;

    // Data is accumulated in a single page of sram before being written to flash.
    if(page_content==NULL){
        page_content = pvPortMalloc(NVM_PAGE_SIZE);
    }
    if(page_content == NULL) return pdFAIL;
    
    // offset within page (row)
    uint32_t write_offset = index % NVM_PAGE_SIZE;

    while(len>0){
        // Address of page containing index
        uint8_t * page_addr = (uint8_t*)NVM_mapMem + ((index / NVM_PAGE_SIZE)*NVM_PAGE_SIZE); 
        
        // page index
        page = NVM_START_PAGE + (index / NVM_PAGE_SIZE);
        
        // If there was no last page go ahead and get the contents from flash
        if(last_page==0xFFFF){
            last_page = page;
            memcpy(page_content, page_addr , NVM_PAGE_SIZE);    // copy entire page (row) from flash into page_content
        }
        if(page > 256) return pdFAIL;   // end of flash
        
        // Limit amount to write to length of the page
        uint32_t write_len = len;
        if(write_len+write_offset > NVM_PAGE_SIZE){
            write_len = NVM_PAGE_SIZE-write_offset;   
        }
        
        // See if the the current page is still cached in last_page
        if(last_page != page){
            // Cache miss.  Commit the last page to flash and reload the cache from the current page
            CySetTemp();
            uint32_t rc = CyWriteRowData((uint8)NVM_ARRAY, last_page, page_content);
            CyFlushCache();
            if(rc) return pdFAIL;
            
            // Reload page_content from flash
            memcpy(page_content, page_addr , NVM_PAGE_SIZE);
        }
        
        // Write buf to the current cached page
        memcpy(page_content + write_offset, buffer , write_len);
        last_page=page;
        
        buffer+=write_len;

        // TODO: The entire if/else could be replaced with:
        // len -= write_len;
        // index += write_len;
        // instead of updating write_offset here, move the calculation of write_index above this loop into the loop at the very top.
        
        if(write_offset){
            // TODO: This is fine, but would be cleaner with len -= write_len;
            len -= (NVM_PAGE_SIZE-write_offset);    // Subtract off the number of bytes written to the cache (may go negative)
            
            // TODO: This works because index is only used to calc the page index, 
            // not the offset in the loop above and write_offset is set to 0 here.
            // index += write_len;  
            index += NVM_PAGE_SIZE;
            
            write_offset = 0;
        }else{
            len -= NVM_PAGE_SIZE;
            index += NVM_PAGE_SIZE;
        }
        n_pages++;
        
        #if NVM_DEBUG
            for(int i=0; i<NVM_PAGE_SIZE; i++){
                ttprintf("%02X:%c ", page_content[i], page_content[i]);   
            }
            ttprintf("\r\n");
            ttprintf("page: %u addr: %04X n_pages %u write_len: %u\r\n", page, page_addr, n_pages, write_len);
        #endif
    }
    
    return pdPASS;
}

// TODO: This is not used
void VMS_init_blk(VMS_BLOCK* blk){
    blk->uid = 0;
    blk->nextBlocks[0] = NO_BLK;
    blk->nextBlocks[1] = NO_BLK;
    blk->nextBlocks[2] = NO_BLK;
    blk->nextBlocks[3] = NO_BLK;
    blk->behavior = NORMAL;
    blk->type = VMS_LIN;
    blk->target = maxOnTime;
    blk->thresholdDirection = RISING;
    blk->targetFactor = 0;
    blk->param1 = 0;
    blk->param2 = 0;
    blk->param3 = 0;
    blk->period = 0;
    blk->flags = 0; 
}

// Prints the contents of blk on the terminal.
void VMS_print_blk(TERMINAL_HANDLE* handle, VMS_BLOCK* blk, uint8_t indent){
    ttprintf("%*sBlock ID: %u @ 0x%08X\r\n", indent, "", blk->uid, blk);
    indent++;
    for(int i=0;i<VMS_MAX_BRANCHES;i++){
        if(blk->nextBlocks[i] == NO_BLK){
            ttprintf("%*sNext %i: No Block\r\n", indent, "", i);   
        }else if(blk->nextBlocks[i] < (VMS_BLOCK*)4096){
            ttprintf("%*sNext %i: ID %u\r\n", indent, "", i, blk->nextBlocks[i]);
        }else{
            ttprintf("%*sNext %i: 0x%08X\r\n", indent, "", i, blk->nextBlocks[i]);
        }
    }
    
    if(blk->offBlock == NO_BLK){
        ttprintf("%*soffBlock: No Block\r\n", indent, "");   
    }else if(blk->offBlock < (VMS_BLOCK*)4096){
        ttprintf("%*soffBlock: ID %u\r\n", indent, "", blk->offBlock);
    }else{
        ttprintf("%*soffBlock: 0x%08X\r\n", indent, "", blk->offBlock);
    }
    

    ttprintf("%*sBehavior: ", indent, "");
    switch(blk->behavior){
        case NORMAL:
            ttprintf("NORMAL");
        break;
        case INVERTED:
            ttprintf("INVERTED");
        break;    
    }
    
    ttprintf("\r\n%*sType: ", indent, "");
    switch(blk->type){
        case VMS_LIN:
            ttprintf("LIN");
        break;
        case VMS_EXP_INV:
            ttprintf("EXP INV");
        break;  
        case VMS_EXP:
            ttprintf("EXP");
        break; 
        case VMS_SIN:
            ttprintf("SIN");
        break;
        case VMS_JUMP:
            ttprintf("JUMP");
        break;
    }
    
    ttprintf("\r\n%*sTarget: ", indent, "");
    switch(blk->target){
        case onTime:
            ttprintf("onTime");
        break;
        case maxOnTime:
            ttprintf("maxOnTime");
        break;  
        case minOnTime:
            ttprintf("minOnTime");
        break;  
        case otCurrent:
            ttprintf("otCurrent");
        break;
        case otTarget:
            ttprintf("otTarget");
        break;
        case otFactor:
            ttprintf("otFactor");
        break;
        case frequency:
            ttprintf("frequency");
        break;
        case freqCurrent:
            ttprintf("freqCurrent");
        break;
        case freqTarget:
            ttprintf("freqTarget");
        break;
        case freqFactor:
            ttprintf("freqFactor");
        break;
        case noise:
            ttprintf("noise");
        break;
        case pTime:
            ttprintf("pTime");
        break;
        case circ1:
            ttprintf("circ1");
        break;
        case circ2:
            ttprintf("circ2");
        break;
        case circ3:
            ttprintf("circ3");
        break;
        case circ4:
            ttprintf("circ4");
        break;
        case CC_102 ... CC_119:
            ttprintf("CC_102 ... CC_119");
        break;
        case KNOWNVAL_MAX:
            ttprintf("KNOWNVAL_MAX");
        break;
        case HyperVoice_Count:
            ttprintf("Hypervoice_Count");
        break;
        case HyperVoice_Phase:
            ttprintf("Hypervoice_Phase");
        break;
    }
    
    ttprintf("\r\n%*sThreshold Direction: ", indent, "");
    switch(blk->thresholdDirection){
        case RISING:
            ttprintf("RISING");
        break;
        case FALLING:
            ttprintf("FALLING");
        break;
        case ANY:
            ttprintf("ANY");
        break;
        case NONE:
            ttprintf("NONE");
        break;
    }
    
    ttprintf("\r\n%*sTarget factor: %i\r\n", indent, "", blk->targetFactor);
    ttprintf("%*sParam 1: %i\r\n", indent, "", blk->param1);
    ttprintf("%*sParam 2: %i\r\n", indent, "", blk->param2);
    ttprintf("%*sParam 3: %i\r\n", indent, "", blk->param3);
    ttprintf("%*sPeriod: %u\r\n", indent, "", blk->period);
    ttprintf("%*sFlags: 0x%08X\r\n\r\n", indent, "", blk->flags);
}

// Prints the contents of map on the terminal.  
// Returns a pointer to the next MAPTABLE_HEADER or NULL if none.
MAPTABLE_HEADER* VMS_print_map(TERMINAL_HANDLE* handle, MAPTABLE_HEADER* map){
    ttprintf("\r\nProgram: %u - %s\r\n", map->programNumber, map->name);
    
    // The MAPTABLE_ENTRY struct is immediately followed by a variable number of MAPTABLE_ENTRY structs.
    MAPTABLE_ENTRY* ptr = (MAPTABLE_ENTRY*)(map+1);
    for(uint32_t i=0;i<map->listEntries;i++){
        ttprintf("   Note range: %u - %u\r\n", ptr->startNote, ptr->endNote);
        ttprintf("   Note frequency: %u\r\n", ptr->data.noteFreq);
        ttprintf("   Target ontime : %u\r\n", ptr->data.targetOT);
        ttprintf("   Flags         : 0x%04x\r\n", ptr->data.flags);
        ttprintf("   Start block   : 0x%04x\r\n\r\n", ptr->data.VMS_Startblock);
        ptr++;
    }
    
    // Return pointer to next MAPTABLE_HEADER or NULL if none.
    map = (MAPTABLE_HEADER*)ptr;
    if(map->listEntries){
        return map;
    }else{
        return NULL;
    }
}

// Returns the number of blocks starting with the specified block
uint32_t nvm_get_blk_cnt(const VMS_BLOCK* blk){
    // TODO: could be replaced with:
    // uint32_t cnt = 0;
    // while(blk[cnt].uid != 0)
    //    ++cnt;
    // return cnt;
    
    uint32_t cnt=0;
    while(1){
        if(blk->uid==0) break;
        blk++;
        cnt++;
    }
    return cnt;
}

// Handles the nvm command to display the maps or blocks on the terminal
uint8_t CMD_nvm(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("nvm [maps|blocks]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(strcmp(args[0], "maps") == 0){
        MAPTABLE_HEADER* map = (MAPTABLE_HEADER*) NVM_mapMem;
    
        while(map){
            map = VMS_print_map(handle, map);
        }
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(strcmp(args[0], "blocks") == 0){
        uint32_t n_blocks = nvm_get_blk_cnt(VMS_BLKS);
        ttprintf("NVM block count: %u\r\n", n_blocks);
        
        for(uint32_t i=0;i<n_blocks;i++){
            VMS_print_blk(handle, (VMS_BLOCK*)&VMS_BLKS[i], 4);
        }
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    return TERM_CMD_EXIT_SUCCESS; 
}