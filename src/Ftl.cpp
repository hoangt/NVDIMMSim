/*********************************************************************************
*  Copyright (c) 2011-2012, Paul Tschirhart
*                             Peter Enns
*                             Jim Stevens
*                             Ishwar Bhati
*                             Mu-Tien Chang
*                             Bruce Jacob
*                             University of Maryland 
*                             pkt3c [at] umd [dot] edu
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

//Ftl.cpp
//class file for ftl
//
#include "Ftl.h"
#include "ChannelPacket.h"
#include "NVDIMM.h"
#include <cmath>

using namespace NVDSim;
using namespace std;

Ftl::Ftl(Configuration &nv_cfg, Controller *c, Logger *l, NVDIMM *p) :
	cfg(nv_cfg)
{

	numBlocks = cfg.NUM_PACKAGES * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.BLOCKS_PER_PLANE;
	
	// we need an extra page(block) for the gap in start gap
	// we don't use the overprovisioning factor here cause start gap doesn't have a way to use
	// the extra blocks
	if(cfg.wearLevelingScheme == StartGap)
	{
	    numBlocks = numBlocks + 1;
	}

	channel = 0;
	die = 0;
	plane = 0;
	lookupCounter = 0;

	busy = 0;

	addressMap = std::unordered_map<uint64_t, uint64_t>();

	used = vector<vector<bool>>(numBlocks, vector<bool>(cfg.PAGES_PER_BLOCK, false));
	used_page_count = 0;

	dirty = vector<vector<bool>>(numBlocks, vector<bool>(cfg.PAGES_PER_BLOCK, false));
	dirty_page_count = 0;

	transQueue = list<FlashTransaction>();

	controller = c;

	parent = p;

	log = l;

	locked_counter = 0;
	
	saved = false; // indicates that state data has been successfully saved to make sure we don't do it twice
	loaded = false; // indicates that state data has been successfully loaded to make sure we don't do it twice
	dirtied = false; // indicates that the NVM has been predirtied to simulate a used NVM
	ctrl_read_queues_full = false; // these let us know when we can't push anything more out to the controller
	ctrl_write_queues_full = false;

	// Counter to keep track of succesful writes.
	write_counter = 0;
	used_page_count = 0;

	// Counter to keep track of cycles we spend waiting on erases
	// if we wait longer than the length of an erase, we've probably deadlocked
	deadlock_counter = 0;

	// the maximum amount of time we can wait before we're sure we've deadlocked
	// time it takes to read all of the pages in a block
	deadlock_time = cfg.PAGES_PER_BLOCK * (READ_CYCLES + ((divide_params_64b((cfg.NV_PAGE_SIZE*8),cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE) / cfg.CYCLE_TIME) +
					   ((divide_params_64b(cfg.COMMAND_LENGTH,cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE) / cfg.CYCLE_TIME));
	// plus the time it takes to write all of the pages in a block
	deadlock_time += cfg.PAGES_PER_BLOCK * (WRITE_CYCLES + ((divide_params_64b((cfg.NV_PAGE_SIZE*8),cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE) / cfg.CYCLE_TIME) +
					   ((divide_params_64b(cfg.COMMAND_LENGTH,cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE) / cfg.CYCLE_TIME));
	// plus the time it takes to erase the block
	deadlock_time += ERASE_CYCLES + ((divide_params_64b(cfg.COMMAND_LENGTH,cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE) / cfg.CYCLE_TIME);

	write_wait_count = cfg.DELAY_WRITE_CYCLES;

	// start gap variables
	start = 0;
	gap = ((numBlocks * cfg.PAGES_PER_BLOCK)-1)*cfg.NV_PAGE_SIZE; // gap is a physical address

	pending_gap_read = false; // indicates that we're waiting on a read of the data from the new gap location
	gap_read_delayed = false; // indicates that there was no room for the read and we will have to try again
	pending_gap_write = false; // indicates that we were not successful in writing the data to the old gap 
	                           // location and will have to try again
	gap_moving = false; // indicates that we are currently moving the gap so we don't move it extra times
	gap_moved = false; // indicates that we've moved the gap for this interation so we don't move it extra times
	srand(time(NULL)); // initialize rand, I think random shuffle uses rand by default

	// if randomized address space
	// this effectively randomizes the pages but individual addresses will still need to be adjusted
	if(cfg.wearLevelingScheme == StartGap && cfg.RANDOM_ADDR)
	{
	    randomAddrs = vector<uint64_t>((numBlocks * cfg.PAGES_PER_BLOCK)-1);

	    for(uint64_t i = 0; i < (numBlocks * cfg.PAGES_PER_BLOCK)-1; i++)
	    {
		randomAddrs[i] = i; // fill vector with all of the addresses
	    }

	    // actually randomizes stuff
	    std::random_shuffle(randomAddrs.begin(), randomAddrs.end());
	}

	// get the bit lengths of various parts of the address for translating purposes
	// unless we're doing addressing the old way and then whatever
	if(cfg.addressScheme != Default)
	{
		channelBitWidth = nvdimm_log2(cfg.NUM_PACKAGES);
		vaultBitWidth = nvdimm_log2(cfg.DIES_PER_PACKAGE);
		bankBitWidth = nvdimm_log2(cfg.PLANES_PER_DIE);
		rowBitWidth = nvdimm_log2(cfg.BLOCKS_PER_PLANE);
		colBitWidth = nvdimm_log2(cfg.PAGES_PER_BLOCK);
		colOffset = nvdimm_log2(cfg.NV_PAGE_SIZE);
		cout << "offsets are " << "chan: " << channelBitWidth << " r: " << vaultBitWidth << " b: " << bankBitWidth << " row: " << rowBitWidth << " col: " << colBitWidth << " offset: " << colOffset << "\n";
	}
}

ChannelPacket *Ftl::translate(ChannelPacketType type, uint64_t vAddr, uint64_t pAddr){

	uint64_t package, die, plane, block, page;
	//uint64_t tempA, tempB, physicalAddress = pAddr;
	uint64_t physicalAddress = pAddr;

	if (physicalAddress > TOTAL_SIZE - 1){
		ERROR("Inavlid address in Ftl: "<<physicalAddress);
		exit(1);
	}

	if(cfg.wearLevelingScheme == DirectTranslation)
	{
		uint64_t tempA, tempB;
		//if we have a power of two of sections then we can just split on bits
		if(nvdimm_check_power2(cfg.BLOCKS_PER_PLANE))
		{
			// each sequence starts with the low order bits and works up to the high order bits
			switch (cfg.addressScheme){
			case ChannelVaultBankRowCol:
				// need to clear out the zeroed bits below the access level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;			
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;
				break;
			case ColRowBankVaultChannel:
				// need to clear out the zeroed bits below the col level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;
				break;
			case RowBankVaultChannelCol:
				// need to clear out the zeroed bits below the col level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;		
				break;
			case BankVaultChannelColRow:
				// need to clear out the zeroed bits below the col level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;

				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;				
				break;
			case BankVaultColChannelRow:
				// need to clear out the zeroed bits below the col level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;			
				break;
			case BankChannelVaultColRow:
				// need to clear out the zeroed bits below the col level
				physicalAddress = physicalAddress >> colOffset;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> rowBitWidth;
				tempB = physicalAddress << rowBitWidth;
				block = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> colBitWidth;
				tempB = physicalAddress << colBitWidth;
				page = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> vaultBitWidth;
				tempB = physicalAddress << vaultBitWidth;
				die = tempA ^ tempB;
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> channelBitWidth;
				tempB = physicalAddress << channelBitWidth;
				package = tempA ^ tempB;		
				
				tempA = physicalAddress;
				physicalAddress = physicalAddress >> bankBitWidth;
				tempB = physicalAddress << bankBitWidth;
				plane = tempA ^ tempB;			
				break;
			case Default:
				physicalAddress /= cfg.NV_PAGE_SIZE;
				page = physicalAddress % cfg.PAGES_PER_BLOCK;
				physicalAddress /= cfg.PAGES_PER_BLOCK;
				block = physicalAddress % cfg.BLOCKS_PER_PLANE;
				physicalAddress /= cfg.BLOCKS_PER_PLANE;
				plane = physicalAddress % cfg.PLANES_PER_DIE;
				physicalAddress /= cfg.PLANES_PER_DIE;
				die = physicalAddress % cfg.DIES_PER_PACKAGE;
				physicalAddress /= cfg.DIES_PER_PACKAGE;
				package = physicalAddress % cfg.NUM_PACKAGES;
				break;
			default:
				physicalAddress /= cfg.NV_PAGE_SIZE;
				page = physicalAddress % cfg.PAGES_PER_BLOCK;
				physicalAddress /= cfg.PAGES_PER_BLOCK;
				block = physicalAddress % cfg.BLOCKS_PER_PLANE;
				physicalAddress /= cfg.BLOCKS_PER_PLANE;
				plane = physicalAddress % cfg.PLANES_PER_DIE;
				physicalAddress /= cfg.PLANES_PER_DIE;
				die = physicalAddress % cfg.DIES_PER_PACKAGE;
				physicalAddress /= cfg.DIES_PER_PACKAGE;
				package = physicalAddress % cfg.NUM_PACKAGES;
				break;
			}
		}
		else
		{
			cout << "WARNING: Non-power of 2 so using default translation \n";
			physicalAddress /= cfg.NV_PAGE_SIZE;
			page = physicalAddress % cfg.PAGES_PER_BLOCK;
			physicalAddress /= cfg.PAGES_PER_BLOCK;
			block = physicalAddress % cfg.BLOCKS_PER_PLANE;
			physicalAddress /= cfg.BLOCKS_PER_PLANE;
			plane = physicalAddress % cfg.PLANES_PER_DIE;
			physicalAddress /= cfg.PLANES_PER_DIE;
			die = physicalAddress % cfg.DIES_PER_PACKAGE;
			physicalAddress /= cfg.DIES_PER_PACKAGE;
			package = physicalAddress % cfg.NUM_PACKAGES;
		}
	}
	else
	{
		physicalAddress /= cfg.NV_PAGE_SIZE;
		page = physicalAddress % cfg.PAGES_PER_BLOCK;
		physicalAddress /= cfg.PAGES_PER_BLOCK;
		block = physicalAddress % cfg.BLOCKS_PER_PLANE;
		physicalAddress /= cfg.BLOCKS_PER_PLANE;
		plane = physicalAddress % cfg.PLANES_PER_DIE;
		physicalAddress /= cfg.PLANES_PER_DIE;
		die = physicalAddress % cfg.DIES_PER_PACKAGE;
		physicalAddress /= cfg.DIES_PER_PACKAGE;
		package = physicalAddress % cfg.NUM_PACKAGES;
	}

	return new ChannelPacket(type, vAddr, pAddr, page, block, plane, die, package, NULL);
}

bool Ftl::attemptAdd(FlashTransaction &t, std::list<FlashTransaction> *queue, uint64_t queue_limit)
{
    if(queue->size() >= queue_limit && queue_limit != 0)
    {
	return false;
    }
    else
    {
	queue->push_back(t);
	
	if(cfg.LOGGING)
	{
	    // Start the logging for this access.
	    log->access_start(t.address, t.transactionType);
	    log->ftlQueueLength(queue->size());
	    if(cfg.QUEUE_EVENT_LOG)
	    {
		log->log_ftl_queue_event(false, queue);
	    }
	}

	cout << "added transaction to address " << t.address << " on cycle " << currentClockCycle << "\n";
	return true;
    }
}

bool Ftl::checkQueueAddTransaction(FlashTransaction &t)
{
	if(t.transactionType == DATA_WRITE)
	{
		// see if this write replaces another already in the transaction queue
		// also make sure that no read in the queue references the old write
		// if both conditions hold remove the old write from the queue it is no longer neccessary
		list<FlashTransaction>::iterator it;
		int count = 0;
		bool rw_conflict = false;
		//cout << "checking queue for depreciated writes \n";
		for (it = transQueue.begin(); it != transQueue.end(); it++)
		{
			// don't replace the write if we're already working on it
			if((*it).address == t.address && currentTransaction.address != t.address && (*it).transactionType == DATA_WRITE)
			{
				list<FlashTransaction>::iterator it2;
				for(it2 = transQueue.begin(); it2 != it; it2++)
				{
					if((*it2).address == t.address && (*it2).transactionType == DATA_READ)
					{
						rw_conflict = true;
						//cout << "rw conflict detected!! \n";
						break;
					}
				}
				if(!rw_conflict)
				{
					if(cfg.LOGGING)
					{
						// access_process for that write is called here since its over now.
						log->access_ftl_handled(t.address);
					}
					// issue a callback for this write
					if (parent->WriteDataDone != NULL){
						(*parent->WriteDataDone)(parent->systemID, (*it).address, currentClockCycle, true);
					}
					transQueue.erase(it);
					//cout << "Replaced a write!! \n";
					break;
				}
			}
			count++;
		}
		// if we erased the write that this write replaced then we should definitely
		// always have room for this write
		return attemptAdd(t, &transQueue, cfg.FTL_QUEUE_LENGTH);
	}
	else
	{
		bool success =  attemptAdd(t, &transQueue, cfg.FTL_QUEUE_LENGTH);
		if (success == true)
		{
			if( transQueue.size() == 1)
			{
				read_pointer = transQueue.begin();
			}
		}
		return success;
	}
	return false;
}

bool Ftl::addTransaction(FlashTransaction &t){
    // if we're using start gap then we have to exclude an extra page from the address space
    if(t.address < VIRTUAL_TOTAL_SIZE*1024)
    {
	// we are going to favor reads over writes
	// so writes get put into a special lower prioirty queue
	if(cfg.FTL_QUEUE_HANDLING)
	{
	    return checkQueueAddTransaction(t);
	}
	// no scheduling, so just shove everything into the read queue
	else
	{
	    return attemptAdd(t, &transQueue, cfg.FTL_QUEUE_LENGTH);
	}
    }
    
    ERROR("Tried to add a transaction with a virtual address that was out of bounds");
    exit(5001);
}

// this is the main function of the ftl, it will run on every clock tick
void Ftl::update(void){

	// if we're not already doing something see if there is something to do
	if (!busy)
	{
		if (!transQueue.empty()) {
		    busy = 1;
		    currentTransaction = transQueue.front();
		    lookupCounter = LOOKUP_CYCLES;
		}
	}

	// if we're doing something, then do it
	if (busy) 
	{		
	    if (lookupCounter <= 0 && !ctrl_write_queues_full){
		    cout << "issued transaction to address " << currentTransaction.address << " on cycle " << currentClockCycle << "\n";
			switch (currentTransaction.transactionType){
				case DATA_READ:
				{
				    if(!ctrl_read_queues_full)
				    {
					handle_read(false);
				    }
				    break;
				}

				case DATA_WRITE: 
					handle_write(false);
					break;

				// GC type support added for Start Gap
			        case GC_DATA_READ:
				{
				    // only correct if this arrives while we're using start gap
				    if(cfg.wearLevelingScheme == StartGap)
				    {
                                        // right now we just lock up on a gc read if it doesn't go through
				        handle_read(true);
					break;
				    }
				    else
				    {
					ERROR("FTL received an illegal transaction type:" << currentTransaction.transactionType);
					break;
				    }
				}

				case GC_DATA_WRITE:
				{
				    // only correct if this arrives while we're using start gap
				    if(cfg.wearLevelingScheme == StartGap)
				    {
					handle_write(true);
					break;
				    }
				    else
				    {
					ERROR("FTL received an illegal transaction type:" << currentTransaction.transactionType);
					break;
				    }
				}

			        case TRIM:
				        handle_trim();
				        break;

			        case PRESET_PAGE:
				        handle_preset();
				        break;				        
				    
				default:
					ERROR("FTL received an illegal transaction type:" << currentTransaction.transactionType);
					break;
			}
	    } //if lookupCounter is not 0
	    else if(lookupCounter > 0)
	    {
		lookupCounter--;
	    }
	    else if(ctrl_write_queues_full || ctrl_read_queues_full)// if queues full is true, these are reset by the queues not full method
	    {
		locked_counter++;
	    }
	} // Not currently busy.
	/*else {
		if (!transQueue.empty()) {
		    busy = 1;
		    currentTransaction = transQueue.front();
		    lookupCounter = LOOKUP_CYCLES;
		}
		}*/

}

// fake an unmapped read for the disk case by first fast writing the page and then normally reading that page
void Ftl::handle_disk_read(bool gc)
{
    ChannelPacket *commandPacket;
    uint64_t vAddr = currentTransaction.address, pAddr;
    bool done = false;;
    uint64_t block, page;
    write_location loc;

    //=============================================================================
    // the fast write part
    //=============================================================================
    //look for first free physical page starting at the write pointer

    // find the next location for this write
    // calling the appropriate next location scheme
    loc = next_write_location(vAddr);
    // gettting all the useful information
    pAddr = loc.address;
    block = loc.block;
    page = loc.page;
    done = loc.done;

    if (!done)
    {
	deadlock_counter++;
	if(deadlock_counter == deadlock_time)
	{
	    //bad news
	    cout << deadlock_time;
	    ERROR("FLASH DIMM IS COMPLETELY FULL AND DEADLOCKED - If you see this, something has gone horribly wrong.");
	    exit(9001);
	}
    } 
    else 
    {
	// We've found a used page. Now we need to try to add the transaction to the Controller queue.
	
	// first things first, we're no longer in danger of dead locking so reset the counter
	deadlock_counter = 0;
	
	// quick write the page
	ChannelPacket *tempPacket = Ftl::translate(FAST_WRITE, vAddr, pAddr);
	controller->writeToPackage(tempPacket);

	used.at(block).at(page) = true;
	used_page_count++;
	addressMap[vAddr] = pAddr;
	
	// Delete the fast write packet cause its done
	delete tempPacket;
	
	if(cfg.wearLevelingScheme == RoundRobin)
	{
	    //update "write pointer"
	    channel = (channel + 1) % cfg.NUM_PACKAGES;
	    if (channel == 0){
		die = (die + 1) % cfg.DIES_PER_PACKAGE;
		if (die == 0)
		    plane = (plane + 1) % cfg.PLANES_PER_DIE;
	    }
	}
	//=============================================================================
	// the read part
	//=============================================================================    
	// so now we can read
	// now make a read to that page we just quickly wrote
	commandPacket = Ftl::translate(READ, vAddr, addressMap[vAddr]);
	
	//send the read to the controller
	bool result = controller->addPacket(commandPacket);
	if(result)
	{
	    if(cfg.LOGGING && !gc)
	    {
		// Update the logger (but not for GC_READ).
		log->read_mapped();
	    }
	    popFrontTransaction();
	    busy = 0;    
	}
	else
	{
	    // Delete the packet if it is not being used to prevent memory leaks.
	    delete commandPacket;
	    
	    ctrl_write_queues_full = true;	
	    log->locked_up(currentClockCycle);
	}
    }
}

void Ftl::handle_read(bool gc)
{
    ChannelPacket *commandPacket;
    uint64_t vAddr = currentTransaction.address;
    // Check to see if the vAddr exists in the address map.
    if (addressMap.find(vAddr) == addressMap.end())
    {
	if (gc)
	{
	    ERROR("GC tried to move data that wasn't there.");
	    exit(1);
	}
	
	// if we are disk reading then we want to map an unmapped read and treat is normally
	if(cfg.DISK_READ)
	{
	    handle_disk_read(gc);
	}
	else
	{
	    
	    // If not, then this is an unmapped read.
	    // We return a fake result immediately.
	    // In the future, this could be an error message if we want.
	    if(cfg.LOGGING)
	    {
		// Update the logger
		log->read_unmapped();
		
		// access_process for this read is called here since this ends now.
		log->access_process(vAddr, vAddr, 0, READ);
		
		// stop_process for this read is called here since this ends now.
		log->access_stop(vAddr, vAddr);
	    }
	    
	    // Miss, nothing to read so return garbage.
	    controller->returnUnmappedData(FlashTransaction(RETURN_DATA, vAddr, (void *)0xdeadbeef));
	    
	    popFrontTransaction();
	    busy = 0;
	}
    } 
    else 
    {	
	ChannelPacketType read_type;
	if (gc)
	  read_type = GC_READ;
	else
	  read_type = READ;
	commandPacket = Ftl::translate(read_type, vAddr, addressMap[vAddr]);
	
	//send the read to the controller
	bool result = controller->addPacket(commandPacket);
	if(result)
	{
	    if(cfg.LOGGING && !gc)
	    {
		// Update the logger (but not for GC_READ).
		log->read_mapped();
	    }
	    popFrontTransaction();
	    busy = 0;
	    
	}
	else
	{
	    // Delete the packet if it is not being used to prevent memory leaks.
	    delete commandPacket;
	    ctrl_write_queues_full = true;	
	    log->locked_up(currentClockCycle);
	}
    }
}

void Ftl::write_used_handler(uint64_t vAddr)
{
    // used and dirty are mutually exclusive states for round robin wear leveling and so we need to unmark
    // this page as used and make it dirty

    // technically this page won't stop being used with start gap but because its address might change slightly
    // on this rewrite, we temporarily mark it as unused on a rewrite

    // we're going to write this data somewhere else for wear-leveling purposes however we will probably 
    // want to reuse this block for something at some later time so mark it as unused because it is
    used[addressMap[vAddr] / BLOCK_SIZE][(addressMap[vAddr] / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK] = false;   
    used_page_count--;

    // here we do mark the unused page as dirty so that we can know how long it will take to write to it
    dirty[addressMap[vAddr] / BLOCK_SIZE][(addressMap[vAddr] / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK] = true;
    dirty_page_count ++;
    
    //cout << "USING FTL's WRITE_USED_HANDLER!!!\n";
}

void Ftl::write_success(uint64_t block, uint64_t page, uint64_t vAddr, uint64_t pAddr, bool gc, bool mapped)
{
    // Set the used bit for this page to true.
    used[block][page] = true;
    used_page_count++;
	
    // Pop the transaction from the transaction queue.
    popFrontTransaction();
	
    // The FTL is no longer busy.
    busy = 0;
	
    // Update the write counter.
    write_counter++;
	
    // we've updated the write counter so now its safe to reset the gap movement bool
    gap_moved = false;

    if (cfg.LOGGING && !gc)
    {
	if (mapped)
	    log->write_mapped();
	else
	    log->write_unmapped();
    }
	
    // Update the address map.
    addressMap[vAddr] = pAddr;
}

write_location Ftl::next_write_location(uint64_t vAddr)
{
    if(cfg.wearLevelingScheme == StartGap)
    {
		return start_gap_write_location(vAddr);
    }
	else if(cfg.wearLevelingScheme == DirectTranslation)
	{
		// in direct translation, we just use the vAddr as the pAddr
		return direct_write_location(vAddr);
	}
    else
    {
		return round_robin_write_location(vAddr);
    }
}

write_location Ftl::direct_write_location(uint64_t vAddr)
{
	write_location loc;

	loc.address = vAddr;
	loc.block = vAddr / BLOCK_SIZE;
	loc.page = (vAddr / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK;
    loc.done = true;

    return loc;
}
    
write_location Ftl::round_robin_write_location(uint64_t vAddr)
{
    uint64_t addr_start;
    uint64_t pAddr;
    bool done = false;
    uint64_t block, page, tmp_block, tmp_page;
    write_location loc;

    //look for first free physical page starting at the write pointer
    addr_start = cfg.BLOCKS_PER_PLANE * (plane + cfg.PLANES_PER_DIE * (die + cfg.DIES_PER_PACKAGE * channel));
	    
    // Search from the current write pointer to the end of the flash for a free page.
    for (block = addr_start ; block < TOTAL_SIZE / BLOCK_SIZE && !done; block++)
    {
	for (page = 0 ; page < cfg.PAGES_PER_BLOCK  && !done ; page++)
	{
	    if (!used[block][page] && !dirty[block][page])
	    {
		tmp_block = block;
		tmp_page = page;
		pAddr = (block * BLOCK_SIZE + page * cfg.NV_PAGE_SIZE);
		done = true;
	    }
	}
    }
    block = tmp_block;
    page = tmp_page;
    
    // If we didn't find a free page after scanning to the end. Scan from the beginning
    // to the write pointer
    if (!done)
    {							
	for (block = 0 ; block < start / BLOCK_SIZE && !done; block++)
	{
	    for (page = 0 ; page < cfg.PAGES_PER_BLOCK  && !done; page++)
	    {
		if (!used[block][page] && !dirty[block][page])
		{
		    tmp_block = block;
		    tmp_page = page;
		    pAddr = (block * BLOCK_SIZE + page * cfg.NV_PAGE_SIZE);
		    done = true;
		}
	    }
	}
	block = tmp_block;
	page = tmp_page;
    }

    loc.address = pAddr;
    loc.block = block;
    loc.page = page;
    loc.done = done;
    return loc;
}

// this gap rotation code was copied in several places so it gets its own function
void Ftl::gap_rotate(void)
{
    if(gap == 0)
    {
	gap = ((numBlocks * cfg.PAGES_PER_BLOCK)-1)*cfg.NV_PAGE_SIZE;
	start = start + cfg.NV_PAGE_SIZE;
	if(start >= (numBlocks * cfg.PAGES_PER_BLOCK)-1)
	{
	    start = 0;
	}
    }
    else
    {
	gap = gap - cfg.NV_PAGE_SIZE;
    }

    gap_moving = false;
    gap_moved = true;
}

// we are doing this at the FTL level because the gap can move across the entire physical memory
// space and so cannot be an internal die operation
void Ftl::gap_movement(void)
{
    FlashTransaction trans;
    uint64_t vAddr;
    ChannelPacket *commandPacket;
   
    // reverse address lookup
    if(gap == 0)
    {
	// if the gap has reached the top of the physical memory, it must be moved
	// back to the end, so we need to read whatever data is now at the end so
	// we can eventually copy it into the old gap location
	vAddr = (((numBlocks * cfg.PAGES_PER_BLOCK)-1)*cfg.NV_PAGE_SIZE)-start;
	vAddr = vAddr - cfg.NV_PAGE_SIZE; // because the vAddr will always be greater than the gap here
    }
    else
    {
	// if the gap is somewhere in the middle of the address space then we need to
	// read the data that is at the next gap location
	vAddr = gap-start-cfg.NV_PAGE_SIZE; // vAddr = [GAP-1] (from Start Gap paper)
	if(vAddr >= gap)
	    vAddr = vAddr-cfg.NV_PAGE_SIZE;
    }

    // Check to see if the vAddr exists in the address map.
    if (addressMap.find(vAddr) == addressMap.end())
    {
	// if not then there was nothing to move and we're done here
	gap_rotate();
    }
    // If we do need to move something, issue the appropriate command to the controller
    else
    {
	// schedule a read to pull the current data at the new gap location
	// reusing the GC transaction type to keep stuff simple here since we want to treat this
	// transfer the same way we would the transfers involved in a GC operation
	commandPacket = Ftl::translate(GC_READ, vAddr, addressMap[vAddr]);

	// log this new thing
	if(cfg.LOGGING)
	{
	    // Start the logging for this access.
	    log->access_start(vAddr, GC_DATA_READ);
	}
	
	//send the read to the controller
	bool result = controller->addPacket(commandPacket);
	if(result)
	{
	    // let everyone know that a read for the gap is pending
	    pending_gap_read = true;
	}
	else
	{
	    // Delete the packet if it is not being used to prevent memory leaks.
	    delete commandPacket;
	    ctrl_write_queues_full = true;	
	    log->locked_up(currentClockCycle);
	    gap_read_delayed = true;
	}
    }
}

void Ftl::handle_gap_write(uint64_t write_vAddr)
{
    ChannelPacket *commandPacket, *dataPacket;

    // the virtual address should be the same as the read even though we've moved the gap
    // so we pull the virtual address from the gap read callback
    dataPacket = Ftl::translate(DATA, write_vAddr, gap);
    commandPacket = Ftl::translate(GC_WRITE, write_vAddr, gap);

    // Check to see if there is enough room for both packets in the queue (need two open spots).
    bool queue_open = controller->checkQueueWrite(dataPacket);
    
    // no room so we can't place the write there right now
    if (!queue_open)
    {
	// These packets are not being used. Since they were dynamically allocated, we must delete them to prevent
	// memory leaks.
	delete dataPacket;
	delete commandPacket;
	pending_gap_write = true;
    }
    // had room, place the write
    else if (queue_open)
    {
	// log this new thing
	if(cfg.LOGGING)
	{
	    // Start the logging for this access.
	    log->access_start(write_vAddr, GC_DATA_WRITE);
	}

	// Add the packets to the controller queue.
	// Do not need to check the return values for these since checkQueueWrite() was called.
	controller->addPacket(dataPacket);
	controller->addPacket(commandPacket);
	
	gap_rotate();
    }
}

write_location Ftl::start_gap_write_location(uint64_t vAddr)
{
    uint64_t pAddr, iAddr;
    write_location loc;

    // if address space randomization is being used
    if(cfg.RANDOM_ADDR)
    {
	// have to get the contiguous page number for this vAddr
	uint64_t page_num = vAddr/cfg.NV_PAGE_SIZE;
	uint64_t offset = vAddr%cfg.NV_PAGE_SIZE;
	iAddr = randomAddrs[page_num]; // get our random address for this virtual address
	iAddr = iAddr * cfg.NV_PAGE_SIZE;
	iAddr = iAddr + offset;
    }
    else
    {						
	iAddr = vAddr;
    }

    // if its time to adjust the gap then do so if we're not already taking care of it
    if(write_counter != 0 && write_counter%cfg.GAP_WRITE_INTERVAL == 0 && !gap_moving && !gap_moved)
    {
	gap_moving = true;
	// start the gap movement process
	gap_movement();
    }

    // get the address
    // this will use the old gap placement while gap movement is taking place as the start
    // and gap values are only updated once the data has been safely moved from the new
    // gap location
    pAddr = (iAddr + start) % (((numBlocks * cfg.PAGES_PER_BLOCK)-1)*cfg.NV_PAGE_SIZE);
    if(pAddr >= gap)
    {
	pAddr = pAddr + cfg.NV_PAGE_SIZE;
    }

    loc.address = pAddr;
    loc.block = pAddr / BLOCK_SIZE;
    loc.page = (pAddr / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK;
    loc.done = true;
    return loc;
}

void Ftl::handle_write(bool gc)
{
    uint64_t vAddr = currentTransaction.address, pAddr;
    ChannelPacket *commandPacket, *dataPacket;
    bool done = false;
    uint64_t block, page;
    write_location loc;
    uint64_t itr_count = 0;

    // Mapped is used to indicate to the logger that a write was mapped or unmapped.
    bool mapped = false;

    if (addressMap.find(vAddr) != addressMap.end())
    {
	if(cfg.wearLevelingScheme == DirectTranslation)
	{
	    pAddr = addressMap[vAddr];	
	    block = pAddr / BLOCK_SIZE;
	    page = (pAddr / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK;
	    done = true;
	}
	else
	{
	    write_used_handler(vAddr);
	}
	mapped = true;	
    }

    // find the next location for this write
    if (mapped == false || cfg.wearLevelingScheme != DirectTranslation)
    {
	// calling the appropriate next location scheme
        loc = next_write_location(vAddr);
	// gettting all the useful information
	pAddr = loc.address;
	block = loc.block;
	page = loc.page;
	done = loc.done;
    }
	    
    if (!done)
    {
	deadlock_counter++;
	if(deadlock_counter == deadlock_time)
	{
	    //bad news
	    cout << deadlock_time;
	    ERROR("FLASH DIMM IS COMPLETELY FULL AND DEADLOCKED - If you see this, something has gone horribly wrong.");
	    exit(9001);
	}
    } 
    else 
    {
	// We've found a used page. Now we need to try to add the transaction to the Controller queue.
	
	// first things first, we're no longer in danger of dead locking so reset the counter
	deadlock_counter = 0;
	
	//send write to controller	
	ChannelPacketType write_type;
	// if this page is dirty and there is no garbage collection, we must issue an erase instead of a write
	// this should only occur with PCM as NAND and NOR will have to use GC
	if (!cfg.GARBAGE_COLLECT && dirty[block][page])
	    write_type = SET_WRITE;
	else if (gc)
	    write_type = GC_WRITE;
	else
	    write_type = WRITE;

	dataPacket = Ftl::translate(DATA, vAddr, pAddr);
	commandPacket = Ftl::translate(write_type, vAddr, pAddr);
	
	// Check to see if there is enough room for both packets in the queue (need two open spots).
	bool queue_open = controller->checkQueueWrite(dataPacket);
	
	// no room so we can't place the write there right now
	if (!queue_open)
	{
	    // These packets are not being used. Since they were dynamically allocated, we must delete them to prevent
	    // memory leaks.
	    delete dataPacket;
	    delete commandPacket;
	    
	    ctrl_write_queues_full = true;
	    
	}
	// had room, place the write
	else if (queue_open)
	{
	    // Add the packets to the controller queue.
	    // Do not need to check the return values for these since checkQueueWrite() was called
	    controller->addPacket(dataPacket);
	    controller->addPacket(commandPacket);
	    
	    // made this a function cause the code was repeated a bunch of places
	    write_success(block, page, vAddr, pAddr, gc, mapped);
	    
	    // if we are using the real write pointer, then update it
	    if(itr_count == 0 && cfg.wearLevelingScheme == RoundRobin)
	    {
		//update "write pointer"
		channel = (channel + 1) % cfg.NUM_PACKAGES;
		if (channel == 0){
		    die = (die + 1) % cfg.DIES_PER_PACKAGE;
		    if (die == 0)
			plane = (plane + 1) % cfg.PLANES_PER_DIE;
		}
	    }
	}
    }
}

uint64_t Ftl::get_ptr(void) {
	// Return a pointer to the current plane.
	return cfg.NV_PAGE_SIZE * cfg.PAGES_PER_BLOCK * cfg.BLOCKS_PER_PLANE * 
		(plane + cfg.PLANES_PER_DIE * (die + cfg.NUM_PACKAGES * channel));
}

void Ftl::handle_trim(void)
{
        uint64_t vAddr = currentTransaction.address;

	//safety first
	if(addressMap.find(vAddr) != addressMap.end())
	{
	    // this page has been identified as no longer in use, so mark it as such
	    used[addressMap[vAddr] / BLOCK_SIZE][(addressMap[vAddr] / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK] = false;

	    // however the data on this page is still present so it is now dirty
	    // here we do mark the unused page as dirty so that we can know how long it will take to write to it
	    dirty[addressMap[vAddr] / BLOCK_SIZE][(addressMap[vAddr] / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK] = true;
	    dirty_page_count++;
	}

	// Pop the transaction from the transaction queue.
	popFrontTransaction();
	
	// The FTL is no longer busy.
	busy = 0;
}

void Ftl::handle_preset(void)
{
        uint64_t vAddr = currentTransaction.address;
	ChannelPacket *commandPacket;
	write_location loc;
	
	//safety first
	if(addressMap.find(vAddr) != addressMap.end())
	{
	    loc = next_write_location(vAddr);
	    
	    commandPacket = Ftl::translate(PRESET_WRITE, vAddr, addressMap[vAddr]);
	    //send the read to the controller
	    bool result = controller->addPacket(commandPacket);

	    if(result)
	    {
		// this page is no longer dirty because we have erased it now
		dirty[addressMap[vAddr] / BLOCK_SIZE][(addressMap[vAddr] / cfg.NV_PAGE_SIZE) % cfg.PAGES_PER_BLOCK] = false;
		dirty_page_count--;
		
		// Pop the transaction from the transaction queue.
		popFrontTransaction();
	
		// The FTL is no longer busy.
		busy = 0;
	    }
	    else
	    {
		// Delete the packet if it is not being used to prevent memory leaks.
		delete commandPacket;
		ctrl_write_queues_full = true;	
		log->locked_up(currentClockCycle);
	    }
	}
	else
	{
	    // Pop the transaction from the transaction queue.
	    popFrontTransaction();
	
	    // The FTL is no longer busy.
	    busy = 0;
	}
}

void Ftl::popFrontTransaction()
{
	transQueue.pop_front();
	if(cfg.LOGGING && cfg.QUEUE_EVENT_LOG)
	{
	    log->log_ftl_queue_event(false, &transQueue);
	}
}

void Ftl::powerCallback(void) 
{
    if(cfg.LOGGING)
    {
	vector<vector<double> > temp = log->getEnergyData();
	if(temp.size() == 2)
	{
		controller->returnPowerData(temp[0], temp[1]);
	}
	else if(temp.size() == 3)
	{
		controller->returnPowerData(temp[0], temp[1], temp[2]);
	}
	else if(temp.size() == 4)
	{
		controller->returnPowerData(temp[0], temp[1], temp[2], temp[3]);
	}
	else if(temp.size() == 6)
	{
		controller->returnPowerData(temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
	}
    }
}

void Ftl::sendQueueLength(void)
{
	if(cfg.LOGGING == true)
	{
		log->ftlQueueLength(transQueue.size());
	}
}

// function to set the NVM as more dirty as if it'd been running for a long time
void Ftl::preDirty(void)
{
    if(cfg.PERCENT_FULL > 0 && !dirtied)
    {
	// use virtual blocks here so we don't deadlock the entire NVM
	uint numBlocks = cfg.NUM_PACKAGES * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.VIRTUAL_BLOCKS_PER_PLANE;
	for(uint64_t i = 0; i < numBlocks; i++)
	{
	    for(uint64_t j = 0; j < (cfg.PAGES_PER_BLOCK*(cfg.PERCENT_FULL/100)); j++)
	    {
		dirty[i][j] = true;
		dirty_page_count++;
		used[i][j] = true;
		used_page_count++;
	    }
	}

	dirtied = true;
    }
}
void Ftl::saveNVState(void)
{
     if(cfg.ENABLE_NV_SAVE && !saved)
    {
	ofstream save_file;
	save_file.open(cfg.NV_SAVE_FILE, ios_base::out | ios_base::trunc);
	if(!save_file)
	{
	    cout << "ERROR: Could not open NVDIMM state save file: " << cfg.NV_SAVE_FILE << "\n";
	    abort();
	}
	
	cout << "NVDIMM is saving the used table, dirty table and address map \n";

	// save the address map
	save_file << "AddressMap \n";
	std::unordered_map<uint64_t, uint64_t>::iterator it;
	for (it = addressMap.begin(); it != addressMap.end(); it++)
	{
	    save_file << (*it).first << " " << (*it).second << " \n";
	}

        // save the dirty table
	save_file << "Dirty \n";
	for(uint64_t i = 0; i < dirty.size(); i++)
	{
	    for(uint64_t j = 0; j < dirty[i].size(); j++)
	    {
		save_file << dirty[i][j] << " ";
	    }
	    save_file << "\n";
	}

	// save the used table
	save_file << "Used";
	for(uint64_t i = 0; i < used.size(); i++)
	{
	    save_file << "\n";
	    for(uint64_t j = 0; j < used[i].size()-1; j++)
	    {
		save_file << used[i][j] << " ";
	    }
	    save_file << used[i][used[i].size()-1];
	}

	save_file.close();
	saved = true;
    }
}


void Ftl::loadNVState(void)
{
    if(cfg.ENABLE_NV_RESTORE && !loaded)
    {
	ifstream restore_file;
	restore_file.open(cfg.NV_RESTORE_FILE);
	if(!restore_file)
	{
	    cout << "ERROR: Could not open NVDIMM restore file: " << cfg.NV_RESTORE_FILE << "\n";
	    abort();
	}

	cout << "NVDIMM is restoring the system from file " << cfg.NV_RESTORE_FILE <<"\n";

	// restore the data
	uint64_t doing_used = 0;
	uint64_t doing_dirty = 0;
	uint64_t doing_addresses = 0;
	uint64_t row = 0;
	uint64_t column = 0;
	uint64_t first = 0;
	uint64_t key = 0;
	uint64_t pAddr = 0;
	uint64_t vAddr = 0;

	std::unordered_map<uint64_t,uint64_t> tempMap;

	std::string temp;
	
	while(!restore_file.eof())
	{ 
	    restore_file >> temp;
	    
	    // these comparisons make this parser work but they are dependent on the ordering of the data in the state file
	    // if the state file changes these comparisons may also need to be changed
	    if(temp.compare("Used") == 0)
	    {
		doing_used = 1;
		doing_addresses = 0;
		doing_dirty = 0;
		
		row = 0;
		column = 0;
	    }
	    else if(temp.compare("Dirty") == 0)
	    {
		doing_used = 0;
		doing_dirty = 1;
		doing_addresses = 0;
		
		row = 0;
		column = 0;
	    }
	    else if(temp.compare("AddressMap") == 0)
	    {
		doing_used = 0;
		doing_dirty = 0;
		doing_addresses = 1;

		row = 0;
		column = 0;
	    }
	    // restore used data
	    else if(doing_used == 1)
	    {
		used[row][column] = convert_uint64_t(temp);

                // this page was used need to issue fake write
		if(temp.compare("1") == 0 && dirty[row][column] != 1)
		{
		    pAddr = (row * BLOCK_SIZE + column * cfg.NV_PAGE_SIZE);
		    vAddr = tempMap[pAddr];
		    ChannelPacket *tempPacket = Ftl::translate(FAST_WRITE, vAddr, pAddr);
		    controller->writeToPackage(tempPacket);

		    used_page_count++;
		}		

		column++;
		if(column >= cfg.PAGES_PER_BLOCK)
		{
		    row++;
		    column = 0;
		}
	    }
	    // restore dirty data
	    else if(doing_dirty == 1)
	    {
		dirty[row][column] = convert_uint64_t(temp);
		column++;
		if(column >= cfg.PAGES_PER_BLOCK)
		{
		    row++;
		    column = 0;
		}
	    }
	    // restore address map data
	    else if(doing_addresses == 1)
	    {	
		if(first == 0)
		{
		    first = 1;
		    key = convert_uint64_t(temp);
		}
		else
		{
		    addressMap[key] = convert_uint64_t(temp);
		    tempMap[convert_uint64_t(temp)] = key;
		    first = 0;
		}
	    }   
	}

	restore_file.close();
	loaded = true;
    }
}

void Ftl::queuesNotFull(void)
{
    ctrl_read_queues_full = false;
    ctrl_write_queues_full = false;   
    log->unlocked_up(locked_counter);
    locked_counter = 0;

    // if we had been trying to write the data from the new gap location
    // go ahead and try that again first
    if(pending_gap_write)
    {
	pending_gap_write = false;
	handle_gap_write(gap_write_vAddr);
    }
    
    // if we had been trying to read the data from the new gap location
    // go ahead and try that again first
    if(gap_read_delayed)
    {
	gap_read_delayed = false;
	gap_movement();
    }
}

// handles the gap movement read complete
// once the read is done, we create a write for that data and send it
void Ftl::GCReadDone(uint64_t vAddr)
{   
    if(cfg.wearLevelingScheme == StartGap && pending_gap_read)
    {
	pending_gap_read = false;
	gap_write_vAddr = vAddr;
	handle_gap_write(vAddr);
    }
}
