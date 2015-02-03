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

//Controller.cpp
//Class files for controller

#include "Controller.h"
#include "NVDIMM.h"
#include "FrontBuffer.h"

using namespace NVDSim;

Controller::Controller(Configuration &nv_cfg, NVDIMM* parent, Logger* l) :
	cfg(nv_cfg)
{

	cfg = nv_cfg;
	parentNVDIMM = parent;
	log = l;

	channelBeatsLeft = vector<uint64_t>(cfg.NUM_PACKAGES, 0);

	ctrlQueues = vector<list <ChannelPacket *> >(cfg.NUM_PACKAGES, list<ChannelPacket *>());
	outgoingPackets = vector<ChannelPacket *>(cfg.NUM_PACKAGES, 0);

	pendingPackets = vector<list <ChannelPacket *> >(cfg.NUM_PACKAGES, list<ChannelPacket *>());

	channel_writing = new bool [cfg.NUM_PACKAGES]; // used to let up know that we need to send the write command
	channel_write_pointer = vector<list<ChannelPacket*>::iterator>(cfg.NUM_PACKAGES, list<ChannelPacket*>::iterator());
	for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
	{
		channel_write_pointer[i] = ctrlQueues[i].end();
	}

	paused = new bool [cfg.NUM_PACKAGES];
	forced_writing = new bool [cfg.NUM_PACKAGES]; // used when scheduling to avoid descending into FIFO mode
	busy_scoreboard = new bool** [cfg.NUM_PACKAGES]; // I'm not sure if this is better or worse than just calling dieBusy before picking a potential packet
	                                                 // this could be seen as keeping two copies of the same information around but its also a more simplified
	                                                 // version of that information so its useful in the context of this controller
	for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
	{
	    paused[i] = false;
	    forced_writing[i] = false;
	}

	die_counter = 0;
	currentClockCycle = 0;	

	REFRESH_CTRL_PERIOD = 0;

	if(cfg.REFRESH_ENABLE)
	{
		// In real DRAM systems refreshes are divided up into 8192 parts so that the entire rank is refreshed
		// with 8192 refreshes each of which takes some longer refresh time
		// The refresh period takes this value into account so we don't need to worry about how many columns or rows are
		// being refreshed at a time, we can just assume that its been divided into 8k refreshes
		// The number of refreshes required to fully refresh a device is usually specified in its white paper
		// In LPDDR when they do per bank refresh, the number of refreshes is multiplied by the number of banks
		// as a result the period between refreshes becomes shorter (divided by the number of ranks),
		// also the time to complete a refresh is also reduced
		// 
		if(cfg.refreshLevel == PerChannel)
		{
			REFRESH_CTRL_PERIOD = REFRESH_PERIOD_CYCLES * cfg.DIES_PER_PACKAGE;
		}
		else if(cfg.refreshLevel == PerVault)
		{
			REFRESH_CTRL_PERIOD = REFRESH_PERIOD_CYCLES;
		}
		// per bank default
		else
		{
			REFRESH_CTRL_PERIOD = REFRESH_PERIOD_CYCLES / cfg.PLANES_PER_DIE;
		}
		cout << "Refresh level is: " << cfg.refreshLevel << "\n";
		cout << "Refresh control period is: " << REFRESH_CTRL_PERIOD << "\n";
		// staggering the refresh countdowns similar to how DRAMSim2 does it
		stringstream scs;
		uint64_t adjusted_refresh_period;
		scs << ((REFRESH_CTRL_PERIOD)/cfg.NUM_PACKAGES);
		scs >> adjusted_refresh_period;
		cout << "Adjusted refresh period is " << adjusted_refresh_period << "\n";

		for (uint64_t i=0; i < cfg.NUM_PACKAGES; i++)
		{	
			uint64_t temp_refresh = adjusted_refresh_period*(i+1);
			refreshCountdown.push_back((uint) (temp_refresh));
		}

		die_rpointer = vector<uint64_t>(cfg.NUM_PACKAGES, 0);
		refreshing = vector<vector<bool> >(cfg.NUM_PACKAGES, vector<bool>(cfg.DIES_PER_PACKAGE, 0));
	}
}

void Controller::attachPackages(vector<Package> *packages){
	this->packages= packages;
}

void Controller::attachFrontBuffer(FrontBuffer *fb){
    this->front_buffer = fb;
}

void Controller::returnReadData(const FlashTransaction  &trans){
	if(parentNVDIMM->ReturnReadData!=NULL){
	    (*parentNVDIMM->ReturnReadData)(parentNVDIMM->systemID, trans.address, currentClockCycle, true);
	}
	parentNVDIMM->numReads++;
}

void Controller::returnUnmappedData(const FlashTransaction  &trans){
	if(parentNVDIMM->ReturnReadData!=NULL){
	    (*parentNVDIMM->ReturnReadData)(parentNVDIMM->systemID, trans.address, currentClockCycle, false);
	}
	parentNVDIMM->numReads++;
}

void Controller::returnCritLine(ChannelPacket *busPacket){
	if(parentNVDIMM->CriticalLineDone!=NULL){
	    (*parentNVDIMM->CriticalLineDone)(parentNVDIMM->systemID, busPacket->virtualAddress, currentClockCycle, true);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy, vector<double> erase_energy,
		vector<double> vpp_idle_energy, vector<double> vpp_access_energy, vector<double> vpp_erase_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(6, vector<double>(cfg.NUM_PACKAGES, 0.0));
		for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * cfg.VCC;
			power_data[1][i] = access_energy[i] * cfg.VCC;
			power_data[2][i] = erase_energy[i] * cfg.VCC;
			power_data[3][i] = vpp_idle_energy[i] * cfg.VPP;
			power_data[4][i] = vpp_access_energy[i] * cfg.VPP;
			power_data[5][i] = vpp_erase_energy[i] * cfg.VPP;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy, vector<double> vpp_idle_energy,
		vector<double> vpp_access_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(4, vector<double>(cfg.NUM_PACKAGES, 0.0));
		for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * cfg.VCC;
			power_data[1][i] = access_energy[i] * cfg.VCC;
			power_data[2][i] = vpp_idle_energy[i] * cfg.VPP;
			power_data[3][i] = vpp_access_energy[i] * cfg.VPP;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy, vector<double> erase_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(3, vector<double>(cfg.NUM_PACKAGES, 0.0));
		for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * cfg.VCC;
			power_data[1][i] = access_energy[i] * cfg.VCC;
			power_data[2][i] = erase_energy[i] * cfg.VCC;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(2, vector<double>(cfg.NUM_PACKAGES, 0.0));
		for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * cfg.VCC;
			power_data[1][i] = access_energy[i] * cfg.VCC;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::receiveFromChannel(ChannelPacket *busPacket){
	cout << "controller get bus packet back from channel for address " << busPacket->virtualAddress << " on cycle " << currentClockCycle << "\n";
	// READ is now done. Log it and call delete
	if(cfg.LOGGING == true)
	{
		log->access_stop(busPacket->virtualAddress, busPacket->physicalAddress);
	}

	// Put in the returnTransaction queue 
	switch (busPacket->busPacketType)
	{
		case READ:
			returnTransaction.push_back(FlashTransaction(RETURN_DATA, busPacket->virtualAddress, busPacket->data));
			break;
		case GC_READ:
			// Nothing to do.
			break;
		default:
			ERROR("Illegal busPacketType " << busPacket->busPacketType << " in Controller::receiveFromChannel\n");
			abort();
			break;
	}
}

// this is only called on a write as the name suggests
/*bool Controller::checkQueueWrite(ChannelPacket *p)
{
	if ((ctrlQueues[p->package].size() + 1 < cfg.CTRL_QUEUE_LENGTH) || (cfg.CTRL_QUEUE_LENGTH == 0))
		return true;
	else
		return false;
	return false;
	}*/

bool Controller::addPacket(ChannelPacket *p){
	if ((ctrlQueues[p->package].size() < cfg.CTRL_QUEUE_LENGTH) || (cfg.CTRL_QUEUE_LENGTH == 0))
	{
		ctrlQueues[p->package].push_back(p);
		
		if(cfg.LOGGING)
		{
			log->ctrlQueueSingleLength(p->package, ctrlQueues[p->package].size());
			if(cfg.QUEUE_EVENT_LOG)
			{
				switch (p->busPacketType)
				{
				case READ:
				case GC_READ:
				case GC_WRITE:
				case ERASE:
					log->log_ctrl_queue_event(false, p->package, &ctrlQueues[p->package]);
					break;
				case WRITE:
				case DATA:
				case SET_WRITE:
					log->log_ctrl_queue_event(true, p->package, &ctrlQueues[p->package]);
					break;
				case PRESET_WRITE:
					break;
				default:
					ERROR("Illegal busPacketType " << p->busPacketType << " in Controller::addPacket\n");
					break;
				}
			}
		}
		cout << "received controller packet to address " << p->virtualAddress << " on cycle " << currentClockCycle << "\n";
		return true;
	}
	else
	{
	    return false;
	}
}

bool Controller::readdPacket(ChannelPacket *p)
{
	if ((ctrlQueues[p->package].size() < cfg.CTRL_QUEUE_LENGTH) || (cfg.CTRL_QUEUE_LENGTH == 0))
	{
		list<ChannelPacket *>::iterator it;
		it = ctrlQueues[p->package].begin();
		it++;
		ctrlQueues[p->package].insert(it,p);
		return true;
	}
	else
	{
		return false;
	}
}

bool Controller::checkHazards(uint64_t virtualAddress, list<uint64_t> write_addresses)
{
	list<uint64_t>::iterator haz_it;
	for(haz_it = write_addresses.begin(); haz_it != write_addresses.end(); haz_it++)
	{
		if((*haz_it) == virtualAddress)
		{
			return true;
		}
	}
	return false;
}

void Controller::update(void){    
	uint64_t i;	
	//Look through queues and send oldest packets to the appropriate channel
	for (i = 0; i < cfg.NUM_PACKAGES; i++){
		//cout << "channel write pointer for channel " << i << " is starting at " << (*channel_write_pointer[i])->virtualAddress << "\n";
		bool done = 0;
		// used to temporarily skip a refresh command to a dies/planes that is currenty busy so that other commands can be issued to other dies/planes
		bool refresh_skip = false;
		uint64_t die_rskipped = cfg.DIES_PER_PACKAGE+1;
		// used to prevent the scheduling of write commands for data that could not yet be sent
		bool write_skip = false;
		Channel* channel_pointer;
		// iterate throught the queue for this channel each time until we find the oldest thing that we can actually issue
		list<ChannelPacket*>::iterator ctrl_it = ctrlQueues[i].begin();
		// use this iterator to save what we issue when scheduling so we can erase it later
		//list<ChannelPacket*>::iterator temp_it = ctrl_it;
		// make a temporary list of the write addresses to check for hazards
		list<uint64_t> temp_write_addrs;
		while(!done)
		{		       			
			// make sure that we have something to do and we're not already doing something
			if ((ctrl_it != ctrlQueues[i].end() || (cfg.REFRESH_ENABLE && refreshCountdown[i] <= 0 && !refresh_skip)) && outgoingPackets[i]==NULL)
			{				
				//cout << "ctrl it is pointed to a packet of type " << (*ctrl_it)->busPacketType << " with address " << (*ctrl_it)->virtualAddress << "\n";
				// check the status of the die
				// ***** This is the write cancelation and write pausing stuff ************************************************************************************
				if((cfg.WRITE_PAUSING || cfg.WRITE_CANCELATION) && ctrl_it != ctrlQueues[i].end())
				{
					writePausingCancelation(*ctrl_it);
				}
				
				// this part of the code is basically deciding what packet we're going to try this round
				// **********************************
				// if we are simulating refreshes and its time to issue an autorefresh to this
				// die, then do so
				ChannelPacket *potentialPacket;
				if(cfg.REFRESH_ENABLE && refreshCountdown[i] <= 0 && !refresh_skip && !channel_writing[i])
				{
					// autorefresh so we don't care about the address, just the command
					potentialPacket = new ChannelPacket(AUTO_REFRESH, 0, 0, 0, 0, 0, die_rpointer[i], i, NULL);
				}
				// if we previously sent data on this channel, then the next thing we send has to be a write command
				// we try this first on each update, if we can't then we go through the queue and try to issue to other dies
				else if(channel_writing[i] && channel_write_pointer[i] != ctrlQueues[i].end() && !write_skip)
				{
					potentialPacket = *channel_write_pointer[i];
					ctrl_it = channel_write_pointer[i];
				}
				// if we are simulating read around write scheduling, then see where the first read is
				else if(cfg.SCHEDULE)
				{
					// check the thresholds to see if we have to force some write to make room in the queue
					if(!forced_writing[i] && ctrlQueues[i].size() > cfg.WRITE_HIGH_THRESHOLD)
					{
						forced_writing[i] = true;
					}
					else if(forced_writing[i] && ctrlQueues[i].size() < cfg.WRITE_LOW_THRESHOLD)
					{
						forced_writing[i] = false;
					}

					// if the front of the queue is data 
					// and we currently are not at the threshold for issuing writes
					// then we need to look for a read
					if(((*ctrl_it)->busPacketType == WRITE || (*ctrl_it)->busPacketType == GC_WRITE) && !forced_writing[i])
					{
						// save the  current iterator in case there are no reads to find
						list<ChannelPacket*>::iterator ctrl_save = ctrl_it;
						
						bool read_found = false;
						while(!read_found)
						{
							ctrl_it++;					
							
							// are we at the end?
							if(ctrl_it == ctrlQueues[i].end())
							{
								break;
							}
							
							//cout << "checking packet of type " << (*ctrl_it)->busPacketType << " with address " << (*ctrl_it)->virtualAddress << "\n";

							// if this is a write command push its address onto our temp list
							if((*ctrl_it)->busPacketType == WRITE || (*ctrl_it)->busPacketType == GC_WRITE)
							{
								temp_write_addrs.push_back((*ctrl_it)->virtualAddress);
							}			
							// we need to check for data hazards
							else if((*ctrl_it)->busPacketType == READ)
							{
								// this might be way too slow for this, we'll see
								if(!checkHazards((*ctrl_it)->virtualAddress, temp_write_addrs))
								{
									read_found = true;
									break;
								}
							}
						}
						if(read_found)
						{
							//cout << "Found read \n";
							//cout << "Used packet of type " << (*ctrl_it)->busPacketType << " with address " << (*ctrl_it)->virtualAddress << "\n";
							potentialPacket = *ctrl_it;
						}
						else
						{
							//cout << "Used front packet \n";
							//cout << "It was packet of type " << (*ctrl_save)->busPacketType << " with address " << (*ctrl_save)->virtualAddress << "\n";
							potentialPacket = *ctrl_save;
							ctrl_it = ctrl_save;
						}
					}
					// otherwise if the front of the queue is a write then data has already been sent
					// and we should just go through with this (or cancel if we're being that complicated)
					// OR if its not the other two things, then its a read and we're good
					// we could have skipped though and in that case we need to be careful that we don't issue a read that
					// is going to that write location
					else
					{					
						// this might be way too slow for this, we'll see
						if(!checkHazards((*ctrl_it)->virtualAddress, temp_write_addrs))
						{
							//cout << "Front not data so uesd that \n";
							//cout << "It was packet of type " << (*ctrl_it)->busPacketType << " with address " << (*ctrl_it)->virtualAddress << "\n";
							potentialPacket = *ctrl_it;
						}
						else
						{
							ctrl_it++;
							continue;
						}
					}
				}
				// NOT SCHEDULING
				// we have something we can do
				else if(ctrl_it != ctrlQueues[i].end())
				{
					// if our current command is a write then we first need to try to send a data packet
					if((*ctrl_it)->busPacketType == WRITE)
					{
						// create a copy of the write packet but make it a data packet
						potentialPacket = new ChannelPacket(DATA, (*ctrl_it)->virtualAddress, (*ctrl_it)->physicalAddress, (*ctrl_it)->page, (*ctrl_it)->block, 
										    (*ctrl_it)->plane, (*ctrl_it)->die, (*ctrl_it)->package, NULL);
					}
					else
					{
						potentialPacket = *ctrl_it;
					}
				}
				// Can't refresh and don't have anything else to do
				else
				{
					done = true;
					continue;
				}
				
				// figure out which channel we'll be using depending on what we're sending, data or command stuff
				if(cfg.DEVICE_DATA_CHANNEL && potentialPacket->busPacketType == DATA)
				{
					channel_pointer = (*packages)[i].data_channel;
				}
				else
				{
					channel_pointer = (*packages)[i].channel;
				}

				//cout << "the potential packet that we've settled on for channel " << i << " is a " << potentialPacket->busPacketType << "\n";
				
				// this part of the code is sending the packet that the previous section decided on
				// ******************************
				// repeat refresh so don't do anything
				// don't issue commands to dies/planes that have a refresh pending
				if((potentialPacket->busPacketType != AUTO_REFRESH && !(refresh_skip && die_rskipped == potentialPacket->die)) || (allDiesRefreshReady(i) && outgoingPackets[i] == NULL))
				{
					// see if this die is ready for a new transaciton
					// no point in doing any other calculation otherwise
					// both channels point to the same dies so it doesn't matter which channel pointer this is
					PlaneState dieBusy = channel_pointer->buffer->dies[potentialPacket->die]->isDieBusy(potentialPacket->plane, potentialPacket->block); 
					//cout << "the die busy status is " << dieBusy << "\n";
					// should allow us to send write data to a buffer that is currently writing
					if(((potentialPacket->busPacketType == DATA && (dieBusy == CAN_ACCEPT_DATA || dieBusy == FREE)) ||
					    // should allow us to send a write command to a plane that has a loaded cache register 		      
					    ((potentialPacket->busPacketType == WRITE  || potentialPacket->busPacketType == GC_WRITE) && (dieBusy == WAITING || dieBusy == CAN_WRITE)) ||
					    // should allow us to send a read command to a plane that has a free data reg
					    // this allows us to interleave reads so that while one sending data back from the cache
					    // reg, the other can start reading from the array into the data reg
					    ((potentialPacket->busPacketType == READ || potentialPacket->busPacketType == GC_READ) && (dieBusy == CAN_READ || dieBusy == FREE))) ||
					   (cfg.REFRESH_ENABLE && potentialPacket->busPacketType == AUTO_REFRESH && channel_pointer->buffer->dies[potentialPacket->die]->canDieRefresh()))
					{
						// the die can accomodate this operation
						// see if the channel is free
						if (channel_pointer->obtainChannel(0, CONTROLLER, potentialPacket)){
							
							if(cfg.LOGGING && cfg.QUEUE_EVENT_LOG)
							{
								switch (potentialPacket->busPacketType)
								{
								case READ:
								case GC_READ:
								case ERASE:
									log->log_ctrl_queue_event(false, ctrlQueues[i].front()->package, &ctrlQueues[i]);
									break;
								case WRITE:
								case GC_WRITE:
								case SET_WRITE:
								case DATA:
									log->log_ctrl_queue_event(true, ctrlQueues[i].front()->package, &ctrlQueues[i]);
									break;
								case FAST_WRITE:
								case PRESET_WRITE:
									break;
								case AUTO_REFRESH:
								case SELF_REFRESH:
									break;
								}
							}
							
							// we're now sending this packet out so mark it as such
							potentialPacket->busPacketStatus = OUTBOUND;
							outgoingPackets[i] = potentialPacket;							
							if(cfg.REFRESH_ENABLE && potentialPacket->busPacketType == AUTO_REFRESH)
							{
								cout << "issued refresh \n";
								refreshCountdown[i] = REFRESH_CTRL_PERIOD;
								die_rpointer[i]++;
								if(die_rpointer[i] >= cfg.DIES_PER_PACKAGE)
								{
									die_rpointer[i] = 0;
								}
							}
							else if(potentialPacket->busPacketType == DATA)
							{
								cout << "issued a data packet \n";
								channel_write_pointer[i] = ctrl_it;
								//cout << "channel write pointer for channel " << i << " is pointed at " << (*channel_write_pointer[i])->virtualAddress << "\n";
								channel_writing[i] = true;
							}
							else
							{
								ctrlQueues[i].erase(ctrl_it);
								parentNVDIMM->queuesNotFull();
								channel_writing[i] = false;
								channel_write_pointer[i] = ctrlQueues[i].end();
							}
							
							// Calculate the time it takes to send this packet to the devices
							if(cfg.BUFFERED)
							{
								switch (outgoingPackets[i]->busPacketType){
								case DATA:
									// Note: cfg.NV_PAGE_SIZE is multiplied by 8 since the parameter is given in B and this is how many bits
									// are in 1 Byte.
									channelBeatsLeft[i] = divide_params_64b((cfg.NV_PAGE_SIZE*8),cfg.CHANNEL_WIDTH); 
									break;
								default:
									channelBeatsLeft[i] = divide_params_64b(cfg.COMMAND_LENGTH,cfg.CHANNEL_WIDTH);
								break;
								}							
							}
							// unbuffered and front buffered both have the controller directly connected to the devices
							else
							{
								switch (outgoingPackets[i]->busPacketType){
								case DATA:
									// Note: cfg.NV_PAGE_SIZE is multiplied by 8 since the parameter is given in B and this is how many bits
									// are in 1 Byte.
									if(cfg.DEVICE_DATA_CHANNEL)
									{
										// the controller cannot send data to the die faster than the die can receive
										channelBeatsLeft[i] = divide_params_64b(divide_params_64b((cfg.NV_PAGE_SIZE*8),cfg.DEVICE_DATA_WIDTH) * cfg.DEVICE_CYCLE, cfg.CYCLE_TIME); 
										//cout << "channel beats left " << channelBeatsLeft[i] << "\n";
									}
									else
									{
										channelBeatsLeft[i] = divide_params_64b(divide_params_64b((cfg.NV_PAGE_SIZE*8),cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE, cfg.CYCLE_TIME); 
									}
									break;
								default:
									channelBeatsLeft[i] = divide_params_64b(divide_params_64b(cfg.COMMAND_LENGTH,cfg.DEVICE_WIDTH) * cfg.DEVICE_CYCLE, cfg.CYCLE_TIME);
									break;
								}
								
								//DRAM chip bus turn around delay
								if(cfg.CHANNEL_TURN_ENABLE && !channel_pointer->lastOwner(CONTROLLER))
								{
									channelBeatsLeft[i] += CHANNEL_TURN_CYCLES;
								}							
							}
							cout << "issued controller packet of type " << potentialPacket->busPacketType << " to address " << potentialPacket->virtualAddress << " on cycle " << currentClockCycle << "\n";
							done = true;
						}
						// couldn't get the channel 
						else
						{
							// so just move on to the next channel since no pending command will be able to use this channel during this cycle
							done = true;
							// if this was a data packet then we'll recreate it next time and we should delete it here
							if(potentialPacket->busPacketType == DATA)
							{
								delete potentialPacket;
							}
						}
					}
					// die not available
					else
					{
						// see if we just failed to schedule a write packet
						// if so, then say we're skipping writes for now and start looking for
						// something else to do this update
						if(potentialPacket->busPacketType == WRITE && !write_skip)
						{
							write_skip = true;
							ctrl_it = ctrlQueues[i].begin();
							continue;
						}
						else if(potentialPacket->busPacketType != AUTO_REFRESH)
						{
							// try the next command in the queue to see if its die is available
							ctrl_it++;	
							continue;
						}
						else
						{
							done = true;
							if(potentialPacket->busPacketType == DATA)
							{
								delete potentialPacket;
							}
						}	
					}
				}
				// command was a refresh and not all required units are ready for the refresh
				else if(potentialPacket->busPacketType == AUTO_REFRESH)
				{
					// if we're refreshing whole channels then we're done here
					if(cfg.refreshLevel == PerChannel)
					{
						done = true;
					}
					// otherwise we might be able to do something with the other dies
					else
					{
						refresh_skip = true;
						die_rskipped = die_rpointer[i];
						continue;
					}
				}
				// command was not a refresh but we still couldn't issue, try something else
				else if(outgoingPackets[i] == NULL)				
				{
					ctrl_it++;
					if(potentialPacket->busPacketType == DATA)
					{
						delete potentialPacket;
					}
					continue;
				}
				// we actually are doing something already for this thing somehow so we're done
				else
				{
					if(potentialPacket->busPacketType == DATA)
					{
						delete potentialPacket;
					}
					done = true;
				}

			}
			// no pending command for this channel and not time for a refresh
			else
			{
				done = true;
			}
		} 
	}
	
	//Use the buffer code for the NVDIMMS to calculate the actual transfer time
	if(cfg.BUFFERED)
	{	
		uint64_t i;
		//Check for commands/data on a channel. If there is, see if it is done on channel
		for (i= 0; i < outgoingPackets.size(); i++){
			if (outgoingPackets[i] != NULL){
				if(paused[outgoingPackets[i]->package] == true && 
				   !(*packages)[outgoingPackets[i]->package].channel->isBufferFull(CONTROLLER, outgoingPackets[i]->busPacketType, 
												   outgoingPackets[i]->die))
				{
					if ((*packages)[outgoingPackets[i]->package].channel->obtainChannel(0, CONTROLLER, outgoingPackets[i])){
						paused[outgoingPackets[i]->package] = false;
					}
				}
				if ((*packages)[outgoingPackets[i]->package].channel->hasChannel(CONTROLLER, 0) && paused[outgoingPackets[i]->package] == false){
					if (channelBeatsLeft[i] == 0){
						(*packages)[outgoingPackets[i]->package].channel->releaseChannel(CONTROLLER, 0);
						pendingPackets[i].push_back(outgoingPackets[i]);
						outgoingPackets[i] = NULL;
					}else if ((*packages)[outgoingPackets[i]->package].channel->notBusy()){
						if(cfg.CUT_THROUGH)
						{
							if(!(*packages)[outgoingPackets[i]->package].channel->isBufferFull(CONTROLLER, outgoingPackets[i]->busPacketType, 
															   outgoingPackets[i]->die))
							{
								(*packages)[outgoingPackets[i]->package].channel->sendPiece(CONTROLLER, outgoingPackets[i]->busPacketType, 
															    outgoingPackets[i]->die, outgoingPackets[i]->plane);
								channelBeatsLeft[i]--;
							}
							else
							{
								(*packages)[outgoingPackets[i]->package].channel->releaseChannel(CONTROLLER, 0);
								paused[outgoingPackets[i]->package] = true;
							}
						}
						else
						{
							if((outgoingPackets[i]->busPacketType == DATA && channelBeatsLeft[i] == divide_params((cfg.NV_PAGE_SIZE*8),cfg.CHANNEL_WIDTH)) ||
							   (outgoingPackets[i]->busPacketType != DATA && channelBeatsLeft[i] == divide_params(cfg.COMMAND_LENGTH,cfg.CHANNEL_WIDTH)))
							{
								if(!(*packages)[outgoingPackets[i]->package].channel->isBufferFull(CONTROLLER, outgoingPackets[i]->busPacketType, 
																   outgoingPackets[i]->die))
								{
									(*packages)[outgoingPackets[i]->package].channel->sendPiece(CONTROLLER, outgoingPackets[i]->busPacketType, 
																    outgoingPackets[i]->die, outgoingPackets[i]->plane);
									channelBeatsLeft[i]--;
								}
								else
								{
									(*packages)[outgoingPackets[i]->package].channel->releaseChannel(CONTROLLER, 0);
									paused[outgoingPackets[i]->package] = true;
								}
							}
							else
							{
								(*packages)[outgoingPackets[i]->package].channel->sendPiece(CONTROLLER, outgoingPackets[i]->busPacketType, 
															    outgoingPackets[i]->die, outgoingPackets[i]->plane);
								channelBeatsLeft[i]--;					    
							}
						}
					}
				}
			}
		}
		//Directly calculate the expected transfer time 
	}
	else
	{
		// cfg.BUFFERED NOT TRUE CASE...
		uint64_t i;
		Channel* channel_pointer;
		//Check for commands/data on a channel. If there is, see if it is done on channel
		for (i= 0; i < outgoingPackets.size(); i++)
		{
			if (outgoingPackets[i] != NULL)
			{
				if(cfg.DEVICE_DATA_CHANNEL && outgoingPackets[i]->busPacketType == DATA)
				{
					channel_pointer = (*packages)[outgoingPackets[i]->package].data_channel;
				}
				else
				{
					channel_pointer = (*packages)[outgoingPackets[i]->package].channel;
				}
				if (channel_pointer->hasChannel(CONTROLLER, 0)){
					channelBeatsLeft[i]--;
					cout << "we're sending stuff on channel " << i << " and we have " << channelBeatsLeft[i] << " cycles left at " << currentClockCycle << "\n";
					if (channelBeatsLeft[i] == 0){
						if(cfg.REFRESH_ENABLE && cfg.refreshLevel == PerChannel && outgoingPackets[i]->busPacketType == AUTO_REFRESH )
						{
							for(uint64_t d = 0; d < cfg.DIES_PER_PACKAGE; d++)
							{
								ChannelPacket *tempPacket = new ChannelPacket(AUTO_REFRESH, 0, 0, 0, 0, 0, d, outgoingPackets[i]->package, NULL);
								channel_pointer->sendToBuffer(tempPacket);
							}
						}
						else
						{
							channel_pointer->sendToBuffer(outgoingPackets[i]);
						}
						
						channel_pointer->releaseChannel(CONTROLLER, 0);
						outgoingPackets[i] = NULL;
					}
				}
			}
		}
	}

	//See if any read data is ready to return
	while (!returnTransaction.empty())
	{
		if(cfg.FRONT_BUFFER)
		{
			// attempt to add the return transaction to the host channel buffer
			bool return_success = front_buffer->addTransaction(returnTransaction.back());
			if(return_success)
			{
				// attempt was successful so pop the transaction and try again
				returnTransaction.pop_back();
			}
			else
			{
				// attempt failed so stop trying to add things for now
				break;
			}
		}
		else
		{
			//call return callback
			returnReadData(returnTransaction.back());
			returnTransaction.pop_back();
		}
	}
	
	//update the refresh counters
	if(cfg.REFRESH_ENABLE)
	{
		uint64_t i;
		for (i = 0; i < refreshCountdown.size(); i++)
		{
			if(refreshCountdown[i] > 0)
				refreshCountdown[i]--;
		}
	}
}

void Controller::writePausingCancelation(ChannelPacket* front_packet)
{
	PlaneState status = (*packages)[front_packet->package].dies[front_packet->die]->isDieBusy(front_packet->plane, front_packet->block);		   
			
	// if the die/plane is writing and we have a read waiting then see how far into this iteration		    
	if((status == CAN_ACCEPT_DATA || status == PLANE_WRITING) && front_packet->busPacketType == READ)
	{			
		// also make sure that we don't have a read after write hazard here
		if((*packages)[front_packet->package].dies[front_packet->die]->isCurrentPAddr(front_packet->plane, front_packet->block, front_packet->physicalAddress))
		{
			uint writeIterationCyclesLeft = (*packages)[front_packet->package].dies[front_packet->die]->returnWriteIterationCycle(front_packet->plane);	
			
			//if we're far enough into a write iteration, then attempt a pause
			// if this read is accessing the same block, then we need to cancel the write
			if(writeIterationCyclesLeft < (cfg.WRITE_ITERATION_CYCLES/2) && (*packages)[front_packet->package].dies[front_packet->die]->isCurrentBlock(front_packet->plane, front_packet->block) && cfg.WRITE_PAUSING)
			{
				(*packages)[front_packet->package].dies[front_packet->die]->writePause(front_packet->plane);
			}
			// if we've still got a ways to go with this write or we're using the same block, go ahead and cancel it
			else if(cfg.WRITE_CANCELATION)
			{
				(*packages)[front_packet->package].dies[front_packet->die]->writeCancel(front_packet->plane);
			}
		}
	}
}

bool Controller::allDiesRefreshReady(uint64_t package)
{
	if(cfg.REFRESH_ENABLE)
	{
		
		if(cfg.refreshLevel == PerChannel)
		{
			uint64_t free_count = 0;
			for(uint64_t d = 0; d < cfg.DIES_PER_PACKAGE; d++)
			{
				if((*packages)[package].channel->canDieRefresh(d))
					free_count++;
			}
			if(free_count == cfg.DIES_PER_PACKAGE)
			{
				return true;
			}
			else
			{
				//cout << "all dies not free \n";
				//cout << "free count for package " << package << " was " << free_count << "\n";
				return false;
			}
		}
		else if(cfg.refreshLevel == PerBank || cfg.refreshLevel == PerVault)
		{
			return (*packages)[package].channel->canDieRefresh(die_rpointer[package]);
		}
		else
		{
			return false;
		}
	}
	else
	{
		return true;
	}
}

bool Controller::dataReady(uint64_t package, uint64_t die, uint64_t plane)
{
    if(!ctrlQueues[package].empty())
    {
	if(ctrlQueues[package].front()->busPacketType == READ && ctrlQueues[package].front()->plane == plane)
	{
	    return 1;
	}
	return 0;
    }
    return 0;
}

void Controller::sendQueueLength(void)
{
    vector<uint64_t> temp = vector<uint64_t>(cfg.NUM_PACKAGES, 0);
	for(uint64_t i = 0; i < ctrlQueues.size(); i++)
	{
		temp[i] = ctrlQueues[i].size();
	}
	if(cfg.LOGGING == true)
	{
		log->ctrlQueueLength(temp);
	}
}

void Controller::writeToPackage(ChannelPacket *packet)
{
	return (*packages)[packet->package].dies[packet->die]->writeToPlane(packet);
}

void Controller::bufferDone(uint64_t package, uint64_t die, uint64_t plane)
{
	for (uint64_t i = 0; i < pendingPackets.size(); i++){
		std::list<ChannelPacket *>::iterator it;
		for(it = pendingPackets[i].begin(); it != pendingPackets[i].end(); it++){
		    if ((*it) != NULL && (*it)->package == package && (*it)->die == die && (*it)->plane == plane){
				(*packages)[(*it)->package].channel->sendToBuffer((*it));
				pendingPackets[i].erase(it);
				break;
		    }
		}
	}
}

bool Controller::readWaiting(uint64_t package, uint64_t die, uint64_t plane)
{
    return false;
}
