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

	paused = new bool [cfg.NUM_PACKAGES];
	for(uint64_t i = 0; i < cfg.NUM_PACKAGES; i++)
	{
	    paused[i] = false;
	}

	die_counter = 0;
	currentClockCycle = 0;

	REFRESH_CTRL_PERIOD = 0;

	if(cfg.REFRESH_ENABLE)
	{
		// TO DO: Make sure that this math is valid with non-Flash-like hierarchies
		// refreshing all the banks on a channel simultaneously so we don't have
		// to issue as many refreshes
		if(cfg.refreshLevel == PerChannel)
		{
			REFRESH_CTRL_PERIOD = cfg.REFRESH_PERIOD / (cfg.PAGES_PER_BLOCK * cfg.BLOCKS_PER_PLANE);
		}
		else if(cfg.refreshLevel == PerVault)
		{
			REFRESH_CTRL_PERIOD = cfg.REFRESH_PERIOD / (cfg.PAGES_PER_BLOCK * cfg.BLOCKS_PER_PLANE * cfg.PLANES_PER_DIE);
		}
		// per bank default
		else
		{
			REFRESH_CTRL_PERIOD = cfg.REFRESH_PERIOD / (cfg.PAGES_PER_BLOCK * cfg.BLOCKS_PER_PLANE * cfg.PLANES_PER_DIE * cfg.DIES_PER_PACKAGE);
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

	// Delete the ChannelPacket since READ is done. This must be done to prevent memory leaks.
	delete busPacket;
}

// this is only called on a write as the name suggests
bool Controller::checkQueueWrite(ChannelPacket *p)
{
	if ((ctrlQueues[p->package].size() + 1 < cfg.CTRL_QUEUE_LENGTH) || (cfg.CTRL_QUEUE_LENGTH == 0))
		return true;
	else
		return false;
	return false;
}

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

void Controller::update(void){    
	uint64_t i;	
	//Look through queues and send oldest packets to the appropriate channel
	for (i = 0; i < cfg.NUM_PACKAGES; i++){
		bool done = 0;
		// used to temporarily skip a refresh command to a die that is currenty busy so that other commands can be issued to other dies
		bool refresh_skip = false;
		// used to prevent the scheduling of write commands for data that could not yet be sent
		bool data_skip = false;
		Channel* channel_pointer;
		// iterate throught the queue for this channel each time until we find the oldest thing that we can actually issue
		list<ChannelPacket*>::iterator ctrl_it = ctrlQueues[i].begin();
		while(!done)
		{
			if(ctrl_it != ctrlQueues[i].end())
			{
				// see if we need to skip the current command because its a write whose data couldn't be sent
				if(((*ctrl_it)->busPacketType == WRITE || (*ctrl_it)->busPacketType == GC_WRITE) && data_skip)
				{
					// just advance the iterator in this case to skip the write
					ctrl_it++;
					// reset data skip so it doesn't mess up some later date / write pair
					data_skip = false;
				}
			}
			
			if ((ctrl_it != ctrlQueues[i].end() || (cfg.REFRESH_ENABLE && refreshCountdown[i] <= 0 && !refresh_skip)) && outgoingPackets[i]==NULL)
			{
				// check the status of the die
				// ***** This is the write cancelation and write pausing stuff ************************************************************************************
				if((cfg.WRITE_PAUSING || cfg.WRITE_CANCELATION) && ctrl_it != ctrlQueues[i].end())
				{
					writePausingCancelation(*ctrl_it);
				}
				
				// if we are simulating refreshes and its time to issue an autorefresh to this
				// die, then do so
				ChannelPacket *potentialPacket;
				if(cfg.REFRESH_ENABLE && refreshCountdown[i] <= 0 && !refresh_skip)
				{
					if(!ctrlQueues[i].empty())
					{
						// this prevents a refersh from coming between a write and data packet
						if(ctrlQueues[i].front()->busPacketType == WRITE)
						{
							potentialPacket = *ctrl_it;
						}
						else
						{
							// autorefresh so we don't care about the address, just the command
							potentialPacket = new ChannelPacket(AUTO_REFRESH, 0, 0, 0, 0, 0, die_rpointer[i], i, NULL);
						}
					}
					else
					{
						// autorefresh so we don't care about the address, just the command
						potentialPacket = new ChannelPacket(AUTO_REFRESH, 0, 0, 0, 0, 0, die_rpointer[i], i, NULL);
					}
				}
				else
				{
					potentialPacket = *ctrl_it;
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
				
				// repeat refresh so don't do anything
				if(potentialPacket->busPacketType != AUTO_REFRESH || (allDiesRefreshReady(i) && outgoingPackets[i] == NULL))
				{
					// see if this die is ready for a new transaciton
					// no point in doing any other calculation otherwise
					// both channels point to the same dies so it doesn't matter which channel pointer this is
					int dieBusy = channel_pointer->buffer->dies[potentialPacket->die]->isDieBusy(potentialPacket->plane); 
					if(((dieBusy == 0) ||
					    // should allow us to send write data to a buffer that is currently writing
					    (potentialPacket->busPacketType == DATA && dieBusy == 2) ||
					    // should allow us to send a write command to a plane that has a loaded cache register 		      
					    ((potentialPacket->busPacketType == WRITE  || potentialPacket->busPacketType == GC_WRITE) && (dieBusy == 3 || dieBusy == 5)) ||
					    // should allow us to send a read command to a plane that has a free data reg
					    // this allows us to interleave reads so that while one sending data back from the cache
					    // reg, the other can start reading from the array into the data reg
					    ((potentialPacket->busPacketType == READ || potentialPacket->busPacketType == GC_READ) && (dieBusy == 5))) ||
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
							
							outgoingPackets[i] = potentialPacket;
							if(cfg.REFRESH_ENABLE && potentialPacket->busPacketType == AUTO_REFRESH)
							{
								refreshCountdown[i] = REFRESH_CTRL_PERIOD;
							}
							else
							{
								ctrlQueues[i].erase(ctrl_it);
								parentNVDIMM->queuesNotFull();
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
							done = true;;
						}
						// couldn't get the channel 
						else
						{
							// so just move on to the next channel since no pending command will be able to use this channel during this cycle
							done = true;
						}
					}
					// die not available
					else
					{
						// see if we just failed to schedule a data packet
						// if so make sure we don't schedule its write without it
						if((*ctrl_it)->busPacketType == DATA)
							data_skip = true;
						// try the next command in the queue to see if its die is available
						ctrl_it++;
						continue;
					}
				}
				// command was a refresh and not all required units are ready for the refresh
				else
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
						continue;
					}
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
	while (!returnTransaction.empty()){
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
	int status = (*packages)[front_packet->package].dies[front_packet->die]->isDieBusy(front_packet->plane);		   
			
	// if the die/plane is writing and we have a read waiting then see how far into this iteration		    
	if((status == 2 || status == 6) && front_packet->busPacketType == READ)
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
