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

using namespace NVDSim;

Controller::Controller(NVDIMM* parent, Logger* l){
	parentNVDIMM = parent;
	log = l;

	channelBeatsLeft = vector<uint>(NUM_PACKAGES, 0);

	readQueues = vector<vector<list <ChannelPacket *> > >(NUM_PACKAGES, vector<list<ChannelPacket *> >(DIES_PER_PACKAGE, list<ChannelPacket * >()));
	writeQueues = vector<vector<list <ChannelPacket *> > >(NUM_PACKAGES, vector<list<ChannelPacket *> >(DIES_PER_PACKAGE, list<ChannelPacket * >()));
	//writeQueues = vector<list <ChannelPacket *> >(DIES_PER_PACKAGE, list<ChannelPacket *>());
	outgoingPackets = vector<ChannelPacket *>(NUM_PACKAGES, 0);

	pendingPackets = vector<list <ChannelPacket *> >(NUM_PACKAGES, list<ChannelPacket *>());

	paused = new bool [NUM_PACKAGES];
	die_pointers = new uint64_t [NUM_PACKAGES];
	for(uint64_t i = 0; i < NUM_PACKAGES; i++)
	{
	    paused[i] = false;
	    die_pointers[i] = 0;
	}

	done = 0;
	die_counter = 0;
	currentClockCycle = 0;
}

void Controller::attachPackages(vector<Package> *packages){
	this->packages= packages;
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
		vector<vector<double>> power_data = vector<vector<double>>(6, vector<double>(NUM_PACKAGES, 0.0));
		for(uint i = 0; i < NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * VCC;
			power_data[1][i] = access_energy[i] * VCC;
			power_data[2][i] = erase_energy[i] * VCC;
			power_data[3][i] = vpp_idle_energy[i] * VPP;
			power_data[4][i] = vpp_access_energy[i] * VPP;
			power_data[5][i] = vpp_erase_energy[i] * VPP;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy, vector<double> vpp_idle_energy,
		vector<double> vpp_access_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(4, vector<double>(NUM_PACKAGES, 0.0));
		for(uint i = 0; i < NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * VCC;
			power_data[1][i] = access_energy[i] * VCC;
			power_data[2][i] = vpp_idle_energy[i] * VPP;
			power_data[3][i] = vpp_access_energy[i] * VPP;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy, vector<double> erase_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(3, vector<double>(NUM_PACKAGES, 0.0));
		for(uint i = 0; i < NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * VCC;
			power_data[1][i] = access_energy[i] * VCC;
			power_data[2][i] = erase_energy[i] * VCC;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::returnPowerData(vector<double> idle_energy, vector<double> access_energy) {
	if(parentNVDIMM->ReturnPowerData!=NULL){
		vector<vector<double>> power_data = vector<vector<double>>(2, vector<double>(NUM_PACKAGES, 0.0));
		for(uint i = 0; i < NUM_PACKAGES; i++)
		{
			power_data[0][i] = idle_energy[i] * VCC;
			power_data[1][i] = access_energy[i] * VCC;
		}
		(*parentNVDIMM->ReturnPowerData)(parentNVDIMM->systemID, power_data, currentClockCycle, false);
	}
}

void Controller::receiveFromChannel(ChannelPacket *busPacket){
	// READ is now done. Log it and call delete
	if(LOGGING == true)
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
    if(CTRL_SCHEDULE)
    {
	if ((writeQueues[p->package][p->die].size() + 1 < CTRL_WRITE_QUEUE_LENGTH) || (CTRL_WRITE_QUEUE_LENGTH == 0))
	    return true;
	else
	    return false;
    }
    else
    {
	if ((readQueues[p->package][p->die].size() + 1 < CTRL_READ_QUEUE_LENGTH) || (CTRL_READ_QUEUE_LENGTH == 0))
	    return true;
	else
	    return false;
    }
}

bool Controller::addPacket(ChannelPacket *p){
    if(CTRL_SCHEDULE)
    {
	// If there is not room in the command queue for this packet, then return false.
	// If CTRL_QUEUE_LENGTH is 0, then infinite queues are allowed.
	switch (p->busPacketType)
	{
	case READ:
        case ERASE:
	    if ((readQueues[p->package][p->die].size() < CTRL_READ_QUEUE_LENGTH) || (CTRL_READ_QUEUE_LENGTH == 0))
		readQueues[p->package][p->die].push_back(p);
	    else	
	        return false;
	    break;
        case WRITE:
        case DATA:
	     if ((writeQueues[p->package][p->die].size() < CTRL_WRITE_QUEUE_LENGTH) || (CTRL_WRITE_QUEUE_LENGTH == 0))
	     {
		 // search the write queue to check if this write overwrites some other write
		 // this should really only happen if we're doing in place writing though (no gc)
		 if(!GARBAGE_COLLECT)
		 {
		     list<ChannelPacket *>::iterator it;
		     for (it = writeQueues[p->package][p->die].begin(); it != writeQueues[p->package][p->die].end(); it++)
		     {
			 if((*it)->virtualAddress == p->virtualAddress && (*it)->busPacketType == p->busPacketType)
			 {
			     if(LOGGING)
			     {		
				 // access_process for that write is called here since its over now.
				 log->access_process((*it)->virtualAddress, (*it)->physicalAddress, (*it)->package, WRITE);
				 
				 // stop_process for that write is called here since its over now.
				 log->access_stop((*it)->virtualAddress, (*it)->physicalAddress);
			     }
			     //call write callback
			     if (parentNVDIMM->WriteDataDone != NULL){
				 (*parentNVDIMM->WriteDataDone)(parentNVDIMM->systemID, (*it)->virtualAddress, currentClockCycle,true);
			     }
			     writeQueues[(*it)->package][(*it)->die].erase(it, it++);
			     break;
			 }
		     }   
		 }
		 writeQueues[p->package][p->die].push_back(p);
		 break;
	     }
	     else
	     {
		 return false;
	     }
	case GC_READ:
	case GC_WRITE:
	    // Try to push the gc stuff to the front of the read queue in order to give them priority
	    if ((readQueues[p->package][p->die].size() < CTRL_READ_QUEUE_LENGTH) || (CTRL_READ_QUEUE_LENGTH == 0))
		readQueues[p->package][p->die].push_front(p);	
	    else
		return false;
	    break;
	default:
	    ERROR("Illegal busPacketType " << p->busPacketType << " in Controller::receiveFromChannel\n");
	    break;
	}
    
	if(LOGGING && QUEUE_EVENT_LOG)
	{
	    switch (p->busPacketType)
	    {
	    case READ:
	    case GC_READ:
	    case GC_WRITE:
	    case ERASE:
		log->log_ctrl_queue_event(false, p->package, &readQueues[p->package][p->die]);
		break;
	    case WRITE:
	    case DATA:
		log->log_ctrl_queue_event(true, p->package, &writeQueues[p->package][p->die]);
		break;
	    default:
		ERROR("Illegal busPacketType " << p->busPacketType << " in Controller::receiveFromChannel\n");
		break;
	    }
	}
	return true;
    }
    // Not scheduling so everything goes to the read queue
    else
    {
	if ((readQueues[p->package][p->die].size() < CTRL_READ_QUEUE_LENGTH) || (CTRL_READ_QUEUE_LENGTH == 0))
	{
	    readQueues[p->package][p->die].push_back(p);
    
	    if(LOGGING && QUEUE_EVENT_LOG)
	    {
		log->log_ctrl_queue_event(false, p->package, &readQueues[p->package][p->die]);
	    }
	    return true;
	}
	else
	{
	    return false;
	}
    }
}

// just cleaning up some of the code
// this was repeated half a dozen times in the code below
bool Controller::nextDie(uint64_t package)
{
    die_pointers[package]++;
    if (die_pointers[package] >= DIES_PER_PACKAGE)
    {
	die_pointers[package] = 0;
    }
    die_counter++;
    // if we loop the number of dies, then we're done
    if (die_counter >= DIES_PER_PACKAGE)
    {
	return 1;
    }
    return 0;
}

void Controller::update(void){
    // schedule the next operation for each die
    if(CTRL_SCHEDULE)
    {
	bool write_queue_handled = false;
	uint64_t i;	
	//loop through the channels to find a packet for each
	for (i = 0; i < NUM_PACKAGES; i++)
	{
	    // loop through the dies per package to find the packet
	    die_counter = 0;
	    done = 0;
	    while (!done)
	    {
		// do we need to issue a write
		// *** NOTE: We need to review this write condition for out new design ***
		if((CTRL_WRITE_ON_QUEUE_SIZE == true && writeQueues[i][die_pointers[i]].size() >= CTRL_WRITE_QUEUE_LIMIT)) //||
		    //(CTRL_WRITE_ON_QUEUE_SIZE == false && writeQueues[i][die_pointers[i]].size() >= CTRL_WRITE_QUEUE_LENGTH-1))
		{
		    if (!writeQueues[i][die_pointers[i]].empty() && outgoingPackets[i]==NULL){
			done = beginWriteSend(i);
		    }
		    // this queue is empty, move on
		    else
		    {
			done = nextDie(i);
		    }
		}
		// if we don't have to issue a write check to see if there is a read to send
		// we're reusing the same die pointer for reads and writes because if a die was just given a read or write
		// it can't do anything else with it so the die counters sort've act like a dies in use marker
		else if (!readQueues[i][die_pointers[i]].empty() && outgoingPackets[i]==NULL){
		    if(queue_access_counter == 0 && readQueues[i][die_pointers[i]].front()->busPacketType != GC_READ && 
		       readQueues[i][die_pointers[i]].front()->busPacketType != GC_WRITE && !writeQueues[i][die_pointers[i]].empty())
		    {
			//see if this read can be satisfied by something in the write queue
			list<ChannelPacket *>::iterator it;
			for (it = writeQueues[i][die_pointers[i]].begin(); it != writeQueues[i][die_pointers[i]].end(); it++)
			{
			    if((*it)->virtualAddress == readQueues[i][die_pointers[i]].front()->virtualAddress)
			    {
				if(LOGGING)
				{		
				    // access_process for the read we're satisfying  is called here since we're doing it here.
				    log->access_process(readQueues[i][die_pointers[i]].front()->virtualAddress, readQueues[i][die_pointers[i]].front()->physicalAddress, 
							readQueues[i][die_pointers[i]].front()->package, READ);
				}
				queue_access_counter = QUEUE_ACCESS_TIME;
				write_queue_handled = true;
				break;
				// done for now with this channel and die but don't advance the die counter
				// cause we have to get through the queue access counter for it
				done = 1;
			    }
			}
		    }
		    else if(queue_access_counter > 0)
		    {
			queue_access_counter--;
			write_queue_handled = true;
			if(queue_access_counter == 0)
			{
			    if(LOGGING)
			    {
				// stop_process for this read is called here since this ends now.
				log->access_stop(readQueues[i][die_pointers[i]].front()->virtualAddress, readQueues[i][die_pointers[i]].front()->virtualAddress);
			    }
			    
			    returnReadData(FlashTransaction(RETURN_DATA, readQueues[i][die_pointers[i]].front()->virtualAddress, readQueues[i][die_pointers[i]].front()->data));
			    readQueues[i][die_pointers[i]].pop_front();
			    parentNVDIMM->queuesNotFull();
			    // managed to place something so we're done with this channel
			    // advance the die pointer since this die is now busy
			    die_pointers[i]++;
			    if (die_pointers[i] >= DIES_PER_PACKAGE)
			    {
				die_pointers[i] = 0;
			    }
			    done = 1;
			}
		    }
		    else if(!write_queue_handled)
		    {
			done = beginReadSend(i);
		    }
		}
		// if there are no reads to send see if we're allowed to send a write instead
		else if (CTRL_IDLE_WRITE == true && !writeQueues[i][die_pointers[i]].empty() && outgoingPackets[i]==NULL){
		    //if we can get the channel
		    done = beginWriteSend(i);
		}
		// queue was empty, move on
		else
		{
		    done = nextDie(i);
		}
	    }
	}
    }
    // not scheduling so everything comes from the read queue
    else
    {
	uint64_t i;	
	//Look through queues and send oldest packets to the appropriate channel
	for (i = 0; i < NUM_PACKAGES; i++){
	    // loop through the dies per package to find the packet
	    die_counter = 0;
	    done = 0;
	    while (!done)
	    {
		if (!readQueues[i][die_pointers[i]].empty() && outgoingPackets[i]==NULL){
		    //if we can get the channel
		    if (getChannel(i, readQueues[i][die_pointers[i]].front())){
			outgoingPackets[i] = readQueues[i][die_pointers[i]].front();
			if(LOGGING && QUEUE_EVENT_LOG)
			{
			    switch (readQueues[i][die_pointers[i]].front()->busPacketType)
			    {
			    case READ:
			    case GC_READ:
			    case ERASE:
				log->log_ctrl_queue_event(false, readQueues[i][die_pointers[i]].front()->package, &readQueues[i][die_pointers[i]]);
				break;
			    case WRITE:
			    case GC_WRITE:
			    case DATA:
				log->log_ctrl_queue_event(true, readQueues[i][die_pointers[i]].front()->package, &readQueues[i][die_pointers[i]]);
				break;
			    case FAST_WRITE:
				break;
			    }
			}
			readQueues[i][die_pointers[i]].pop_front();
			parentNVDIMM->queuesNotFull();
			switch (outgoingPackets[i]->busPacketType){
			case DATA:
			    // Note: NV_PAGE_SIZE is multiplied by 8192 since the parameter is given in KB and this is how many bits
			    // are in 1 KB (1024 * 8).
			    channelBeatsLeft[i] = divide_params((NV_PAGE_SIZE*8192), getChannelWidth(outgoingPackets[i]->busPacketType)); 
			    break;
			default:
			    channelBeatsLeft[i] = divide_params(COMMAND_LENGTH, getChannelWidth(outgoingPackets[i]->busPacketType));
			    break;
			}
			// managed to place something so we're done with this channel
			// advance the die pointer since this die is now busy
			die_pointers[i]++;
			if (die_pointers[i] >= DIES_PER_PACKAGE)
			{
			    die_pointers[i] = 0;
			}
			done = 1;
		    }
		    // couldn't get the channel so... Next die
		    else
		    {
			done = nextDie(i);
		    }
		}
		// this queue is empty so move on
		else
		{
		    done = nextDie(i);
		}
	    }
	}
    }
	
    //Use the buffer code for the NVDIMMS to calculate the actual transfer time
    if(BUFFERED)
    {	
	uint64_t i;
	//Check for commands/data on a channel. If there is, see if it is done on channel
	for (i= 0; i < outgoingPackets.size(); i++){
	    if (outgoingPackets[i] != NULL){
		if(paused[outgoingPackets[i]->package] == true && 
		   !checkBuffer(i, outgoingPackets[i]->busPacketType))
		{
		    if (getChannel(i, outgoingPackets[i])){
			paused[outgoingPackets[i]->package] = false;
		    }
		}
		if (checkChannel(i, outgoingPackets[i]->busPacketType) && paused[outgoingPackets[i]->package] == false){
		    if (channelBeatsLeft[i] == 0){
			releaseChannel(i, outgoingPackets[i]->busPacketType);
			pendingPackets[i].push_back(outgoingPackets[i]);
			outgoingPackets[i] = NULL;
		    }else if (checkBusy(i, outgoingPackets[i]->busPacketType)){
			    if(CUT_THROUGH)
			    {
				if(!checkBuffer(i, outgoingPackets[i]->busPacketType))
				    {
					sendPiece(i, outgoingPackets[i]->busPacketType);
					channelBeatsLeft[i]--;
				    }
				    else
				    {
					releaseChannel(i, outgoingPackets[i]->busPacketType);
					    paused[outgoingPackets[i]->package] = true;
				    }
			    }
			    else
			    {
				if((outgoingPackets[i]->busPacketType == DATA && 
				    channelBeatsLeft[i] == divide_params((NV_PAGE_SIZE*8192), getChannelWidth(outgoingPackets[i]->busPacketType))) ||
				   (outgoingPackets[i]->busPacketType != DATA && 
				    channelBeatsLeft[i] == divide_params(COMMAND_LENGTH, getChannelWidth(outgoingPackets[i]->busPacketType))))
				{
				    if(!checkBuffer(i, outgoingPackets[i]->busPacketType))
				    {
					sendPiece(i, outgoingPackets[i]->busPacketType);
					channelBeatsLeft[i]--;
				    }
				    else
				    {
					releaseChannel(i, outgoingPackets[i]->busPacketType);
					paused[outgoingPackets[i]->package] = true;
				    }
				}
				else
				{
				    sendPiece(i, outgoingPackets[i]->busPacketType);
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
	// BUFFERED NOT TRUE CASE...
	// Cannot support different channels without a buffer so no different channel checks here
	uint64_t i;
	//Check for commands/data on a channel. If there is, see if it is done on channel
	for (i= 0; i < outgoingPackets.size(); i++){
	    if (outgoingPackets[i] != NULL && (*packages)[outgoingPackets[i]->package].channel->hasChannel(CONTROLLER, 0)){
		channelBeatsLeft[i]--;
		if (channelBeatsLeft[i] == 0){
		    (*packages)[outgoingPackets[i]->package].channel->sendToBuffer(outgoingPackets[i]);
		    (*packages)[outgoingPackets[i]->package].channel->releaseChannel(CONTROLLER, 0);
		    outgoingPackets[i] = NULL;
		}
	    }
	}
    }

    //See if any read data is ready to return
    while (!returnTransaction.empty()){
	//call return callback
	returnReadData(returnTransaction.back());
	returnTransaction.pop_back();
    }
}

bool Controller::dataReady(uint64_t package, uint64_t die, uint64_t plane)
{
    if(!readQueues[package][die].empty())
    {
	if(readQueues[package][die].front()->busPacketType == READ && readQueues[package][die].front()->plane == plane)
	{
	    return 1;
	}
	return 0;
    }
    return 0;
}

bool Controller::getChannel(uint64_t package, ChannelPacket *p)
{
    if(p->busPacketType == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[package].requestChan->obtainChannel(0, CONTROLLER, REQUEST_DATA, p);
	}
	else
	{
	    return (*packages)[package].channel->obtainChannel(0, CONTROLLER, RESPONSE_DATA, p);
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	    return (*packages)[package].addrChan->obtainChannel(0, CONTROLLER, ADDR, p);
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[package].requestChan->obtainChannel(0, CONTROLLER, REQUEST_DATA, p);
	}
	else
	{
	    return (*packages)[package].channel->obtainChannel(0, CONTROLLER, RESPONSE_DATA, p);
	}
    }
    return 0;
}

bool Controller::beginWriteSend(uint64_t package)
{
    if (getChannel(package, writeQueues[package][die_pointers[package]].front())){
	outgoingPackets[package] = writeQueues[package][die_pointers[package]].front();
	if(LOGGING && QUEUE_EVENT_LOG)
	{
	    log->log_ctrl_queue_event(true, writeQueues[package][die_pointers[package]].front()->package, &writeQueues[package][die_pointers[package]]);
	}
	writeQueues[package][die_pointers[package]].pop_front();
	parentNVDIMM->queuesNotFull();
	
	switch (outgoingPackets[package]->busPacketType){
	case DATA:
	    // Note: NV_PAGE_SIZE is multiplied by 8192 since the parameter is given in KB and this is how many bits
	    // are in 1 KB (1024 * 8).
	    channelBeatsLeft[package] = divide_params((NV_PAGE_SIZE*8192), getChannelWidth(outgoingPackets[package]->busPacketType)); 
	    break;
	default:
	    channelBeatsLeft[package] = divide_params(COMMAND_LENGTH, getChannelWidth(outgoingPackets[package]->busPacketType));
	    break;
	}
	// managed to place something so we're done with this channel
	// advance the die pointer since this die is now busy
	die_pointers[package]++;
	if (die_pointers[package] >= DIES_PER_PACKAGE)
	{
	    die_pointers[package] = 0;
	}
	return 1;
    }
    // couldn't get the channel so go to the next die
    else
    {
	return nextDie(package);
    }
}

bool Controller::beginReadSend(uint64_t package)
{
    if (getChannel(package, readQueues[package][die_pointers[package]].front())){
	outgoingPackets[package] = readQueues[package][die_pointers[package]].front();
	if(LOGGING && QUEUE_EVENT_LOG)
	{
	    log->log_ctrl_queue_event(false, readQueues[package][die_pointers[package]].front()->package, &readQueues[package][die_pointers[package]]);
	}
	readQueues[package][die_pointers[package]].pop_front();
	parentNVDIMM->queuesNotFull();
	
	channelBeatsLeft[package] = divide_params(COMMAND_LENGTH, getChannelWidth(outgoingPackets[package]->busPacketType));
	// managed to place something so we're done with this channel
	// advance the die pointer since this die is now busy
	die_pointers[package]++;
	if (die_pointers[package] >= DIES_PER_PACKAGE)
	{
	    die_pointers[package] = 0;
	}
	return 1;
    }
    // couldn't get the channel so go to the next die
    else
    {
	return nextDie(package);
    }
}

bool Controller::checkBuffer(uint64_t packet, ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->isBufferFull(CONTROLLER, outgoingPackets[packet]->busPacketType, outgoingPackets[packet]->die);
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->isBufferFull(CONTROLLER, outgoingPackets[packet]->busPacketType, outgoingPackets[packet]->die);
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	     return (*packages)[outgoingPackets[packet]->package].addrChan->isBufferFull(CONTROLLER, outgoingPackets[packet]->busPacketType, outgoingPackets[packet]->die);
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->isBufferFull(CONTROLLER, outgoingPackets[packet]->busPacketType, outgoingPackets[packet]->die);
	}
	else
	{
	     return (*packages)[outgoingPackets[packet]->package].channel->isBufferFull(CONTROLLER, outgoingPackets[packet]->busPacketType, outgoingPackets[packet]->die);
	}
    }
    return 0;
}

bool Controller::checkChannel(uint64_t packet, ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->hasChannel(CONTROLLER, 0);
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->hasChannel(CONTROLLER, 0);
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	     return (*packages)[outgoingPackets[packet]->package].addrChan->hasChannel(CONTROLLER, 0);
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->hasChannel(CONTROLLER, 0);
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->hasChannel(CONTROLLER, 0);
	}
    }
    return 0;
}

bool Controller::checkBusy(uint64_t packet, ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->notBusy();
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->notBusy();
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	     return (*packages)[outgoingPackets[packet]->package].addrChan->notBusy();
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->notBusy();
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->notBusy();
	}
    }
    return 0;
}

bool Controller::releaseChannel(uint64_t packet, ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->releaseChannel(CONTROLLER, 0);
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->releaseChannel(CONTROLLER, 0);
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	     return (*packages)[outgoingPackets[packet]->package].addrChan->releaseChannel(CONTROLLER, 0);
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return (*packages)[outgoingPackets[packet]->package].requestChan->releaseChannel(CONTROLLER, 0);
	}
	else
	{
	    return (*packages)[outgoingPackets[packet]->package].channel->releaseChannel(CONTROLLER, 0);
	}
    }
    return 0;
}

void Controller::sendPiece(uint64_t packet, ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    (*packages)[outgoingPackets[packet]->package].requestChan->sendPiece(CONTROLLER, outgoingPackets[packet]->busPacketType, 
									    outgoingPackets[packet]->die, outgoingPackets[packet]->plane);
	}
	else
	{
	    (*packages)[outgoingPackets[packet]->package].channel->sendPiece(CONTROLLER, outgoingPackets[packet]->busPacketType, 
									outgoingPackets[packet]->die, outgoingPackets[packet]->plane);
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	    (*packages)[outgoingPackets[packet]->package].addrChan->sendPiece(CONTROLLER, outgoingPackets[packet]->busPacketType, 
									 outgoingPackets[packet]->die, outgoingPackets[packet]->plane);
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    (*packages)[outgoingPackets[packet]->package].requestChan->sendPiece(CONTROLLER, outgoingPackets[packet]->busPacketType, 
									    outgoingPackets[packet]->die, outgoingPackets[packet]->plane);
	}
	else
	{
	    (*packages)[outgoingPackets[packet]->package].channel->sendPiece(CONTROLLER, outgoingPackets[packet]->busPacketType, 
									outgoingPackets[packet]->die, outgoingPackets[packet]->plane);
	}
    }
}

uint64_t Controller::getChannelWidth(ChannelPacketType type)
{
    if(type == DATA)
    {
	if(ENABLE_REQUEST_CHANNEL)
	{
	    return REQUEST_CHANNEL_WIDTH;
	}
	else
	{
	    return CHANNEL_WIDTH;
	}
    }
    // commands
    else
    {
	if(ENABLE_ADDR_CHANNEL)
	{
	    return ADDR_CHANNEL_WIDTH;
	}
	else if(ENABLE_REQUEST_CHANNEL)
	{
	    return REQUEST_CHANNEL_WIDTH;
	}
	else
	{
	    return CHANNEL_WIDTH;
	}
    }
    return 0;
}

void Controller::sendQueueLength(void)
{
	vector<uint64_t> temp = vector<uint64_t>(writeQueues.size(),0);
	for(uint i = 0; i < writeQueues.size(); i++)
	{
		temp[i] = writeQueues[i].size();
	}
	if(LOGGING == true)
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
	for (uint i = 0; i < pendingPackets.size(); i++){
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
