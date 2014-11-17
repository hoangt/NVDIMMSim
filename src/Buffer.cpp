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

//Buffer.cpp
//Buffer functions

#include "Buffer.h"

using namespace std;
using namespace NVDSim;

Buffer::Buffer(Configuration &nv_cfg, uint64_t i):
	cfg(nv_cfg)
{
	
	cfg = nv_cfg;
    id = i;

    outData = vector<list <BufferPacket *> >(cfg.DIES_PER_PACKAGE, list<BufferPacket *>());
    inData = vector<list <BufferPacket *> >(cfg.DIES_PER_PACKAGE, list<BufferPacket *>());

    outDataSize = new uint64_t [cfg.DIES_PER_PACKAGE];
    inDataSize = new uint64_t [cfg.DIES_PER_PACKAGE];
    cyclesLeft = new uint64_t [cfg.DIES_PER_PACKAGE];
    outDataLeft = new uint64_t [cfg.DIES_PER_PACKAGE];
    critData = new uint64_t [cfg.DIES_PER_PACKAGE];
    inDataLeft = new uint64_t [cfg.DIES_PER_PACKAGE];
    waiting =  new bool [cfg.DIES_PER_PACKAGE];

    for(uint64_t i = 0; i < cfg.DIES_PER_PACKAGE; i++){
	outDataSize[i] = 0;
	inDataSize[i] = 0;
	cyclesLeft[i] = 0;
	outDataLeft[i] = 0;
	critData[i] = 0;
	inDataLeft[i] = 0;
	waiting[i] = false;
   }

    sendingDie = 0;
    sendingPlane = 0;

    dieLookingUp = cfg.DIES_PER_PACKAGE + 1;
    lookupTimeLeft = BUFFER_LOOKUP_CYCLES;
    

}

void Buffer::attachDie(Die *d){
    dies.push_back(d);
}

void Buffer::attachChannel(Channel *c){
    channel = c;
}

void Buffer::attachDataChannel(Channel *c){
	data_channel = c;
}

void Buffer::sendToDie(ChannelPacket *busPacket){
    dies[busPacket->die]->receiveFromBuffer(busPacket);
}

void Buffer::sendToController(ChannelPacket *busPacket){
    channel->sendToController(busPacket);
}

void Buffer::sendDataToController(ChannelPacket *busPacket){
    data_channel->sendToController(busPacket);
}

bool Buffer::sendPiece(SenderType t, int type, uint64_t die, uint64_t plane){
    if(t == CONTROLLER)
    {
      if(cfg.IN_BUFFER_SIZE == 0 || inDataSize[die] <= (cfg.IN_BUFFER_SIZE-(cfg.CHANNEL_WIDTH)))
	{
	    if(!inData[die].empty() && inData[die].back()->type == type && inData[die].back()->plane == plane &&
	       type == 5 && inData[die].back()->number < (cfg.NV_PAGE_SIZE*8))
	    {
		inData[die].back()->number = inData[die].back()->number + cfg.CHANNEL_WIDTH;
		inDataSize[die] = inDataSize[die] + cfg.CHANNEL_WIDTH;
	    }
	    else if(!inData[die].empty() && inData[die].back()->type == type && inData[die].back()->plane == plane && 
		    type != 5 && inData[die].back()->number < cfg.COMMAND_LENGTH)
	    {
		inData[die].back()->number = inData[die].back()->number + cfg.CHANNEL_WIDTH;
		inDataSize[die] = inDataSize[die] + cfg.CHANNEL_WIDTH;
	    }
	    else
	    {	
		BufferPacket* myPacket = new BufferPacket();
		myPacket->type = type;
		myPacket->number = cfg.CHANNEL_WIDTH;
		myPacket->plane = plane;
		inData[die].push_back(myPacket);
		inDataSize[die] = inDataSize[die] + cfg.CHANNEL_WIDTH;
	    }
	    return true;
	}
	else
	{
	    //cout << "controller sent packet to buffer " << die << " and plane " << plane << " that didn't fit \n";
	    //cout << "packet type was " << type << "\n";
	    return false;
	}
    }
    else if(t == BUFFER)
    {
	if(cfg.OUT_BUFFER_SIZE == 0 || outDataSize[die] <= (cfg.OUT_BUFFER_SIZE-cfg.DEVICE_WIDTH))
	{
	    if(!outData[die].empty() && outData[die].back()->type == type && outData[die].back()->plane == plane &&
	       outData[die].back()->number < (cfg.NV_PAGE_SIZE*8)){
		outData[die].back()->number = outData[die].back()->number + cfg.DEVICE_WIDTH;
		outDataSize[die] = outDataSize[die] + cfg.DEVICE_WIDTH;
		// if ths was the last piece of this packet, tell the die
		if( outData[die].back()->number >= (cfg.NV_PAGE_SIZE*8))
		{
		    dies[die]->bufferLoaded();
		}
	    }else{
		BufferPacket* myPacket = new BufferPacket();
		myPacket->type = type;
		myPacket->number = cfg.DEVICE_WIDTH;
		myPacket->plane = plane;
		outData[die].push_back(myPacket);
		outDataSize[die] = outDataSize[die] + cfg.DEVICE_WIDTH;
	    }
	    return true;
	}
	else
	{
	    return false;
	}
    }
    return false;
}

bool Buffer::isFull(SenderType t, ChannelPacketType bt, uint64_t die)
{
    if(t == CONTROLLER)
    {
      if(cfg.IN_BUFFER_SIZE == 0)
      {
	      return false;
      }
      else if(cfg.CUT_THROUGH && inDataSize[die] <= (cfg.IN_BUFFER_SIZE-cfg.CHANNEL_WIDTH) && waiting[die] == false)
      {
	      return false;
      }
      else if(!cfg.CUT_THROUGH && bt == 5 && inDataSize[die] <= (cfg.IN_BUFFER_SIZE-(divide_params((cfg.NV_PAGE_SIZE*8), cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH)))
      {
	      return false;
      }
      else if(!cfg.CUT_THROUGH && bt != 5 && inDataSize[die] <= (cfg.IN_BUFFER_SIZE-(divide_params(cfg.COMMAND_LENGTH, cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH)))
      {
	      return false;
      }
      else
      {
	      //return false;
	      return true;
      }
    }
    else if(t == BUFFER)
    {
	    if(cfg.OUT_BUFFER_SIZE == 0)
	    {
		    return false;
	    }
	    if(cfg.CUT_THROUGH && outDataSize[die] <= (cfg.OUT_BUFFER_SIZE-cfg.DEVICE_WIDTH))
	    {
		    return false;
	    }
	    else if(!cfg.CUT_THROUGH && outDataSize[die] <= (cfg.OUT_BUFFER_SIZE-(divide_params((cfg.NV_PAGE_SIZE*8), cfg.DEVICE_WIDTH)*cfg.DEVICE_WIDTH)))
	    {
		    return false;
	    }
	    else
	    {
		    return true;
	    }
    }
    return true;
}
	    
void Buffer::update(void){
    for(uint64_t i = 0; i < cfg.DIES_PER_PACKAGE; i++){
	// moving data into a die
	//==================================================================================
	// if we're not already busy writing stuff
	if(!inData[i].empty())
	{
	    // *NOTE* removed check for inDataLeft == inData which i think was to make sure this didn't get called when things where just empty
	    // if it is a command, set the number of beats if we've not set them yet
	    if(inData[i].front()->type != 5)
	    {
		// first time we've dealt with this command so we need to set our values
		if(inDataLeft[i] == 0 && waiting[i] != true && inData[i].front()->number >= cfg.COMMAND_LENGTH)
		{
			if(BUFFER_LOOKUP_CYCLES != 0)
			{
				if(dieLookingUp == cfg.DIES_PER_PACKAGE+1)
				{
					dieLookingUp = i;
					lookupTimeLeft = BUFFER_LOOKUP_CYCLES;
				}
				else if(dieLookingUp == i)
				{
					if(lookupTimeLeft > 0)
					{
						lookupTimeLeft--;
					}
					if(lookupTimeLeft == 0)
					{
						dieLookingUp = cfg.DIES_PER_PACKAGE+1;
						inDataLeft[i] = cfg.COMMAND_LENGTH;
						cyclesLeft[i] = divide_params(cfg.DEVICE_CYCLE,cfg.CYCLE_TIME);
						processInData(i);
					}
				}
			}
			else
			{
				inDataLeft[i] = cfg.COMMAND_LENGTH;
				cyclesLeft[i] = divide_params(cfg.DEVICE_CYCLE,cfg.CYCLE_TIME);
				processInData(i);
			}
		}
		// need to make sure either enough data has been transfered to the buffer to warrant
		// sending out more data or all of the data for this particular packet has already
		// been loaded into the buffer
		else if(inData[i].front()->number >= ((cfg.COMMAND_LENGTH-inDataLeft[i])+cfg.DEVICE_WIDTH) ||
			(inData[i].front()->number >= cfg.COMMAND_LENGTH))
		{
		    processInData(i);
		}
	    }
	    // its not a command but it is the first time we've dealt with this data
	    else if(inDataLeft[i] == 0 && waiting[i] != true)
	    {
		// cut through routing enabled
		// starting the transaction as soon as we have enough data to send one beat
		if(cfg.CUT_THROUGH && inData[i].front()->number >= cfg.DEVICE_WIDTH)
		{
		    inDataLeft[i] = (cfg.NV_PAGE_SIZE*8);
		    cyclesLeft[i] = divide_params(cfg.DEVICE_CYCLE,cfg.CYCLE_TIME);
		    processInData(i);
		}
		// don't do cut through routing
		// wait until we have the whole page before sending
		else if(!cfg.CUT_THROUGH && inData[i].front()->number >= (cfg.NV_PAGE_SIZE*8))
		{
		    inDataLeft[i] = (cfg.NV_PAGE_SIZE*8);
		    cyclesLeft[i] = divide_params(cfg.DEVICE_CYCLE,cfg.CYCLE_TIME);
		    processInData(i);
		}
	    }
	    // its not a command and its not the first time we've seen it but we still need to make sure either
	    // there is enough data to warrant sending out the data or all of the data for this particular packet has already
	    // been loaded into the buffer	    
	    else if (inData[i].front()->number >= (((cfg.NV_PAGE_SIZE*8)-inDataLeft[i])+cfg.DEVICE_WIDTH) ||
		     (inData[i].front()->number >= (cfg.NV_PAGE_SIZE*8)))
	    {
		processInData(i);
	    }
	}    
	    	
	// moving data away from die
	//====================================================================================
	// first scan through to see if we have stuff to send if we're not busy
	if(!outData[i].empty())
	{
	    // we're sending data as quickly as we get it
	    if(cfg.CUT_THROUGH && outData[i].front()->number >= cfg.CHANNEL_WIDTH)
	    {
		prepareOutChannel(i);
	    }
	    // waiting to send data until we have a whole page to send
	    else if(!cfg.CUT_THROUGH && outData[i].front()->number >= (cfg.NV_PAGE_SIZE*8))
	    {
		prepareOutChannel(i);
	    }
	}
    }
}

void Buffer::prepareOutChannel(uint64_t die)
{
    // see if we have control of the channel
    if (channel->hasChannel(BUFFER, id) && sendingDie == die && sendingPlane == outData[die].front()->plane)
    {
	if((outData[die].front()->number >= (((cfg.NV_PAGE_SIZE*8)-outDataLeft[die])+cfg.CHANNEL_WIDTH)) ||
	   (outData[die].front()->number >= (cfg.NV_PAGE_SIZE*8)))
	{
	    processOutData(die);
	}
    }
    // if we don't have the channel, get it
    else if (channel->obtainChannel(id, BUFFER, NULL)){
	outDataLeft[die] = (cfg.NV_PAGE_SIZE*8);
	sendingDie = die;
	sendingPlane = outData[die].front()->plane;
	processOutData(die);
    }
}

void Buffer::processInData(uint64_t die){

    // count down the time it takes for the device to latch the data
    if(cyclesLeft[die] > 0)
    {
	cyclesLeft[die]--;
    }

    // device has finished latching this particular piece of data
    if(cyclesLeft[die] == 0)
    {	    
	// do we have data to send
	if(inDataLeft[die] > 0)
	{
	    // set the device latching cycle for this next piece of data
	    cyclesLeft[die] = divide_params(cfg.DEVICE_CYCLE,cfg.CYCLE_TIME);
	    // subtract this chunk of data from the data we need to send to be done
	    if(inDataLeft[die] >= cfg.DEVICE_WIDTH)
	    {
		//cout << "sending data to die \n";
		inDataLeft[die] = inDataLeft[die] - cfg.DEVICE_WIDTH;
		if(cfg.CUT_THROUGH)
		{
			inDataSize[die] = inDataSize[die] - cfg.DEVICE_WIDTH;
		}
	    }
	    // if we only had a tiny amount left to send just set remaining count to zero
	    // to avoid negative numbers here which break things
	    else
	    {
		    if(cfg.CUT_THROUGH)
		    {
			    inDataSize[die] = inDataSize[die] - inDataLeft[die];
		    }
		inDataLeft[die] = 0;
	    }
	}
    }
    
    // we're done here
    if(inDataLeft[die] == 0)
    {
	//cout << dies[die]->isDieBusy(inData[die].front()->plane) << "\n";
	//cout << inData[die].front()->type << "\n";
	if(dies[die]->isDieBusy(inData[die].front()->plane) == 0 ||
	   (inData[die].front()->type == 5 && dies[die]->isDieBusy(inData[die].front()->plane) == 2) ||
	   (inData[die].front()->type != 5 && dies[die]->isDieBusy(inData[die].front()->plane) == 3))
	{   
	    channel->bufferDone(id, die, inData[die].front()->plane);
	    if(!cfg.CUT_THROUGH)
	    {
		    if(inData[die].front()->type == 5)
		    {
			    if( inDataSize[die] >= (divide_params((cfg.NV_PAGE_SIZE*8), cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH))
			    {
				    inDataSize[die] = inDataSize[die] - (divide_params((cfg.NV_PAGE_SIZE*8), cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH);
			    }
			    else
			    {
				    inDataSize[die] = 0;
			    }
		    }
		    else
		    {
			    if(inDataSize[die] >= (divide_params(cfg.COMMAND_LENGTH, cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH))
			    {
				    inDataSize[die] = inDataSize[die] - (divide_params(cfg.COMMAND_LENGTH, cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH);
			    }
			    else
			    {
				   inDataSize[die] = 0;  
			    }
		    }
	    }
	    inData[die].pop_front();
	    waiting[die] = false;
	}
	else
	{
	    waiting[die] = true;
	}
    }
}

void Buffer::processOutData(uint64_t die){
    // deal with the critical line first stuff first
    if(critData[die] >= 512 && critData[die] < 512+cfg.CHANNEL_WIDTH && channel->notBusy())
    {
	dies[die]->critLineDone();
    }
    
    // got the channel and we have stuff to send so send it
    if(outDataLeft[die] > 0 && channel->notBusy()){
	channel->sendPiece(BUFFER,outData[die].front()->type,die,outData[die].front()->plane);
	
	if(outDataLeft[die] >= cfg.CHANNEL_WIDTH)
	{
	    outDataLeft[die] = outDataLeft[die] - cfg.CHANNEL_WIDTH;
	    if(cfg.CUT_THROUGH)
	    {
		    outDataSize[die] = outDataSize[die] - cfg.CHANNEL_WIDTH;
	    }
	}
	else
	{
	    if(cfg.CUT_THROUGH)
	    {
		    outDataSize[die] = outDataSize[die] - outDataLeft[die];
	    }
	    outDataLeft[die] = 0;
	}
	critData[die] = critData[die] + cfg.CHANNEL_WIDTH;
    }
    
    // we're done here
    if(outDataLeft[die] == 0 && channel->notBusy())
    {
	    if(!cfg.CUT_THROUGH)
	    {
		     if( outDataSize[die] >= (divide_params((cfg.NV_PAGE_SIZE*8), cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH))
		     {
			     outDataSize[die] = outDataSize[die] - (divide_params((cfg.NV_PAGE_SIZE*8), cfg.CHANNEL_WIDTH)*cfg.CHANNEL_WIDTH);
		     }
		     else
		     {
			     outDataSize[die] = 0;
		     }
	    }
	dies[die]->bufferDone(outData[die].front()->plane);
	channel->releaseChannel(BUFFER,id);
	critData[die] = 0;
	outData[die].pop_front();
    }
}

bool Buffer::dataReady(uint64_t die, uint64_t plane)
{					       
    if(!outData[die].empty())
    {
	if(outData[die].front()->type == 0 && outData[die].front()->plane == plane)
	{
	    return true;
	}
	return false;
    }
    return false;
}
