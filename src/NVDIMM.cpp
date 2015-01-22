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


//NVDIMM.cpp
//Class file for nonvolatile memory dimm system wrapper

#include "NVDIMM.h"
#include "Init.h"

using namespace std;

namespace NVDSim
{
    NVDIMM::NVDIMM(uint64_t id, string iniFile, string pwd, string trc) :
	ini(iniFile),
	cDirectory(pwd)
    {
	uint64_t i, j;
	systemID = id;

	cfg = Init::read(ini);
	
	// this is where the overprovisioning happens
	if(cfg.wearLevelingScheme == StartGap)
	{
	    // we need an extra page(block) for the gap in start gap
	    // we don't use the overprovisioning factor here cause start gap doesn't have a way to use
	    // the extra blocks
	    // we take care of that in Ftl.cpp
	    cfg.BLOCKS_PER_PLANE = (uint64_t)(cfg.VIRTUAL_BLOCKS_PER_PLANE);
	}
	else
	{
	    cfg.BLOCKS_PER_PLANE = (uint64_t)(cfg.VIRTUAL_BLOCKS_PER_PLANE * cfg.PBLOCKS_PER_VBLOCK);
	}

	if(cfg.LOGGING == 1)
	{
	    PRINT("Logs are being generated");
	}else{
	    PRINT("Logs are not being generated");
	}
	PRINT("\nDevice Information:\n");
	PRINT("Device Type: "<<cfg.DEVICE_TYPE);
	uint64_t bits_per_GB = (uint64_t)(1024*1024)*(1024);
	PRINT("Size (GB): "<<TOTAL_SIZE/bits_per_GB);
	PRINT("Block Size: "<<BLOCK_SIZE);
	PRINT("Plane Size: "<<PLANE_SIZE);
	PRINT("Die Size: "<<DIE_SIZE);
	PRINT("Package Size: "<<PACKAGE_SIZE);
	PRINT("Total Size (KB): "<<TOTAL_SIZE);
	PRINT("Packages/Channels: "<<cfg.NUM_PACKAGES);
	PRINT("Page size: "<<cfg.NV_PAGE_SIZE);
	if(cfg.GARBAGE_COLLECT == 1)
	{
	  PRINT("Device is using garbage collection");
	}else{
	  PRINT("Device is not using garbage collection");
	}
	if(cfg.BUFFERED == 1 || cfg.FRONT_BUFFER == 1)
	{
	  PRINT("Memory is using a buffer between the channel and dies");
	}else{
	  PRINT("Memory channels are directly connected to dies");
	}
	
	switch(cfg.wearLevelingScheme)
	{
	case RoundRobin:
	    PRINT("Memory is using a round robin wear leveling scheme");
	    break;
	case StartGap:
	    PRINT("Memory is using the start gap wear leveling scheme");
	    break;
	case DirectTranslation:
	    PRINT("Memory is using a direct address translation wear leveling scheme");
	    break;
	default:
	    break;
	}

	if(cfg.wearLevelingScheme == StartGap && cfg.PAGES_PER_BLOCK != 1)
	{
	    PRINT("WARNING: PCM blocks should consist of just 1 page but that is not the case here");
	}
	
	PRINT("\nTiming Info:\n");
	PRINT("Read time: "<<cfg.READ_TIME);
	PRINT("Write Time: "<<cfg.WRITE_TIME);
	PRINT("Erase time: "<<cfg.ERASE_TIME);
	PRINT("Channel latency for data: "<<cfg.CHANNEL_CYCLE);
	PRINT("Channel width for data: "<<cfg.CHANNEL_WIDTH);
	PRINT("Device latency for data: "<<cfg.DEVICE_CYCLE);
	PRINT("Device width for data: "<<cfg.DEVICE_WIDTH)
	if(USE_EPOCHS == 1)
	{
	    PRINT("Device is using epoch data logging");
	}
	PRINT("Epoch Time: "<<cfg.EPOCH_CYCLES);
	PRINT("");
	
	
	if((cfg.GARBAGE_COLLECT == 0) && (cfg.DEVICE_TYPE.compare("NAND") == 0 || cfg.DEVICE_TYPE.compare("NOR") == 0))
	{
	  ERROR("Device is Flash and must use garbage collection");
	  exit(-1);
	}

	if(cfg.DEVICE_TYPE.compare("P8P") == 0)
	  {
	    if(cfg.ASYNC_READ_I == 0.0)
	      {
		WARNING("No asynchronous read current supplied, using 0.0");
	      }
	    else if(cfg.VPP_STANDBY_I == 0.0)
	       {
		PRINT("VPP standby current data missing for P8P, using 0.0");
	      }
	    else if(cfg.VPP_READ_I == 0.0)
	      {
		PRINT("VPP read current data missing for P8P, using 0.0");
	      }
	    else if(cfg.VPP_WRITE_I == 0.0)
	       {
		PRINT("VPP write current data missing for P8P, using 0.0");
	      }
	    else if(cfg.VPP_ERASE_I == 0.0)
	       {
		PRINT("VPP erase current data missing for P8P, using 0.0");
	      }
	    else if(cfg.VPP == 0.0)
	      {
		PRINT("VPP power data missing for P8P, using 0.0");
	      }
	  }
		
	if(cfg.DEVICE_TYPE.compare("P8P") == 0 && cfg.GARBAGE_COLLECT == 1)
	{
	    // if we're not logging we probably shouldn't even initialize the logger
	    if(cfg.LOGGING)
	    {
		log = new P8PGCLogger(cfg);
	    }
	    else
	    {
		log = NULL;
	    }
	    controller= new Controller(cfg, this, log);
	    ftl = new GCFtl(cfg, controller, log, this);
	}
	else if(cfg.DEVICE_TYPE.compare("P8P") == 0 && cfg.GARBAGE_COLLECT == 0)
	{
	    if(cfg.LOGGING)
	    {
		log = new P8PLogger(cfg);
	    }
	    else
	    {
		log = NULL;
	    }
	    controller= new Controller(cfg, this, log);
	    ftl = new Ftl(cfg, controller, log, this);
	}
	else if(cfg.GARBAGE_COLLECT == 1)
	{
	    if(cfg.LOGGING)
	    {
		log = new GCLogger(cfg);
	    }
	    else
	    {
		log = NULL;
	    }
	    controller= new Controller(cfg, this, log);
	    ftl = new GCFtl(cfg, controller, log, this);
	}
	else
	{
	    if(cfg.LOGGING)
	    {
		log = new Logger(cfg);
	    }
	    else
	    {
		log = NULL;
	    }
	    controller= new Controller(cfg, this, log);
	    ftl = new Ftl(cfg, controller, log, this);
	}
	packages= new vector<Package>();

	if (cfg.DIES_PER_PACKAGE > INT_MAX){
		ERROR("Too many dies.");
		exit(1);
	}

	// sanity checks
	
	if(cfg.DEVICE_DATA_CHANNEL)
	{
		for (i= 0; i < cfg.NUM_PACKAGES; i++){
			Package pack = {new Channel(cfg, i), new Channel(cfg, i), new Buffer(cfg, i), vector<Die *>()};
			//pack.channel= new Channel();
			pack.channel->attachController(controller);
			pack.channel->attachBuffer(pack.buffer);
			pack.buffer->attachChannel(pack.channel);
			pack.data_channel->attachController(controller);
			pack.data_channel->attachBuffer(pack.buffer);
			pack.buffer->attachDataChannel(pack.data_channel);
			for (j= 0; j < cfg.DIES_PER_PACKAGE; j++){
				Die *die= new Die(cfg, this, log, j);
				die->attachToBuffer(pack.buffer);
				pack.buffer->attachDie(die);
				pack.dies.push_back(die);
			}
			packages->push_back(pack);
		}
	}
	else
	{
		for (i= 0; i < cfg.NUM_PACKAGES; i++){
			Package pack = {new Channel(cfg, i), NULL, new Buffer(cfg, i), vector<Die *>()};
			//pack.channel= new Channel();
			pack.channel->attachController(controller);
			pack.channel->attachBuffer(pack.buffer);
			pack.buffer->attachChannel(pack.channel);
			for (j= 0; j < cfg.DIES_PER_PACKAGE; j++){
				Die *die= new Die(cfg, this, log, j);
				die->attachToBuffer(pack.buffer);
				pack.buffer->attachDie(die);
				pack.dies.push_back(die);
			}
			packages->push_back(pack);
		}
	}
	controller->attachPackages(packages);

	frontBuffer = new FrontBuffer(cfg, this, ftl);
	controller->attachFrontBuffer(frontBuffer);

	ReturnReadData= NULL;
	WriteDataDone= NULL;

	epoch_count = 0;
	epoch_cycles = 0;

	numReads= 0;
	numWrites= 0;
	numErases= 0;
	currentClockCycle= 0;
	cycles_left = new uint64_t [cfg.NUM_PACKAGES];
	for(uint64_t h = 0; h < cfg.NUM_PACKAGES; h++){
	    cycles_left[h] = 0;
	}
	// the channel and buffers are running faster than the other parts of the system
	/*if(cfg.CYCLE_TIME > cfg.CHANNEL_CYCLE)
	{
	    channel_cycles_per_cycle = (uint64_t)(((float)cfg.CYCLE_TIME / (float)cfg.CHANNEL_CYCLE) + 0.50f);
	    faster_channel = true;
	}
	else if(cfg.CYCLE_TIME <= cfg.CHANNEL_CYCLE)
	{
	    channel_cycles_per_cycle = (uint64_t)(((float)cfg.CHANNEL_CYCLE / (float)cfg.CYCLE_TIME) + 0.50f);
	    faster_channel = false;
	}
	cout << "the faster cycles computed was: " << channel_cycles_per_cycle << " \n";*/

	// counters for cross clock domain calculations
	system_clock_counter = 0.0;
	nv_clock_counter1 = 0.0;
	nv_clock_counter2 = 0.0;
	controller_clock_counter = 0.0;
	nv_clock_counter3 = new float [cfg.NUM_PACKAGES];
	channel_clock_counter = new float [cfg.NUM_PACKAGES];
	for(uint64_t c = 0; c < cfg.NUM_PACKAGES; c++){
	    nv_clock_counter3[c] = 0.0;
	    channel_clock_counter[c] = 0.0;
	}
	
	ftl->loadNVState();
	
	ftl->preDirty();

	channels = cfg.NUM_PACKAGES;
	dies_per_package = cfg.DIES_PER_PACKAGE;
	planes_per_die = cfg.PLANES_PER_DIE;
	blocks_per_plane = cfg.BLOCKS_PER_PLANE;
	pages_per_block = cfg.PAGES_PER_BLOCK;
	page_size = cfg.NV_PAGE_SIZE;
    }

// static allocator for the library interface
    NVDIMM *getNVDIMMInstance(uint64_t id, string iniFile, string pwd, string trc)
    {
	return new NVDIMM(id, iniFile, pwd, trc);
    }

// outdated static allocator for the library interface to preserve backwards compatability
    NVDIMM *getNVDIMMInstance(uint64_t id, string iniFile, string unused, string pwd, string trc)
    {
	return new NVDIMM(id, iniFile, pwd, trc);
    }

    bool NVDIMM::add(FlashTransaction &trans){
	if(cfg.FRONT_BUFFER)
	{
	    return frontBuffer->addTransaction(trans);
	}
	else
	{
	    return ftl->addTransaction(trans);
	}
    }

    bool NVDIMM::addTransaction(bool isWrite, uint64_t addr){
	TransactionType type = isWrite ? DATA_WRITE : DATA_READ;
	FlashTransaction trans = FlashTransaction(type, addr, NULL);
	if(cfg.FRONT_BUFFER)
	{
	    return frontBuffer->addTransaction(trans);
	}
	else
	{
	    return ftl->addTransaction(trans);
	}
    }

    string NVDIMM::SetOutputFileName(string tracefilename){
	return "";
    }

    void NVDIMM::RegisterCallbacks(Callback_t *readCB, Callback_t *writeCB, Callback_v *Power){
	ReturnReadData = readCB;
	CriticalLineDone = NULL;
	WriteDataDone = writeCB;
	ReturnPowerData = Power;
    }

    void NVDIMM::RegisterCallbacks(Callback_t *readCB,  Callback_t *critLineCB, Callback_t *writeCB, Callback_v *Power)
    {
	ReturnReadData = readCB;
	CriticalLineDone = critLineCB;
	WriteDataDone = writeCB;
	ReturnPowerData = Power;
    }

    void NVDIMM::printStats(void){
	if(cfg.LOGGING == true)
	{
	    log->print(currentClockCycle);
	}
    }

    void NVDIMM::saveStats(void){
	if(cfg.LOGGING == true)
	{
	    log->save(currentClockCycle, epoch_count);
	}
	ftl->saveNVState();
    }

    void NVDIMM::update(void)
    {
	uint64_t i, j;
	Package package;

	//update the system clock counters
	system_clock_counter += cfg.SYSTEM_CYCLE;

	while(nv_clock_counter1 < system_clock_counter)
	{
	    nv_clock_counter1 += cfg.CYCLE_TIME;

	    //cout << "updating ftl \n";
	    ftl->update();
	    ftl->step();
	    
	    if(cfg.BUFFERED)
	    {
		nv_clock_counter2 += cfg.CYCLE_TIME;
		while(controller_clock_counter < nv_clock_counter2)
		{
		    controller_clock_counter += cfg.CHANNEL_CYCLE;
		    controller->update();
		    controller->step();
		}

		if(controller_clock_counter == nv_clock_counter2)
		{
		    nv_clock_counter2 = 0.0;
		    controller_clock_counter = 0.0;
		}
		/*
		if(faster_channel)
		{
		    for(uint64_t c = 0; c < channel_cycles_per_cycle; c++)
		    {
			controller->update();
			controller->step();
		    }
		}
		else
		{
		    // reset the update counter and update the controller
		    if(controller_cycles_left == 0)
		    {
			controller->update();
			controller->step();
			controller_cycles_left = channel_cycles_per_cycle;
		    }
		    
		    controller_cycles_left = controller_cycles_left - 1;
		}
		*/
	    }
	    else if(cfg.FRONT_BUFFER)
	    {
		nv_clock_counter2 += cfg.CYCLE_TIME;
		while(controller_clock_counter < nv_clock_counter2)
		{
		    controller_clock_counter += cfg.CHANNEL_CYCLE;
		    frontBuffer->update();
		    frontBuffer->step();
		}

		if(controller_clock_counter == nv_clock_counter2)
		{
		    nv_clock_counter2 = 0.0;
		    controller_clock_counter = 0.0;
		}

		controller->update();
		controller->step();
	    }
	    else
	    {
		controller->update();
		controller->step();
	    }
	
	    for (i= 0; i < packages->size(); i++){
		package= (*packages)[i];
		if(cfg.BUFFERED)
		{
		    nv_clock_counter3[i] += cfg.CYCLE_TIME;
		    while(channel_clock_counter[i] < nv_clock_counter3[i])
		    {
			channel_clock_counter[i] += cfg.CHANNEL_CYCLE;
			package.channel->update();
			package.buffer->update();
		    }

		    if(channel_clock_counter[i] == nv_clock_counter3[i])
		    {
			channel_clock_counter[i] = 0.0;
			nv_clock_counter3[i] = 0.0;
		    }
		    /*if(faster_channel)
		    {
			for(uint64_t c = 0; c < channel_cycles_per_cycle; c++)
			{
			    package.channel->update();
			    package.buffer->update();
			}
		    }
		    else
		    {
			// reset the update counter and update the channel
			if(cycles_left[i] == 0)
			{
			    package.channel->update();
			    package.buffer->update();
			    cycles_left[i] = channel_cycles_per_cycle;
			}
			
			cycles_left[i] = cycles_left[i] - 1;
		    }*/
		}
		else
		{
		    package.channel->update();
		    package.buffer->update();
		}		
		for (j= 0; j < package.dies.size() ; j++)
		{
			package.dies[j]->update();
			package.dies[j]->step();
		}
	    }

	    if(cfg.LOGGING == true)
	    {
		log->update();
	    }
	    
	    step();
	    
	    //saving stats at the end of each epoch
	    if(USE_EPOCHS)
	    {
		ftl->sendQueueLength();
		controller->sendQueueLength();
		if(epoch_cycles >= cfg.EPOCH_CYCLES)
		{
		    if(cfg.LOGGING == true)
		    {
			log->save_epoch(currentClockCycle, epoch_count);
			log->ftlQueueReset();
			log->ctrlQueueReset();
		    }
		    epoch_count++;
		    epoch_cycles = 0;		
		}
		else
		{
		    epoch_cycles++;
		}
	    }
	}

	if(nv_clock_counter1 == system_clock_counter)
	{
	    nv_clock_counter1 = 0.0;
	    system_clock_counter = 0.0;
	}

	//cout << "NVDIMM successfully updated" << endl;
    }

    void NVDIMM::powerCallback(void){
	ftl->powerCallback();
    }

//If either of these methods are called it is because HybridSim called them
//therefore the appropriate system setting should be set
    void NVDIMM::saveNVState(string filename){
	cfg.ENABLE_NV_SAVE = 1;
	cfg.NV_SAVE_FILE = filename;
	cout << "got to save state in nvdimm \n";
	cout << "save file was " << cfg.NV_SAVE_FILE << "\n";
	ftl->saveNVState();
    }

    void NVDIMM::loadNVState(string filename){
	cfg.ENABLE_NV_RESTORE = 1;
	cfg.NV_RESTORE_FILE = filename;
	ftl->loadNVState();
    }

    void NVDIMM::queuesNotFull(void)
    {
	ftl->queuesNotFull();
    }

    void NVDIMM::GCReadDone(uint64_t vAddr)
    {
	ftl->GCReadDone(vAddr);
    }
}
