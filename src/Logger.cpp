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

#include "Logger.h"

using namespace NVDSim;
using namespace std;

Logger::Logger()
{
    	num_accesses = 0;
	num_reads = 0;
	num_writes = 0;

	num_unmapped = 0;
	num_mapped = 0;

	num_read_unmapped = 0;
	num_read_mapped = 0;
	num_write_unmapped = 0;
	num_write_mapped = 0;

	average_latency = 0;
	average_read_latency = 0;
	average_write_latency = 0;
	average_queue_latency = 0;

	ftl_queue_length = 0;
	ctrl_queue_length = vector<uint64_t>(NUM_PACKAGES, 0);

	max_ftl_queue_length = 0;
	max_ctrl_queue_length = vector<uint64_t>(NUM_PACKAGES, 0);

	if(PLANE_STATE_LOG)
	{
	    first_state_log = true;
	
	    plane_states = new PlaneStateType **[NUM_PACKAGES];
	    for(uint i = 0; i < NUM_PACKAGES; i++){
		plane_states[i] = new PlaneStateType *[DIES_PER_PACKAGE];
		for(uint j = 0; j < DIES_PER_PACKAGE; j++){
		    plane_states[i][j] = new PlaneStateType[PLANES_PER_DIE];
		    for(uint k = 0; k < PLANES_PER_DIE; k++){
			plane_states[i][j][k] = IDLE;
		    }
		}
	    }
	}

	if(QUEUE_EVENT_LOG)
	{
	    first_ftl_read_log = true;
	    first_ftl_write_log = true;
	    first_ctrl_read_log = new bool [NUM_PACKAGES];
	    first_crtl_write_log = new bool [NUM_PACKAGES];
	    
	    for(uint i = 0; i < NUM_PACKAGES; i++){
		first_ctrl_read_log[i] = true;
		first_crtl_write_log[i] = true;
	    }
	}

	idle_energy = vector<double>(NUM_PACKAGES, 0.0); 
	access_energy = vector<double>(NUM_PACKAGES, 0.0);        
}

void Logger::update()
{
    	//update idle energy
	//since this is already subtracted from the access energies we just do it every time
	for(uint i = 0; i < (NUM_PACKAGES); i++)
	{
	  idle_energy[i] += STANDBY_I;
	}

	this->step();
}

void Logger::access_start(uint64_t addr)
{
	access_queue.push_back(pair <uint64_t, uint64_t>(addr, currentClockCycle));
}

// Using virtual addresses here right now
void Logger::access_process(uint64_t addr, uint64_t paddr, uint package, ChannelPacketType op)
{
        // Get entry off of the access_queue.
	uint64_t start_cycle = 0;
	bool found = false;
	list<pair <uint64_t, uint64_t>>::iterator it;
	for (it = access_queue.begin(); it != access_queue.end(); it++)
	{
		uint64_t cur_addr = (*it).first;
		uint64_t cur_cycle = (*it).second;

		if (cur_addr == addr)
		{
			start_cycle = cur_cycle;
			found = true;
			access_queue.erase(it);
			break;
		}
	}

	if (!found)
	{
		cerr << "ERROR: NVLogger.access_process() called with address not in the access_queue. address=0x" << hex << addr << "\n" << dec;
		abort();
	}

	if (op == READ || op == GC_READ)
	{
	    if(access_map[addr][paddr].size() > 2)
	    {
		cerr << "ERROR: NVLogger.access_process() called for a read with more than one entry already in access_map. address=0x" << hex << addr << "\n" << dec;
		abort();
	    }
	}
	else
	{
	    if(access_map[addr][paddr].size() != 0)
	    {
		cerr << "ERROR: NVLogger.access_process() called with address already in access_map. address=0x" << hex << addr << "\n" << dec;
		abort();
	    }
	}

	AccessMapEntry a;
	a.start = start_cycle;
	a.op = op;
	a.process = this->currentClockCycle;
	a.pAddr = paddr;
	a.package = package;
	//access_map[addr] = std::unordered_map<uint64_t, AccessMapEntry>();
	access_map[addr][paddr].push_back(a);
	//cout << "access map now has size " << access_map[addr][paddr].size() << "\n";
	
	this->queue_latency(a.process - a.start);
}

void Logger::access_stop(uint64_t addr, uint64_t paddr)
{
        if (access_map[addr][paddr].empty())
	{
		cerr << "ERROR: NVLogger.access_stop() called with address not in access_map. address=" << hex << addr << "\n" << dec;
		abort();
	}

	AccessMapEntry a = access_map[addr][paddr].front();
	a.stop = this->currentClockCycle;
	access_map[addr][paddr].front() = a;

	// Log cache event type.
	if (a.op == READ)
	{
	    //update access energy figures
	    access_energy[a.package] += (READ_I - STANDBY_I) * READ_TIME/2;
	    this->read();
	    this->read_latency(a.stop - a.start);
	}	         
	else
	{
	    //update access energy figures
	    access_energy[a.package] += (WRITE_I - STANDBY_I) * WRITE_TIME/2;
	    this->write();    
	    this->write_latency(a.stop - a.start);
	    if(WEAR_LEVEL_LOG)
	    {
		if(writes_per_address.count(a.pAddr) == 0)
		{
		    writes_per_address[a.pAddr] = 1;
		}
		else
		{
		    writes_per_address[a.pAddr]++;
		}
	    }
	}
	
	access_map[addr][paddr].pop_front();
	if(access_map[addr][paddr].empty())
	{
	    access_map[addr].erase(paddr);
	    if(access_map.count(addr) == 0)
	    {
		access_map.erase(addr);
	    }
	}
}

void Logger::log_ftl_queue_event(bool write, std::list<FlashTransaction> *queue)
{
    if(!write)
    {
	if(first_ftl_read_log == true)
	{
	    string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	    const char * command = command_str.c_str();
	    int sys_done = system(command);
	    if (sys_done != 0)
	    {
		WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	    }
	    savefile.open(LOG_DIR+"FtlReadQueue.log", ios_base::out | ios_base::trunc);
	    savefile<<"FTL Read Queue Log \n";
	    first_ftl_read_log = false;
	}
	else
	{
	    savefile.open(LOG_DIR+"FtlReadQueue.log", ios_base::out | ios_base::app);
	}
    }
    else if(write)
    {
	if(first_ftl_write_log == true)
	{
	    string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	    const char * command = command_str.c_str();
	    int sys_done = system(command);
	    if (sys_done != 0)
	    {
		WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	    }
	    savefile.open(LOG_DIR+"FtlWriteQueue.log", ios_base::out | ios_base::trunc);
	    savefile<<"FTL Write Queue Log \n";
	    first_ftl_write_log = false;
	}
	else
	{
	    savefile.open(LOG_DIR+"FtlWriteQueue.log", ios_base::out | ios_base::app);
	}
    }

    savefile<<"Clock cycle: "<<currentClockCycle<<"\n";
    std::list<FlashTransaction>::iterator it;
    for (it = queue->begin(); it != queue->end(); it++)
    {
	savefile<<"Address: "<<(*it).address<<", Transaction Type: "<<(*it).transactionType<<"\n";
    }
    savefile<<"\n";
    
    savefile.close();
}

void Logger::log_ctrl_queue_event(bool write, uint64_t number, std::list<ChannelPacket*> *queue)
{
    if(!write)
    {					       
	std::string file = "CtrlReadQueue";
        std::stringstream temp;
	temp << number;
	file += temp.str();
        file += ".log";
	if(first_ctrl_read_log[number] == true)
	{
	    string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	    const char * command = command_str.c_str();
	    int sys_done = system(command);
	    if (sys_done != 0)
	    {
		WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	    }
	    savefile.open(LOG_DIR+file, ios_base::out | ios_base::trunc);
	    savefile<<"Controller Read Queue " << number << " Log \n";
	    first_ctrl_read_log[number] = false;
	}
	else
	{
	    savefile.open(LOG_DIR+file, ios_base::out | ios_base::app);
	}
    }
    else if(write)
    {
	std::string file = "CtrlWriteQueue";
	std::stringstream temp;
	temp << number;
	file += temp.str();
	file += ".log";
	if(first_crtl_write_log[number] == true)
	{
	    string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	    const char * command = command_str.c_str();
	    int sys_done = system(command);
	    if (sys_done != 0)
	    {
		WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	    }
	    savefile.open(LOG_DIR+file, ios_base::out | ios_base::trunc);
	    savefile<<"Controller Write Queue " << number << " Log \n";
	    first_crtl_write_log[number] = false;
	}
	else
	{
	    savefile.open(LOG_DIR+file, ios_base::out | ios_base::app);
	}
    }

    savefile<<"Clock cycle: "<<currentClockCycle<<"\n";
    std::list<ChannelPacket*>::iterator it;
    for (it = queue->begin(); it != queue->end(); it++)
    {
	savefile<<"Address: "<<(*it)->virtualAddress<<", Transaction Type: "<<(*it)->busPacketType<<"\n";
    }
    savefile<<"\n";
    
    savefile.close();
}

void Logger::log_plane_state(uint64_t package, uint64_t die, uint64_t plane, PlaneStateType op)
{
    plane_states[package][die][plane] = op;
    
    if(first_state_log == 0)
    {
	string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	const char * command = command_str.c_str();
	int sys_done = system(command);
	if (sys_done != 0)
	{
	    WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	}
	savefile.open(LOG_DIR+"PlaneState.log", ios_base::out | ios_base::trunc);
	savefile<<"Plane State Log \n";
	first_state_log = 1;
    }
    else
    {
	savefile.open(LOG_DIR+"PlaneState.log", ios_base::out | ios_base::app);
    }

    savefile<<"Clock cycle: "<<currentClockCycle<<"\n";
    for(uint i = 0; i < NUM_PACKAGES; i++){
	for(uint j = 0; j < DIES_PER_PACKAGE; j++){
	    for(uint k = 0; k < PLANES_PER_DIE; k++){
		savefile<<plane_states[i][j][k]<<" ";
	    }
	    savefile<<"   ";
	}
	savefile<<"\n";
    }
    savefile<<"\n";

    savefile.close();
}

void Logger::read()
{
	num_accesses += 1;
	num_reads += 1;
}

void Logger::write()
{
	num_accesses += 1;
	num_writes += 1;
}

void Logger::mapped()
{
	num_mapped += 1;
}

void Logger::unmapped()
{
	num_unmapped += 1;
}

void Logger::read_mapped()
{
	this->mapped();
	num_read_mapped += 1;
}

void Logger::read_unmapped()
{
	this->unmapped();
	num_read_unmapped += 1;
}

void Logger::write_mapped()
{
	this->mapped();
	num_write_mapped += 1;
}

void Logger::write_unmapped()
{
	this->unmapped();
	num_write_unmapped += 1;
}

void Logger::read_latency(uint64_t cycles)
{
    average_read_latency += cycles;
}

void Logger::write_latency(uint64_t cycles)
{
    average_write_latency += cycles;
}

void Logger::queue_latency(uint64_t cycles)
{
    average_queue_latency += cycles;
}

double Logger::unmapped_rate()
{
    return (double)num_unmapped / num_accesses;
}

double Logger::read_unmapped_rate()
{
    return (double)num_read_unmapped / num_reads;
}

double Logger::write_unmapped_rate()
{
    return (double)num_write_unmapped / num_writes;
}

double Logger::calc_throughput(uint64_t cycles, uint64_t accesses)
{
    if(cycles != 0)
    {
	return ((((double)accesses / (double)cycles) * (1.0/(CYCLE_TIME * 0.000000001)) * NV_PAGE_SIZE));
    }
    else
    {
	return 0;
    }
}

double Logger::divide(double num, double denom)
{
    if(denom == 0)
    {
	return 0.0;
    }
    else
    {
	return (num / denom);
    }
}

void Logger::ftlQueueLength(uint64_t length)
{
    if(length > ftl_queue_length){
	ftl_queue_length = length;
    }

    if(length > max_ftl_queue_length){
	max_ftl_queue_length = length;
    }
}

void Logger::ftlQueueLength(uint64_t length, uint64_t length2)
{
    if(length > ftl_queue_length){
	ftl_queue_length = length;
    }

    if(length > max_ftl_queue_length){
	max_ftl_queue_length = length;
    }
}

void Logger::ctrlQueueLength(vector<uint64_t> length)
{
    for(uint i = 0; i < length.size(); i++)
    {
	if(length[i] > ctrl_queue_length[i])
	{
	    ctrl_queue_length[i] = length[i];
	}
	if(length[i] > max_ctrl_queue_length[i])
	{
	    max_ctrl_queue_length[i] = length[i];
	}
    }
}

void Logger::ftlQueueReset()
{
    ftl_queue_length = 0;
}

void Logger::ctrlQueueReset()
{
    for(uint i = 0; i < ctrl_queue_length.size(); i++)
    {
	ctrl_queue_length[i] = 0;
    }
}

void Logger::save(uint64_t cycle, uint epoch) 
{
        // Power stuff
	// Total power used
	vector<double> total_energy = vector<double>(NUM_PACKAGES, 0.0);
	
        // Average power used
	vector<double> ave_idle_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> ave_access_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> average_power = vector<double>(NUM_PACKAGES, 0.0);

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    if(cycle != 0)
	    {
		total_energy[i] = (idle_energy[i] + access_energy[i]) * VCC;
		ave_idle_power[i] = (idle_energy[i] * VCC) / cycle;
		ave_access_power[i] = (access_energy[i] * VCC) / cycle;
		average_power[i] = total_energy[i] / cycle;
	    }
	    else
	    {
		total_energy[i] = 0;
		ave_idle_power[i] = 0;
		ave_access_power[i] = 0;
		average_power[i] = 0;
	    }
	}

	string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	const char * command = command_str.c_str();
	int sys_done = system(command);
	if (sys_done != 0)
	{
	    WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	}
	savefile.open(LOG_DIR+"NVDIMM.log", ios_base::out | ios_base::trunc);
	savefile<<"NVDIMM Log \n";

	if (!savefile) 
	{
	    ERROR("Cannot open NVDIMM.log");
	    exit(-1); 
	}

	savefile<<"\nData for Full Simulation: \n";
	savefile<<"===========================\n";
	savefile<<"\nAccess Data: \n";
	savefile<<"========================\n";	    
	savefile<<"Cycles Simulated: "<<cycle<<"\n";
	savefile<<"Accesses completed: "<<num_accesses<<"\n";
	savefile<<"Reads completed: "<<num_reads<<"\n";
	savefile<<"Writes completed: "<<num_writes<<"\n";
	savefile<<"Number of Unmapped Accesses: " <<num_unmapped<<"\n";
	savefile<<"Number of Mapped Accesses: " <<num_mapped<<"\n";
	savefile<<"Number of Unmapped Reads: " <<num_read_unmapped<<"\n";
	savefile<<"Number of Mapped Reads: " <<num_read_mapped<<"\n";
	savefile<<"Number of Unmapped Writes: " <<num_write_unmapped<<"\n";
	savefile<<"Number of Mapped Writes: " <<num_write_mapped<<"\n";
	savefile<<"Unmapped Rate: " <<unmapped_rate()<<"\n";
	savefile<<"Read Unmapped Rate: " <<read_unmapped_rate()<<"\n";
	savefile<<"Write Unmapped Rate: " <<write_unmapped_rate()<<"\n";

	savefile<<"\nThroughput and Latency Data: \n";
	savefile<<"========================\n";
	savefile<<"Average Read Latency: " <<(divide((float)average_read_latency,(float)num_reads))<<" cycles";
	savefile<<" (" <<(divide((float)average_read_latency,(float)num_reads)*CYCLE_TIME)<<" ns)\n";
	savefile<<"Average Write Latency: " <<divide((float)average_write_latency,(float)num_writes)<<" cycles";
	savefile<<" (" <<(divide((float)average_write_latency,(float)num_writes))*CYCLE_TIME<<" ns)\n";
	savefile<<"Average Queue Latency: " <<divide((float)average_queue_latency,(float)num_accesses)<<" cycles";
	savefile<<" (" <<(divide((float)average_queue_latency,(float)num_accesses))*CYCLE_TIME<<" ns)\n";
	savefile<<"Total Throughput: " <<this->calc_throughput(cycle, num_accesses)<<" KB/sec\n";
	savefile<<"Read Throughput: " <<this->calc_throughput(cycle, num_reads)<<" KB/sec\n";
	savefile<<"Write Throughput: " <<this->calc_throughput(cycle, num_writes)<<" KB/sec\n";

	savefile<<"\nQueue Length Data: \n";
	savefile<<"========================\n";
	savefile<<"Maximum Length of Ftl Queue: " <<max_ftl_queue_length<<"\n";
	for(uint i = 0; i < max_ctrl_queue_length.size(); i++)
	{
	    savefile<<"Maximum Length of Controller Queue for Package " << i << ": "<<max_ctrl_queue_length[i]<<"\n";
	}

	if(WEAR_LEVEL_LOG)
	{
	    savefile<<"\nWrite Frequency Data: \n";
	    savefile<<"========================\n";
	    unordered_map<uint64_t, uint64_t>::iterator it;
	    for (it = writes_per_address.begin(); it != writes_per_address.end(); it++)
	    {
		savefile<<"Address "<<(*it).first<<": "<<(*it).second<<" writes\n";
	    }
	}

	savefile<<"\nPower Data: \n";
	savefile<<"========================\n";

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    savefile<<"Package: "<<i<<"\n";
	    savefile<<"Accumulated Idle Energy: "<<(idle_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<" mJ\n";
	    savefile<<"Accumulated Access Energy: "<<(access_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<" mJ\n";
	    savefile<<"Total Energy: "<<(total_energy[i] * (CYCLE_TIME * 0.000000001))<<" mJ\n\n";
	 
	    savefile<<"Average Idle Power: "<<ave_idle_power[i]<<" mW\n";
	    savefile<<"Average Access Power: "<<ave_access_power[i]<<" mW\n";
	    savefile<<"Average Power: "<<average_power[i]<<" mW\n\n";
	}

	savefile<<"\n=================================================\n";

	savefile.close();

	if(USE_EPOCHS && !RUNTIME_WRITE)
	{
	    list<EpochEntry>::iterator it;
	    for (it = epoch_queue.begin(); it != epoch_queue.end(); it++)
	    {
		write_epoch(&(*it));
	    }
	}
}

void Logger::print(uint64_t cycle) 
{
        // Power stuff
	// Total power used
	vector<double> total_energy = vector<double>(NUM_PACKAGES, 0.0);
	
        // Average power used
	vector<double> ave_idle_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> ave_access_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> average_power = vector<double>(NUM_PACKAGES, 0.0);

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    total_energy[i] = (idle_energy[i] + access_energy[i]) * VCC;
	    ave_idle_power[i] = (idle_energy[i] * VCC) / cycle;
	    ave_access_power[i] = (access_energy[i] * VCC) / cycle;
	    average_power[i] = total_energy[i] / cycle;
	}

	cout<<"Reads completed: "<<num_reads<<"\n";
	cout<<"Writes completed: "<<num_writes<<"\n";

	cout<<"\nPower Data: \n";
	cout<<"========================\n";

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    cout<<"Package: "<<i<<"\n";
	    cout<<"Accumulated Idle Energy: "<<(idle_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<"mJ\n";
	    cout<<"Accumulated Access Energy: "<<(access_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<"mJ\n";
	    cout<<"Total Energy: "<<(total_energy[i] * (CYCLE_TIME * 0.000000001))<<"mJ\n\n";
	 
	    cout<<"Average Idle Power: "<<ave_idle_power[i]<<"mW\n";
	    cout<<"Average Access Power: "<<ave_access_power[i]<<"mW\n";
	    cout<<"Average Power: "<<average_power[i]<<"mW\n\n";
	}
}

vector<vector<double> > Logger::getEnergyData(void)
{
    vector<vector<double> > temp = vector<vector<double> >(2, vector<double>(NUM_PACKAGES, 0.0));
    for(uint i = 0; i < NUM_PACKAGES; i++)
    {
	temp[0][i] = idle_energy[i];
	temp[1][i] = access_energy[i];
    }
    return temp;
}

void Logger::save_epoch(uint64_t cycle, uint epoch)
{
    EpochEntry this_epoch;
    this_epoch.cycle = cycle;
    this_epoch.epoch = epoch;

    this_epoch.num_accesses = num_accesses;
    this_epoch.num_reads = num_reads;
    this_epoch.num_writes = num_writes;
	
    this_epoch.num_unmapped = num_unmapped;
    this_epoch.num_mapped = num_mapped;

    this_epoch.num_read_unmapped = num_read_unmapped;
    this_epoch.num_read_mapped = num_read_mapped;
    this_epoch.num_write_unmapped = num_write_unmapped;
    this_epoch.num_write_mapped = num_write_mapped;
		
    this_epoch.average_latency = average_latency;
    this_epoch.average_read_latency = average_read_latency;
    this_epoch.average_write_latency = average_write_latency;
    this_epoch.average_queue_latency = average_queue_latency;

    this_epoch.ftl_queue_length = ftl_queue_length;

    this_epoch.writes_per_address = writes_per_address;

    for(uint i = 0; i < ctrl_queue_length.size(); i++)
    {
	this_epoch.ctrl_queue_length[i] = ctrl_queue_length[i];
    }

    for(uint i = 0; i < NUM_PACKAGES; i++)
    {	
	this_epoch.idle_energy[i] = idle_energy[i]; 
	this_epoch.access_energy[i] = access_energy[i]; 
    }

    EpochEntry temp_epoch;

    temp_epoch = this_epoch;

    if(epoch > 0)
    {
	this_epoch.cycle -= last_epoch.cycle;
	
	this_epoch.num_accesses -= last_epoch.num_accesses;
	this_epoch.num_reads -= last_epoch.num_reads;
	this_epoch.num_writes -= last_epoch.num_writes;
	
	this_epoch.num_unmapped -= last_epoch.num_unmapped;
	this_epoch.num_mapped -= last_epoch.num_mapped;
	
	this_epoch.num_read_unmapped -= last_epoch.num_read_unmapped;
	this_epoch.num_read_mapped -= last_epoch.num_read_mapped;
	this_epoch.num_write_unmapped -= last_epoch.num_write_unmapped;
	this_epoch.num_write_mapped -= last_epoch.num_write_mapped;
	
	this_epoch.average_latency -= last_epoch.average_latency;
	this_epoch.average_read_latency -= last_epoch.average_read_latency;
	this_epoch.average_write_latency -= last_epoch.average_write_latency;
	this_epoch.average_queue_latency -= last_epoch.average_queue_latency;
	
	for(uint i = 0; i < NUM_PACKAGES; i++)
	{	
	    this_epoch.idle_energy[i] -= last_epoch.idle_energy[i]; 
	    this_epoch.access_energy[i] -= last_epoch.access_energy[i]; 
	}
    }
    
    if(RUNTIME_WRITE)
    {
	write_epoch(&this_epoch);
    }
    else
    {
	epoch_queue.push_front(this_epoch);
    }

    last_epoch = temp_epoch;
}

void Logger::write_epoch(EpochEntry *e)
{
    	if(e->epoch == 0 && RUNTIME_WRITE)
	{
	    string command_str = "test -e "+LOG_DIR+" || mkdir "+LOG_DIR;
	    const char * command = command_str.c_str();
	    int sys_done = system(command);
	    if (sys_done != 0)
	    {
		WARNING("Something might have gone wrong when nvdimm attempted to makes its log directory");
	    }
	    savefile.open(LOG_DIR+"NVDIMM_EPOCH.log", ios_base::out | ios_base::trunc);
	    savefile<<"NVDIMM_EPOCH Log \n";
	}
	else
	{
	    savefile.open(LOG_DIR+"NVDIMM_EPOCH.log", ios_base::out | ios_base::app);
	}

	if (!savefile) 
	{
	    ERROR("Cannot open PowerStats.log");
	    exit(-1); 
	}

	// Power stuff
	// Total power used
	vector<double> total_energy = vector<double>(NUM_PACKAGES, 0.0);
	
        // Average power used
	vector<double> ave_idle_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> ave_access_power = vector<double>(NUM_PACKAGES, 0.0);
	vector<double> average_power = vector<double>(NUM_PACKAGES, 0.0);

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    if(e->cycle != 0)
	    {
		total_energy[i] = (e->idle_energy[i] + e->access_energy[i]) * VCC;
		ave_idle_power[i] = (e->idle_energy[i] * VCC) / e->cycle;
		ave_access_power[i] = (e->access_energy[i] * VCC) / e->cycle;
		average_power[i] = total_energy[i] / e->cycle;
	    }
	    else
	    {
		total_energy[i] = 0;
		ave_idle_power[i] = 0;
		ave_access_power[i] = 0;
		average_power[i] = 0;
	    }
	}

	savefile<<"\nData for Epoch: "<<e->epoch<<"\n";
	savefile<<"===========================\n";
	savefile<<"\nAccess Data: \n";
	savefile<<"========================\n";	
	savefile<<"Cycles Simulated: "<<e->cycle<<"\n";
	savefile<<"Accesses completed: "<<e->num_accesses<<"\n";
	savefile<<"Reads completed: "<<e->num_reads<<"\n";
	savefile<<"Writes completed: "<<e->num_writes<<"\n";
	savefile<<"Number of Unmapped Accesses: " <<e->num_unmapped<<"\n";
	savefile<<"Number of Mapped Accesses: " <<e->num_mapped<<"\n";
	savefile<<"Number of Unmapped Reads: " <<e->num_read_unmapped<<"\n";
	savefile<<"Number of Mapped Reads: " <<e->num_read_mapped<<"\n";
	savefile<<"Number of Unmapped Writes: " <<e->num_write_unmapped<<"\n";
	savefile<<"Number of Mapped Writes: " <<e->num_write_mapped<<"\n";

	savefile<<"\nThroughput and Latency Data: \n";
	savefile<<"========================\n";
	savefile<<"Average Read Latency: " <<(divide((float)e->average_read_latency,(float)e->num_reads))<<" cycles";
	savefile<<" (" <<(divide((float)e->average_read_latency,(float)e->num_reads)*CYCLE_TIME)<<" ns)\n";
	savefile<<"Average Write Latency: " <<divide((float)e->average_write_latency,(float)e->num_writes)<<" cycles";
	savefile<<" (" <<(divide((float)e->average_write_latency,(float)e->num_writes))*CYCLE_TIME<<" ns)\n";
	savefile<<"Average Queue Latency: " <<divide((float)e->average_queue_latency,(float)e->num_accesses)<<" cycles";
	savefile<<" (" <<(divide((float)e->average_queue_latency,(float)e->num_accesses))*CYCLE_TIME<<" ns)\n";
	savefile<<"Total Throughput: " <<this->calc_throughput(e->cycle, e->num_accesses)<<" KB/sec\n";
	savefile<<"Read Throughput: " <<this->calc_throughput(e->cycle, e->num_reads)<<" KB/sec\n";
	savefile<<"Write Throughput: " <<this->calc_throughput(e->cycle, e->num_writes)<<" KB/sec\n";

	savefile<<"\nQueue Length Data: \n";
	savefile<<"========================\n";
	savefile<<"Length of Ftl Queue: " <<e->ftl_queue_length<<"\n";
	for(uint i = 0; i < e->ctrl_queue_length.size(); i++)
	{
	    savefile<<"Length of Controller Queue for Package " << i << ": "<<e->ctrl_queue_length[i]<<"\n";
	}

	if(WEAR_LEVEL_LOG)
	{
	    savefile<<"\nWrite Frequency Data: \n";
	    savefile<<"========================\n";
	    unordered_map<uint64_t, uint64_t>::iterator it;
	    for (it = e->writes_per_address.begin(); it != e->writes_per_address.end(); it++)
	    {
		savefile<<"Address "<<(*it).first<<": "<<(*it).second<<" writes\n";
	    }
	}

	savefile<<"\nPower Data: \n";
	savefile<<"========================\n";

	for(uint i = 0; i < NUM_PACKAGES; i++)
	{
	    savefile<<"Package: "<<i<<"\n";
	    savefile<<"Accumulated Idle Energy: "<<(e->idle_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<" mJ\n";
	    savefile<<"Accumulated Access Energy: "<<(e->access_energy[i] * VCC * (CYCLE_TIME * 0.000000001))<<" mJ\n";
	    savefile<<"Total Energy: "<<(total_energy[i] * (CYCLE_TIME * 0.000000001))<<" mJ\n\n";
	 
	    savefile<<"Average Idle Power: "<<ave_idle_power[i]<<" mW\n";
	    savefile<<"Average Access Power: "<<ave_access_power[i]<<" mW\n";
	    savefile<<"Average Power: "<<average_power[i]<<" mW\n\n";
	}

	savefile<<"\n-------------------------------------------------\n";
	
	savefile.close();
}

