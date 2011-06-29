#ifndef NVGCLOGGER_H
#define NVGCLOGGER_H

#include <iostream>
#include <fstream>
#include <vector>

#include "FlashConfiguration.h"
#include "Logger.h"

namespace NVDSim
{
    class GCLogger: public Logger
    {
    public:
	GCLogger();
	
	// operations
	void erase();
	void gcread();
	void gcwrite();

	void erase_latency(uint64_t cycles);
	void gcread_latency(uint64_t cycles);
	void gcwrite_latency(uint64_t cycles);
	
	void save(uint64_t cycle, uint epoch);
	void print(uint64_t cycle);

	void update();

	void access_process(uint64_t addr, uint64_t paddr, uint package, ChannelPacketType op);
	void access_stop(uint64_t paddr);

	//Accessors for power data
	//Writing correct object oriented code up in this piece, what now?
	std::vector<std::vector<double>> getEnergyData(void);
	
	void save_epoch(uint64_t cycle, uint epoch);

	// State
	uint64_t num_erases;
	uint64_t num_gcreads;
	uint64_t num_gcwrites;
		
	uint64_t average_erase_latency;
	uint64_t average_gcread_latency;
	uint64_t average_gcwrite_latency;

	// Power Stuff
	// This is computed per package
	std::vector<double> erase_energy;

	class EpochEntry
	{
	public:
	    uint64_t cycle;
	    uint64_t epoch;

	    uint64_t num_accesses;
	    uint64_t num_reads;
	    uint64_t num_writes;
	    uint64_t num_erases;
	    uint64_t num_gcreads;
	    uint64_t num_gcwrites;

	    uint64_t num_unmapped;
	    uint64_t num_mapped;

	    uint64_t num_read_unmapped;
	    uint64_t num_read_mapped;
	    uint64_t num_write_unmapped;
	    uint64_t num_write_mapped;
		
	    uint64_t average_latency;
	    uint64_t average_read_latency;
	    uint64_t average_write_latency;
	    uint64_t average_erase_latency;
	    uint64_t average_queue_latency;
	    uint64_t average_gcread_latency;
	    uint64_t average_gcwrite_latency;

	    uint64_t ftl_queue_length;
	    std::vector<uint64_t> ctrl_queue_length;

	    std::unordered_map<uint64_t, uint64_t> writes_per_address;

	    std::vector<double> idle_energy;
	    std::vector<double> access_energy;
	    std::vector<double> erase_energy;

	    EpochEntry()
	    {
		num_accesses = 0;
		num_reads = 0;
		num_writes = 0;
		num_erases = 0;
		num_gcreads = 0;
		num_gcwrites = 0;

		num_unmapped = 0;
		num_mapped = 0;

		num_read_unmapped = 0;
		num_read_mapped = 0;
		num_write_unmapped = 0;
		num_write_mapped = 0;
		
		average_latency = 0;
		average_read_latency = 0;
		average_write_latency = 0;
		average_erase_latency = 0;
		average_queue_latency = 0;
		average_gcread_latency = 0;
		average_gcwrite_latency = 0;
		
		ftl_queue_length = 0;
		ctrl_queue_length = std::vector<uint64_t>(NUM_PACKAGES, 0);

		idle_energy = std::vector<double>(NUM_PACKAGES, 0.0); 
		access_energy = std::vector<double>(NUM_PACKAGES, 0.0); 
		erase_energy = std::vector<double>(NUM_PACKAGES, 0.0);
	    }
	};

	// Store system snapshot from last epoch to compute this epoch
	EpochEntry last_epoch;

	// Store the data from each epoch for printing at the end of the simulation
	std::list<EpochEntry> epoch_queue;

	void write_epoch(EpochEntry *e);
    };   
}

#endif
