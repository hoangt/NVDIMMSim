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

#ifndef NVINIT_H
#define NVINIT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "FlashConfiguration.h"
#include "Util.h"

using namespace std;

namespace NVDSim 
{
	class Configuration 
	{		
		public:
			bool DEBUG_TRANSLATION;
	
			bool SCHEDULE;
			bool WRITE_ON_QUEUE_SIZE;
			uint64_t WRITE_QUEUE_LIMIT;
			bool IDLE_WRITE;
			bool DELAY_WRITE;
			uint64_t DELAY_WRITE_CYCLES;
			bool DISK_READ;
			bool FTL_QUEUE_HANDLING;
	
			WearLevelingScheme wearLevelingScheme;
			uint64_t GAP_WRITE_INTERVAL;
			bool RANDOM_ADDR;
			
			AddressScheme addressScheme;
		
			bool REFRESH_ENABLE;
			uint64_t REFRESH_PERIOD;
			RefreshScheme refreshLevel;
			
			bool RW_INTERLEAVE_ENABLE;
			
			bool WRITE_PAUSING;
			bool WRITE_CANCELATION;
			uint64_t WRITE_ITERATION_CYCLES;
			
			bool FRONT_BUFFER;
			uint64_t REQUEST_BUFFER_SIZE;
			uint64_t RESPONSE_BUFFER_SIZE;
			bool BUFFERED;
			bool CUT_THROUGH;
			uint64_t IN_BUFFER_SIZE;
			uint64_t OUT_BUFFER_SIZE;
			
			bool CRIT_LINE_FIRST;
			
			bool LOGGING;
			std::string LOG_DIR;
			bool WEAR_LEVEL_LOG;
			bool RUNTIME_WRITE; 
			bool PER_PACKAGE;
			bool QUEUE_EVENT_LOG;
			bool PLANE_STATE_LOG;
			bool WRITE_ARRIVE_LOG;
			bool READ_ARRIVE_LOG;
			bool CONCURRENCY_LOG;
			uint64_t BLOCK_HISTOGRAM_MAX;
			uint64_t BLOCK_HISTOGRAM_BIN;
			
			bool ENABLE_NV_SAVE;
			std::string NV_SAVE_FILE;
			bool ENABLE_NV_RESTORE;
			std::string NV_RESTORE_FILE;
			
			std::string DEVICE_TYPE;
			uint64_t NUM_PACKAGES;
			uint64_t DIES_PER_PACKAGE;
			uint64_t PLANES_PER_DIE;
			uint64_t BLOCKS_PER_PLANE;
			uint64_t VIRTUAL_BLOCKS_PER_PLANE;
			uint64_t PAGES_PER_BLOCK; // when in PCM mode this should always be 1 because pages are the erase granularity for PCM
			uint64_t NV_PAGE_SIZE;
			float DEVICE_CYCLE;
			uint64_t DEVICE_WIDTH;
			bool DEVICE_DATA_CHANNEL;
			uint64_t DEVICE_DATA_WIDTH;
			bool CHANNEL_TURN_ENABLE;
			uint64_t CHANNEL_TURN_TIME;
			
			float CHANNEL_CYCLE; //default channel, becomes up channel when down channel is enabled
			uint64_t CHANNEL_WIDTH;
			
			bool ENABLE_COMMAND_CHANNEL;
			uint64_t COMMAND_CHANNEL_WIDTH;
			
			bool ENABLE_REQUEST_CHANNEL;
			uint64_t REQUEST_CHANNEL_WIDTH;
			
			bool GARBAGE_COLLECT;
			bool PRESTATE;
			float PERCENT_FULL;
			
			uint64_t READ_TIME;
			uint64_t WRITE_TIME;
			uint64_t ERASE_TIME;
			uint64_t COMMAND_LENGTH;
			uint64_t LOOKUP_TIME;
			uint64_t BUFFER_LOOKUP_TIME;
			uint64_t QUEUE_ACCESS_TIME;
			
			uint64_t REFRESH_TIME;
			bool OPEN_ROW_ENABLE;
			uint64_t ROW_HIT_TIME;
			
			uint64_t EPOCH_CYCLES;
			float CYCLE_TIME;
			float SYSTEM_CYCLE;
			
			uint64_t FTL_QUEUE_LENGTH;
			uint64_t CTRL_QUEUE_LENGTH;
			
			float READ_I;
			float WRITE_I;
			float ERASE_I;
			float STANDBY_I;
			float IN_LEAK_I;
			float OUT_LEAK_I;
			float VCC;
			float ASYNC_READ_I;
			float VPP_STANDBY_I;
			float VPP_READ_I;
			float VPP_WRITE_I;
			float VPP_ERASE_I;
			float VPP;
			
			float IDLE_GC_THRESHOLD;
			float FORCE_GC_THRESHOLD;
			float PBLOCKS_PER_VBLOCK;
			
			bool DEBUG_INIT;

			// defining a constructor here so we can get default values
	                Configuration():
			        DEBUG_TRANSLATION(0),
				SCHEDULE(0),
				WRITE_ON_QUEUE_SIZE(0),
				WRITE_QUEUE_LIMIT(20),
				IDLE_WRITE(0),
				DELAY_WRITE(0),
				DELAY_WRITE_CYCLES(50),
				DISK_READ(1),
				FTL_QUEUE_HANDLING(1),
				//wearLevelingScheme(direct_translation),
				GAP_WRITE_INTERVAL(5),
				RANDOM_ADDR(1),
				//addressScheme(0),
				REFRESH_ENABLE(1),
				REFRESH_PERIOD(40000000),
				//refreshLevel(0),
				RW_INTERLEAVE_ENABLE(0),
				WRITE_PAUSING(0),
				WRITE_CANCELATION(0),
				WRITE_ITERATION_CYCLES(20),
				FRONT_BUFFER(0),
				REQUEST_BUFFER_SIZE(3276800),
				RESPONSE_BUFFER_SIZE(3276800),
				BUFFERED(0),
				CUT_THROUGH(0),
				IN_BUFFER_SIZE(330000),
				OUT_BUFFER_SIZE(330000),
				CRIT_LINE_FIRST(0),
				LOGGING(1),
				LOG_DIR("nvdimm_ps_logs/"),
				WEAR_LEVEL_LOG(0),
				RUNTIME_WRITE(1), 
				PER_PACKAGE(0),
				QUEUE_EVENT_LOG(0),
				PLANE_STATE_LOG(0),
				WRITE_ARRIVE_LOG(0),
				READ_ARRIVE_LOG(0),
				CONCURRENCY_LOG(1),
				BLOCK_HISTOGRAM_MAX(2000),
				BLOCK_HISTOGRAM_BIN(50),			
				ENABLE_NV_SAVE(0),
				NV_SAVE_FILE("state/nvdimm_state.txt"),
				ENABLE_NV_RESTORE(0),
				NV_RESTORE_FILE("state/nvdimm_state.txt"),
				DEVICE_TYPE("PCM"),
				NUM_PACKAGES(16),
				DIES_PER_PACKAGE(1),
				PLANES_PER_DIE(8),
				BLOCKS_PER_PLANE(512),
				VIRTUAL_BLOCKS_PER_PLANE(512),
				PAGES_PER_BLOCK(32), // when in PCM mode this should always be 1 because pages are the erase granularity for PCM
				NV_PAGE_SIZE(64),
				DEVICE_CYCLE(0.3),
				DEVICE_WIDTH(8),
				DEVICE_DATA_CHANNEL(1),
				DEVICE_DATA_WIDTH(64),
				CHANNEL_TURN_ENABLE(1),
				CHANNEL_TURN_TIME(10),
				CHANNEL_CYCLE(0.25), //default channel, becomes up channel when down channel is enabled
				CHANNEL_WIDTH(32),
				ENABLE_COMMAND_CHANNEL(0),
				COMMAND_CHANNEL_WIDTH(8),
				ENABLE_REQUEST_CHANNEL(1),
				REQUEST_CHANNEL_WIDTH(32),
				GARBAGE_COLLECT(0),
				PRESTATE(0),
				PERCENT_FULL(0),
				READ_TIME(25),
				WRITE_TIME(50),
				ERASE_TIME(150),
				COMMAND_LENGTH(56),
				LOOKUP_TIME(0),
				BUFFER_LOOKUP_TIME(30),
				QUEUE_ACCESS_TIME(75),
				REFRESH_TIME(40),
				OPEN_ROW_ENABLE(0),
				ROW_HIT_TIME(12),
				EPOCH_CYCLES(200000),
				CYCLE_TIME(1.51),
				SYSTEM_CYCLE(1.51),
				FTL_QUEUE_LENGTH(10),
				CTRL_QUEUE_LENGTH(0),
				READ_I(15),
				WRITE_I(35),
				ERASE_I(35),
				STANDBY_I(0.08),
				IN_LEAK_I(0.01),
				OUT_LEAK_I(0.01),
				VCC(3.3),
				ASYNC_READ_I(30),
				VPP_STANDBY_I(0.0002),
				VPP_READ_I(0.002),
				VPP_WRITE_I(0.05),
				VPP_ERASE_I(0.05),
				VPP(3.3),
				IDLE_GC_THRESHOLD(0.70),
				FORCE_GC_THRESHOLD(1.0),
				PBLOCKS_PER_VBLOCK(1.0),
				DEBUG_INIT(0){}
	};

	class Init 
	{		
		public:
		static Configuration read(string inifile);
	};
}


#endif
