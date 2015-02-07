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

#ifndef NVFTL_H
#define NVFTL_H
//Ftl.h
//header file for the ftl

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include "SimObj.h"
#include "FlashConfiguration.h"
#include "Init.h"
#include "ChannelPacket.h"
#include "FlashTransaction.h"
#include "Controller.h"
#include "Logger.h"
#include "Util.h"

namespace NVDSim{
        typedef struct 
	{
	    uint64_t address;
	    uint64_t block;
	    uint64_t page;
	    bool done;
	} write_location;

        class NVDIMM;
	class Ftl : public SimObj{

		public:
	                Ftl(Configuration &nv_cfg, Controller *c, Logger *l, NVDIMM *p);

			ChannelPacket *translate(ChannelPacketType type, uint64_t vAddr, uint64_t pAddr);
			bool attemptAdd(FlashTransaction &t, std::list<FlashTransaction> *queue, uint64_t queue_limit);
			bool checkQueueAddTransaction(FlashTransaction &t);
			virtual bool addTransaction(FlashTransaction &t);
			virtual void update(void);
			void handle_disk_read(bool gc);
			void handle_read(bool gc);
			virtual void write_used_handler(uint64_t vAddr);
			void write_success(uint64_t block, uint64_t page, uint64_t vAddr, uint64_t pAddr, bool gc, bool mapped);
			write_location next_write_location(uint64_t vAddr);
			write_location round_robin_write_location(uint64_t vAddr);
			write_location direct_write_location(uint64_t vAddr);
			void gap_rotate(void);
			void gap_movement(void);
			void handle_gap_write(uint64_t write_vAddr);
			uint64_t randomize_address_space(uint64_t vAddr);
			write_location start_gap_write_location(uint64_t vAddr);
			void handle_write(bool gc);
			void handle_trim(void);
			void handle_preset(void);
			uint64_t get_ptr(void); 
			void inc_ptr(void); 

			void popFrontTransaction();

			void sendQueueLength(void);
			
			void powerCallback(void);

			void preDirty(void);

			void saveNVState(void);
			void loadNVState(void);

			void queuesNotFull(void);
			void flushWriteQueues(void);

			virtual void GCReadDone(uint64_t vAddr);

			Configuration &cfg;
		       
			Controller *controller;

			NVDIMM *parent;

			Logger *log;

			// temp stuff **************************
			uint64_t locked_counter;
			// *************************************



		protected:
			int numBlocks;

			std::ifstream scriptfile;
			uint64_t write_cycle;
			uint64_t write_addr;
			uint64_t write_pack;
			uint64_t write_die;
			uint64_t write_plane;
			FlashTransaction writeTransaction;

			bool gc_flag;
			uint64_t channel, die, plane, lookupCounter;
			uint64_t temp_channel, temp_die, temp_plane;
			uint64_t max_queue_length;
			FlashTransaction currentTransaction;
			bool busy;

			uint64_t deadlock_counter;
			uint64_t deadlock_time;
			uint64_t write_counter;
			uint64_t used_page_count;
			uint64_t dirty_page_count;
			uint64_t write_wait_count;
			std::list<FlashTransaction>::iterator read_pointer; // stores location of the last place we tried in the read queue

			bool saved; // indicates that state data has been successfully saved to make sure we don't do it twice
			bool loaded; // indicates that state data has been successfully loaded to make sure we don't do it twice
			bool dirtied; // indicates that the NVM has been predirtied to simulate a used NVM
			bool ctrl_write_queues_full; // these let us know when we can't push anything more out to the controller
			bool ctrl_read_queues_full;

			// start gap variables
			uint64_t start;
			uint64_t gap;
			bool pending_gap_read;
			bool gap_read_delayed;
			bool pending_gap_write;
			bool gap_moving;
			bool gap_moved;
			uint64_t gap_write_vAddr; // we save this in case the ctrl queues are full and we need to reuse it
			std::unordered_map<uint64_t,uint64_t> randomMap; // address map for the randomized version of start-gap
			std::vector<uint64_t> randomAddrs; // vector of scrambled addresses

			std::unordered_map<uint64_t,uint64_t> addressMap; // address map for translation
			std::vector<vector<bool>> used;
			std::vector<vector<bool>> dirty;
			std::list<FlashTransaction> transQueue; 

			// address translation variables
			unsigned	totalBitWidth;
			unsigned	channelBitWidth;
			unsigned	vaultBitWidth;
			unsigned	bankBitWidth;
			unsigned	rowBitWidth;
			unsigned	colBitWidth;
			unsigned	colOffset;
	};
}
#endif
