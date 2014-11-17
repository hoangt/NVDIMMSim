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

#ifndef NVFLASHCONF_H
#define NVFLASHCONF_H
//SysemConfiguration.h
//Configuration values, headers, and macros for the whole system
//

#include <iostream>
#include <cstdlib>
#include <string>

#include <vector>
#include <unordered_map>
#include <queue>
#include <list>
#include <stdint.h>
#include <limits.h>

#include "Util.h"

//sloppily reusing #defines from dramsim
#ifndef ERROR
#define ERROR(str) std::cerr<<"[ERROR ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl;
#endif

#ifndef WARNING
#define WARNING(str) std::cout<<"[WARNING ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl;
#endif

// enables outputs
# define OUTPUT 1

#ifdef DEBUG_BUILD
	#ifndef DEBUG
	#define DEBUG(str) std::cout<< str <<std::endl;
	#endif
	#ifndef DEBUGN
	#define DEBUGN(str) std::cout<< str;
	#endif
#else
	#ifndef DEBUG
	#define DEBUG(str) ;
	#endif
	#ifndef DEBUGN
	#define DEBUGN(str) ;
	#endif
#endif

#ifndef NO_OUTPUT
	#ifndef PRINT
	#define PRINT(str)  if(OUTPUT) { std::cerr <<str<<std::endl; }
	#endif
	#ifndef PRINTN
	#define PRINTN(str) if(OUTPUT) { std::cerr <<str; }
	#endif
#else
	#undef DEBUG
	#undef DEBUGN
	#define DEBUG(str) ;
	#define DEBUGN(str) ;
	#ifndef PRINT
	#define PRINT(str) ;
	#endif
	#ifndef PRINTN
	#define PRINTN(str) ;
	#endif
#endif

// Power Callback Options
#define Power_Callback 1
#define Verbose_Power_Callback 0

namespace NVDSim{

enum WearLevelingScheme
{
    RoundRobin,
    StartGap,
    DirectTranslation
};

enum RefreshScheme
{
	PerBank,
	PerVault,
	PerChannel
};

enum AddressScheme
{
	ChannelVaultBankRowCol,
	ColRowBankVaultChannel,
	RowBankVaultChannelCol,
	BankVaultChannelColRow,
	BankVaultColChannelRow,
	BankChannelVaultColRow,
	Default
}; 

// Debugging Options
extern bool DEBUG_TRANSLATION;

// constants
#define BITS_PER_KB 8192

#define CHANNEL_TURN_CYCLES divide_params_64b(cfg.CHANNEL_TURN_TIME, cfg.CYCLE_TIME)

#define BLOCK_SIZE (cfg.NV_PAGE_SIZE * cfg.PAGES_PER_BLOCK)
#define PLANE_SIZE (cfg.NV_PAGE_SIZE * cfg.BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define DIE_SIZE (cfg.NV_PAGE_SIZE * cfg.PLANES_PER_DIE * cfg.BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define PACKAGE_SIZE (cfg.NV_PAGE_SIZE * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define TOTAL_SIZE (cfg.NV_PAGE_SIZE * cfg.NUM_PACKAGES * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)

#define VIRTUAL_PLANE_SIZE (cfg.NV_PAGE_SIZE * cfg.VIRTUAL_BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define VIRTUAL_DIE_SIZE (cfg.NV_PAGE_SIZE * cfg.PLANES_PER_DIE * cfg.VIRTUAL_BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define VIRTUAL_PACKAGE_SIZE (cfg.NV_PAGE_SIZE * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.VIRTUAL_BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)
#define VIRTUAL_TOTAL_SIZE (cfg.NV_PAGE_SIZE * cfg.NUM_PACKAGES * cfg.DIES_PER_PACKAGE * cfg.PLANES_PER_DIE * cfg.VIRTUAL_BLOCKS_PER_PLANE * cfg.PAGES_PER_BLOCK)

#define READ_CYCLES (divide_params_64b(cfg.READ_TIME, cfg.CYCLE_TIME))
#define WRITE_CYCLES (divide_params_64b(cfg.WRITE_TIME, cfg.CYCLE_TIME))
#define ERASE_CYCLES (divide_params_64b(cfg.ERASE_TIME, cfg.CYCLE_TIME))
#define LOOKUP_CYCLES (divide_params_64b(cfg.LOOKUP_TIME, cfg.CYCLE_TIME))
#define BUFFER_LOOKUP_CYCLES (divide_params_64b(cfg.BUFFER_LOOKUP_TIME, cfg.CHANNEL_CYCLE)) // we use chcannel cycles here cause that is how the buffer is updated
#define QUEUE_ACCESS_CYCLES (divide_params_64b(cfg.QUEUE_ACCESS_TIME, cfg.CYCLE_TIME))

// some DRAM specific timings
#define REFRESH_CYCLES divide_params_64b(cfg.REFRESH_TIME, cfg.CYCLE_TIME)
#define ROW_HIT_CYCLES divide_params_64b(cfg.ROW_HIT_TIME, cfg.CYCLE_TIME)

#define USE_EPOCHS (cfg.EPOCH_CYCLES > 0)

//namespace NVDSim{
	typedef void (*returnCallBack_t)(uint64_t id, uint64_t addr, uint64_t clockcycle);
}
#endif
