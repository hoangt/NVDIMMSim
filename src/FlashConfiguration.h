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

//sloppily reusing #defines from dramsim
#ifndef ERROR
#define ERROR(str) std::cerr<<"[ERROR ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl;
#endif

#ifndef WARNING
#define WARNING(str) std::cout<<"[WARNING ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl;
#endif

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

// Logging Options
extern bool WEAR_LEVEL_LOG;
extern bool RUNTIME_WRITE;
extern bool PER_PACKAGE;

// Save and Restore Options
extern bool ENABLE_NV_SAVE;
extern std::string NVDIMM_SAVE_FILE;
extern bool ENABLE_NV_RESTORE;
extern std::string NVDIMM_RESTORE_FILE;

extern std::string DEVICE_TYPE;
extern uint NUM_PACKAGES;
extern uint DIES_PER_PACKAGE;
extern uint PLANES_PER_DIE;
extern uint BLOCKS_PER_PLANE;
extern uint VIRTUAL_BLOCKS_PER_PLANE;
extern uint PAGES_PER_BLOCK;
extern uint NV_PAGE_SIZE;
extern float DEVICE_CYCLE;
extern float CHANNEL_CYCLE;
extern uint DEVICE_WIDTH;
extern uint CHANNEL_WIDTH;
// does the device use garbage collection 
extern bool GARBAGE_COLLECT;

#define GC GARBAGE_COLLECT

extern float IDLE_GC_THRESHOLD;
extern float FORCE_GC_THRESHOLD;
extern float PBLOCKS_PER_VBLOCK;

#define BLOCK_SIZE (NV_PAGE_SIZE * PAGES_PER_BLOCK)
#define PLANE_SIZE (NV_PAGE_SIZE * BLOCKS_PER_PLANE * PAGES_PER_BLOCK)
#define DIE_SIZE (NV_PAGE_SIZE * PLANES_PER_DIE * BLOCKS_PER_PLANE * PAGES_PER_BLOCK)
#define PACKAGE_SIZE (NV_PAGE_SIZE * DIES_PER_PACKAGE * PLANES_PER_DIE * BLOCKS_PER_PLANE * PAGES_PER_BLOCK)
#define TOTAL_SIZE (NV_PAGE_SIZE * NUM_PACKAGES * DIES_PER_PACKAGE * PLANES_PER_DIE * BLOCKS_PER_PLANE * PAGES_PER_BLOCK)

#define VIRTUAL_PLANE_SIZE (NV_PAGE_SIZE * VIRTUAL_BLOCKS_PER_PLANE * PAGES_PER_BLOCK)
#define VIRTUAL_DIE_SIZE (NV_PAGE_SIZE * PLANES_PER_DIE * VIRTUAL_BLOCKS_PER_PLANE * PAGES_PER_BLOCK)
#define VIRTUAL_PACKAGE_SIZE (NV_PAGE_SIZE * DIES_PER_PACKAGE * PLANES_PER_DIE * VIRTUAL_BLOCKS_PER_PLANE PAGES_PER_BLOCK)
#define VIRTUAL_TOTAL_SIZE (NV_PAGE_SIZE * NUM_PACKAGES * DIES_PER_PACKAGE * PLANES_PER_DIE * VIRTUAL_BLOCKS_PER_PLANE * PAGES_PER_BLOCK)

// TODO: Expand this to use the timing values from datasheets
extern uint READ_TIME;
extern uint WRITE_TIME;
extern uint ERASE_TIME;
extern uint COMMAND_LENGTH; //in bits, including address
extern uint LOOKUP_TIME;
// in nanoseconds
extern float CYCLE_TIME;

extern uint EPOCH_TIME;
#define USE_EPOCHS (EPOCH_TIME > 0)
extern uint FTL_QUEUE_LENGTH;
extern uint CTRL_QUEUE_LENGTH;

// Power stuff
extern double READ_I;
extern double WRITE_I;
extern double ERASE_I;
extern double STANDBY_I;
extern double IN_LEAK_I;
extern double OUT_LEAK_I;
extern double VCC;

// PCM specific power stuff
extern double ASYNC_READ_I;
extern double VPP_STANDBY_I;
extern double VPP_READ_I;
extern double VPP_WRITE_I;
extern double VPP_ERASE_I;
extern double VPP;

extern uint OUTPUT;

namespace NVDSim{
	typedef void (*returnCallBack_t)(uint id, uint64_t addr, uint64_t clockcycle);
}
#endif
