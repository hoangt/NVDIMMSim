#ifndef NVGCFTL_H
#define NVGCFTL_H
//GCFtl.h
//header file for the ftl with garbage collection

#include <iostream>
#include <fstream>
#include "SimObj.h"
#include "FlashConfiguration.h"
#include "ChannelPacket.h"
#include "FlashTransaction.h"
#include "Controller.h"
#include "Ftl.h"
#include "Logger.h"
#include "GCLogger.h"

namespace NVDSim{
        class NVDIMM;
	class GCFtl : public Ftl{
		public:
	                GCFtl(Controller *c, Logger *l, NVDIMM *p);
			bool addTransaction(FlashTransaction &t);
			void addGcTransaction(FlashTransaction &t);
			void update(void);
			bool checkGC(void); 
			void runGC(void);

			void saveNVState(void);
			void loadNVState(void);

		protected:
			uint gc_status, panic_mode;
			uint64_t used_page_count, start_erase;
			    
			std::vector<vector<bool>> dirty;
	};
}
#endif
