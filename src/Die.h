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

#ifndef NVDIE_H
#define NVDIE_H
//Die.h
//header file for the Die class

#include "SimObj.h"
#include "FlashConfiguration.h"
#include "Init.h"
#include "ChannelPacket.h"
#include "Plane.h"
#include "Logger.h"
#include "Util.h"

namespace NVDSim{

	enum PlaneState
	{
		FREE,
		BUSY,
		CAN_RW,
		CAN_READ,
		CAN_ACCEPT_DATA,
		WAITING,
		PLANE_WRITING
	};

	class Buffer;
	class Controller;
	class NVDIMM;
	class Ftl;
	class Die : public SimObj{
		public:
	                Die(Configuration &nv_cfg, NVDIMM *parent, Logger *l, uint64_t id);
			void attachToBuffer(Buffer *buff);
			void receiveFromBuffer(ChannelPacket *busPacket);
			PlaneState isDieBusy(uint64_t plane, uint64_t block);
			void testDieRefresh(uint64_t plane, uint64_t block, bool write);
			bool canDieRefresh();
			void update(void);
			void channelDone(void);
			void bufferDone(uint64_t plane);
			void bufferLoaded(void);
			void critLineDone(void);
			uint returnWriteIterationCycle(uint64_t plane);
			bool writePause(uint64_t plane);
			bool writeResume(uint64_t plane);
			bool writeCancel(uint64_t plane);
			bool isCurrentBlock(uint64_t plane, uint64_t block);
			bool isCurrentPAddr(uint64_t plane, uint64_t block, uint64_t physAddr);
			void addRefreshes(ChannelPacket *busPacket);
			void updateRefreshPointer();

			// for fast forwarding
			void writeToPlane(ChannelPacket *packet);

			Configuration &cfg;
			uint64_t id;

		private:
			NVDIMM *parentNVDIMM;
			Buffer *buffer;
			Logger *log;
			bool sending;
			uint64_t dataCyclesLeft; //cycles per device beat
			uint64_t deviceBeatsLeft; //device beats per page
			uint64_t critBeat; //device beat when first cache line will have been sent, used for crit line first
			std::queue<ChannelPacket *> returnDataPackets;
			std::queue<ChannelPacket *> pendingDataPackets;
			std::vector<Plane> planes;
			std::vector<ChannelPacket *> currentCommands;
			std::vector<ChannelPacket *> pausedCommands;

			// DRAM row locality support
			std::vector<uint64_t> open_row;

			//for logging purposes
			std::vector<bool> refresh_blocked; 

			// refresh stuff
			uint64_t plane_rpointer;
			std::vector<bool> refreshing; //just a bit to check if something is refreshing

			uint64_t *pausedCyclesLeft;
			uint64_t *controlCyclesLeft;
			uint64_t *writeIterationCyclesLeft;
	};
}
#endif
