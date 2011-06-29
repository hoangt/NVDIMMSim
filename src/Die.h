#ifndef NVDIE_H
#define NVDIE_H
//Die.h
//header file for the Die class

#include "SimObj.h"
#include "FlashConfiguration.h"
#include "ChannelPacket.h"
#include "Plane.h"
#include "Logger.h"
#include "Util.h"

namespace NVDSim{

	class Channel;
	class Controller;
	class NVDIMM;
	class Ftl;
	class Die : public SimObj{
		public:
	                Die(NVDIMM *parent, Logger *l, uint id);
			void attachToChannel(Channel *chan);
			void receiveFromChannel(ChannelPacket *busPacket);
			int isDieBusy(uint plane);
			void update(void);

			void channelDone(void);

			// for fast forwarding
			void writeToPlane(ChannelPacket *packet);

		private:
			uint id;
			NVDIMM *parentNVDIMM;
			Channel *channel;
			Logger *log;
			uint dataCyclesLeft; //cycles per device beat
			uint deviceBeatsLeft; //device beats per page			
			std::queue<ChannelPacket *> returnDataPackets;
			std::vector<Plane> planes;
			std::vector<ChannelPacket *> currentCommands;
			uint *controlCyclesLeft;
	};
}
#endif
