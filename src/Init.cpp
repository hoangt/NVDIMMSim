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

#include "Init.h"

using namespace std;

// these are the values that are extern'd in FlashConfiguration.h so that they
// have global scope even though they are set by Init

namespace NVDSim 
{	
	// makes a new configuration object and returns it to the caller
	Configuration Init::read(string inifile)
	{
		ifstream inFile;
		char tmp[256];
		string tmp2;
		list<string> lines;

		inFile.open(inifile);
		if (!inFile.is_open())
		{
			cerr << "ERROR: Failed to load NVDIMM's Ini file: " << inifile << "\n";
			abort();
		}

		// put the text file into a data structure we can work with
		while(!inFile.eof())
		{
			inFile.getline(tmp, 256);
			tmp2 = (string)tmp;

			// Filter comments out.
			size_t pos = tmp2.find("#");
			tmp2 = tmp2.substr(0, pos);

			// Strip whitespace from the ends.
			tmp2 = strip(tmp2);

			// Filter newlines out.
			if (tmp2.empty())
				continue;

			// Add it to the lines list.
			lines.push_back(tmp2);
		}
		inFile.close();

		
		// make a new configuration object
		Configuration cfg = Configuration();

		list<string>::iterator it;
		for (it = lines.begin(); it != lines.end(); it++)
		{
			list<string> split_line = split((*it), "=", 2);

			if (split_line.size() != 2)
			{
				cerr << "ERROR: Parsing ini failed on line: " << (*it) << "\n";
				cerr << "There should be exactly one '=' per line\n";
				abort();
			}

			string key = split_line.front();
			string value = split_line.back();

			// Place the value into the appropriate global.
			// Debugging Options
			if (key.compare("DEBUG_TRANSLATION") == 0)
				convert_bool(cfg.DEBUG_TRANSLATION, value, key);

			// Scheduling Options
			else if (key.compare("SCHEDULE") == 0)
				convert_bool(cfg.SCHEDULE, value, key);
			else if (key.compare("WRITE_HIGH_THRESHOLD") == 0)
				convert_uint64_t(cfg.WRITE_HIGH_THRESHOLD, value, key);
			else if (key.compare("WRITE_LOW_THRESHOLD") == 0)
				convert_uint64_t(cfg.WRITE_LOW_THRESHOLD, value, key);
			else if (key.compare("IDLE_WRITE") == 0)
				convert_bool(cfg.IDLE_WRITE, value, key);
			else if (key.compare("DELAY_WRITE") == 0)
				convert_bool(cfg.DELAY_WRITE, value, key);
			else if (key.compare("DELAY_WRITE_CYCLES") == 0)
				convert_uint64_t(cfg.DELAY_WRITE_CYCLES, value, key);
			
			else if (key.compare("FTL_QUEUE_HANDLING") == 0)
				convert_bool(cfg.FTL_QUEUE_HANDLING, value, key);

			// Wear Leveling Options
			else if (key.compare("WEAR_LEVELING_SCHEME") == 0)
			{
				if(value.compare("round_robin") == 0)
				{
					cfg.wearLevelingScheme = RoundRobin;
				}
				else if(value.compare("start_gap") == 0)
				{
					cfg.wearLevelingScheme = StartGap;
				}
				else if(value.compare("direct_translation") == 0)
				{
					cfg.wearLevelingScheme = DirectTranslation;
				}
				else
				{
					cout << "WARNING: unknown wear leveling policy '"<<value<<"'; valid values are 'round_robin' and 'direct_translation'. Defaulting to round_robin \n";
					cfg.wearLevelingScheme = RoundRobin;
				}
			}
			else if (key.compare("GAP_WRITE_INTERVAL") == 0)
				convert_uint64_t(cfg.GAP_WRITE_INTERVAL, value, key);
			else if (key.compare("RANDOM_ADDR") == 0)
				convert_bool(cfg.RANDOM_ADDR, value, key);

			// Addressing Options
			else if (key.compare("ADDRESS_SCHEME") == 0)
			{
				if(value.compare("channel_vault_bank_row_col") == 0)
				{
					cfg.addressScheme = ChannelVaultBankRowCol;
				}
				else if(value.compare("col_row_bank_vault_channel") == 0)
				{
					cfg.addressScheme = ColRowBankVaultChannel;
				}
				else if(value.compare("row_bank_vault_channel_col") == 0)
				{
					cfg.addressScheme = RowBankVaultChannelCol;
				}
				else if(value.compare("bank_vault_channel_col_row") == 0)
				{
					cfg.addressScheme = BankVaultChannelColRow;
				}
				else if(value.compare("bank_vault_col_channel_row") == 0)
				{
					cfg.addressScheme = BankVaultColChannelRow;
				}
				else if(value.compare("bank_channel_vault_col_row") == 0)
				{
					cfg.addressScheme = BankChannelVaultColRow;
				}
				else if(value.compare("default") == 0)
				{
					cfg.addressScheme = Default;
				}
				else
				{
					cout << "WARNING: unknown address policy '"<<value<<"'; valid values are 'channel_vault_bank_row_col','col_row_bank_vault_channel', 'row_bank_vault_channel_col', and 'Default'. Defaulting to Default \n";
					cfg.addressScheme = Default;
				}
			}
			
			// Refresh Support
			else if (key.compare("REFRESH_ENABLE") == 0)
				convert_bool(cfg.REFRESH_ENABLE, value, key);
			else if (key.compare("REFRESH_PERIOD") == 0)
				convert_uint64_t(cfg.REFRESH_PERIOD, value, key);
			else if (key.compare("REFRESH_LEVEL") == 0)
			{
				if(value.compare("per_bank") == 0)
				{
					cfg.refreshLevel = PerBank;
				}
				else if(value.compare("per_vault") == 0)
				{
					cfg.refreshLevel = PerVault;
				}
				else if(value.compare("per_channel") == 0)
				{
					cfg.refreshLevel = PerChannel;
				}
				else
				{
					cout << "WARNING: unknown refresh policy '"<<value<<"'; valid values are 'per_bank','per_vault', and 'per_channel'. Defaulting to per_bank \n";
					cfg.refreshLevel = PerBank;
				}
			}
			
			// Read / Write Interleaving Options
			else if (key.compare("RW_INTERLEAVE_ENABLE") == 0)
				convert_bool(cfg.RW_INTERLEAVE_ENABLE, value, key);

			// Write Blocking Avoidance
			else if (key.compare("WRITE_PAUSING") == 0)
				convert_bool(cfg.WRITE_PAUSING, value, key);
			else if (key.compare("WRITE_CANCELATION") == 0)
				convert_bool(cfg.WRITE_CANCELATION, value, key);
			else if (key.compare("WRITE_ITERATION_CYCLES") == 0)
				convert_uint64_t(cfg.WRITE_ITERATION_CYCLES, value, key);
			
			// SSD Options
			else if (key.compare("DISK_READ") == 0)
				convert_bool(cfg.DISK_READ, value, key);			

			// Buffering Options
			else if (key.compare("FRONT_BUFFER") == 0)
				convert_bool(cfg.FRONT_BUFFER, value, key);
			else if (key.compare("REQUEST_BUFFER_SIZE") == 0)
				convert_uint64_t(cfg.REQUEST_BUFFER_SIZE, value, key);
			else if (key.compare("RESPONSE_BUFFER_SIZE") == 0)
				convert_uint64_t(cfg.RESPONSE_BUFFER_SIZE, value, key);
			else if (key.compare("BUFFERED") == 0)
				convert_bool(cfg.BUFFERED, value, key);
			else if (key.compare("CUT_THROUGH") == 0)
				convert_bool(cfg.CUT_THROUGH, value, key);
			else if (key.compare("IN_BUFFER_SIZE") == 0)
				convert_uint64_t(cfg.IN_BUFFER_SIZE, value, key);
			else if (key.compare("OUT_BUFFER_SIZE") == 0)
				convert_uint64_t(cfg.OUT_BUFFER_SIZE, value, key);

			// Critical Cache Line First Options
			else if (key.compare("CRIT_LINE_FIRST") == 0)
				convert_bool(cfg.CRIT_LINE_FIRST, value, key);

			// Logging Options
			else if (key.compare("LOGGING") == 0)
				convert_bool(cfg.LOGGING, value, key);
			else if (key.compare("LOG_DIR") == 0)
				cfg.LOG_DIR = value;
			else if (key.compare("WEAR_LEVEL_LOG") == 0)
				convert_bool(cfg.WEAR_LEVEL_LOG, value, key);
			else if (key.compare("RUNTIME_WRITE") == 0)
				convert_bool(cfg.RUNTIME_WRITE, value, key);
			else if (key.compare("PER_PACKAGE") == 0)
				convert_bool(cfg.PER_PACKAGE, value, key);
			else if (key.compare("QUEUE_EVENT_LOG") == 0)
				convert_bool(cfg.QUEUE_EVENT_LOG, value, key);
			else if (key.compare("PLANE_STATE_LOG") == 0)
				convert_bool(cfg.PLANE_STATE_LOG, value, key);
			else if (key.compare("WRITE_ARRIVE_LOG") == 0)
				convert_bool(cfg.WRITE_ARRIVE_LOG, value, key);
			else if (key.compare("READ_ARRIVE_LOG") == 0)
				convert_bool(cfg.READ_ARRIVE_LOG, value, key);
			else if (key.compare("CONCURRENCY_LOG") == 0)
				convert_bool(cfg.CONCURRENCY_LOG, value, key);
			else if (key.compare("BLOCK_HISTOGRAM_MAX") == 0)
				convert_uint64_t(cfg.BLOCK_HISTOGRAM_MAX, value, key);
			else if (key.compare("BLOCK_HISTOGRAM_BIN") == 0)
				convert_uint64_t(cfg.BLOCK_HISTOGRAM_BIN, value, key);

			// Save and Restore Options
			else if (key.compare("ENABLE_NV_SAVE") == 0)
				convert_bool(cfg.ENABLE_NV_SAVE, value, key);
			else if (key.compare("NV_SAVE_FILE") == 0)
				cfg.NV_SAVE_FILE = value;
			else if (key.compare("ENABLE_NV_RESTORE") == 0)
				convert_bool(cfg.ENABLE_NV_RESTORE, value, key);
			else if (key.compare("NV_RESTORE_FILE") == 0)
				cfg.NV_RESTORE_FILE = value;

			// General Organization Options
			else if (key.compare("DEVICE_TYPE") == 0)
				cfg.DEVICE_TYPE = value;
			else if (key.compare("NUM_PACKAGES") == 0)
				convert_uint64_t(cfg.NUM_PACKAGES, value, key);
			else if (key.compare("DIES_PER_PACKAGE") == 0)
				convert_uint64_t(cfg.DIES_PER_PACKAGE, value, key);
			else if (key.compare("PLANES_PER_DIE") == 0)
				convert_uint64_t(cfg.PLANES_PER_DIE, value, key);
			else if (key.compare("VIRTUAL_BLOCKS_PER_PLANE") == 0)
				convert_uint64_t(cfg.VIRTUAL_BLOCKS_PER_PLANE, value, key);
			else if (key.compare("PAGES_PER_BLOCK") == 0)
				convert_uint64_t(cfg.PAGES_PER_BLOCK, value, key);
			else if (key.compare("NV_PAGE_SIZE") == 0)
				convert_uint64_t(cfg.NV_PAGE_SIZE, value, key);

			// Channel Options
			else if (key.compare("DEVICE_CYCLE") == 0)
				convert_float(cfg.DEVICE_CYCLE, value, key);
			else if (key.compare("DEVICE_WIDTH") == 0)
				convert_uint64_t(cfg.DEVICE_WIDTH, value, key);
			else if (key.compare("DEVICE_DATA_CHANNEL") == 0)
				convert_bool(cfg.DEVICE_DATA_CHANNEL, value, key);
			else if (key.compare("DEVICE_DATA_WIDTH") == 0)
				convert_uint64_t(cfg.DEVICE_DATA_WIDTH, value, key);
			else if (key.compare("CHANNEL_TURN_ENABLE") == 0)
				convert_bool(cfg.CHANNEL_TURN_ENABLE, value, key);
			else if (key.compare("CHANNEL_TURN_TIME") == 0)
				convert_uint64_t(cfg.CHANNEL_TURN_TIME, value, key);
			else if (key.compare("CHANNEL_CYCLE") == 0)
				convert_float(cfg.CHANNEL_CYCLE, value, key);
			else if (key.compare("CHANNEL_WIDTH") == 0)
				convert_uint64_t(cfg.CHANNEL_WIDTH, value, key);
			else if (key.compare("ENABLE_COMMAND_CHANNEL") == 0)
				convert_bool(cfg.ENABLE_COMMAND_CHANNEL, value, key);
			else if (key.compare("COMMAND_CHANNEL_WIDTH") == 0)
				convert_uint64_t(cfg.COMMAND_CHANNEL_WIDTH, value, key);
			else if (key.compare("ENABLE_REQUEST_CHANNEL") == 0)
				convert_bool(cfg.ENABLE_REQUEST_CHANNEL, value, key);
			else if (key.compare("REQUEST_CHANNEL_WIDTH") == 0)
				convert_uint64_t(cfg.REQUEST_CHANNEL_WIDTH, value, key);

			// Garbage Collection Options
			else if (key.compare("GARBAGE_COLLECT") == 0)
				convert_bool(cfg.GARBAGE_COLLECT, value, key);
			else if (key.compare("PRESTATE") == 0)
				convert_bool(cfg.PRESTATE, value, key);
			else if (key.compare("PERCENT_FULL") == 0)
				convert_float(cfg.PERCENT_FULL, value, key);
			else if (key.compare("IDLE_GC_THRESHOLD") == 0)
				convert_float(cfg.IDLE_GC_THRESHOLD, value, key);
			else if (key.compare("FORCE_GC_THRESHOLD") == 0)
				convert_float(cfg.FORCE_GC_THRESHOLD, value, key);
			else if (key.compare("PBLOCKS_PER_VBLOCK") == 0)
				convert_float(cfg.PBLOCKS_PER_VBLOCK, value, key);
			
			// Timing Options
			else if (key.compare("READ_TIME") == 0)
				convert_uint64_t(cfg.READ_TIME, value, key);
			else if (key.compare("WRITE_TIME") == 0)
				convert_uint64_t(cfg.WRITE_TIME, value, key);
			else if (key.compare("ERASE_TIME") == 0)
				convert_uint64_t(cfg.ERASE_TIME, value, key);
			else if (key.compare("COMMAND_LENGTH") == 0)
				convert_uint64_t(cfg.COMMAND_LENGTH, value, key);
			else if (key.compare("LOOKUP_TIME") == 0)
				convert_uint64_t(cfg.LOOKUP_TIME, value, key);
			else if (key.compare("BUFFER_LOOKUP_TIME") == 0)
				convert_uint64_t(cfg.BUFFER_LOOKUP_TIME, value, key);
			else if (key.compare("QUEUE_ACCESS_TIME") == 0)
				convert_uint64_t(cfg.QUEUE_ACCESS_TIME, value, key);
			else if (key.compare("REFRESH_TIME") == 0)
				convert_uint64_t(cfg.REFRESH_TIME, value, key);
			else if (key.compare("OPEN_ROW_ENABLE") == 0)
				convert_bool(cfg.OPEN_ROW_ENABLE, value, key);
			else if (key.compare("ROW_HIT_TIME") == 0)
				convert_uint64_t(cfg.ROW_HIT_TIME, value, key);
			else if (key.compare("EPOCH_CYCLES") == 0)
				convert_uint64_t(cfg.EPOCH_CYCLES, value, key);
			else if (key.compare("CYCLE_TIME") == 0)
				convert_float(cfg.CYCLE_TIME, value, key);
			else if (key.compare("SYSTEM_CYCLE") == 0)
				convert_float(cfg.SYSTEM_CYCLE, value, key);

			// Queue Options
			else if (key.compare("FTL_QUEUE_LENGTH") == 0)
				convert_uint64_t(cfg.FTL_QUEUE_LENGTH, value, key);
			else if (key.compare("CTRL_QUEUE_LENGTH") == 0)
				convert_uint64_t(cfg.CTRL_QUEUE_LENGTH, value, key);

			// Power Settings
			else if (key.compare("READ_I") == 0)
				convert_float(cfg.READ_I, value, key);
			else if (key.compare("WRITE_I") == 0)
				convert_float(cfg.WRITE_I, value, key);
			else if (key.compare("ERASE_I") == 0)
				convert_float(cfg.ERASE_I, value, key);
			else if (key.compare("STANDBY_I") == 0)
				convert_float(cfg.STANDBY_I, value, key);
			else if (key.compare("IN_LEAK_I") == 0)
				convert_float(cfg.IN_LEAK_I, value, key);
			else if (key.compare("OUT_LEAK_I") == 0)
				convert_float(cfg.OUT_LEAK_I, value, key);
			else if (key.compare("VCC") == 0)
				convert_float(cfg.VCC, value, key);
			else if (key.compare("ASYNC_READ_I") == 0)
				convert_float(cfg.ASYNC_READ_I, value, key);
			else if (key.compare("VPP_STANDBY_I") == 0)
				convert_float(cfg.VPP_STANDBY_I, value, key);
			else if (key.compare("VPP_READ_I") == 0)
				convert_float(cfg.VPP_READ_I, value, key);
			else if (key.compare("VPP_WRITE_I") == 0)
				convert_float(cfg.VPP_WRITE_I, value, key);
			else if (key.compare("VPP_ERASE_I") == 0)
				convert_float(cfg.VPP_ERASE_I, value, key);
			else if (key.compare("VPP") == 0)
				convert_float(cfg.VPP, value, key);
			
			// Something ain't right
			else
			{
				cerr << "ERROR: Illegal key/value pair in NVDIMM ini file: " << key << "=" << value << "\n";
				cerr << "This could either be due to an illegal key or the incorrect value type for a key\n";
				abort();
			}
		}
		
		return cfg;
	}
}
