#*********************************************************************************
#  Copyright (c) 2013-2014, Paul Tschirhart
#                             Jim Stevens
#                             Peter Enns
#                             Ishwar Bhati
#                             Mu-Tien Chang
#                             Bruce Jacob
#                             University of Maryland 
#                             pkt3c [at] umd [dot] edu
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  
#     * Redistributions of source code must retain the above copyright notice,
#        this list of conditions and the following disclaimer.
#  
#     * Redistributions in binary form must reproduce the above copyright notice,
#        this list of conditions and the following disclaimer in the documentation
#        and/or other materials provided with the distribution.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#********************************************************************************

#USAGE: python new_schedule_analysis.py <write log> <plane log>

import sys
import math

# capacity parameters
cfg.NUM_PACKAGES = 32
cfg.DIES_PER_PACKAGE = 4
cfg.PLANES_PER_DIE = 1
cfg.BLOCKS_PER_PLANE = 32
cfg.PAGES_PER_BLOCK = 48
cfg.NV_PAGE_SIZE=32768 # in bits

# timing parameters
cfg.DEVICE_CYCLE = 2.5
cfg.CHANNEL_CYCLE = 0.15
cfg.DEVICE_WIDTH = 8
cfg.CHANNEL_WIDTH = 8

# get the log files
write_log = open(sys.argv[1], 'r')
plane_log = open(sys.argv[2], 'r')

# what mode are we using (analysis of traffic, analysis of predictors)
mode = sys.argv[3]

# see if we've specified whether or not we want to append to this file or not
if len(sys.argv) == 6:
	file_out = 1 # just so we know later to write to a file
	if sys.argv[4] == 'Append':
		output_file = open(sys.argv[5], 'a')
	else:
		output_file = open(sys.argv[5], 'w')
# if we didn't but we still provided an output file assume we're not appending
elif len(sys.argv) == 5:
	file_out = 1 # just so we know later to write to a file
	output_file = open(sys.argv[4], 'w')

# preparse everything into files
plane_data = []
write_data = []
