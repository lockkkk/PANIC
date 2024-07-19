#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2021.1 (64-bit)
#
# Filename    : elaborate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for elaborating the compiled design
#
# Generated by Vivado on Sat Apr 09 00:16:45 CDT 2022
# SW Build 3247384 on Thu Jun 10 19:36:07 MDT 2021
#
# IP Build 3246043 on Fri Jun 11 00:30:35 MDT 2021
#
# usage: elaborate.sh
#
# ****************************************************************************
set -Eeuo pipefail
# elaborate design
echo "xelab -wto 90cb01782e6644c8b22cb3e74266aba8 --incr --debug typical --relax --mt 8 -L xil_defaultlib -L uvm -L unisims_ver -L unimacro_ver -L secureip -L xpm --snapshot packet_gen_pipeline_behav xil_defaultlib.packet_gen_pipeline xil_defaultlib.glbl -log elaborate.log"
xelab -wto 90cb01782e6644c8b22cb3e74266aba8 --incr --debug typical --relax --mt 8 -L xil_defaultlib -L uvm -L unisims_ver -L unimacro_ver -L secureip -L xpm --snapshot packet_gen_pipeline_behav xil_defaultlib.packet_gen_pipeline xil_defaultlib.glbl -log elaborate.log

