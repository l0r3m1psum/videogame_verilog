#!/bin/sh

set -e

t=ball_paddle_top
f=ulx3s_game

yosys -p "synth_ecp5 -top $t -json ${f}.json" ${f}.v
nextpnr-ecp5 --85k \
	--json ${f}.json \
	--lpf ulx3s_v20.lpf \
	--textcfg ulx3s_out.config \
	--package CABGA381
# nextpnr-ecp5 --json ${f}.json --lpf ulx3s_v20.lpf --package CABGA381 --gui
ecppack ulx3s_out.config ulx3s.bit
if [ "$1" != "only" ]
then
	fujprog ulx3s.bit
fi
