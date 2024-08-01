#!/bin/sh

set -e

f=myblinky

yosys -p "synth_ecp5 -json ${f}.json" ${f}.v
nextpnr-ecp5 --85k \
	--json ${f}.json \
	--lpf ulx3s_v20.lpf \
	--textcfg ulx3s_out.config \
	--package CABGA381
ecppack ulx3s_out.config ulx3s.bit
fujprog ulx3s.bit
