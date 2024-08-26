#!/bin/sh

set -e

top=my_ball_paddle_top_tb
f=ulx3s_game

iverilog -g2005 -gstrict-expr-width -Wall \
	-s $top \
	-o ${f}.vvp ${f}.v
vvp ${f}.vvp
if [ "$1" != "only" ]
then
	gtkwave ${top}.vcd
fi