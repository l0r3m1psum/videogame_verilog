#!/bin/sh

set -e

top=player_stats_tb

iverilog -g2005 -gstrict-expr-width -Wall \
	-s $top \
	-o ulxs3_game.vvp ulx3s_game.v
vvp ulxs3_game.vvp
gtkwave ${top}.vcd
