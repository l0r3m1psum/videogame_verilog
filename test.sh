#!/bin/sh

set -e

top=digits_tb

iverilog -g2005 -gstrict-expr-width -Wall \
	-s $top \
	-o ulxs3_game.vvp ulxs3_game.v
vvp ulxs3_game.vvp
# gtkwave ${top}.vcd