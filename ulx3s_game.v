// 800x525 -> 640x480
module hvsync_generator_hdmi(
	input pixclk, rst,
	output hSync, vSync, DrawArea,
	output reg [9:0] CounterX, CounterY
);
	initial begin
		CounterX = 0;
		CounterY = 0;
	end

	always @(posedge pixclk) begin
		if (rst) begin
			CounterX <= 0;
			CounterY <= 0;
		end else begin
			CounterX <= CounterX == 799 ? 0 : CounterX + 1;
			if (CounterX == 799) CounterY <= CounterY == 524 ? 0 : CounterY + 1;
		end
	end

	assign hSync = 656 <= CounterX && CounterX < 752;
	assign vSync = 490 <= CounterY && CounterY < 492;
	assign DrawArea = CounterX < 640 && CounterY < 480;
endmodule

module hvsync_tb;
	reg clk = 0, rst = 0;
	always #1 clk = ~clk;
	
	wire hSync, vSync, DrawArea;
	wire [9:0] CounterX, CounterY;

	hvsync_generator_hdmi dut(clk, rst, hSync, vSync, DrawArea, CounterX, CounterY);
	
	initial begin
		// $monitor("%d hSync=%b vSync=%b DrawArea=%b CounterX=%3d CounterY=%3d",
		// 	$time, hSync, vSync, DrawArea, CounterX, CounterY);
		$dumpfile("hvsync_tb.vcd");
		$dumpvars;
		rst = 1;
		#2 // @(posedge clk); should nicely solve this!
		rst = 0;
		#(800*525 * 2) // since every clock cycle is 2 units of time
		$finish();
	end
endmodule

module TMDS_encoder(
	input clk,
	input [7:0] VD, // video data (red, green or blue)
	input [1:0] CD, // control data
	input VDE,      // video data enable, to choose between CD (when VDE=0) and VD (when VDE=1)
	output reg [9:0] TMDS = 0
);
    wire [3:0] Nb1s = VD[0] + VD[1] + VD[2] + VD[3] + VD[4] + VD[5] + VD[6] + VD[7];
    wire XNOR = (Nb1s>4'd4) || (Nb1s==4'd4 && VD[0]==1'b0);
    wire [8:0] q_m = {~XNOR, q_m[6:0] ^ VD[7:1] ^ {7{XNOR}}, VD[0]};

    reg [3:0] balance_acc = 0;
    wire [3:0] balance = q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7] - 4'd4;
    wire balance_sign_eq = (balance[3] == balance_acc[3]);
    wire invert_q_m = (balance==0 || balance_acc==0) ? ~q_m[8] : balance_sign_eq;
    wire [3:0] balance_acc_inc = balance - ({q_m[8] ^ ~balance_sign_eq} & ~(balance==0 || balance_acc==0));
    wire [3:0] balance_acc_new = invert_q_m ? balance_acc-balance_acc_inc : balance_acc+balance_acc_inc;
    wire [9:0] TMDS_data = {invert_q_m, q_m[8], q_m[7:0] ^ {8{invert_q_m}}};
    wire [9:0] TMDS_code = CD[1] ? (CD[0] ? 10'b1010101011 : 10'b0101010100) : (CD[0] ? 10'b0010101011 : 10'b1101010100);

    always @(posedge clk) TMDS <= VDE ? TMDS_data : TMDS_code;
    always @(posedge clk) balance_acc <= VDE ? balance_acc_new : 4'h0;
endmodule

// TODO: remove the need for this by using EHXPLLL directly https://www.digikey.it/en/maker/projects/2028ce62001b4cb69335f48e127fa366
// Taken from https://github.com/emard/ulx3s-misc/blob/master/examples/ecp5pll/hdl/sv/ecp5pll.sv
// https://github.com/emard/ulx3s-misc/blob/master/examples/dvi/top/top_vgatest.v
// Simpler version but hard to undestand what it id doing
// https://github.com/lawrie/ulx3s_examples/blob/master/hdmi/pll.v
// Hardcore stuff
// https://zipcpu.com/dsp/2017/12/14/logic-pll.html
module ecp5pll #(
		parameter integer in_hz       =  25000000,
		parameter integer out0_hz     =  25000000,
		parameter integer out0_deg    =         0, // keep 0
		parameter integer out0_tol_hz =         0, // tolerance: if freq differs more, then error
		parameter integer out1_hz     =         0,
		parameter integer out1_deg    =         0,
		parameter integer out1_tol_hz =         0,
		parameter integer out2_hz     =         0,
		parameter integer out2_deg    =         0,
		parameter integer out2_tol_hz =         0,
		parameter integer out3_hz     =         0,
		parameter integer out3_deg    =         0,
		parameter integer out3_tol_hz =         0,
		parameter integer reset_en    =         0,
		parameter integer standby_en  =         0,
		parameter integer dynamic_en  =         0
	) (
		input        clk_i,
		output [3:0] clk_o,
		input        reset,
		input        standby,
		input  [1:0] phasesel,
		input        phasedir, phasestep, phaseloadreg,
		output       locked
	);

	localparam PFD_MIN =   3125000;
	localparam PFD_MAX = 400000000;
	localparam VCO_MIN = 400000000;
	localparam VCO_MAX = 800000000;
	localparam VCO_OPTIMAL = (VCO_MIN+VCO_MAX)/2;

	function integer abs(input integer x);
		abs = x > 0 ? x : -x;
	endfunction
	function integer min(input integer x, y);
		min = x < y ? x : y;
	endfunction
	function integer max(input integer x, y);
		max = x > y ? x : y;
	endfunction

	function [32*3-1:0] F_ecp5pll(input integer x /* ignored */);
	begin: a
		integer input_div, input_div_min, input_div_max;
		integer output_div, output_div_min, output_div_max;
		integer feedback_div, feedback_div_min, feedback_div_max;
		integer fvco, fout;
		integer error, error_prev;
		integer params_fvco;
		integer div1, div2, div3;
		integer params_refclk_div, params_feedback_div, params_output_div;

		params_fvco = 0;
		error_prev = 'h7FFF_FFFF;
		input_div_min = max(1, in_hz/PFD_MAX);
		input_div_max = min(128, in_hz/PFD_MIN);
		for(input_div = input_div_min;
			input_div <= input_div_max;
			input_div=input_div+1) begin

			feedback_div = out0_hz / 1000000 * input_div < 2000
				? out0_hz * input_div / in_hz
				: out0_hz / in_hz * input_div;
			feedback_div_min = max(1, feedback_div);
			feedback_div_max = min(80, feedback_div+1);
			for(feedback_div = feedback_div_min;
				feedback_div <= feedback_div_max;
				feedback_div = feedback_div+1) begin

				output_div_min = max(1, (VCO_MIN/feedback_div) / (in_hz/input_div));
				output_div_max = min(128, (VCO_MAX/feedback_div) / (in_hz/input_div));
				fout = in_hz * feedback_div / input_div;
				for(output_div = output_div_min;
					output_div <= output_div_max;
					output_div=output_div+1) begin

					fvco = fout * output_div;
					error = abs(fout-out0_hz)
						+ (out1_hz > 0 ? abs(fvco/(fvco >= out1_hz ? fvco/out1_hz : 1)-out1_hz) : 0)
						+ (out2_hz > 0 ? abs(fvco/(fvco >= out2_hz ? fvco/out2_hz : 1)-out2_hz) : 0)
						+ (out3_hz > 0 ? abs(fvco/(fvco >= out3_hz ? fvco/out3_hz : 1)-out3_hz) : 0);
					if(error < error_prev
						|| (error == error_prev && abs(fvco-VCO_OPTIMAL) < abs(params_fvco-VCO_OPTIMAL))
					) begin
						error_prev          = error;
						params_refclk_div   = input_div;
						params_feedback_div = feedback_div;
						params_output_div   = output_div;
						params_fvco         = fvco;
					end
				end
			end
		end
		F_ecp5pll = {params_refclk_div, params_feedback_div, params_output_div};
	end
	endfunction

	function integer F_primary_phase(input integer output_div, deg);
	begin: a
		integer phase_compensation;
		integer phase_count_x8;

		phase_compensation = (output_div+1)/2*8-8+output_div/2*8; // output_div/2*8 = 180 deg shift
		phase_count_x8     = phase_compensation + 8*output_div*deg/360;
		F_primary_phase    = phase_count_x8 > 1023
			? phase_count_x8 % (output_div*8) // wraparound 360 deg
			: phase_count_x8;
	end
	endfunction

	function integer F_secondary_phase(input integer sfreq, sphase);
	begin: a
		integer div, freq;
		integer phase_compensation, phase_count_x8;

		phase_count_x8 = 0;
		if(sfreq > 0) begin
			div = params_fvco >= sfreq ? params_fvco/sfreq : 1;
			freq = params_fvco/div;
			phase_compensation = div*8-8;
			phase_count_x8 = phase_count_x8 > 1023
				? phase_count_x8 % (div*8) // wraparound 360 deg
				: phase_compensation + 8*div*sphase/360;
		end

		F_secondary_phase = phase_count_x8;
	end
	endfunction
	
	function integer F_secondary_divisor(input integer sfreq, params_fvco);
		F_secondary_divisor = sfreq > 0 && params_fvco >= sfreq ? params_fvco/sfreq : 1;
	endfunction

	localparam F_ecp5pll_out = F_ecp5pll(0);
	localparam params_refclk_div       = F_ecp5pll_out[32*3-1:32*2];
	localparam params_feedback_div     = F_ecp5pll_out[32*2-1:32*1];
	localparam params_output_div       = F_ecp5pll_out[32*1-1:32*0];
	localparam params_fout             = in_hz * params_feedback_div / params_refclk_div;
	localparam params_fvco             = params_fout * params_output_div;

	// localparam params_primary_phase_x8 = F_ecp5pll(3);
	localparam params_primary_cphase   = F_primary_phase(params_output_div, out0_deg) / 8;
	localparam params_primary_fphase   = F_primary_phase(params_output_div, out0_deg) % 8;

	localparam params_secondary1_div      = F_secondary_divisor(out1_hz, params_fvco);
	localparam params_secondary1_cphase   = F_secondary_phase  (out1_hz, out1_deg) / 8;
	localparam params_secondary1_fphase   = F_secondary_phase  (out1_hz, out1_deg) % 8;
	localparam params_secondary2_div      = F_secondary_divisor(out2_hz, params_fvco);
	localparam params_secondary2_cphase   = F_secondary_phase  (out2_hz, out2_deg) / 8;
	localparam params_secondary2_fphase   = F_secondary_phase  (out2_hz, out2_deg) % 8;
	localparam params_secondary3_div      = F_secondary_divisor(out3_hz, params_fvco);
	localparam params_secondary3_cphase   = F_secondary_phase  (out3_hz, out3_deg) / 8;
	localparam params_secondary3_fphase   = F_secondary_phase  (out3_hz, out3_deg) % 8;

	// check if generated frequencies are out of range
	localparam error_out0_hz =               abs(out0_hz - params_fout)                         > out0_tol_hz;
	localparam error_out1_hz = out1_hz > 0 ? abs(out1_hz - params_fvco / params_secondary1_div) > out1_tol_hz : 0;
	localparam error_out2_hz = out2_hz > 0 ? abs(out2_hz - params_fvco / params_secondary2_div) > out2_tol_hz : 0;
	localparam error_out3_hz = out3_hz > 0 ? abs(out3_hz - params_fvco / params_secondary3_div) > out3_tol_hz : 0;
	// diamond: won't compile this, comment it out. Workaround follows using division by zero

	if(error_out0_hz) $error("out0_hz tolerance exceeds out0_tol_hz");
	if(error_out1_hz) $error("out1_hz tolerance exceeds out1_tol_hz");
	if(error_out2_hz) $error("out2_hz tolerance exceeds out2_tol_hz");
	if(error_out3_hz) $error("out3_hz tolerance exceeds out3_tol_hz");

	// diamond: trigger error with division by zero, doesn't accept $error()
	localparam trig_out0_hz = error_out0_hz ? 1/0 : 0;
	localparam trig_out1_hz = error_out1_hz ? 1/0 : 0;
	localparam trig_out2_hz = error_out2_hz ? 1/0 : 0;
	localparam trig_out3_hz = error_out3_hz ? 1/0 : 0;

	wire [1:0] PHASESEL_HW = phasesel-1;
	wire CLKOP; // internal

	// TODO: frequencies in MHz if passed as "attributes"
	// will appear in diamond *.mrp file like "Output Clock(P) Frequency (MHz):"
	// but I don't know how to pass string parameters for this:
	// (* FREQUENCY_PIN_CLKI="025.000000" *)
	// (* FREQUENCY_PIN_CLKOP="023.345678" *)
	// (* FREQUENCY_PIN_CLKOS="034.234567" *)
	// (* FREQUENCY_PIN_CLKOS2="111.345678" *)
	// (* FREQUENCY_PIN_CLKOS3="123.456789" *)
	(* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
	EHXPLLL #(
		.CLKI_DIV     (params_refclk_div),
		.CLKFB_DIV    (params_feedback_div),
		.FEEDBK_PATH  ("CLKOP"),

		.OUTDIVIDER_MUXA("DIVA"),
		.CLKOP_ENABLE ("ENABLED"),
		.CLKOP_DIV    (params_output_div),
		.CLKOP_CPHASE (params_primary_cphase),
		.CLKOP_FPHASE (params_primary_fphase),

		.OUTDIVIDER_MUXB("DIVB"),
		.CLKOS_ENABLE (out1_hz > 0 ? "ENABLED" : "DISABLED"),
		.CLKOS_DIV    (params_secondary1_div),
		.CLKOS_CPHASE (params_secondary1_cphase),
		.CLKOS_FPHASE (params_secondary1_fphase),

		.OUTDIVIDER_MUXC("DIVC"),
		.CLKOS2_ENABLE(out2_hz > 0 ? "ENABLED" : "DISABLED"),
		.CLKOS2_DIV   (params_secondary2_div),
		.CLKOS2_CPHASE(params_secondary2_cphase),
		.CLKOS2_FPHASE(params_secondary2_fphase),

		.OUTDIVIDER_MUXD("DIVD"),
		.CLKOS3_ENABLE(out3_hz > 0 ? "ENABLED" : "DISABLED"),
		.CLKOS3_DIV   (params_secondary3_div),
		.CLKOS3_CPHASE(params_secondary3_cphase),
		.CLKOS3_FPHASE(params_secondary3_fphase),

		.INTFB_WAKE   ("DISABLED"),
		.STDBY_ENABLE (standby_en ? "ENABLED" : "DISABLED"),
		.PLLRST_ENA   (  reset_en ? "ENABLED" : "DISABLED"),
		.DPHASE_SOURCE(dynamic_en ? "ENABLED" : "DISABLED"),
		.PLL_LOCK_MODE(0)
	) pll_inst (
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(clk_i),
		.CLKOP(CLKOP),
		.CLKOS (clk_o[1]),
		.CLKOS2(clk_o[2]),
		.CLKOS3(clk_o[3]),
		.CLKFB(CLKOP),
		.CLKINTFB(),
		.PHASESEL1(PHASESEL_HW[1]),
		.PHASESEL0(PHASESEL_HW[0]),
		.PHASEDIR(phasedir),
		.PHASESTEP(phasestep),
		.PHASELOADREG(phaseloadreg),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.ENCLKOS(1'b0),
		.ENCLKOS2(1'b0),
		.ENCLKOS3(1'b0),
		.LOCK(locked)
	);
	assign clk_o[0] = CLKOP;

endmodule

// Adapted from https://www.fpga4fun.com/HDMI.html
module hdmi(
	input pixclk,
	input hSync, vSync, DrawArea,
	input [2:0] rgb,
	output [2:0] TMDS,
	output TMDS_clock
);
	wire [7:0] red = {8{rgb[0]}}, green = {8{rgb[1]}}, blue = {8{rgb[2]}};
	
	wire [9:0] TMDS_red, TMDS_green, TMDS_blue;
	TMDS_encoder encode_R(.clk(pixclk), .VD(red  ), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_red));
	TMDS_encoder encode_G(.clk(pixclk), .VD(green), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_green));
	TMDS_encoder encode_B(.clk(pixclk), .VD(blue ), .CD({vSync,hSync}), .VDE(DrawArea), .TMDS(TMDS_blue));
	
	// In the rest of this module we emit the TMDS encoded data 10 times faster than our clock so that each
	// clock cycle we emit the 10 bit necessary to draw a pixel.

	wire clk_TMDS;  // 25MHz x 10 = 250MHz
`ifdef SYNTHESIS
	wire [3:0] clocks;
	wire locked;
	ecp5pll #(.in_hz(25000000), .out0_hz(250000000))
	ecp5pll_inst(.clk_i(pixclk), .clk_o(clocks),
		.reset(1'b0),
		.standby(1'b0),
		.phasesel(2'b00),
		.phasedir(1'b0), .phasestep(1'b0), .phaseloadreg(1'b0),
		.locked(locked)
	);
	assign clk_TMDS = clocks[0];
`else
	// https://www.chipverify.com/verilog/verilog-clock-generator
	// assiming 1 clock cycle is 10 units of time
	reg simulated_pll = 0;
	always begin
		#1 simulated_pll = 0;
		#1 simulated_pll = 1;
	end
	assign clk_TMDS = simulated_pll;
`endif

	reg [9:0] TMDS_shift_red=0, TMDS_shift_green=0, TMDS_shift_blue=0;
	reg [3:0] TMDS_mod10=0;  // modulus 10 counter
	reg TMDS_shift_load=0;
	always @(posedge clk_TMDS) TMDS_shift_load <= (TMDS_mod10==4'd9);
	always @(posedge clk_TMDS) begin
		TMDS_shift_red   <= TMDS_shift_load ? TMDS_red   : TMDS_shift_red  [9:1];
		TMDS_shift_green <= TMDS_shift_load ? TMDS_green : TMDS_shift_green[9:1];
		TMDS_shift_blue  <= TMDS_shift_load ? TMDS_blue  : TMDS_shift_blue [9:1];
		TMDS_mod10 <= (TMDS_mod10==4'd9) ? 4'd0 : TMDS_mod10+4'd1;
	end

	assign TMDS[2] = TMDS_shift_red[0];
	assign TMDS[1] = TMDS_shift_green[0];
	assign TMDS[0] = TMDS_shift_blue[0];
	assign TMDS_clock = pixclk;
endmodule

module test_pattern_top(
	input clk_25mhz,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);

	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	wire hsync, vsync, display_on;
	wire [9:0] hpos, vpos;
	hvsync_generator_hdmi hvsync_gen(clk_25mhz, 1'b0, hsync, vsync, display_on, hpos, vpos);

	wire r = display_on & (((hpos&7)==0) | ((vpos&7)==0));
	wire g = display_on & vpos[4];
	wire b = display_on & hpos[4];
	wire [2:0] rgb = {b,g,r};

	hdmi out(
		.pixclk(clk_25mhz),
		.hSync(hsync),
		.vSync(vsync),
		.DrawArea(display_on),
		.rgb(rgb),
		.TMDS(gpdi_dp[2:0]),
		.TMDS_clock(gpdi_dp[3])
	);
endmodule

// Divides the input clock signal by 2^i where i=1..N
module divider
	#(parameter N = 16)
	(input clk, input rst, output reg [N-1:0] out);

	always @(posedge clk) begin
		if (rst) out <= 0;
		else out <= out + 1;
	end
endmodule

module divider_top(
	input clk_25mhz,
	output [7:0] led,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);
	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	localparam N = 28;
	wire [N-1:0] clk_div;
	divider #(N) div(.clk(clk_25mhz), .rst(btn[1]), .out(clk_div));
	assign led = clk_div[N-1:N-1-8];
endmodule

module digits10(
	input [3:0] digit,    // digit 0-9
	input [2:0] yofs,     // vertical offset (0-4)
	output reg [4:0] bits // output (5 bits)
);

	wire [6:0] caseexpr = {digit,yofs};
	always @(*)
		case (caseexpr) /*{w:5,h:5,count:10}*/
			7'o000: bits = 5'b11111;
			7'o001: bits = 5'b10001;
			7'o002: bits = 5'b10001;
			7'o003: bits = 5'b10001;
			7'o004: bits = 5'b11111;
			// 7'o005-7'o007 = 0
			7'o010: bits = 5'b01100;
			7'o011: bits = 5'b00100;
			7'o012: bits = 5'b00100;
			7'o013: bits = 5'b00100;
			7'o014: bits = 5'b11111;
			// 7'o015-7'o017 = 0
			7'o020: bits = 5'b11111;
			7'o021: bits = 5'b00001;
			7'o022: bits = 5'b11111;
			7'o023: bits = 5'b10000;
			7'o024: bits = 5'b11111;
			// 7'o025-7'o027 = 0
			7'o030: bits = 5'b11111;
			7'o031: bits = 5'b00001;
			7'o032: bits = 5'b11111;
			7'o033: bits = 5'b00001;
			7'o034: bits = 5'b11111;
			// 7'o035-7'o037 = 0
			7'o040: bits = 5'b10001;
			7'o041: bits = 5'b10001;
			7'o042: bits = 5'b11111;
			7'o043: bits = 5'b00001;
			7'o044: bits = 5'b00001;
			// 7'o045-7'o047 = 0
			7'o050: bits = 5'b11111;
			7'o051: bits = 5'b10000;
			7'o052: bits = 5'b11111;
			7'o053: bits = 5'b00001;
			7'o054: bits = 5'b11111;
			// 7'o055-7'o057 = 0
			7'o060: bits = 5'b11111;
			7'o061: bits = 5'b10000;
			7'o062: bits = 5'b11111;
			7'o063: bits = 5'b10001;
			7'o064: bits = 5'b11111;
			// 7'o065-7'o067 = 0
			7'o070: bits = 5'b11111;
			7'o071: bits = 5'b00001;
			7'o072: bits = 5'b00001;
			7'o073: bits = 5'b00001;
			7'o074: bits = 5'b00001;
			// 7'o075-7'o077 = 0
			7'o100: bits = 5'b11111;
			7'o101: bits = 5'b10001;
			7'o102: bits = 5'b11111;
			7'o103: bits = 5'b10001;
			7'o104: bits = 5'b11111;
			// 7'o105-7'o107 = 0
			7'o110: bits = 5'b11111;
			7'o111: bits = 5'b10001;
			7'o112: bits = 5'b11111;
			7'o113: bits = 5'b00001;
			7'o114: bits = 5'b11111;
			// 7'o115-7'o177 = 0
			default: bits = 0;
	endcase
endmodule

module digits_tb;
	reg clk = 0, rst = 0;
	always #1 clk = ~clk;
	
	wire hSync, vSync, display_on;
	wire [9:0] hpos, vpos;

	hvsync_generator_hdmi hvsync_gen(clk, rst, hSync, vSync, display_on, hpos, vpos);

	wire [3:0] digit = hpos[7:4];
	wire [2:0] xofs = hpos[3:1];
	wire [2:0] yofs = vpos[3:1];
	wire [4:0] bits;

	digits10 dut(.digit(digit), .yofs(yofs), .bits(bits));

	wire [2:0] rgb = {1'b0,display_on && xofs >= 3'b011 && bits[xofs ^ 3'b111],1'b0};

	initial begin
		$dumpfile("digits_tb.vcd");
		$dumpvars;
		rst = 1;
		#2
		rst = 0;
		#(800*525 * 2) // since every clock cycle is 2 units of time
		$finish();
	end
endmodule

module digits10_top(
	input clk_25mhz,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);

	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	wire hsync, vsync, display_on;
	wire [9:0] hpos, vpos;
	hvsync_generator_hdmi hvsync_gen(clk_25mhz, 1'b0, hsync, vsync, display_on, hpos, vpos);

	wire [3:0] digit = hpos[7:4];
	wire [2:0] xofs = hpos[3:1];
	wire [2:0] yofs = vpos[3:1];
	wire [4:0] bits;

	digits10 numbers(.digit(digit), .yofs(yofs), .bits(bits));

	wire r = display_on && 0;
	wire g = display_on && xofs >= 3'b011 && bits[xofs ^ 3'b111];
	wire b = display_on && 0;
	wire [2:0] rgb = {b,g,r};
	
	hdmi out(
		.pixclk(clk_25mhz),
		.hSync(hsync),
		.vSync(vsync),
		.DrawArea(display_on),
		.rgb(rgb),
		.TMDS(gpdi_dp[2:0]),
		.TMDS_clock(gpdi_dp[3])
	);
endmodule

module bouncing_ball_top(
	input clk_25mhz,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);
	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	wire hsync, vsync, display_on;
	wire [9:0] hpos, vpos;
	hvsync_generator_hdmi hvsync_gen(clk_25mhz, 1'b0, hsync, vsync, display_on, hpos, vpos);

	localparam ball_horiz_initial = 128;
	localparam ball_vert_initial = 128;
	localparam BALL_SIZE = 4;

	reg [9:0] ball_hpos = ball_horiz_initial, ball_vpos = ball_vert_initial;
	reg [9:0] ball_hvel = -2, ball_vvel = 2;

	always @(posedge vsync) begin
		ball_hpos <= ball_hpos + ball_hvel;
		ball_vpos <= ball_vpos + ball_vvel;
	end

	wire ball_vert_collide  = ball_vpos >= 480 - BALL_SIZE;
	wire ball_horiz_collide = ball_hpos >= 640 - BALL_SIZE;
	always @(posedge ball_vert_collide)  ball_vvel <= -ball_vvel;
	always @(posedge ball_horiz_collide) ball_hvel <= -ball_hvel;

	// offset of ball position from video beam
	wire [9:0] ball_hdiff = hpos - ball_hpos;
	wire [9:0] ball_vdiff = vpos - ball_vpos;

	wire ball_hgfx = ball_hdiff < BALL_SIZE;
	wire ball_vgfx = ball_vdiff < BALL_SIZE;
	wire ball_gfx  = ball_hgfx && ball_vgfx;

	wire grid_gfx = (((hpos&7)==0) && ((vpos&7)==0));
	wire r = display_on && (ball_hgfx | ball_gfx);
	wire g = display_on && (grid_gfx  | ball_gfx);
	wire b = display_on && (ball_vgfx | ball_gfx);
	wire [2:0] rgb = {b,g,r};

	hdmi out(
		.pixclk(clk_25mhz),
		.hSync(hsync),
		.vSync(vsync),
		.DrawArea(display_on),
		.rgb(rgb),
		.TMDS(gpdi_dp[2:0]),
		.TMDS_clock(gpdi_dp[3])
	);
endmodule

module player_stats(
	input reset,
	output reg [3:0] score0,
	output reg [3:0] score1,
	output reg [3:0] lives,
	input incscore,
	input declives
);

	initial begin
		score0 = 0;
		score1 = 0;
		lives = 3;
	end

	always @(posedge incscore or posedge reset) begin
		if (reset) begin
			score0 <= 0;
			score1 <= 0;
		end else if (score0 == 9) begin
			score0 <= 0;
			score1 <= score1 == 9 ? 0 : score1+1;
		end else begin
			score0 <= score0 + 1;
		end
	end

	always @(posedge declives or posedge reset) begin
		if (reset)
			lives <= 3;
		else if (lives != 0)
			lives <= lives - 1;
	end
endmodule

`ifndef SYNTHESIS
module player_stats_tb;
	reg rst = 0;

	reg incscore = 0, declives = 0;
	wire [3:0] score0, score1, lives;
	player_stats dut(rst, score0, score1, lives, incscore, declives);
	always #1 begin
		incscore = ~incscore;
		declives = ~declives;
	end

	initial begin
		$dumpfile("player_stats_tb.vcd");
		$dumpvars;
		rst = 1;
		@(posedge incscore);
		rst = 0;
		repeat (100) @(posedge incscore);
		$finish();
	end
endmodule
`endif

module scoreboard_generator(
	input [3:0] score0,
	input [3:0] score1,
	input [3:0] lives,
	input [9:0] vpos,
	input [9:0] hpos,
	output board_gfx
);

	reg [3:0] score_digit;
	wire [4:0] score_bits;

	always @(*) begin
		case (hpos[7:5])
			1: score_digit = score1;
			2: score_digit = score0;
			6: score_digit = lives;
			default: score_digit = 15; // no digit
		endcase
	end

	digits10 digits(
		.digit(score_digit),
		.yofs(vpos[4:2]),
		.bits(score_bits)
	);

	wire [2:0] xofs = hpos[4:2];
	assign board_gfx = xofs >= 3'b011 && score_bits[xofs ^ 3'b111];
endmodule

// TODO: aggiustare dimensioni della grafica per far si che usino bene lo
// schermo.
module ball_paddle_top(
	input clk_25mhz,
	input [6:0] btn,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);
	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	wire reset = ~btn[0];

	wire hsync, vsync, display_on;
	wire [9:0] hpos, vpos;
	hvsync_generator_hdmi hvsync_gen(clk_25mhz, 1'b0, hsync, vsync, display_on, hpos, vpos);

	localparam BRICKS_H = 16, BRICKS_V = 8;

	localparam BALL_DIR_LEFT = 0, BALL_DIR_RIGHT = 1;
	localparam BALL_DIR_UP = 0, BALL_DIR_DOWN = 1;

	localparam PADDLE_WIDTH = 31;
	localparam BALL_SIZE = 6;

	reg [9:0] paddle_x = 0;

	reg [9:0] ball_x, ball_y;
	reg ball_dir_x;   // ball X direction (0=left, 1=right)
	reg ball_speed_x; // ball speed (0=1 pixel/frame, 1=2 pixels/frame)
	reg ball_dir_y;   // ball Y direction (0=up, 1=down)

	// reg brick_array [0:BRICKS_H*BRICKS_V-1] = 0; // 16*8 = 128 bits
	// active-low
	reg [0:BRICKS_H*BRICKS_V-1] brick_array = 0; // 16*8 = 128 bits

	wire [3:0] score0, score1, lives;
	reg incscore;
	reg declives = 0;  // TODO

	wire score_gfx;
	player_stats stats(
		.reset(reset),
		.score0(score0), .score1(score1), .lives(lives),
		.incscore(incscore), .declives(declives)
	);
	scoreboard_generator score_gen(
		.score0(score0), .score1(score1), .lives(lives),
		.vpos(vpos), .hpos(hpos),
		.board_gfx(score_gfx)
	);

	wire [5:0] hcell = hpos[8:3], vcell = vpos[8:3];
	wire lr_border = hcell==0 || hcell==31;

	// TODO: unsigned compare doesn't work in JS
	wire [9:0] paddle_rel_x = ((hpos-paddle_x) & 9'h1ff);

	wire paddle_gfx = (vcell == 28) && (paddle_rel_x < PADDLE_WIDTH);

	wire [9:0] ball_rel_x = (hpos - ball_x);
	wire [9:0] ball_rel_y = (vpos - ball_y);

	wire ball_gfx = ball_rel_x < BALL_SIZE && ball_rel_y < BALL_SIZE;

	reg static_collidable_gfx; // main graphics signal (bricks and borders)
	reg brick_present;
	reg [6:0] brick_index;
	wire brick_gfx = lr_border || (brick_present && vpos[2:0] != 0 && hpos[3:1] != 4);

	// scan bricks: compute brick_index and brick_present flag
	always @(posedge clk_25mhz)
		// see if we are scanning brick area
		if (vpos[8:6] == 1 && !lr_border) begin
			// every 16th pixel, starting at 8
			if (hpos[3:0] == 8) begin
				brick_index <= {vpos[5:3], hpos[7:4]};
			end
			// every 17th pixel
			else if (hpos[3:0] == 9) begin
				brick_present <= !brick_array[brick_index];
			end
		end else begin
			brick_present <= 0;
		end

	always @(posedge vsync) // TODO: use a divider with hsync signal instead.
		     if (btn[5]) paddle_x <= paddle_x - 1;
		else if (btn[6]) paddle_x <= paddle_x + 1;

	wire ball_static_pixel_collide = static_collidable_gfx & ball_gfx;

	reg ball_collide_paddle = 0;
	reg [3:0] ball_collide_bits = 0;

	// compute ball collisions with paddle and playfield
	always @(posedge clk_25mhz)
		// clear all collide bits for frame
		if (vsync) begin
			ball_collide_bits <= 0;
			ball_collide_paddle <= 0;
		end else begin
			if (ball_static_pixel_collide) begin
				// did we collide w/ paddle?
				if (paddle_gfx) ball_collide_paddle <= 1;
				// ball has 4 collision quadrants
				if (!ball_rel_x[2] & !ball_rel_y[2]) ball_collide_bits[0] <= 1;
				if ( ball_rel_x[2] & !ball_rel_y[2]) ball_collide_bits[1] <= 1;
				if (!ball_rel_x[2] &  ball_rel_y[2]) ball_collide_bits[2] <= 1;
				if ( ball_rel_x[2] &  ball_rel_y[2]) ball_collide_bits[3] <= 1;
			end
		end

	// compute ball collisions with brick and increment score
	always @(posedge clk_25mhz)
		if (ball_static_pixel_collide && brick_present) begin
			brick_array[brick_index] <= 1;
			incscore <= 1; // increment score
		end else begin
			incscore <= 0; // reset incscore
		end

	// computes position of ball in relation to center of paddle
	wire signed [9:0] ball_paddle_dx = ball_x - paddle_x + 8;

	// ball bounce: determine new velocity/direction
	always @(posedge vsync or posedge reset) begin
		if (reset) begin
			ball_dir_y <= BALL_DIR_DOWN;
		end else if (ball_collide_paddle) begin
			// bounces upward off of paddle
			ball_dir_y <= BALL_DIR_UP;
			// which side of paddle, left/right?
			ball_dir_x <= (ball_paddle_dx < 20) ? BALL_DIR_LEFT : BALL_DIR_RIGHT;
			// hitting with edge of paddle makes it fast
			ball_speed_x <= ball_collide_bits != 4'b1100;
		end else begin
			// collided with playfield
			// TODO: can still slip through corners
			// compute left/right bounce
			casez (ball_collide_bits)
				4'b01?1: ball_dir_x <= BALL_DIR_RIGHT; // left edge/corner
				4'b1101: ball_dir_x <= BALL_DIR_RIGHT; // left corner
				4'b101?: ball_dir_x <= BALL_DIR_LEFT;  // right edge/corner
				4'b1110: ball_dir_x <= BALL_DIR_LEFT;  // right corner
				default: ;
			endcase
			// compute top/bottom bounce
			casez (ball_collide_bits)
				4'b1011: ball_dir_y <= BALL_DIR_DOWN;
				4'b0111: ball_dir_y <= BALL_DIR_DOWN;
				4'b001?: ball_dir_y <= BALL_DIR_DOWN;
				4'b0001: ball_dir_y <= BALL_DIR_DOWN;
				4'b0100: ball_dir_y <= BALL_DIR_UP;
				4'b1?00: ball_dir_y <= BALL_DIR_UP;
				4'b1101: ball_dir_y <= BALL_DIR_UP;
				4'b1110: ball_dir_y <= BALL_DIR_UP;
				default: ;
			endcase
		end
	end

	// ball motion: update ball position
	always @(negedge vsync or posedge reset) begin
		if (reset) begin
			// reset ball position to top center
			ball_x <= 128;
			ball_y <= 180;
		end else begin
			// move ball horizontal and vertical position
			if (ball_dir_x == BALL_DIR_RIGHT)
				ball_x <= ball_x + (ball_speed_x?1:0) + 1;
			else
				ball_x <= ball_x - (ball_speed_x?1:0) - 1;
			ball_y <= ball_y + (ball_dir_y==BALL_DIR_DOWN?1:-1);
		end
	end

	always @* begin
		case (vcell)
			0,1,2: static_collidable_gfx = score_gfx;
			3: static_collidable_gfx = 0;
			4: static_collidable_gfx = 1; // top border
			8,9,10,11,12,13,14,15: static_collidable_gfx = brick_gfx;
			28: static_collidable_gfx = paddle_gfx | lr_border;
			29: static_collidable_gfx = hpos[0] ^ vpos[0]; // bottom border
			default: static_collidable_gfx = lr_border;
		endcase
	end

	wire grid_gfx = (((hpos&7)==0) || ((vpos&7)==0));
	wire r = ball_gfx | paddle_gfx;
	wire g = static_collidable_gfx | ball_gfx;
	wire b = grid_gfx | ball_gfx | brick_present;
	wire [2:0] rgb = {3{display_on}} & {b,g,r};

	hdmi out(
		.pixclk(clk_25mhz), .hSync(hsync), .vSync(vsync),
		.DrawArea(display_on), .rgb(rgb),
		.TMDS(gpdi_dp[2:0]), .TMDS_clock(gpdi_dp[3])
	);
endmodule

`ifndef SYNTHESIS
module ball_paddle_top_tb;
	reg clk = 0, rst = 0;
	always #10 clk = ~clk;

	wire [3:0] gpdi_dp;
	wire ignore;
	ball_paddle_top dut(
		.clk_25mhz(clk),
		.btn({6'b000000, ~rst}),
		.gpdi_dp(gpdi_dp),
		.wifi_gpio0(ignore)
	);

	initial begin
		$dumpfile("ball_paddle_top_tb.vcd");
		$dumpvars(0, dut);
		$dumpoff;
		rst = 1;
		@(posedge clk);
		dut.ball_x = 240;
		rst = 0;
		$dumpon;
		// https://www.chipverify.com/verilog/verilog-delay-control
		// This does not seem to terminate...
		wait (dut.ball_static_pixel_collide);
		repeat(800*525) @(posedge clk);
		$finish();
	end
endmodule
`endif

module my_ball_paddle_top(
	input clk_25mhz,
	input [6:0] btn,
	output [3:0] gpdi_dp,
	output wifi_gpio0
);
	// Tie GPIO0, keep board from rebooting
	assign wifi_gpio0 = 1'b1;

	wire reset = ~btn[0];

	wire hsync, vsync, display_on;
	wire [9:0] hpos, vpos;
	hvsync_generator_hdmi hvsync_gen(clk_25mhz, reset, hsync, vsync, display_on, hpos, vpos);

	reg [5:0] hpos_div10 = 0, vpos_div10 = 0;
	reg [3:0] hpos_mod10 = 0, vpos_mod10 = 0;
	reg should_sample_on_hsync = 0;
	always @(posedge clk_25mhz) begin
		if (vsync) begin
			hpos_mod10 <= 0; hpos_div10 <= 0;
			vpos_mod10 <= 0; vpos_div10 <= 0;
		end else if (hsync) begin
			hpos_mod10 <= 0;
			if (should_sample_on_hsync) begin
				if (vpos_mod10 == 9) begin
					vpos_mod10 <= 0;
					vpos_div10 <= vpos_div10 + 1;
				end else
					vpos_mod10 <= vpos_mod10 + 1;
				// vpos_mod10 <= vpos_mod10 == 9 ? 0 : vpos_mod10 + 1;
				should_sample_on_hsync <= 0;
			end
		end else if (display_on) begin
			if (hpos_mod10 == 9) begin
				hpos_mod10 <= 0;
				hpos_div10 <= hpos_div10 + 1;
			end else
				hpos_mod10 <= hpos_mod10 + 1;
			// hpos_mod10 <= hpos_mod10 == 9 ? 0 : hpos_mod10 + 1;
			should_sample_on_hsync <= 1;
		end
	end

	localparam BALL_SIZE = 6;
	localparam PADDLE_WIDTH = 31;

	wire [5:0] hcell = hpos_div10, vcell = vpos_div10;

	wire lr_border = hcell == 0 || hcell == 63;
	wire grid_gfx = hpos_mod10 == 0 || vpos_mod10 == 0;

	reg [9:0] ball_x   = 640/2;
	reg [9:0] ball_y   = 480/2;
	reg [9:0] paddle_x = 640/2;

	// When the delta is negative since it is a 2's complement number it is just
	// interpreted as a big number.
	wire [9:0] ball_rel_x   = hpos - ball_x;
	wire [9:0] ball_rel_y   = vpos - ball_y;
	wire [9:0] paddle_rel_x = hpos - paddle_x;

	wire ball_gfx   = ball_rel_x < BALL_SIZE && ball_rel_y < BALL_SIZE;
	wire paddle_gfx = (vcell == 46) && (paddle_rel_x < PADDLE_WIDTH);

	wire ball_static_pixel_collide = static_collidable_gfx & ball_gfx;

	reg static_collidable_gfx = 0;
	always @(*) begin
		case (vcell)
			0,1,2: static_collidable_gfx = 0;
			3: static_collidable_gfx = 0;
			4: static_collidable_gfx = 1; // top border
			// 5,6,7:
			8,9,10,11,12,13,14,15: static_collidable_gfx = 0 | lr_border; // brick rows 1-8
			// 16,17,18,19,20,21,22,23,24,25,26,27
			28: static_collidable_gfx = 0 | lr_border;
			47: static_collidable_gfx = hpos[0] ^ vpos[0]; // bottom border
			default: static_collidable_gfx = lr_border; // left/right borders
		endcase
	end

	wire r = ball_gfx | paddle_gfx;
	wire g = static_collidable_gfx;
	wire b = grid_gfx;
	wire [2:0] rgb = {b,g,r};

	hdmi out(
		.pixclk(clk_25mhz),
		.hSync(hsync),
		.vSync(vsync),
		.DrawArea(display_on),
		.rgb(rgb),
		.TMDS(gpdi_dp[2:0]),
		.TMDS_clock(gpdi_dp[3])
	);
endmodule

`ifndef SYNTHESIS
module my_ball_paddle_top_tb;
	reg clk = 0, rst = 0;
	always #10 clk = ~clk;

	wire [3:0] gpdi_dp;
	wire ignore;
	my_ball_paddle_top dut(
		.clk_25mhz(clk),
		.btn({6'b000000, ~rst}),
		.gpdi_dp(gpdi_dp),
		.wifi_gpio0(ignore)
	);

	initial begin
		$dumpfile("my_ball_paddle_top_tb.vcd");
		$dumpvars;
		rst = 1;
		@(posedge clk);
		rst = 0;
		repeat(800*525) @(posedge clk);
		$finish();
	end
endmodule
`endif