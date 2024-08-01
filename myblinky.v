// Divides the input clock signal by 2^i where i=1..N
module divider
	#(parameter N = 16)
	(input clk, input rst, output reg [N-1:0] out);

	always @(posedge clk) begin
		if (rst) out <= 0;
		else out <= out + 1;
	end
endmodule

module hvsync_generator (
		input clk,
		input rst,
		output reg hsync,
		output reg vsync,
		output display_on,
		output reg [8:0] hpos,
		output reg [8:0] vpos
	);

	parameter H_DISPLAY       = 256; // horizontal display width
	parameter H_BACK          =  23; // horizontal left border (back porch)
	parameter H_FRONT         =   7; // horizontal right border (front porch)
	parameter H_SYNC          =  23; // horizontal sync width
	// vertical constants
	parameter V_DISPLAY       = 240; // vertical display height
	parameter V_TOP           =   5; // vertical top border
	parameter V_BOTTOM        =  14; // vertical bottom border
	parameter V_SYNC          =   3; // vertical sync # lines
	// derived constants
	parameter H_SYNC_START    = H_DISPLAY + H_FRONT;
	parameter H_SYNC_END      = H_DISPLAY + H_FRONT + H_SYNC - 1;
	parameter H_MAX           = H_DISPLAY + H_BACK + H_FRONT + H_SYNC - 1;
	parameter V_SYNC_START    = V_DISPLAY + V_BOTTOM;
	parameter V_SYNC_END      = V_DISPLAY + V_BOTTOM + V_SYNC - 1;
	parameter V_MAX           = V_DISPLAY + V_TOP + V_BOTTOM + V_SYNC - 1;

	wire hmaxxed = (hpos == H_MAX) || reset; // set when hpos is maximum
	wire vmaxxed = (vpos == V_MAX) || reset; // set when vpos is maximum

	always @(posedge clk) begin
		// horizontal position counter
		hsync <= (hpos >= H_SYNC_START && hpos <= H_SYNC_END);
		if (hmaxxed) hops <= 0;
		else hpos <= hpos + 1;
		// vertical position counter
		vsync <= (vpos <= V_SYNC_START && vpos <= V_SYNC_END);
		if (hmaxxed && vmaxxed) vpos <= 0;
		else vpos <= vpos + 1;
	end

	// set when the the beam is in visible frame
	assign display_on = (hpos < H_DISPLAY) && (vpos < V_DISPLAY);
endmodule

// Adapted from https://www.fpga4fun.com/HDMI.html
module HDMI_test(
	input pixclk,  // 25MHz
	output [2:0] TMDS,
	output TMDS_clock,
);
    reg [9:0] CounterX=0, CounterY=0;
    reg hSync, vSync, DrawArea;
    begin
        always @(posedge pixclk) DrawArea <= (CounterX<640) && (CounterY<480);
        always @(posedge pixclk) CounterX <= (CounterX==799) ? 0 : CounterX+1;
        always @(posedge pixclk) if(CounterX==799) CounterY <= (CounterY==524) ? 0 : CounterY+1;
        always @(posedge pixclk) hSync <= (CounterX>=656) && (CounterX<752);
        always @(posedge pixclk) vSync <= (CounterY>=490) && (CounterY<492);
    end

    reg [7:0] red, green, blue;
    begin
        wire [7:0] W = {8{CounterX[7:0]==CounterY[7:0]}}; // White
        wire [7:0] A = {8{CounterX[7:5]==3'h2 && CounterY[7:5]==3'h2}}; // Alpha
        always @(posedge pixclk) red <= ({CounterX[5:0] & {6{CounterY[4:3]==~CounterX[4:3]}}, 2'b00} | W) & ~A;
        always @(posedge pixclk) green <= (CounterX[7:0] & {8{CounterY[6]}} | W) & ~A;
        always @(posedge pixclk) blue <= CounterY[7:0] | W | A;
    end

    wire [9:0] TMDS_red, TMDS_green, TMDS_blue;
    begin
        TMDS_encoder encode_R(.clk(pixclk), .VD(red  ), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_red));
        TMDS_encoder encode_G(.clk(pixclk), .VD(green), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_green));
        TMDS_encoder encode_B(.clk(pixclk), .VD(blue ), .CD({vSync,hSync}), .VDE(DrawArea), .TMDS(TMDS_blue));
    end
    
    // In the rest of this module we emit the TMDS encoded data 10 times faster than our clock so that each
    // clock cycle we emit the 10 bit necessary to draw a pixel.

    wire clk_TMDS;  // 25MHz x 10 = 250MHz
    begin
        wire [3:0] clocks;
        ecp5pll #(.in_hz(25000000), .out0_hz(250000000)) ecp5pll_inst(.clk_i(pixclk), .clk_o(clocks));
        assign clk_TMDS = clocks[0];
    end

    reg [9:0] TMDS_shift_red=0, TMDS_shift_green=0, TMDS_shift_blue=0;
    begin
        reg [3:0] TMDS_mod10=0;  // modulus 10 counter
        reg TMDS_shift_load=0;
        always @(posedge clk_TMDS) TMDS_shift_load <= (TMDS_mod10==4'd9);
        always @(posedge clk_TMDS) begin
            TMDS_shift_red   <= TMDS_shift_load ? TMDS_red   : TMDS_shift_red  [9:1];
            TMDS_shift_green <= TMDS_shift_load ? TMDS_green : TMDS_shift_green[9:1];
            TMDS_shift_blue  <= TMDS_shift_load ? TMDS_blue  : TMDS_shift_blue [9:1];
            TMDS_mod10 <= (TMDS_mod10==4'd9) ? 4'd0 : TMDS_mod10+4'd1;
        end
    end

    assign TMDS[2] = TMDS_shift_red[0];
    assign TMDS[1] = TMDS_shift_green[0];
    assign TMDS[0] = TMDS_shift_blue[0];
    assign TMDS_clock = pixclk;
endmodule

module TMDS_encoder(
	input clk,
	input [7:0] VD,  // video data (red, green or blue)
	input [1:0] CD,  // control data
	input VDE,  // video data enable, to choose between CD (when VDE=0) and VD (when VDE=1)
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

module top(
		input clk_25mhz,
		input [6:0] btn,
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

	HDMI_test hdmi(.pixclk(clk_25mhz), .TMDS(gpdi_dp[2:0]), .TMDS_clock(gpdi_dp[3]));
endmodule

// TODO: remove the need for this by using EHXPLLL directly https://www.digikey.it/en/maker/projects/2028ce62001b4cb69335f48e127fa366
// Taken from https://github.com/emard/ulx3s-misc/blob/master/examples/ecp5pll/hdl/sv/ecp5pll.sv
// https://github.com/emard/ulx3s-misc/blob/master/examples/dvi/top/top_vgatest.v
// Simpler version but hard to undestand what it id doing
// https://github.com/lawrie/ulx3s_examples/blob/master/hdmi/pll.v
module ecp5pll #(
		parameter integer in_hz      =  25000000,
		parameter integer out0_hz    =  25000000,
		parameter integer out0_deg   =         0, // keep 0
		parameter integer out0_tol_hz=         0, // tolerance: if freq differs more, then error
		parameter integer out1_hz    =         0,
		parameter integer out1_deg   =         0,
		parameter integer out1_tol_hz=         0,
		parameter integer out2_hz    =         0,
		parameter integer out2_deg   =         0,
		parameter integer out2_tol_hz=         0,
		parameter integer out3_hz    =         0,
		parameter integer out3_deg   =         0,
		parameter integer out3_tol_hz=         0,
		parameter integer reset_en   =         0,
		parameter integer standby_en =         0,
		parameter integer dynamic_en =         0
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

	function integer F_ecp5pll(input integer x);
	  integer input_div, input_div_min, input_div_max;
	  integer output_div, output_div_min, output_div_max;
	  integer feedback_div, feedback_div_min, feedback_div_max;
	  integer fvco, fout;
	  integer error, error_prev;
	  integer params_fvco;
	  integer div1, div2, div3;

	  integer params_refclk_div;
	  integer params_feedback_div;
	  integer params_output_div;

	  params_fvco = 0;
	  error_prev = 999999999;
	  input_div_min = in_hz/PFD_MAX;
	  if(input_div_min < 1)
	    input_div_min = 1;
	  input_div_max = in_hz/PFD_MIN;
	  if(input_div_max > 128)
	    input_div_max = 128;
	  for(input_div = input_div_min; input_div <= input_div_max; input_div=input_div+1)
	  begin
	    if(out0_hz / 1000000 * input_div < 2000)
	      feedback_div = out0_hz * input_div / in_hz;
	    else
	      feedback_div = out0_hz / in_hz * input_div;
	    feedback_div_min = feedback_div;
	    feedback_div_max = feedback_div+1;
	    if(feedback_div_min < 1)
	      feedback_div_min = 1;
	    if(feedback_div_max > 80)
	      feedback_div_max = 80;
	    for(feedback_div = feedback_div_min; feedback_div <= feedback_div_max; feedback_div = feedback_div+1)
	    begin
	      output_div_min = (VCO_MIN/feedback_div) / (in_hz/input_div);
	      if(output_div_min < 1)
	        output_div_min = 1;
	      output_div_max = (VCO_MAX/feedback_div) / (in_hz/input_div);
	      if(output_div_max > 128)
	        output_div_max = 128;
	      fout = in_hz * feedback_div / input_div;
	      for(output_div = output_div_min; output_div <= output_div_max; output_div=output_div+1)
	      begin
	        fvco = fout * output_div;
	        error = abs(fout-out0_hz)
	              + (out1_hz > 0 ? abs(fvco/(fvco >= out1_hz ? fvco/out1_hz : 1)-out1_hz) : 0)
	              + (out2_hz > 0 ? abs(fvco/(fvco >= out2_hz ? fvco/out2_hz : 1)-out2_hz) : 0)
	              + (out3_hz > 0 ? abs(fvco/(fvco >= out3_hz ? fvco/out3_hz : 1)-out3_hz) : 0);
	        if( error < error_prev
	        || (error == error_prev && abs(fvco-VCO_OPTIMAL) < abs(params_fvco-VCO_OPTIMAL)) )
	        begin
	          error_prev              = error;
	          params_refclk_div       = input_div;
	          params_feedback_div     = feedback_div;
	          params_output_div       = output_div;
	          params_fvco             = fvco;
	        end
	      end
	    end
	  end
	  // FIXME in the future when yosys supports struct
	  if(x==0) F_ecp5pll = params_refclk_div;
	  if(x==1) F_ecp5pll = params_feedback_div;
	  if(x==2) F_ecp5pll = params_output_div;
	endfunction

	function integer F_primary_phase(input integer output_div, deg);
	  integer phase_compensation;
	  integer phase_count_x8;

	  phase_compensation = (output_div+1)/2*8-8+output_div/2*8; // output_div/2*8 = 180 deg shift
	  phase_count_x8     = phase_compensation + 8*output_div*deg/360;
	  if(phase_count_x8 > 1023)
	    phase_count_x8 = phase_count_x8 % (output_div*8); // wraparound 360 deg
	  F_primary_phase = phase_count_x8;
	endfunction

	// FIXME it is inefficient to call F_ecp5pll multiple times
	localparam params_refclk_div       = F_ecp5pll(0);
	localparam params_feedback_div     = F_ecp5pll(1);
	localparam params_output_div       = F_ecp5pll(2);
	localparam params_fout             = in_hz * params_feedback_div / params_refclk_div;
	localparam params_fvco             = params_fout * params_output_div;

	localparam params_primary_phase_x8 = F_ecp5pll(3);
	localparam params_primary_cphase   = F_primary_phase(params_output_div, out0_deg) / 8;
	localparam params_primary_fphase   = F_primary_phase(params_output_div, out0_deg) % 8;

	function integer F_secondary_divisor(input integer sfreq);
	  F_secondary_divisor = 1;
	  if(sfreq > 0)
	    if(params_fvco >= sfreq)
	      F_secondary_divisor = params_fvco/sfreq;
	endfunction

	function integer F_secondary_phase(input integer sfreq, sphase);
	  integer div, freq;
	  integer phase_compensation, phase_count_x8;

	  phase_count_x8 = 0;
	  if(sfreq > 0)
	  begin
	    div = 1;
	    if(params_fvco >= sfreq)
	      div = params_fvco/sfreq;
	    freq = params_fvco/div;
	    phase_compensation = div*8-8;
	    phase_count_x8 = phase_compensation + 8*div*sphase/360;
	    if(phase_count_x8 > 1023)
	      phase_count_x8 = phase_count_x8 % (div*8); // wraparound 360 deg
	  end

	  F_secondary_phase = phase_count_x8;
	endfunction

	localparam params_secondary1_div      = F_secondary_divisor(out1_hz);
	localparam params_secondary1_cphase   = F_secondary_phase  (out1_hz, out1_deg) / 8;
	localparam params_secondary1_fphase   = F_secondary_phase  (out1_hz, out1_deg) % 8;
	localparam params_secondary2_div      = F_secondary_divisor(out2_hz);
	localparam params_secondary2_cphase   = F_secondary_phase  (out2_hz, out2_deg) / 8;
	localparam params_secondary2_fphase   = F_secondary_phase  (out2_hz, out2_deg) % 8;
	localparam params_secondary3_div      = F_secondary_divisor(out3_hz);
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

