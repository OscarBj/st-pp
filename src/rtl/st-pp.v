`timescale 1ns/1ps

module stpp (
	input  wire [4:0]   okUH,
	output wire [3:0]   okHU,
	inout  wire [31:0]  okUHU,
	input  wire [3:0]   okRSVD,
	inout  wire         okAA,

	input  wire         sys_clk_p,
	input  wire         sys_clk_n,
	
	output wire [3:0]   led,
	
	inout  wire [15:0]  ddr3_dq,
	output wire [14:0]  ddr3_addr,
	output wire [2 :0]  ddr3_ba,
	output wire [0 :0]  ddr3_ck_p,
	output wire [0 :0]  ddr3_ck_n,
	output wire [0 :0]  ddr3_cke,
	output wire         ddr3_cas_n,
	output wire         ddr3_ras_n,
	output wire         ddr3_we_n,
	output wire [0 :0]  ddr3_odt,
	output wire [1 :0]  ddr3_dm,
	inout  wire [1 :0]  ddr3_dqs_p,
	inout  wire [1 :0]  ddr3_dqs_n,
	output wire         ddr3_reset_n
	);

// OK RAMTest Parameters
localparam BLOCK_SIZE = 128; // 512 bytes / 4 bytes per word, 
localparam FIFO_SIZE = 1023; // note that Xilinx does not allow use of the full 1024 words
localparam BUFFER_HEADROOM = 20; // headroom for the FIFO count to account for latency

// Capability bitfield, used to indicate features supported by this bitfile:
// [0] - Supports passing calibration status through FrontPanel
localparam CAPABILITY = 16'h0001;

wire          init_calib_complete;
reg           sys_rst;

wire [29 :0]  app_addr;
wire [2  :0]  app_cmd;
wire          app_en;
wire          app_rdy;
wire [127:0]  app_rd_data;
wire          app_rd_data_end;
wire          app_rd_data_valid;
wire [127:0]  app_wdf_data;
wire          app_wdf_end;
reg [15 :0]  app_wdf_mask;
wire          app_wdf_rdy;
wire          app_wdf_wren;

wire          clk;
wire          rst;

// Front Panel

// Target interface bus:
wire         okClk;
wire [112:0] okHE;
wire [64:0]  okEH;

wire [31:0]  ep00wire;

wire         pipe_in_read;
wire [127:0] pipe_in_data;
wire [7:0]   pipe_in_rd_count;
wire [9:0]   pipe_in_wr_count;
wire         pipe_in_valid;
wire         pipe_in_full;
wire         pipe_in_empty;
reg          pipe_in_ready;

wire         pipe_out_write;
wire [127:0] pipe_out_data;
wire [9:0]   pipe_out_rd_count;
wire [7:0]   pipe_out_wr_count;
wire         pipe_out_full;
wire         pipe_out_empty;
reg          pipe_out_ready;

// Pipe Fifos
wire         pi0_ep_write;
wire         po0_ep_read;
wire [31:0]  pi0_ep_dataout;
wire [31:0]  po0_ep_datain;

// Register Bridge
wire [31:0] regAddress;
wire [31:0] regDataOut;
reg [31:0] regStore_host[15:0];
reg [31:0] regStore_fpga[15:0];
initial regStore_host[0] = 32'h2fd; // feature brightness treshold
initial regStore_host[1] = 32'b1; // pixel 1-bit transform treshold
initial regStore_host[2] = 32'h4e520c8;
reg [31:0] regDataIn;
wire regRead;
wire regWrite;
// wire [7:0]regBlock;
// wire [7:0]regWord;

// Circuit behavior
// assign regBlock = regAddress[31:24];
// assign regWord = regAddress[7:0];
	
//Define a block with a limited register range to store data
always @ (posedge okClk) begin
     if(regWrite) begin
        if(regAddress <= 15) begin // For writing pipeline parameters - use registers fpga reads from
            regStore_host[regAddress] <= regDataOut;
        end
    end else if (regRead) begin
        if(regAddress <= 15) begin // For reading metrics - use registers that fpga writes to
            regDataIn <= regStore_fpga[regAddress];
        end
    end
end

// Registers for muxing ddr controller
reg [2:0]       app_cmd_main = 3'b000;
reg             app_en_main;
reg [28 :0]     app_addr_main;

reg [127:0]     app_wdf_data_main;
reg             app_wdf_end_main;
reg             app_wdf_wren_main;

reg [127:0]     img_wdf_data;
reg             img_wdf_wren;
reg             img_wdf_end;
reg             img_en;
reg             img_cmd;
reg [28 :0]     img_addr;
reg [28 :0]     img_rd_addr;
reg [28 :0]     img_wr_addr;

// Registers for testing
reg            led_test_1 = 1'b0;
reg             led_test_2 = 1'b0;
reg             match;
initial match = 1'b1;
reg             done;
initial done = 1'b1;
reg box_filter_done;
initial box_filter_done = 1'b1;
reg ccl_done;
initial ccl_done = 1'b1;
reg feature_rendering_done;
initial feature_rendering_done = 1'b1;
// Calcluate average value of 9 - 8 bit int
function [7:0] windowAvg;
input   [23:0] px1,px2,px3;//,px4,px5,px6,px7,px8,px9;

integer tot;
integer i; // constant
integer f;
begin
    i = 28;
    windowAvg = 8'b0;
    tot = px1[23-:8] + px1[15-:8] + px1[7-:8] + px2[15-:8] + px2[7-:8] + px2[23-:8] + px3[15-:8] + px3[7-:8] + px3[23-:8];// + px4 + px5 + px6 + px7 + px8 + px9;
    f = tot * i;
    windowAvg = f >> 8;
    // windowAvg = windowAvg > 8'b1 ? 8'hff : 8'b0; // tresholding & transformation
end
endfunction

// Led output utility
function [3:0] xem7305_led;
input [3:0] a;
integer i;
begin
	for(i=0; i<4; i=i+1) begin: u
		xem7305_led[i] = (a[i]==1'b1) ? (1'b0) : (1'bz);
	end
end
endfunction

assign led = xem7305_led({match,led_test_2,led_test_1,ep00wire[3]});

	//MIG Infrastructure Reset
reg [31:0] rst_cnt;
initial rst_cnt = 32'b0;
always @(posedge okClk) begin
	if(rst_cnt < 32'h0800_0000) begin
		rst_cnt <= rst_cnt + 1;
		sys_rst <= 1'b1;
	end
	else begin
		sys_rst <= 1'b0;
	end
end

// Metrics

//Timer
integer timer_cnt;
initial timer_cnt = 32'b0;

integer timer_cnt2;
initial timer_cnt2 = 32'b0;

integer box_filter_timer;
initial box_filter_timer = 32'b0;

integer ccl_timer;
initial ccl_timer = 32'b0;

integer feature_rendering_timer;
initial feature_rendering_timer = 32'b0;

always @(posedge clk) begin
    // Box filter timer
    if(box_filter_done) begin
        regStore_fpga[1] <= box_filter_timer;
    end
    else begin
        box_filter_timer <= box_filter_timer + 1;
    end

    // CCL timer
    if(ccl_done) begin
        regStore_fpga[2] <= ccl_timer;
    end
    else begin
        ccl_timer <= ccl_timer + 1;
    end

    // feature_rendering timer
    if(feature_rendering_done) begin
        regStore_fpga[3] <= feature_rendering_timer;
    end
    else begin
        feature_rendering_timer <= feature_rendering_timer + 1;
    end
    
    // Pipeline timer
    if(!ep00wire[3] & done) begin
        regStore_fpga[0] <= timer_cnt;
    end
    else if(ep00wire[3]) begin
        box_filter_timer <= 0;
        ccl_timer <= 0;
        feature_rendering_timer <= 0;
        timer_cnt <= 0;
        timer_cnt2 <= 0;
        match <= 1'b1;
    end
    else begin
        timer_cnt <= timer_cnt + 1;
    end

    // Host register defined timer
	if(timer_cnt2 > regStore_host[2]) begin
		match <= 1'b0;
	end
    else begin
        timer_cnt2 <= timer_cnt2 + 1;
    end
end

//Memory access counter


// MIG User Interface instantiation
u_ddr3_256_16 u_ddr3_256_16 (
	// Memory interface ports
	.ddr3_addr                      (ddr3_addr),
	.ddr3_ba                        (ddr3_ba),
	.ddr3_cas_n                     (ddr3_cas_n),
	.ddr3_ck_n                      (ddr3_ck_n),
	.ddr3_ck_p                      (ddr3_ck_p),
	.ddr3_cke                       (ddr3_cke),
	.ddr3_ras_n                     (ddr3_ras_n),
	.ddr3_reset_n                   (ddr3_reset_n),
	.ddr3_we_n                      (ddr3_we_n),
	.ddr3_dq                        (ddr3_dq),
	.ddr3_dqs_n                     (ddr3_dqs_n),
	.ddr3_dqs_p                     (ddr3_dqs_p),
	.init_calib_complete            (init_calib_complete),
	
	.ddr3_dm                        (ddr3_dm),
	.ddr3_odt                       (ddr3_odt),
	// Application interface ports
	.app_addr                       (app_addr_main),
	.app_cmd                        (app_cmd_main),
	.app_en                         (app_en_main),
	.app_wdf_data                   (app_wdf_data_main),
	.app_wdf_end                    (app_wdf_end_main),
	.app_wdf_wren                   (app_wdf_wren_main),
	
	.app_rd_data                    (app_rd_data),
	.app_rd_data_end                (app_rd_data_end),
	.app_rd_data_valid              (app_rd_data_valid),
	.app_rdy                        (app_rdy),
	.app_wdf_rdy                    (app_wdf_rdy),
	.app_sr_req                     (1'b0),
	.app_sr_active                  (),
	.app_ref_req                    (1'b0),
	.app_ref_ack                    (),
	.app_zq_req                     (1'b0),
	.app_zq_ack                     (),
	.ui_clk                         (clk),
	.ui_clk_sync_rst                (rst),
	
	.app_wdf_mask                   (app_wdf_mask),
	
	// System Clock Ports
	.sys_clk_p                      (sys_clk_p),
	.sys_clk_n                      (sys_clk_n),
	
	.sys_rst                        (sys_rst)
	);
	
reg [269:0]     bram_wea;
reg [1:0]       bram_addra;
reg [1:0]       bram_addrb;
reg [2159:0]    bram_dina;
wire [2159:0]   bram_doutb;

bram_dual_port (
    .clka(clk),
    .wea(bram_wea),
    .addra(bram_addra),
    .dina(bram_dina),
    .clkb(clk),
    .addrb(bram_addrb),
    .doutb(bram_doutb)
);

// OK MIG DDR3 Testbench Instatiation
ddr3_test ddr3_tb (
	.clk                (clk),
    .reset              (ep00wire[2] | rst),
    .reads_en           (ep00wire[0]),
    .writes_en          (ep00wire[1]),
	.calib_done         (init_calib_complete),

	.ib_re              (pipe_in_read),
	.ib_data            (pipe_in_data),

	.ib_count           (pipe_in_rd_count),
	.ib_valid           (pipe_in_valid),
	.ib_empty           (pipe_in_empty),
	
	.ob_we              (pipe_out_write),
	.ob_data            (pipe_out_data),
	.ob_count           (pipe_out_wr_count),
	.ob_full            (pipe_out_full),
	
	.app_rdy            (app_rdy),
	.app_en             (app_en),
	.app_cmd            (app_cmd),
	.app_addr           (app_addr),
	
	.app_rd_data        (app_rd_data),
	.app_rd_data_end    (app_rd_data_end),
	.app_rd_data_valid  (app_rd_data_valid),
	
	.app_wdf_rdy        (app_wdf_rdy),
	.app_wdf_wren       (app_wdf_wren),
	.app_wdf_data       (app_wdf_data),
	.app_wdf_end        (app_wdf_end)
	);

integer rd_img_col;
integer rd_img_row;

integer wr_img_row;
reg [8:0] img_row;

reg [1:0] buf_addr_col_0;
reg [1:0] buf_addr_col_1;
reg [1:0] buf_addr_col_2;

localparam bram_addr_0 = 2'b00;
localparam bram_addr_1 = 2'b01;
localparam bram_addr_2 = 2'b11;

reg [23:0] bram_rd_0;
reg [23:0] bram_rd_1;
reg [23:0] bram_rd_2;

reg [127:0] app_rd_data_buf;
reg [127:0] app_wdf_data_buf;

reg [8:0] rd_col_tot;
reg [8:0] wr_col_tot;
reg [8:0] wr_row_tot;

integer rd_data;
integer wr_data;

localparam ADDRESS_INCREMENT    = 5'd8; // BL8 Burst Mode BL
localparam ADDRESS_1            = 16'd64808; // Processing/ binary operations Address (NOTE: Divisible with 8)
localparam ADDRESS_2            = 0; // Frontpanel Address
localparam ADDRESS_3            = 17'd129616; // 
localparam RW_COUNT_ASCII       = 64800; // Nr of 128bit reads/writes to cover data
localparam RW_COUNT_B           = 8100; // Nr of 128bit reads/writes to cover data

reg write_mode_1;
reg read_mode_1;

reg write_img_1;
reg read_img_1;

reg [7:0] b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16;

integer pipeline_stage;
localparam box_filter_1 = 0;
localparam box_filter_2 = 1;
localparam ccl = 2;
localparam feature_rendering = 3;
localparam centroiding = 4;
localparam centroid_rendering = 5;

reg [7:0] label; // label/ pixel value - limited by 8 bit int value - new tresholding needs to be done if this is exceeded
reg [35:0] feature_coordinates [254:0]; // store two x,y coordinates for each feature (min & max)
reg [35:0] feature_coordinate;
reg [7:0] frame_feature_idx;
reg [7:0] centroid_feature_idx;
reg [7:0] feature; // features that labels can be mapped into - components are assigned labels that are assigned to a feature DO NOT RESET THIS
reg [7:0] feature_table [254:0]; // feature table
reg [31:0] feature_brightness [254:0]; // brightness table - sum of feature pixels are kept here
reg [17:0] feature_centroids [254:0];
reg [99:0] feature_px_coordinates [254:0]; // one hot encoded pixel coordinates for centroid calculation
reg [31:0] pixel_count;
reg [29 :0] px_addr;

reg [7:0] tmp_feature;

reg [15:0] centroid_columns [9:0];
reg [15:0] centroid_rows [9:0];
reg [3:0] col_max;
reg [3:0] row_max;
reg [8:0] col_idx; // NOTE BIT WIDTH
reg [8:0] row_idx;

reg calc_centroid;
reg rendering_centroid;

// reg [31:0] tmp_px_x1y1;
// reg [31:0] tmp_px_x1y2;
// reg [31:0] tmp_px_y1x2;
// reg [31:0] tmp_px_y2x2;

integer center_x;
integer center_y;
reg [3:0] center_dx;
reg [3:0] center_dy;

integer px_offset;
// Memory interface multiplexer
always begin

    // Image processing
    if(!done) begin
        app_addr_main <= img_addr;
        app_cmd_main <= img_cmd;
        app_en_main <= img_en;
        app_wdf_wren_main <= img_wdf_wren;
        app_wdf_data_main <= img_wdf_data;
        app_wdf_end_main <= img_wdf_end;
    end
    // Front Panel
    else begin
    
        app_cmd_main <= app_cmd;
        app_en_main <= app_en;
        app_addr_main <= app_addr;
        
        app_wdf_wren_main <= app_wdf_wren;
        app_wdf_data_main <= app_wdf_data;
        app_wdf_end_main <= app_wdf_end;
        
    end
end

reg [39:0] state;

// One hot encoded states
localparam  s_idle     =        40'b00000000_00000000_00000000_00000000_00000000_00000001,
            s_park     =        40'b00000000_00000000_00000000_00000000_00000000_00000010,
            
            // 3x3 window buffering
            s_window_00 =       40'b00000000_00000000_00000000_00000000_00000000_00000100,
            s_window_01 =       40'b00000000_00000000_00000000_00000000_00000000_00001000,
            s_window_02 =       40'b00000000_00000000_00000000_00000000_00000000_00010000,
            s_window_03 =       40'b00000000_00000000_00000000_00000000_00000000_00100000,
            s_window_04 =       40'b00000000_00000000_00000000_00000000_00000000_01000000,
            s_window_05 =       40'b00000000_00000000_00000000_00000000_00000000_10000000,

            // Box filter processing
            s_box_filter =      40'b00000000_00000000_00000000_00000000_00000001_00000000,

            // Read line buffer into sdram
            s_rd_linebuf_2 =    40'b00000000_00000000_00000000_00000000_00000010_00000000,
            s_rd_linebuf_3 =    40'b00000000_00000000_00000000_00000000_00000100_00000000,
            s_rd_linebuf_4 =    40'b00000000_00000000_00000000_00000000_00001000_00000000,
            s_rd_linebuf_5 =    40'b00000000_00000000_00000000_00000000_00010000_00000000,
            
            // Write to line buffer from sdram
            s_wr_linebuf_0  =   40'b00000000_00000000_00000000_00000000_00100000_00000000,
            s_wr_linebuf_1  =   40'b00000000_00000000_00000000_00000000_01000000_00000000,
            s_wr_linebuf_2  =   40'b00000000_00000000_00000000_00000000_10000000_00000000,
            s_wr_linebuf_3  =   40'b00000000_00000000_00000000_00000001_00000000_00000000,
            s_wr_linebuf_4  =   40'b00000000_00000000_00000000_00000010_00000000_00000000,
            s_wr_linebuf_5  =   40'b00000000_00000000_00000000_00000100_00000000_00000000,

            // Connected compnent labeling
            s_read_5 =      40'b00000000_00000000_00000000_00001000_00000000_00000000,
            s_ccl_2 =           40'b00000000_00000000_00000000_00010000_00000000_00000000,
            s_ccl_3 =           40'b00000000_00000000_00000000_00100000_00000000_00000000,
            s_ccl_4 =           40'b00000000_00000000_00000000_01000000_00000000_00000000,
            s_ccl_5 =           40'b00000000_00000000_00000000_10000000_00000000_00000000,
            s_centroid_1 =      40'b00000000_00000000_00000001_00000000_00000000_00000000,

            // Write connected compnent features table to image
            s_read_0 =          40'b00000000_00000000_00000010_00000000_00000000_00000000,
            s_read_1 =          40'b00000000_00000000_00000100_00000000_00000000_00000000,
            s_write_1 =         40'b00000000_00000000_00001000_00000000_00000000_00000000,
            s_write_2 =         40'b00000000_00000000_00010000_00000000_00000000_00000000,
            s_write_3 =         40'b00000000_00000000_00100000_00000000_00000000_00000000,
            s_read_2 =          40'b00000000_00000000_01000000_00000000_00000000_00000000,

            s_centroid_2 =      40'b00000000_00000000_10000000_00000000_00000000_00000000,
            s_centroid_3 =      40'b00000000_00000001_00000000_00000000_00000000_00000000,

            s_read_4 =   40'b00000000_00000010_00000000_00000000_00000000_00000000,
            s_write_8 =             40'b00000000_00000100_00000000_00000000_00000000_00000000,
            s_write_7 =             40'b00000000_00001000_00000000_00000000_00000000_00000000,
            s_read_3 =              40'b00000000_00010000_00000000_00000000_00000000_00000000,
            s_write_4 =             40'b00000000_00100000_00000000_00000000_00000000_00000000,
            s_write_5 =             40'b00000000_01000000_00000000_00000000_00000000_00000000,
            s_write_6 =             40'b00000000_10000000_00000000_00000000_00000000_00000000;

// sdram ui clk drives pipeline state machine
always @(posedge clk) begin

    // Init registers
	if (!ep00wire[3] & done) begin
		state           <= s_idle;
        led_test_2      <= 1'b0; // Led off

        read_img_1      <= 1'b1;
        write_img_1     <= 1'b0;
        write_mode_1    <= 1'b0;
        read_mode_1     <= 1'b0;

        img_rd_addr  <= ADDRESS_1;
        img_wr_addr  <= ADDRESS_2;

        rd_img_col <= 0;
        rd_img_row <= 2159;
        wr_img_row <= 2159;
        img_row <= 269;
        rd_col_tot <= 0;
        wr_col_tot <= 0;
        wr_row_tot <= 0;

        rd_data <= 127;
        wr_data <= 127;

        buf_addr_col_0 <= bram_addr_0;
        buf_addr_col_1 <= bram_addr_1;
        buf_addr_col_2 <= bram_addr_2;
        
        bram_addra <= bram_addr_0;
        bram_addrb <= bram_addr_0;

        pipeline_stage <= box_filter_1;
        
        label <= 8'b1;
        feature <= 8'b1;
        
        // feature_coordinates <= {255{36'b0}};
        // feature_table <= {255{8'b0}};

        frame_feature_idx <= 8'b1;
        centroid_feature_idx <= 8'b1;

        pixel_count <= 32'b0;
        app_wdf_mask    <= 16'h0000;

        calc_centroid <= 1'b0;
	end 
    else if(ep00wire[3]) begin
        done <= 1'b0;
        box_filter_done <= 1'b0;
        led_test_2      <= 1'b1; // Led on during pipeline
    end
    else begin
        // Clear memory interface registers
        img_en          <= 1'b0;
        img_wdf_wren    <= 1'b0;
        img_wdf_end     <= 1'b0;
        bram_wea        <= 270'b0;
        app_wdf_mask    <= 16'hffff;
        
		case (state)
        
        // PARK STATE - Go Here when done
		    s_park: begin
		      done <= 1'b1;
		    end
		
        // IDLE STATE - Go Here between single read & write operations
			s_idle: begin
				
                if (write_img_1==1'b1) begin
                    img_addr <= img_wr_addr;
                end
                if (read_img_1==1'b1) begin
                    img_addr <= img_rd_addr;
                    state <= s_wr_linebuf_0;
                end
                // if(write_mode_1 == 1) begin
                //     img_addr <= img_wr_addr;
                //     state <= s_write_1;
                // end
                // if(read_mode_1 == 1) begin
                //     img_addr <= img_rd_addr;
                //     state <= s_read_0;
                // end
			end
            
            /////////////// Write 3x3 mean to sdram

            // Column 0 (LEFT)
            s_window_00: begin
                bram_addrb  <= buf_addr_col_0; // Read left column
                state <= s_window_01;
            end

            s_window_01: begin
                // Read 3 rows
                bram_rd_0 <= bram_doutb[wr_img_row-:24];
                bram_addrb  <= buf_addr_col_1; // Next address
                state <= s_window_02;
            end

            // Column 1 (CENTER)
            s_window_02: begin
                bram_addrb <= buf_addr_col_1; // Read center column
                state <= s_window_03;
            end

            s_window_03: begin
                // Read 3 rows
                bram_rd_1 <= bram_doutb[wr_img_row-:24];
                bram_addrb  <= buf_addr_col_2; // Next address
                state <= s_window_04;
            end

            // Column 2 (RIGHT)
            s_window_04: begin
                bram_addrb  <= buf_addr_col_2; // Read right column
                state <= s_window_05;
            end

            s_window_05: begin
                // Read 3 rows
                bram_rd_2 <= bram_doutb[wr_img_row-:24];
                if(pipeline_stage == box_filter_1 | pipeline_stage == box_filter_2) begin
                    state <= s_box_filter;
                end
                else if(pipeline_stage == ccl) begin
                    state <= s_ccl_2;
                end
                else if(pipeline_stage == feature_rendering | pipeline_stage == centroid_rendering) begin
                    state <= s_write_1;
                end
                else begin
                    state <= s_park; // Should not go here...
                end
            end
            
            /////////////// PROCESS PIXELS

            s_box_filter: begin

                // LINE START
                if(wr_img_row == 2159) begin // frist write of line -> allow to set first to default
                    app_wdf_data_buf[wr_data-:16] <= 16'h00;
                    wr_data = wr_data - 16;
                    wr_img_row = wr_img_row - 8;
                end
                // LINE END
                else if (wr_img_row < 31) begin // second to last write of line -> allow to set last to default
                    app_wdf_data_buf[wr_data-:16] <= 16'h00;
                    wr_data = wr_data - 16;
                    wr_img_row = 0;
                end

                // FIRST LINE MID
                else if(wr_col_tot == 0) begin
                    app_wdf_data_buf[wr_data-:8] <= 8'h00;
                    wr_data = wr_data - 8;
                    wr_img_row = wr_img_row - 8;
                    
                end

                // LAST LINE MID
                else if (wr_col_tot == 479) begin
                    app_wdf_data_buf[wr_data-:8] <= 8'h00;
                    wr_data = wr_data - 8;
                    wr_img_row = wr_img_row - 8;
                end

                // MID
                else begin

                    app_wdf_data_buf[wr_data-:8] <= windowAvg( // 3x3 window avg
                        bram_rd_0,bram_rd_1,bram_rd_2
                    );
                    wr_data = wr_data - 8;
                    wr_img_row = wr_img_row - 8;
                end

                state <= s_rd_linebuf_2;
            end

            s_rd_linebuf_2: begin

                // SDRAM Write buffer full
                if (wr_data < 7) begin // output buffer full, proceed to write
                    wr_data <= 127;
                    
                    img_addr <= img_wr_addr;

                    // Write BURST back into ??? order
                    {b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16} = app_wdf_data_buf;

                    if(pipeline_stage == box_filter_2) begin
                        // Binary transformation trough treshold of 1 (0-255)
                        b1 =  b1   > regStore_host[1] ? 8'hff : 8'b0;
                        b2 =  b2   > regStore_host[1] ? 8'hff : 8'b0;
                        b3 =  b3   > regStore_host[1] ? 8'hff : 8'b0;
                        b4 =  b4   > regStore_host[1] ? 8'hff : 8'b0;
                        b5 =  b5   > regStore_host[1] ? 8'hff : 8'b0;
                        b6 =  b6   > regStore_host[1] ? 8'hff : 8'b0;
                        b7 =  b7   > regStore_host[1] ? 8'hff : 8'b0;
                        b8 =  b8   > regStore_host[1] ? 8'hff : 8'b0;
                        b9 =  b9   > regStore_host[1] ? 8'hff : 8'b0;
                        b10 = b10  > regStore_host[1] ? 8'hff : 8'b0;
                        b11 = b11  > regStore_host[1] ? 8'hff : 8'b0;
                        b12 = b12  > regStore_host[1] ? 8'hff : 8'b0;
                        b13 = b13  > regStore_host[1] ? 8'hff : 8'b0;
                        b14 = b14  > regStore_host[1] ? 8'hff : 8'b0;
                        b15 = b15  > regStore_host[1] ? 8'hff : 8'b0;
                        b16 = b16  > regStore_host[1] ? 8'hff : 8'b0;
                    end
                    
                    // img_wdf_data = {b4,b3,b2,b1,b8,b7,b6,b5,b12,b11,b10,b9,b16,b15,b14,b13};
                    img_wdf_data = {b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16};

                    state <= s_rd_linebuf_3;
                end
                // Line buffer end
                else if(wr_img_row < 7) begin // done with current writes, proceed to read
                    wr_img_row <= 2159;
                    img_row <= 269; // ccl
                    
                    if(wr_col_tot != 478 & wr_col_tot > 0) begin // when 479 columns written - allow one more column to be written before park
                        state           <= s_wr_linebuf_4;
                        img_addr        <= img_rd_addr;
                        read_img_1      <= 1'b1;
                        write_img_1     <= 1'b0;
                        img_cmd         <= 3'b001;
                    end

                    wr_col_tot = wr_col_tot + 1;
                    wr_row_tot <= 9'b0;
                end
                // Continue processing
                else begin
                    bram_addrb  <= buf_addr_col_0; // Next address
                    state <= s_window_00; // read more from column    
                end
                
            end
            
            /////////////// Write line buffer to sdram

            s_rd_linebuf_3: begin
                if (app_wdf_rdy == 1'b1) begin
					state <= s_rd_linebuf_4;
				end
            end

            s_rd_linebuf_4: begin
                img_wdf_wren <= 1'b1;
                img_wdf_end <= 1'b1;
                
                if (app_wdf_rdy == 1'b1) begin
                    img_en    <= 1'b1;
					img_cmd <= 3'b000;
                    app_wdf_mask    <= 16'h0000;
                    state <= s_rd_linebuf_5;
                end

            end

            s_rd_linebuf_5: begin
                if (app_rdy == 1'b1) begin
                    img_wr_addr <= img_wr_addr + ADDRESS_INCREMENT;
                    state <= s_rd_linebuf_2;

                end
                else begin
                    img_en    <= 1'b1;
					img_cmd <= 3'b000;
                    app_wdf_mask    <= 16'h0000;
                end
            end

            /////////////// Read sdram to 3 line buffers
            
            s_wr_linebuf_0: begin
                img_en    <= 1'b1;
                img_cmd <= 3'b001;
                state <= s_wr_linebuf_1;
            end
            
            s_wr_linebuf_1: begin
                if (app_rdy == 1'b1) begin
                    img_rd_addr <= img_rd_addr + ADDRESS_INCREMENT;
                    state <= s_wr_linebuf_2;
                end else begin
                    img_en    <= 1'b1;
                    img_cmd <= 3'b001;
                end
            end

            s_wr_linebuf_2: begin
                if (app_rd_data_valid == 1'b1) begin
                    
                    // Read BURST into image pixel order
                    // {b4,b3,b2,b1,b8,b7,b6,b5,b12,b11,b10,b9,b16,b15,b14,b13} = app_rd_data;

                    // app_rd_data_buf = {b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16};
                    app_rd_data_buf = app_rd_data;
                    state <= s_wr_linebuf_3;
                end
            end

            s_wr_linebuf_3: begin
                bram_dina[rd_img_row-:8] <= app_rd_data_buf[rd_data-:8];
                rd_img_row = rd_img_row - 8;
                rd_data = rd_data - 8;

                state <= s_wr_linebuf_4;
            end

            s_wr_linebuf_4: begin
                // Go here when box filter is completed - reset registers
                if(rd_col_tot > 479) begin

                    rd_img_col <= 0;
                    rd_img_row <= 2159;
                    rd_col_tot <= 0;
                    rd_data <= 127;

                    wr_img_row <= 2159;
                    img_row <= 269;
                    wr_col_tot <= 9'b0;
                    wr_row_tot <= 9'b0;
                    wr_data <= 127;

                    buf_addr_col_0 <= bram_addr_0;
                    buf_addr_col_1 <= bram_addr_1;
                    buf_addr_col_2 <= bram_addr_2;
                    
                    bram_addra <= bram_addr_0;
                    bram_addrb <= bram_addr_0;

                    pixel_count <= 32'b0;

                    if(pipeline_stage == box_filter_1) begin
                        // Use same address sapce for ccl operations
                        box_filter_done <= 1'b1;
                        ccl_done <= 1'b0;
                        img_rd_addr  <= ADDRESS_1;
                        img_wr_addr  <= ADDRESS_3;
                        pipeline_stage <= box_filter_2; // binary transform
                        state <= s_idle; // Start read
                    end
                    else if (pipeline_stage == box_filter_2) begin
                        // Use same address sapce for ccl operations
                        img_rd_addr  <= ADDRESS_3;
                        img_wr_addr  <= ADDRESS_3;
                        pipeline_stage <= ccl;
                        state <= s_idle; // Start read
                    end
                    else if(pipeline_stage == ccl) begin
                        ccl_done <= 1'b1;
                        feature_rendering_done <= 1'b0;

                        img_rd_addr  <= ADDRESS_3;
                        img_wr_addr  <= ADDRESS_2;

                        pipeline_stage <= feature_rendering;
                        //state <= s_idle; // Start read

                        frame_feature_idx <= 1'b1;
                        state <= s_write_1;
                        // state <= s_park;
                    end
                    else if(pipeline_stage == feature_rendering) begin
                        feature_rendering_done <= 1'b1;

                        col_idx <= 4'b0;
                        row_idx <= 4'b0;
                        centroid_feature_idx <= 8'b1;

                        pipeline_stage <= centroiding; // rd from addr 2. RD ONLY
                        // state <= s_centroid_0;
                        state <= s_park;
                    end
                    else if(pipeline_stage == centroid_rendering) begin
                        // feature_rendering_done <= 1'b1;
                        state <= s_park;
                    end
                end
                // Go here hen ccl is completed - reset registers
                // else if(rd_col_tot > 479 & pipeline_stage == ccl) begin

                //     write_img_1 <= 1'b0;
                //     read_img_1 <= 1'b0;
                //     read_mode_1 <= 1'b1;
                //     write_mode_1 <= 1'b0;
                //     img_rd_addr  <= ADDRESS_1;
                //     img_wr_addr  <= ADDRESS_1;
                //     state <= s_idle;
                //     wr_data <= 127;
                //     pixel_count <= 32'b0;
                // end
                else if(rd_img_row < 7) begin
                    rd_img_row <= 2159;

                    rd_img_col <= rd_img_col+1;
                    
                    rd_col_tot <= rd_col_tot + 1; // Total column read count
                    state <= s_wr_linebuf_5; // update addressing
                    
                end
                else if (rd_data < 7) begin
                    rd_data <= 127;

                    state <= s_idle; // new read from ddr3
                end
                else begin
                    state <= s_wr_linebuf_3; // read more into column
                end

            end

            s_wr_linebuf_5: begin

                // Next state
                if(rd_col_tot > 2) begin // Goto write
                        // Previous read completed column 0
                    if((rd_img_col-1) == 0) begin // Select which column to read into next time
                        rd_img_col      <= 1; // Next read goes into 0, then 1, then 2, then 0 etc
                        
                        // Update column indexes
                        buf_addr_col_0   <= bram_addr_1;
                        buf_addr_col_1   <= bram_addr_2;
                        buf_addr_col_2   <= bram_addr_0;

                        bram_addra <= bram_addr_0;
                        bram_addrb <= bram_addr_1;
                    end
                    else if((rd_img_col-1) == 1) begin
                        rd_img_col <= 2; // Next read goes into 0, then 1, then 2, then 0 etc
                        
                        // Update column indexes
                        buf_addr_col_0   <= bram_addr_2;
                        buf_addr_col_1   <= bram_addr_0;
                        buf_addr_col_2   <= bram_addr_1;

                        bram_addra <= bram_addr_1;
                        bram_addrb <= bram_addr_2;   
                    end
                    else if((rd_img_col-1) == 2) begin
                        rd_img_col <= 0; // Next read goes into 0, then 1, then 2, then 0 etc
                        
                        // Update column indexes
                        buf_addr_col_0   <= bram_addr_0;
                        buf_addr_col_1   <= bram_addr_1;
                        buf_addr_col_2   <= bram_addr_2;

                        bram_addra <= bram_addr_2;
                        bram_addrb <= bram_addr_0;
                    end

                    bram_wea    <= {270{1'b1}}; // Enable write

                    state <= s_rd_linebuf_2;
                    img_addr <= img_wr_addr;

                    read_img_1      <= 1'b0;
                    write_img_1     <= 1'b1; 

                end
                else begin // Contniue read (for first 3 columns)\

                    if(rd_col_tot > 1) begin // update addresses after first write
                        bram_addra <= bram_addra + 2'b01; // Increment bram write address
                    end
                
                    bram_wea    <= {270{1'b1}}; // Enable write
                    state <= s_wr_linebuf_4; // Read more
                end
                
            end

            /////////////// Connected component labeling

            s_ccl_2: begin

                // LINE START
                if(wr_img_row == 2159) begin // frist write of line -> allow to set first to default
                    app_wdf_data_buf[wr_data-:16] <= 16'h00;
                    state <= s_ccl_4;
                end

                // LINE END
                else if (wr_img_row < 31) begin // second to last write of line -> allow to set last to default
                    app_wdf_data_buf[wr_data-:16] <= 16'h00;
                    state <= s_ccl_4;
                end

                // FIRST LINE MID
                else if(wr_col_tot == 0) begin
                    app_wdf_data_buf[wr_data-:8] <= 8'h00;
                    state <= s_ccl_5;
                end

                // LAST LINE MID
                else if (wr_col_tot == 479) begin
                    app_wdf_data_buf[wr_data-:8] <= 8'h00;
                    state <= s_ccl_5;
                end

                // MID
                else begin

                    if(bram_rd_1[15-:8] > 8'b0) begin // foreground pixel

                        state <= s_ccl_3;

                        if(bram_rd_0[15-:8] > 8'b0) begin // left pixel
                            if(bram_rd_1[23-:8] > 8'b0 & feature_table[bram_rd_1[23-:8]] != feature_table[bram_rd_0[15-:8]]) begin // above pixel has conflicting label - update feature
                                
                                // TODO shift feature table to allow reuse of replaced feature - table shift would result in redraw of image - bad idea
                                
                                // update all feature coordinates
                                feature_coordinates[feature_table[bram_rd_0[15-:8]]] <= {
                                    // Update X1
                                    {feature_coordinates[feature_table[bram_rd_0[15-:8]]][35-:9] < feature_coordinates[feature_table[bram_rd_1[23-:8]]][35-:9] ?
                                    feature_coordinates[feature_table[bram_rd_0[15-:8]]][35-:9] : feature_coordinates[feature_table[bram_rd_1[23-:8]]][35-:9]},
                                    // Update Y1
                                    {feature_coordinates[feature_table[bram_rd_0[15-:8]]][26-:9] < feature_coordinates[feature_table[bram_rd_1[23-:8]]][26-:9] ?
                                    feature_coordinates[feature_table[bram_rd_0[15-:8]]][26-:9] : feature_coordinates[feature_table[bram_rd_1[23-:8]]][26-:9]},
                                    // Update X2
                                    {feature_coordinates[feature_table[bram_rd_0[15-:8]]][17-:9] > feature_coordinates[feature_table[bram_rd_1[23-:8]]][17-:9] ?
                                    feature_coordinates[feature_table[bram_rd_0[15-:8]]][17-:9] : feature_coordinates[feature_table[bram_rd_1[23-:8]]][17-:9]},
                                    // Update Y2
                                    {feature_coordinates[feature_table[bram_rd_0[15-:8]]][8-:9] > feature_coordinates[feature_table[bram_rd_1[23-:8]]][8-:9] ?
                                    feature_coordinates[feature_table[bram_rd_0[15-:8]]][8-:9] : feature_coordinates[feature_table[bram_rd_1[23-:8]]][8-:9]}
                                };
                                // Store feature before overwrite
                                tmp_feature <= feature_table[bram_rd_1[23-:8]];

                                // Map labels to correct features - conflicting labels now point to same feature
                                feature_table[bram_rd_1[23-:8]] <= feature_table[bram_rd_0[15-:8]]; // use pixel value as key - maps to feature

                                // Assign feature brightness
                                if(feature_brightness[feature_table[bram_rd_1[23-:8]]] + feature_brightness[feature_table[bram_rd_0[15-:8]]] <= 32'hffffffff) feature_brightness[feature_table[bram_rd_1[23-:8]]] <= feature_brightness[feature_table[bram_rd_1[23-:8]]] + feature_brightness[feature_table[bram_rd_0[15-:8]]];
                                else feature_brightness[feature_table[bram_rd_1[23-:8]]] <= 32'hffffffff;
                            end
                            else if(feature_coordinates[feature_table[bram_rd_0[15-:8]]][17-:9] < wr_col_tot) begin // Update right edge coordinate
                                 // update feature right edge
                                feature_coordinates[feature_table[bram_rd_0[15-:8]]][17-:9] <= {wr_col_tot};
                                // update feature brightness
                                if(feature_brightness[feature_table[bram_rd_0[15-:8]]] + bram_rd_0[15-:8] <= 32'hffffffff) feature_brightness[feature_table[bram_rd_0[15-:8]]] <= feature_brightness[feature_table[bram_rd_0[15-:8]]] + bram_rd_0[15-:8];
                                else feature_brightness[feature_table[bram_rd_0[15-:8]]] <= 32'hffffffff;
                            end
                            else begin
                                // update feature brightness
                                if(feature_brightness[feature_table[bram_rd_0[15-:8]]] + bram_rd_0[15-:8] <= 32'hffffffff) feature_brightness[feature_table[bram_rd_0[15-:8]]] <= feature_brightness[feature_table[bram_rd_0[15-:8]]] + bram_rd_0[15-:8];
                                else feature_brightness[feature_table[bram_rd_0[15-:8]]] <= 32'hffffffff;
                            end

                            app_wdf_data_buf[wr_data-:8] <= bram_rd_0[15-:8];

                        end
                        else if(bram_rd_1[23-:8] > 8'b0) begin // previous pixel
                            app_wdf_data_buf[wr_data-:8] <= bram_rd_1[23-:8];
                            
                            if(feature_brightness[feature_table[bram_rd_1[23-:8]]] + bram_rd_1[15-:8] <= 32'hffffffff) feature_brightness[feature_table[bram_rd_1[23-:8]]] <= feature_brightness[feature_table[bram_rd_1[23-:8]]] + bram_rd_1[15-:8];
                            else feature_brightness[feature_table[bram_rd_1[23-:8]]] <= 32'hffffffff;
                            // update feature bottom edge

                            // Note img_row goes from 269 to 0 -> smaller nr is row further down
                            if(wr_row_tot > feature_coordinates[feature_table[bram_rd_1[23-:8]]][8:0]) begin
                                feature_coordinates[feature_table[bram_rd_1[23-:8]]][8:0] <= {wr_row_tot};
                            end
                        end
                        else begin // New label - set pixel value and update feature table

                            feature_table[label] <= feature; // assign feature

                                                            // X1       Y1      X2          Y2
                            feature_coordinates[feature] <= {wr_col_tot, wr_row_tot, wr_col_tot, wr_row_tot};
                            // feature_coordinates[feature] <= pixel_count;
                            app_wdf_data_buf[wr_data-:8] <= label; // assign label
                        
                            feature_brightness[feature] <= bram_rd_1[15-:8];

                            label <= label < 255 ? label + 1'b1 : label; // prevent overflow - fix this by adjusting tresholding
                            feature <= feature < 255 ? feature + 1'b1 : feature; // prevent overflow - fix this by adjusting tresholding

                        end
                        
                    end
                    else begin // background pixel
                        app_wdf_data_buf[wr_data-:8] <= bram_rd_1[15-:8];
                        state <= s_ccl_5;
                    end

                end

            end

            s_ccl_3: begin
                // Reset coordinates for merged feature
                if(bram_rd_0[15-:8] > 8'b0 & bram_rd_1[23-:8] > 8'b0 & tmp_feature != feature_table[bram_rd_0[15-:8]]) begin
                    feature_coordinates[tmp_feature] <= 36'b0;
                end
                
                // Copy to line buf
                bram_dina[(wr_img_row-8)-:8] <= app_wdf_data_buf[wr_data-:8];
                
                bram_addrb <= bram_addr_0;
                bram_addra <= buf_addr_col_1; // center column

                bram_wea[img_row] <= 1'b1; // Enable write for current byte

                state <= s_ccl_5;
            end

            s_ccl_4: begin
                wr_data <= wr_data - 16;
                wr_img_row <= wr_img_row == 2159 ? wr_img_row - 8 : 0;
                img_row <= img_row - 2;
                pixel_count <= pixel_count + 2'd2;
                wr_row_tot <= wr_row_tot + 2'd2;

                state <= s_rd_linebuf_2;
            end

            s_ccl_5: begin
                
                wr_data <= wr_data - 8;
                wr_img_row <= wr_img_row - 8;
                img_row <= img_row - 1;
                pixel_count <= pixel_count + 1'd1;
                wr_row_tot <= wr_row_tot + 2'd1;

                state <= s_rd_linebuf_2;
            end

            // s_ccl_6: begin
            //     feature_coordinates[feature_table[bram_rd_1[23-:8]]] <= 36'b0;
            //     feature_table[bram_rd_1[23-:8]] <= feature_table[bram_rd_0[15-:8]];
                
            //     state <= s_ccl_3;
            // end

            // s_write_1: begin

            //     // LINE START
            //     if(wr_img_row == 2159) begin // frist write of line -> allow to set first to default
            //         app_wdf_data_buf[wr_data-:16] <= 16'h00;
            //         state <= s_ccl_4;
            //     end

            //     // LINE END
            //     else if (wr_img_row < 31) begin // second to last write of line -> allow to set last to default
            //         app_wdf_data_buf[wr_data-:16] <= 16'h00;
            //         state <= s_ccl_4;
            //     end

            //     // FIRST LINE MID
            //     else if(wr_col_tot == 0) begin
            //         app_wdf_data_buf[wr_data-:8] <= 8'h00;
            //         state <= s_ccl_5;
            //     end

            //     // LAST LINE MID
            //     else if (wr_col_tot == 479) begin
            //         app_wdf_data_buf[wr_data-:8] <= 8'h00;
            //         state <= s_ccl_5;
            //     end

            //     else begin // IMAGE CENTER
            //         if (pipeline_stage == feature_rendering) state <= s_write_3;
            //         else if (pipeline_stage == centroid_rendering) state <= s_render_centroid_2;
            //     end
                
            // end
            
            s_write_1: begin

                // Reset pixel sum containers
                centroid_columns[4'd0] <= 16'b0;
                centroid_columns[4'd1] <= 16'b0;
                centroid_columns[4'd2] <= 16'b0;
                centroid_columns[4'd3] <= 16'b0;
                centroid_columns[4'd4] <= 16'b0;
                centroid_columns[4'd5] <= 16'b0;
                centroid_columns[4'd6] <= 16'b0;
                centroid_columns[4'd7] <= 16'b0;
                centroid_columns[4'd8] <= 16'b0;
                centroid_columns[4'd9] <= 16'b0;

                centroid_rows[4'd0] <= 16'b0;
                centroid_rows[4'd1] <= 16'b0;
                centroid_rows[4'd2] <= 16'b0;
                centroid_rows[4'd3] <= 16'b0;
                centroid_rows[4'd4] <= 16'b0;
                centroid_rows[4'd5] <= 16'b0;
                centroid_rows[4'd6] <= 16'b0;
                centroid_rows[4'd7] <= 16'b0;
                centroid_rows[4'd8] <= 16'b0;
                centroid_rows[4'd9] <= 16'b0;

                // Feature coordinate table iteration completed
                if(frame_feature_idx > feature) begin
                    app_wdf_data_buf[wr_data-:8] <= bram_rd_1[15-:8];
                    frame_feature_idx <= 8'b1;
                    state <= s_park;
                end
                else if(feature_coordinates[frame_feature_idx] > 36'b0 & feature_brightness[frame_feature_idx] > regStore_host[0]) begin
                    feature_coordinate <= { // apply frame with 1px offset to feature
                        {feature_coordinates[frame_feature_idx][35-:9] - 2'd2},
                        {feature_coordinates[frame_feature_idx][26-:9] - 2'd2},
                        {feature_coordinates[frame_feature_idx][17-:9] + 2'd2},
                        {feature_coordinates[frame_feature_idx][8-:9]  + 2'd2}
                    };

                    row_idx <= 1'b0;
                    col_idx <= 1'b0;

                    state <= s_write_2;
                end
                else begin
                    frame_feature_idx <= frame_feature_idx + 1'b1; // go to next
                end
            end

            s_write_2: begin
                // Set sdram read mode
                img_en    <= 1'b1;
                img_cmd <= 3'b001;

                // Calculcate burst address that includes wanted pixel - Feature
                // px_addr <= ADDRESS_3 + ((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9] - row_idx))) >> 3'b100) << 2'b11;
                
                // Pixel offset from burst start
                // px_offset <= (((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9] - row_idx))) - (((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9] - row_idx))) >> 3'b100) << 2'b11);
                
                px_addr <= ((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) >> 3'b100) << 2'b11;
                px_offset <= 4'd15 - (((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) - (((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) >> 3'b100) << 3'b100);

                center_dx <= (feature_coordinates[frame_feature_idx][17-:9] - feature_coordinates[frame_feature_idx][35-:9]) >> 1'b1;
                //center_dx <= (feature_coordinate[17-:9] - feature_coordinate[35-:9]) >> 1'b1; // X center inside 10 x 10 window. Use as weight
                center_dy <= (feature_coordinates[frame_feature_idx][8-:9] - feature_coordinates[frame_feature_idx][26-:9]) >> 1'b1;
                //center_dy <= (feature_coordinate[26-:9] - feature_coordinate[8-:9]) >> 1'b1; // Y center inside 10 x 10 window. Use as weight

                calc_centroid <= feature_coordinate[17-:9] - feature_coordinate[35-:9] <= 10 & feature_coordinate[8-:9] - feature_coordinate[26-:9] <= 10;

                state <= s_read_2;
            end

            s_write_3: begin

                if(row_idx == 1'b0) begin
                    app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] <= 8'hff;
                    state <= s_write_4;
                end
                else if(col_idx == ((feature_coordinate[17-:9] - feature_coordinate[35-:9]))) begin
                    app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] <= 8'hff;
                    state <= s_write_4;
                end
                else if(col_idx == 1'b0) begin
                    app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] <= 8'hff;
                    state <= s_write_4;
                end
                else if(row_idx == ((feature_coordinate[8-:9] - feature_coordinate[26-:9]))) begin
                    app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] <= 8'hff;
                    state <= s_write_4;
                end
                else begin
                    state <= s_centroid_1; // inside frame
                end

                
            end
            
            s_write_4: begin

                img_wdf_data = app_rd_data_buf;
 
                img_addr <= ADDRESS_2 + px_addr;

                state <= s_write_5;

            end

            /////// WRITE PIXEL

            s_write_5: begin
                if (app_wdf_rdy == 1'b1) begin
                    state <= s_write_6;
				end
            end

            s_write_6: begin
                img_wdf_wren <= 1'b1;
                img_wdf_end <= 1'b1;
                
                if (app_wdf_rdy == 1'b1) begin
                    img_en    <= 1'b1;
                    app_wdf_mask[px_offset] <= 1'b0;
					img_cmd <= 3'b000;
                    state <= s_write_7;
                end

            end

            s_write_7: begin
                if (app_rdy == 1'b1) begin
                    state <= s_write_8; 
                end
                else begin
                    img_en    <= 1'b1;
					img_cmd <= 3'b000;
                    app_wdf_mask[px_offset] <= 1'b0;
                end
            end

            s_centroid_1: begin

                // Small feature -> calculate centroid
                if(calc_centroid & feature_table[app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8]] == frame_feature_idx) begin
                    img_en    <= 1'b1;
                    img_cmd <= 3'b001;
                    state <= s_read_4;
                end
                // Large feature -> render frame center
                else if(~calc_centroid & (col_idx == ((feature_coordinate[17-:9] - feature_coordinate[35-:9]) >> 1'b1) | row_idx == ((feature_coordinate[8-:9] - feature_coordinate[26-:9]) >> 1'b1))) begin
                   app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] <= 8'hff;
                   state <= s_write_4;
                end
                // Large feature - continue read
                else begin
                    state <= s_write_8;
                end
            end

            s_centroid_2: begin
                // Validate index
                if((col_idx - 2'd2) >= 4'd0 & (col_idx - 2'd2) <= 4'd9) centroid_columns[(col_idx - 2'd2)]  <= (centroid_columns[(col_idx - 2'd2)] + (16'hffff - (app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] * ((col_idx - 2'd2) > center_dx ? ((col_idx - 2'd2) - center_dx) : (center_dx - (col_idx - 2'd2))))));
                // Validate index
                if((row_idx - 2'd2) >= 4'd0 & (row_idx - 2'd2) <= 4'd9) centroid_rows[(row_idx- 2'd2)]      <= (centroid_rows[(row_idx - 2'd2)] + (16'hffff - (app_rd_data_buf[((px_offset << 2'b11) + 3'd7)-:8] * ((row_idx - 2'd2) > center_dy ? ((row_idx - 2'd2) - center_dy) : (center_dy - (row_idx - 2'd2))))));
                state <= s_write_8;
            end

            s_centroid_3: begin
                px_addr <= ((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) >> 3'b100) << 2'b11;
                px_offset <= 4'd15 - (((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) - (((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) >> 3'b100) << 3'b100);
                app_rd_data_buf[(((4'd15 - (((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) - (((((feature_coordinate[35-:9]+col_idx)*9'd270)+((feature_coordinate[26-:9]+row_idx))) >> 3'b100) << 3'b100)) << 2'b11) + 3'd7)-:8] <= 8'hff;
                state <= s_write_4;
            end

            s_write_8: begin
                // Render centroid
                if(rendering_centroid & (col_idx < (col_max + 2'd1))) begin

                    state <= s_centroid_3;

                    if(col_idx + 1'b1 == col_max) begin
                        row_idx <= row_max - 1'd1;
                    end

                    if(col_idx == col_max) begin
                        if(row_idx < (row_max + 1'b1)) begin
                            row_idx <= row_idx + 1'b1;
                        end
                        else begin
                            row_idx <= row_max;
                            col_idx <= col_idx + 1'b1;
                        end
                        
                    end
                    else begin
                        col_idx <= col_idx + 1'b1;
                    end

                end
                else if(rendering_centroid) begin
                    rendering_centroid <= 1'b0;
                    frame_feature_idx <= frame_feature_idx + 1'b1; // go to next
                    state <= s_write_1; 
                end
                // Render frame center & calc centroid
                // If more pixels are needed to iterate feature rows
                else if(row_idx < (feature_coordinate[8-:9] - feature_coordinate[26-:9])) begin
                    row_idx <= row_idx + 1'b1;

                    state <= s_write_2;
                end
                // If more pixels are needed to iterate feature columns
                else if(col_idx < (feature_coordinate[17-:9] - feature_coordinate[35-:9])) begin
                    row_idx <= 1'b0;
                    col_idx <= col_idx + 1'b1;

                    state <= s_write_2;
                end
                // Move to next feature
                else begin
                    if(calc_centroid) begin
                        // render centroid
                        row_max = 4'd0;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd1] ? row_max : 4'd1;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd2] ? row_max : 4'd2;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd3] ? row_max : 4'd3;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd4] ? row_max : 4'd4;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd5] ? row_max : 4'd5;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd6] ? row_max : 4'd6;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd7] ? row_max : 4'd7;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd8] ? row_max : 4'd8;
                        row_max = centroid_rows[row_max] > centroid_rows[4'd9] ? row_max : 4'd9;
                        //row_max = 4'd1;
                        //row_max = (feature_coordinates[frame_feature_idx][8-:9] - feature_coordinates[frame_feature_idx][26-:9]) >> 1'b1;

                        col_max = 4'd0;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd1] ? col_max : 4'd1;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd2] ? col_max : 4'd2;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd3] ? col_max : 4'd3;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd4] ? col_max : 4'd4;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd5] ? col_max : 4'd5;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd6] ? col_max : 4'd6;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd7] ? col_max : 4'd7;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd8] ? col_max : 4'd8;
                        col_max = centroid_columns[col_max] > centroid_columns[4'd9] ? col_max : 4'd9;
                        //col_max = 4'd1;
                        //col_max = (feature_coordinates[frame_feature_idx][17-:9] - feature_coordinates[frame_feature_idx][35-:9]) >> 1'b1;

                        rendering_centroid <= 1'b1;
                        
                        row_idx <= row_max;
                        //col_idx <= (feature_coordinate[17-:9] - feature_coordinate[35-:9]) >> 1'b1;
                        col_idx <= col_max - 1'd1;
                        state <= s_centroid_3;

                        feature_coordinate <= { // apply frame with 1px offset to feature
                            {feature_coordinates[frame_feature_idx][35-:9]},
                            {feature_coordinates[frame_feature_idx][26-:9]},
                            {feature_coordinates[frame_feature_idx][17-:9]}, // note 1 not 2
                            {feature_coordinates[frame_feature_idx][8-:9]}
                        };
                    end
                    else begin
                        frame_feature_idx <= frame_feature_idx + 1'b1; // go to next
                        state <= s_write_1; 
                    end
                end
            end

            /////// READ PIXEL

			s_read_2: begin
				if (app_rdy == 1'b1) begin
                    img_addr <= ADDRESS_3 + px_addr;
					state <= s_read_3;

				end else begin
					img_en    <= 1'b1;
					img_cmd <= 3'b001;
				end
			end

			s_read_3: begin
				if (app_rd_data_valid == 1'b1) begin

                    app_rd_data_buf = app_rd_data;
                    state <= s_write_3;
                    
				end
			end

            // Centroid
            s_read_4: begin
				if (app_rdy == 1'b1) begin
                    img_addr <= ADDRESS_2 + px_addr;
					state <= s_read_5;

				end else begin
					img_en    <= 1'b1;
					img_cmd <= 3'b001;
				end
			end

            s_read_5: begin
				if (app_rd_data_valid == 1'b1) begin

                    app_rd_data_buf = app_rd_data;
                    state <= s_centroid_2;
                    
				end
			end

        endcase
	end
end


//Block Throttle
always @(posedge okClk) begin
	// Check for enough space in input FIFO to pipe in another block
	// The count is compared against a reduced size to account for delays in
	// FIFO count updates.
	if(pipe_in_wr_count <= (FIFO_SIZE-BUFFER_HEADROOM-BLOCK_SIZE) ) begin
		pipe_in_ready <= 1'b1;
	end
	else begin
		pipe_in_ready <= 1'b0;
	end

	// Check for enough space in output FIFO to pipe out another block
	if(pipe_out_rd_count >= BLOCK_SIZE) begin
		pipe_out_ready <= 1'b1;
	end
	else begin
		pipe_out_ready <= 1'b0;
	end
end


// Instantiate the okHost and connect endpoints.
wire [65*4-1:0]  okEHx;

okHost okHI(
	.okUH(okUH),
	.okHU(okHU),
	.okRSVD(okRSVD),
	.okUHU(okUHU),
	.okAA(okAA),
	.okClk(okClk),
	.okHE(okHE),
	.okEH(okEH)
	);

okWireOR # (.N(4)) wireOR (okEH, okEHx);
okWireIn       wi01 (.okHE(okHE),                             .ep_addr(8'h00), .ep_dataout(ep00wire));
okWireOut      wo00 (.okHE(okHE), .okEH(okEHx[ 1*65 +: 65 ]), .ep_addr(8'h20), .ep_datain({31'h00, init_calib_complete}));
// okWireOut      wo01 (.okHE(okHE), .okEH(okEHx[ 1*65 +: 65 ]), .ep_addr(8'h3e), .ep_datain(CAPABILITY));
okRegisterBridge regBridge (.okHE(okHE), .okEH(okEHx[ 0*65 +: 65 ]),.ep_write(regWrite),.ep_read(regRead),.ep_address(regAddress),.ep_dataout(regDataOut),.ep_datain(regDataIn));
okBTPipeIn     pi0  (.okHE(okHE), .okEH(okEHx[ 2*65 +: 65 ]), .ep_addr(8'h80), .ep_write(pi0_ep_write), .ep_blockstrobe(), .ep_dataout(pi0_ep_dataout), .ep_ready(pipe_in_ready));
okBTPipeOut    po0  (.okHE(okHE), .okEH(okEHx[ 3*65 +: 65 ]), .ep_addr(8'ha0), .ep_read(po0_ep_read),   .ep_blockstrobe(), .ep_datain(po0_ep_datain),   .ep_ready(pipe_out_ready));

fifo_w32_1024_r128_256 okPipeIn_fifo (
	.rst(ep00wire[2]),
	.wr_clk(okClk),
	.rd_clk(clk),
	.din(pi0_ep_dataout), // Bus [31 : 0]
	.wr_en(pi0_ep_write),
	.rd_en(pipe_in_read),
	.dout(pipe_in_data), // Bus [127 : 0]
	.full(pipe_in_full),
	.empty(pipe_in_empty),
	.valid(pipe_in_valid),
	.rd_data_count(pipe_in_rd_count), // Bus [7 : 0]
	.wr_data_count(pipe_in_wr_count)); // Bus [9 : 0]

fifo_w128_256_r32_1024 okPipeOut_fifo (
	.rst(ep00wire[2]),
	.wr_clk(clk),
	.rd_clk(okClk),
	.din(pipe_out_data), // Bus [127 : 0]
	.wr_en(pipe_out_write),
	.rd_en(po0_ep_read),
	.dout(po0_ep_datain), // Bus [31 : 0]
	.full(pipe_out_full),
	.empty(pipe_out_empty),
	.valid(),
	.rd_data_count(pipe_out_rd_count), // Bus [9 : 0]
	.wr_data_count(pipe_out_wr_count)); // Bus [7 : 0]

endmodule
