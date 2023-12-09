module vga_driver_memory_double_buf	(
  // Clock Inputs
  input         CLOCK_50,    // 50MHz Input 1
  input         CLOCK2_50,   // 50MHz Input 2
  input         CLOCK3_50,   // 50MHz Input 3
  output        SMA_CLKOUT,  // External Clock Output
  input         SMA_CLKIN,   // External Clock Input

  // DPDT Switch
  input  [17:0] SW,          // Toggle Switch[17:0]
  input [1343:0] frameData,

  // VGA
  output        VGA_CLK,     // VGA Clock
  output        VGA_HS,      // VGA H_SYNC
  output        VGA_VS,      // VGA V_SYNC
  output        VGA_BLANK_N, // VGA BLANK
  output        VGA_SYNC_N,  // VGA SYNC
  output reg [7:0]  VGA_R,       // VGA Red[9:0]
  output reg [7:0]  VGA_G,       // VGA Green[9:0]
  output reg [7:0]  VGA_B       // VGA Blue[9:0]
);

  // Turn off all displays.
  assign HEX0 = 7'h7F;
  assign HEX1 = 7'h7F;
  assign HEX2 = 7'h7F;
  assign HEX3 = 7'h7F;
  assign HEX4 = 7'h7F;
  assign HEX5 = 7'h7F;
  assign HEX6 = 7'h7F;
  assign HEX7 = 7'h7F;

  // Set all GPIO to tri-state.
  assign GPIO = 36'hzzzzzzzzz;
  
  // Disable audio codec.
  assign AUD_DACDAT = 1'b0;
  assign AUD_XCK    = 1'b0;

  // Disable DRAM
  assign DRAM_ADDR  = 13'h0;
  assign DRAM_BA  = 2'b0;
  assign DRAM_CAS_N = 1'b1;
  assign DRAM_CKE   = 1'b0;
  assign DRAM_CLK   = 1'b0;
  assign DRAM_CS_N  = 1'b1;
  assign DRAM_Dframe_buf_mem_q    = 32'hzzzz;
  assign DRAM_Dframe_buf_mem_qM   = 4'b0;
  assign DRAM_RAS_N = 1'b1;
  //assign DRAM_UDframe_buf_mem_qM  = 1'b0;
  assign DRAM_WE_N  = 1'b1;

  // Disable flash.
  assign FL_ADDR  = 23'h0;
  assign FL_CE_N  = 1'b1;
  assign FL_Dframe_buf_mem_q    = 8'hzz;
  assign FL_OE_N  = 1'b1;
  assign FL_RST_N = 1'b1;
  assign FL_WE_N  = 1'b1;
  assign FL_WP_N  = 1'b0;

  // Disable LCD.
  assign LCD_BLON = 1'b0;
  assign LCD_frame_buf_mem_data = 8'hzz;
  assign LCD_EN   = 1'b0;
  assign LCD_ON   = 1'b0;
  assign LCD_RS   = 1'b0;
  assign LCD_RW   = 1'b0;

  // Disable OTG.
  assign OTG_ADDR    = 2'h0;
  assign OTG_CS_N    = 1'b1;
  assign OTG_DACK_N  = 2'b11;
  assign OTG_FSPEED  = 1'b1;
  assign OTG_frame_buf_mem_data    = 16'hzzzz;
  assign OTG_LSPEED  = 1'b1;
  assign OTG_RD_N    = 1'b1;
  assign OTG_RST_N   = 1'b1;
  assign OTG_WR_N    = 1'b1;

  // Disable SD
  assign SD_DAT = 4'bzzzz;
  assign SD_CLK = 1'b0;
  assign SD_CMD = 1'b0;

  // Disable SRAM.
  assign SRAM_ADDR = 20'h0;
  assign SRAM_CE_N = 1'b1;
  assign SRAM_Dframe_buf_mem_q   = 16'hzzzz;
  assign SRAM_LB_N = 1'b1;
  assign SRAM_OE_N = 1'b1;
  assign SRAM_UB_N = 1'b1;
  assign SRAM_WE_N = 1'b1;

  // Disable all other peripherals.
  assign I2C_SCLK   = 1'b0;
  //assign TD_RESET_N = 1'b0;
  assign UART_TXD   = 1'b0;
  assign UART_CTS   = 1'b0;
// DONE STANDARD PORT DECLARATION ABOVE

/* HANDLE SIGNALS FOR CIRCUIT */
wire clk;
wire rst;

assign clk = CLOCK_50;
assign rst = SW[17];

wire [17:0]SW_db;
reg [1343:0] frame;

// Module to debounce the switch inputs. Provided by Dr. Jamieson.
debounce_switches db(
.clk(clk),
.rst(rst),
.SW(SW[16:0]), 
.SW_db(SW_db)
);
// VGA DRIVER
wire active_pixels; // is on when we're in the active draw space
wire frame_done;

wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

// Module provided by Dr. Jamieson to draw to the display.
vga_driver the_vga(
.clk(clk),
.rst(rst),

.vga_clk(VGA_CLK),

.hsync(VGA_HS),
.vsync(VGA_VS),

.active_pixels(active_pixels),
.frame_done(frame_done),

.xPixel(x),
.yPixel(y),

.VGA_BLANK_N(VGA_BLANK_N),
.VGA_SYNC_N(VGA_SYNC_N)
);

// Block to interface the data being read from one of the buffers with the vga_driver module.
always @(*)
begin
	/* This part is for taking the memory value read out from memory and sending to the VGA */
	if (S == RFM_INIT_WAIT || S == RFM_INIT_START || S == RFM_DRAWING)
	begin
		{VGA_R, VGA_G, VGA_B} = read_buf_mem_q;
	end
	else // BLACK OTHERWISE
		{VGA_R, VGA_G, VGA_B} = 24'h666666;
end

/* -------------------------------- */
/* 	FSM to control the writing and reading of the framebuffer. */
reg [15:0]i;
reg [7:0]S;
reg [7:0]NS;
parameter 
	START 			= 8'd0,
	// W2M is write to memory
	W2M_INIT 		= 8'd1,
	W2M_COND 		= 8'd2,
	W2M_INC 		= 8'd3,
	W2M_DONE 		= 8'd4,
	// The RFM = READ_FROM_MEMORY reading cycles
	RFM_INIT_START 	= 8'd5,
	RFM_INIT_WAIT 	= 8'd6,
	RFM_DRAWING 	= 8'd7,
    RFM_CHAR_WAIT   = 8'd8,
	ERROR 			= 8'hFF;

parameter MEMORY_SIZE = 16'd19200; // 160*120 // Number of memory spots ... highly reduced since memory is slow
parameter PIXEL_VIRTUAL_SIZE = 16'd4; // Pixels per spot - therefore 4x4 pixels per memory location

/* ACTUAL VGA RESOLUTION */
parameter VGA_WIDTH = 16'd640; 
parameter VGA_HEIGHT = 16'd480;

/* Our reduced RESOLUTION */
parameter VIRTUAL_PIXEL_WIDTH = VGA_WIDTH/PIXEL_VIRTUAL_SIZE; // 160
parameter VIRTUAL_PIXEL_HEIGHT = VGA_HEIGHT/PIXEL_VIRTUAL_SIZE; // 120

reg [7:0] counterLines;
reg [7:0] counterPixels;

/* Calculate NS */
always @(*)
    case(S)
	    START:
				NS = W2M_INIT;
		W2M_INIT: NS = W2M_COND;
		W2M_COND:
			if (i < MEMORY_SIZE)
				NS = W2M_INC;
			else
				NS = W2M_DONE;
		W2M_INC: NS = W2M_COND;
		W2M_DONE: 
			if (frame_done == 1'b1)
				NS = RFM_INIT_START;
			else
				NS = W2M_DONE;
	
		RFM_INIT_START: NS = RFM_INIT_WAIT;
		RFM_INIT_WAIT: 
			if (frame_done == 1'b0)
				NS = RFM_DRAWING;
			else	
				NS = RFM_INIT_WAIT;
		RFM_DRAWING:
			if (frame_done == 1'b1)
				NS = RFM_INIT_START;
			else
				NS = RFM_DRAWING;
		default:	NS = ERROR;
	endcase

// block for resetting the FSM.
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
			S <= START;
	end
	else
	begin
			S <= NS;
	end
end


always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
        /* Reset all variables to start from a known state. 
        current_character and character_buf_mem_address are not accurate
        here as they depend on frame which is assigned here. 
        */
        completedFirstCycle <= 1'b0;
        read_buf_mem_address <= 24'd0;
        readCounterOverall <= 16'd0;
        readLineCounter <= 8'b0;
        readCounter <= 8'd0;
		write_buf_mem_address <= 14'd0;
		write_buf_mem_data <= 24'd0;
		write_buf_mem_wren <= 1'd0;
		i <= 16'd0;
		wr_id <= MEM_INIT_WRITE;
        counterLines <= 8'd0;
        counterPixels <= 8'd0;
        character_done <= 1'b0;
        character_buf_mem_address <= frame[1343:1337];
        character_row_count <= 8'd0;
        character_count <= 8'd0;
        character_buf_mem_wren <= 1'b0;
        current_character <= character_buf_mem_q;
        frame <= frameData;
	end
	else
	begin
		case (S)
			START:
			begin
                /* Ensure all variables start from a known state. 
                current_character and character_buf_mem_address are not accurate
                here as they depend on frame which is assigned here. 
                */
                completedFirstCycle <= 1'b0;
                read_buf_mem_address <= 24'd0;
                readCounterOverall <= 16'd0;
                readLineCounter <= 8'b0;
                readCounter <= 8'd0;
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd0;
				wr_id <= MEM_INIT_WRITE;
                counterLines <= 8'd0;
                counterPixels <= 8'd0;
                character_done <= 1'b0;
                character_row_count <= 8'd0;
                character_count <= 8'd0;
                character_buf_mem_wren <= 1'b0;
                character_buf_mem_address <= frame[1343:1337];
                frame <= frameData;
			end

            W2M_INIT:
			begin
                // Assign character_buf_mem_address here to be accurate. Initialize writing variables.
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd1;
				i <= 16'd0;
                character_buf_mem_address <= frame[1343:1337];
			end
			W2M_COND:
			begin
			end
			W2M_INC: 
			begin
				i <= i + 1'b1;
				write_buf_mem_address <= write_buf_mem_address + 1'b1;
				/* INITIALIZE to a solid color - IF SW[16-14] all off then = BLACK...all on = WHITE  This is irrelevant as game data will be written afterward. This will only be displayed for a frame.*/
				write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF};
			end
			W2M_DONE: 
            begin
                write_buf_mem_wren <= 1'd0; // turn off writing to memory
                current_character <= character_buf_mem_q; // Assign current_character as frame and character_buf_mem_address are accurate now.
                read_buf_mem_address <= 24'd0;
            end
			RFM_INIT_START: 
			begin
                /* Initialize variables at the end of each read and write cycle. Iterate frame 
                so that it is ahead of character_buf_mem_address which enables their iteration at the same time 
                in the write loop.
                */
                write_buf_mem_address <= 14'd0;
                write_buf_mem_wren <= 1'd1;
                counterLines <= 8'd0;
                counterPixels <= 8'd0;
                character_count <= 8'd0;
                character_done <= 1'b0;
                current_character <= character_buf_mem_q;
                completedFirstCycle <= 1'b0;
                frame <= {frame[1336:0], 7'd0};
                
				
				

                /* swap the buffers after each frame...the double buffer */
                if (wr_id == MEM_INIT_WRITE)
                    wr_id <= MEM_M0_READ_M1_WRITE;
                else if (wr_id == MEM_M0_READ_M1_WRITE)
                    wr_id <= MEM_M0_WRITE_M1_READ;
                else
                    wr_id <= MEM_M0_READ_M1_WRITE;


                /*The read structure. Please see the documentation for more information.
		This structure is included in multiple states to ensure data is read continously.*/
                if (y < VGA_HEIGHT && x < VGA_WIDTH)
                begin
                case(readCounter)
                    8'd0: readCounter <= readCounter + 1'b1;
                    8'd1: 
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd2:
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd3:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd4: 
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd5:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd6:
                    begin
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter >= 8'd3)
                                    begin
                                        if (read_buf_mem_address == 24'd19199)
                                        begin
                                            read_buf_mem_address <= 24'd0;
                                            readCounter <= readCounter + 1'b1;
                                        end
                                        else
                                        begin
                                            readCounter <= readCounter + 1'b1;
                                            read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                        end
                                    end
                                    else
                                    begin
                                        readCounter <= readCounter + 1'b1;
                                        read_buf_mem_address <= read_buf_mem_address - 24'd159;                             
                                    end
                                end
                                else
                                begin
                                    read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                    readCounter <= readCounter + 1'b1;
                                end
                    end
                    8'd7:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter >= 8'd3)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 8'd0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= readLineCounter + 1'b1;                                
                            end
                        end
                        else
                        begin
                            readCounter <= 8'd0;
                            readCounterOverall <= readCounterOverall + 1'b1;
                        end
                    end
                endcase
            end
            else
            begin
                readCounterOverall <= 16'd0;
                readCounter <= 8'd0;
            end
			end
			RFM_INIT_WAIT:
			begin
                // Prepare the writing and data writing counters.
                counterLines <= 8'd0;
                character_count <= 8'd0;
                character_done <= 1'b0;

                /*The read structure. Please see the documentation for more information.*/
                if (y < VGA_HEIGHT && x < VGA_WIDTH)
                begin
                case(readCounter)
                    8'd0: readCounter <= readCounter + 1'b1;
                    8'd1: 
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd2:
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd3:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd4: 
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd5:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd6:
                    begin
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter >= 8'd3)
                                    begin
                                        if (read_buf_mem_address == 24'd19199)
                                        begin
                                            read_buf_mem_address <= 24'd0;
                                            readCounter <= readCounter + 1'b1;
                                        end
                                        else
                                        begin
                                            readCounter <= readCounter + 1'b1;
                                            read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                        end
                                    end
                                    else
                                    begin
                                        readCounter <= readCounter + 1'b1;
                                        read_buf_mem_address <= read_buf_mem_address - 24'd159;                             
                                    end
                                end
                                else
                                begin
                                    read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                    readCounter <= readCounter + 1'b1;
                                end
                    end
                    8'd7:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter >= 8'd3)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 8'd0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= readLineCounter + 1'b1;                                
                            end
                        end
                        else
                        begin
                            readCounter <= 8'd0;
                            readCounterOverall <= readCounterOverall + 1'b1;
                        end
                    end
                endcase
            end
            else
            begin
                readCounterOverall <= 16'd0;
                readCounter <= 8'd0;
            end

                    /*Structure to initialize the data to be written so that accurate data is written to the first memory location.
                    This structure will execute one time which will write data to the first memory location and iterate the 
                    data variables once.*/
                    if(completedFirstCycle == 1'b0)
                    begin
                        if ((current_character & 100'd1) == 100'd1)
                        begin
                            write_buf_mem_data <= {8'hFF, 8'hFF, 8'hFF}; 
                            current_character <= {1'b0, current_character[99:1]};
                            write_buf_mem_address <= 14'd1;
                            counterPixels <= 8'd1;
                            completedFirstCycle <= 1'b1;
                            character_buf_mem_address <= frame[1343:1337];
                            frame <= {frame[1336:0], 7'd0};
                        end
                        else
                        begin
                            write_buf_mem_data <= {8'h00, 8'hC7, 8'hFF};
                            current_character <= {1'b0, current_character[99:1]};  
                            write_buf_mem_address <= 14'd1;
                            counterPixels <= 8'd1;
                            completedFirstCycle <= 1'b1;
                            character_buf_mem_address <= frame[1343:1337];
                            frame <= {frame[1336:0], 7'd0};
                        end
                    end
			end	
			RFM_DRAWING:
			begin		

                /*The read structure. Please see the documentation for more information.*/
                if (y < VGA_HEIGHT && x < VGA_WIDTH)
                begin
                case(readCounter)
                    8'd0: readCounter <= readCounter + 1'b1;
                    8'd1: 
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd2:
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd3:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd4: 
                    begin
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd5:
                    begin
                        readCounterOverall <= readCounterOverall + 1'b1;
                        readCounter <= readCounter + 1'b1;
                    end
                    8'd6:
                    begin
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter >= 8'd3)
                                    begin
                                        if (read_buf_mem_address == 24'd19199)
                                        begin
                                            read_buf_mem_address <= 24'd0;
                                            readCounter <= readCounter + 1'b1;
                                        end
                                        else
                                        begin
                                            readCounter <= readCounter + 1'b1;
                                            read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                        end
                                    end
                                    else
                                    begin
                                        readCounter <= readCounter + 1'b1;
                                        read_buf_mem_address <= read_buf_mem_address - 24'd159;                             
                                    end
                                end
                                else
                                begin
                                    read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                    readCounter <= readCounter + 1'b1;
                                end
                    end
                    8'd7:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter >= 8'd3)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 8'd0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= readLineCounter + 1'b1;                                
                            end
                        end
                        else
                        begin
                            readCounter <= 8'd0;
                            readCounterOverall <= readCounterOverall + 1'b1;
                        end
                    end
                endcase
            end
            else
            begin
                readCounterOverall <= 16'd0;
                readCounter <= 8'd0;
            end

                /*The write structure. Please see the documentation for more information.*/				
				if (write_buf_mem_wren == 1'b1)
				begin                    
                    if (counterPixels == 8'd9)
                    begin
                        if (counterLines == 8'd9)
                        begin
                            if (character_count == 8'd15)
                            begin
                                character_count <= 8'd0;
                                counterLines <= 8'd0;
                                counterPixels <= 8'd0;
                                if(write_buf_mem_address >= 15'd19199)
                                begin
                                    write_buf_mem_wren = 1'b0;
                                    write_buf_mem_address <= 15'd0;
                                    character_buf_mem_address <= frame[1343:1337];
                                    counterLines <= 8'd0;
                                    counterPixels <= 8'd0;
                                    character_count <= 8'd0;
                                    character_done <= 1'b0;
                                end
                                else
                                begin
                                        write_buf_mem_address <= write_buf_mem_address + 1'b1;
                                end
                            end
                            else
                            begin
                                counterLines <= 8'd0;
                                counterPixels <= 8'd0;
                                write_buf_mem_address <= write_buf_mem_address - 15'd1439;
                                character_count <= character_count + 1'd1;
                            end
                        end
                        else
                        begin
                            write_buf_mem_address <= write_buf_mem_address + 15'd151;
                            counterPixels <= 8'd0;
                            counterLines <= counterLines + 1'b1;
                        end
                    end
                    else
                    begin
                        if (counterPixels == 8'd7)
                        begin
                            if (counterLines == 8'd9)
                            begin
                            character_done <= 1'b1;
                            write_buf_mem_address <= write_buf_mem_address + 1'b1;
                            counterPixels <= counterPixels + 1'b1;
                            end
                            else
                            begin
                                write_buf_mem_address <= write_buf_mem_address + 1'b1;
                                counterPixels <= counterPixels + 1'b1;
                            end
                        end
                        else
                        begin
                            write_buf_mem_address <= write_buf_mem_address + 1'b1;
                            counterPixels <= counterPixels + 1'b1;
                        end
                    end

                    /*The data writing structure. Please see the documentation for more information.*/
                    if(character_done == 1'b0)
                    begin
                        if ((current_character & 100'd1) == 100'd1)
                        begin
                            write_buf_mem_data <= {8'hFF, 8'hFF, 8'hFF}; 
                            current_character <= {1'b0, current_character[99:1]};
                        end
                        else
                        begin
                            write_buf_mem_data <= {8'h00, 8'hC7, 8'hFF};
                            current_character <= {1'b0, current_character[99:1]};        
                        end
                    end
                    else
                    begin
                        
                        if ((current_character & 100'd1) == 100'd1)
                        begin
                            write_buf_mem_data <= {8'hFF, 8'hFF, 8'hFF}; 
                            current_character <= character_buf_mem_q;
                            character_done <= 1'b0;
                            character_buf_mem_address <= frame[1343:1337];
                            if (write_buf_mem_address >= 15'd19188)
                            begin
                                frame <= frameData;
                            end
                            else
                                frame <= {frame[1336:0], 7'd0};
                        end
                        else
                        begin
                            write_buf_mem_data <= {8'h00, 8'hC7, 8'hFF};  
                            current_character <= character_buf_mem_q;
                            character_done <= 1'b0; 
                            character_buf_mem_address <= frame[1343:1337];
                            if (write_buf_mem_address >= 15'd19188)
                            begin
                                frame <= frameData;
                            end
                            else
                                frame <= {frame[1336:0], 7'd0};
                        end
                    end
				end
				else
                begin
					write_buf_mem_wren <= 1'b0;
                    character_buf_mem_address <= frame[1343:1337];
                    frame <= frameData;
                end
			end	
    	endcase
	end
end

reg completedFirstCycle;
reg [7:0] readCounter;
reg [99:0] current_character;
reg [7:0] readLineCounter;
reg [15:0] readCounterOverall;

reg character_done;
reg [7:0] character_row_count;
reg [7:0] character_count;

/* -------------------------------- */
/* MEMORY to STORE a MINI framebuffer.  Problem is the FPGA's on-chip memory can't hold an entire frame 640*480, so some
form of compression is needed.*/
reg [14:0] frame_buf_mem_address0;
reg [23:0] frame_buf_mem_data0;
reg frame_buf_mem_wren0;
wire [23:0]frame_buf_mem_q0;

/*Character buffer to store our font.*/
reg [6:0] character_buf_mem_address;
reg [99:0] character_buf_mem_data;
reg character_buf_mem_wren;
wire [99:0]character_buf_mem_q;

CharacterArray characters(
character_buf_mem_address,
clk,
character_buf_mem_data,
character_buf_mem_wren,
character_buf_mem_q
);

// First frame buffer.
vga_frame vga_memory0(
	frame_buf_mem_address0,
	clk,
	frame_buf_mem_data0,
	frame_buf_mem_wren0,
	frame_buf_mem_q0);
	
reg [14:0] frame_buf_mem_address1;
reg [23:0] frame_buf_mem_data1;
reg frame_buf_mem_wren1;
wire [23:0]frame_buf_mem_q1;

// Second frame buffer.
vga_frame vga_memory1(
	frame_buf_mem_address1,
	clk,
	frame_buf_mem_data1,
	frame_buf_mem_wren1,
	frame_buf_mem_q1);

/* signals that will be combinationally swapped in each cycle */
reg [1:0]wr_id;	
reg [14:0] write_buf_mem_address;
reg [23:0] write_buf_mem_data;
reg write_buf_mem_wren;
reg [23:0]read_buf_mem_q;
reg [14:0] read_buf_mem_address;

parameter MEM_INIT_WRITE = 2'd0,
		  MEM_M0_READ_M1_WRITE = 2'd1,
		  MEM_M0_WRITE_M1_READ = 2'd2,
		  MEM_ERROR = 2'd3;

/* signals that will be combinationally swapped in each buffer output that swaps between wr_id where wr_id = 0 is for initialize */
always @(*)
begin
	if (wr_id == MEM_INIT_WRITE) // WRITING to BOTH
	begin
		frame_buf_mem_address0 = write_buf_mem_address;
		frame_buf_mem_data0 = write_buf_mem_data;
		frame_buf_mem_wren0 = write_buf_mem_wren;
		frame_buf_mem_address1 = write_buf_mem_address;
		frame_buf_mem_data1 = write_buf_mem_data;
		frame_buf_mem_wren1 = write_buf_mem_wren;
		
		read_buf_mem_q = frame_buf_mem_q1; // doesn't matter
	end
	else if (wr_id == MEM_M0_WRITE_M1_READ) // WRITING to MEM 0 READING FROM MEM 1
	begin
		// MEM 0 - WRITE
		frame_buf_mem_address0 = write_buf_mem_address;
		frame_buf_mem_data0 = write_buf_mem_data;
		frame_buf_mem_wren0 = write_buf_mem_wren;
		// MEM 1 - READ
		frame_buf_mem_address1 = read_buf_mem_address;
		frame_buf_mem_data1 = 24'd0;
		frame_buf_mem_wren1 = 1'b0;
		read_buf_mem_q = frame_buf_mem_q1;
	end
	else //if (wr_id == MEM_M0_READ_M1_WRITE) WRITING to MEM 1 READING FROM MEM 0
	begin
		// MEM 0 - READ
		frame_buf_mem_address0 = read_buf_mem_address;
		frame_buf_mem_data0 = 24'd0;
		frame_buf_mem_wren0 = 1'b0;
		read_buf_mem_q = frame_buf_mem_q0;
		// MEM 1 - WRITE
		frame_buf_mem_address1 = write_buf_mem_address;
		frame_buf_mem_data1 = write_buf_mem_data;
		frame_buf_mem_wren1 = write_buf_mem_wren;
	end
end

endmodule
