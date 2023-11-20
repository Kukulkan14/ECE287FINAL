module vga_driver_memory_double_buf	(
  // Clock Inputs
  input         CLOCK_50,    // 50MHz Input 1
  input         CLOCK2_50,   // 50MHz Input 2
  input         CLOCK3_50,   // 50MHz Input 3
  output        SMA_CLKOUT,  // External Clock Output
  input         SMA_CLKIN,   // External Clock Input

  // Push Button
  input  [3:0]  KEY,         // Pushbutton[3:0]

  // DPDT Switch
  input  [17:0] SW,          // Toggle Switch[17:0]

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
assign rst = KEY[0];

wire [17:0]SW_db;

debounce_switches db(
.clk(clk),
.rst(rst),
.SW(SW), 
.SW_db(SW_db)
);
/* -------------------------------- */

/* DEBUG SIGNALS */
//assign LEDR[0] = active_pixels;

/* -------------------------------- */
// VGA DRIVER
wire active_pixels; // is on when we're in the active draw space
wire frame_done;

wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

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

always @(*)
begin
	/* This part is for taking the memory value read out from memory and sending to the VGA */
	if (S == RFM_INIT_WAIT || S == RFM_INIT_START || S == RFM_DRAWING || S == RFM_CHAR_WAIT)
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
	// The RFM = READ_FROM_MEMOERY reading cycles
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
	case (S)
		START: 	
			if (KEY[1] == 1'b0) // Basically, if you hold down KEY[1] you will initialize the file with the FSM, otherwise you skip and have Rick as your mif initialization
				NS = W2M_INIT;
			else	
				NS = W2M_DONE;
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
            begin
                if (character_done == 1'b1)
                NS = RFM_CHAR_WAIT;
                else
                NS = RFM_DRAWING;
            end
        RFM_CHAR_WAIT: NS = RFM_DRAWING;
		default:	NS = ERROR;
	endcase

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

/* 
The code goes through a write phase (after reset) and an endless read phase once writing is done.

The W2M (write to memory) code is roughly:
for (i = 0; i < MEMORY_SIZE; i++)
	mem[i] = color // where color is a hard coded on off is on SW[16:14] for {R, G, B}

The RFM (read from memory) is synced with the VGA display (via vga_driver modules x and y) which goes row by row
for (y = 0; y < 480; y++) // height
	for (x = 0; x < 640; x++) // width
		color = mem[(x/4 * VP_HEIGHT) + j/4] reads from one of the buffers while you can write to the other buffer
*/
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
        readCounterOverall <= 16'd0;
        readLineCounter <= 1'b0;
        readCounter <= 8'd0;
		write_buf_mem_address <= 14'd0;
		write_buf_mem_data <= 24'd0;
		write_buf_mem_wren <= 1'd0;
		i <= 16'd0;
		wr_id <= MEM_INIT_WRITE;
        counterLines <= 8'd0;
        counterPixels <= 8'd0;
        character_done <= 1'b0;
        character_buf_mem_address <= 15'd0;
        character_row_count <= 8'd0;
        character_count <= 8'd0;
        character_buf_mem_wren <= 1'b0;
	end
	else
	begin
		case (S)
			START:
			begin
                readCounterOverall <= 16'd0;
                readLineCounter <= 1'b0;
                readCounter <= 8'd0;
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd0;
				i <= 16'd0;
				wr_id <= MEM_INIT_WRITE;
                counterLines <= 8'd0;
                counterPixels <= 8'd0;
                character_done <= 1'b0;
                character_buf_mem_address <= 15'd0;
                character_row_count <= 8'd0;
                character_count <= 8'd0;
                character_buf_mem_wren <= 1'b0;
                current_character <= 100'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111110;
			end
			W2M_INIT:
			begin
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd1;
				i <= 16'd0;
			end
			W2M_COND:
			begin
			end
			W2M_INC: 
			begin
				i <= i + 1'b1;
				write_buf_mem_address <= write_buf_mem_address + 1'b1;
				/* INITIALIZE to a solid color - IF SW[16-14] all off then = BLACK...all on = WHITE */
				write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF}; // red, blue, and green done in the combinational part below	
			end
			W2M_DONE: write_buf_mem_wren <= 1'd0; // turn off writing to memory
			RFM_INIT_START: 
			begin
                //readCounter <= 8'd0;
                //read_buf_mem_address <= 24'd0;
                write_buf_mem_address <= 14'd0;
                write_buf_mem_wren <= 1'd0;
                counterLines <= 8'd0;
                counterPixels <= 8'd0;
                character_count <= 8'd0;
                character_done <= 1'b0;
                character_buf_mem_address <= 15'd0;
                current_character <= 100'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111110;
				
				/* swap the buffers after each frame...the double buffer */
				if (wr_id == MEM_INIT_WRITE)
					wr_id <= MEM_M0_READ_M1_WRITE;
				else if (wr_id == MEM_M0_READ_M1_WRITE)
					wr_id <= MEM_M0_WRITE_M1_READ;
				else
					wr_id <= MEM_M0_READ_M1_WRITE;


                if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
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
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter == 1'b1)
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
                    8'd3:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter == 1'b1)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b1;                                
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
                read_buf_mem_address <= 24'd0;
            end



                // if ((y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) | (x == 10'd799)) // or use the active_pixels signal
                // begin
                //     if (readCounter == 8'd3)
                //     begin
                //         if ((readCounterOverall == 16'd639))
                //         begin
                //             if (readLineCounter == 1'b1)
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b0;
                //                 if (read_buf_mem_address == 24'd19199)
                //                 begin
				// 	                read_buf_mem_address <= 24'd0;
                //                 end
                //             end
                //             else
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address - 24'd159;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b1;                                
                //             end
                //         end
                //         else
                //         begin
                //             readCounter <= 8'd0;
                //             readCounterOverall <= readCounterOverall + 1'b1;
                //         end
                //     end
                //     else
                //     begin
                //         if (readCounter == 8'd1)
                //         begin
                //             readCounterOverall <= readCounterOverall + 1'b1;
                //             readCounter <= readCounter + 1'b1;
                //         end
                //         else
                //         begin
                //             if (readCounter == 8'd2)
                //             begin
                //                 if ((readCounterOverall == 16'd639))
                //                 begin
                //                     if (readLineCounter == 1'b1)
                //                     begin
                                        
                //                         if (read_buf_mem_address == 24'd19199)
                //                         begin
                //                             read_buf_mem_address <= 24'd0;
                //                         end
                //                         else
                //                         begin
                //                             read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //                         end
                //                     end
                //                     else
                //                     begin
                //                         readCounterOverall <= 16'd0;
                //                         read_buf_mem_address <= read_buf_mem_address - 24'd159;
                //                         readCounter <= 8'd0;
                //                         readLineCounter <= 1'b1;                                
                //                     end
                //                 end
                //                 else
                //                 begin
                //                     readCounter <= 8'd0;
                //                     readCounterOverall <= readCounterOverall + 1'b1;
                //                 end
                //             end
                //         end
                //     end
                // end
				// if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) // or use the active_pixels signal
				// 	read_buf_mem_address <= read_buf_mem_address + 1'b1;
			end
			RFM_INIT_WAIT:
			begin

                if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
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
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter == 1'b1)
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
                    8'd3:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter == 1'b1)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b1;                                
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
                read_buf_mem_address <= 24'd0;
            end


				// if ((y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) | (x == 10'd799)) // or use the active_pixels signal
                // begin
                //     if (readCounter == 8'd3)
                //     begin
                //         if ((readCounterOverall == 16'd639))
                //         begin
                //             if (readLineCounter == 1'b1)
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b0;
                //                 if (read_buf_mem_address == 24'd19199)
                //                 begin
				// 	                read_buf_mem_address <= 24'd0;
                //                 end
                //             end
                //             else
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address - 24'd159;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b1;                                
                //             end
                //         end
                //         else
                //         begin
                //             readCounter <= 8'd0;
                //         end
                //         readCounterOverall <= readCounterOverall + 1'b1;
                //     end
                //     else
                //     begin
                //         if (readCounter == 8'd1)
                //         begin
                //             readCounterOverall <= readCounterOverall + 1'b1;
                //             readCounter <= readCounter + 1'b1;
                //         end
                //         else
                //         begin
                //             if (readCounter == 8'd2)
                //             begin
                //                 read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //             end
                //             readCounter <= readCounter + 1'b1;
                //         end

                        // if (((readCounterOverall == 24'd0) & (readCounter == 1'b0)))
                        // begin
                        //     readCounter <= readCounter + 1'b1;
                        //     readCounterOverall <= readCounterOverall;
                        // end
                        // else
                        // begin
                        //     readCounter <= readCounter + 1'b1;
                        //     readCounterOverall <= readCounterOverall + 1'b1;
                        // end
                //     end
                // end

                    write_buf_mem_address <= 14'd0;
                    write_buf_mem_wren <= 1'b1;
                    counterLines <= 8'd0;
                    counterPixels <= 8'd0;
                    character_count <= 8'd0;
                    character_done <= 1'b0;
                    character_buf_mem_address <= 15'd0;
                    current_character <= 100'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111110;

                    

			end	
			RFM_DRAWING:
			begin		

                if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
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
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter == 1'b1)
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
                                    readCounter <= readCounter + 1'b1;
                                    read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                end
                    end
                    8'd3:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter == 1'b1)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b1;                                
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
                read_buf_mem_address <= 24'd0;
            end

				// if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
				// 	//read_buf_mem_address <= (y/PIXEL_VIRTUAL_SIZE) * VIRTUAL_PIXEL_HEIGHT + (x/PIXEL_VIRTUAL_SIZE) ;
                //     read_buf_mem_address <= read_buf_mem_address + 1'b1; // need to read every 4 or so cycles because of resolution difference.

                // if ((y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) | (x == 10'd799)) // or use the active_pixels signal
                // begin
                //     if (readCounter == 8'd3)
                //     begin
                //         if ((readCounterOverall == 16'd639))
                //         begin
                //             if (readLineCounter == 1'b1)
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b0;
                //                 if (read_buf_mem_address == 24'd19199)
                //                 begin
				// 	                read_buf_mem_address <= 24'd0;
                //                 end
                //             end
                //             else
                //             begin
                //                 readCounterOverall <= 16'd0;
                //                 read_buf_mem_address <= read_buf_mem_address - 24'd159;
                //                 readCounter <= 8'd0;
                //                 readLineCounter <= 1'b1;                                
                //             end
                //         end
                //         else
                //         begin
                //             readCounter <= 8'd0;
                //         end
                //         readCounterOverall <= readCounterOverall + 1'b1;
                //     end
                //     else
                //     begin
                //         if (readCounter == 8'd1)
                //         begin
                //             readCounterOverall <= readCounterOverall + 1'b1;
                //             readCounter <= readCounter + 1'b1;
                //         end
                //         else
                //         begin
                //             if (readCounter == 8'd2)
                //             begin
                //                 read_buf_mem_address <= read_buf_mem_address + 1'b1;
                //             end
                //             readCounter <= readCounter + 1'b1;
                //         end
                //     end
                // end
				
				if (SW[17] == 1'b1) // When you turn on SWITCH 17, you will draw a RGB pixel (depending on SW[16-14] at location x = SW[13:7] and y = SW[6:0]
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
                                character_done <= 1'b1;
                                if(write_buf_mem_address == 15'd19199)
                                begin
                                    write_buf_mem_wren = 1'b0;
                                    write_buf_mem_address <= 15'd0;
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
                                character_done <= 1'b1;
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
                        write_buf_mem_address <= write_buf_mem_address + 1'b1;
                        counterPixels <= counterPixels + 1'b1;
                        
                    end

                    if(character_done == 1'b0)
                    begin
                        if ((current_character & 100'd1) == 100'd1)
                        begin
                            write_buf_mem_data <= {8'hFF, 8'hFF, 8'hFF};
                            
                        end
                        else
                        begin
                            write_buf_mem_data <= {8'h00, 8'h00, 8'h00}; // nice blue
                            
                        end
                    end
                    
                    
					// write_buf_mem_address <= (SW[13:7]) * VIRTUAL_PIXEL_HEIGHT + (SW[6:0]);
					// write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF};
                    current_character <= {1'b0, current_character[99:1]};
				end
				else
					write_buf_mem_wren <= 1'b0;
			end	
            RFM_CHAR_WAIT:
            begin
                if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
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
                        if ((readCounterOverall == 16'd639))
                                begin
                                    if (readLineCounter == 1'b1)
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
                                    readCounter <= readCounter + 1'b1;
                                    read_buf_mem_address <= read_buf_mem_address + 1'b1;
                                end
                    end
                    8'd3:
                    begin
                        if ((readCounterOverall == 16'd639))
                        begin
                            if (readLineCounter == 1'b1)
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b0;
                            end
                            else
                            begin
                                readCounterOverall <= 16'd0;
                                readCounter <= 8'd0;
                                readLineCounter <= 1'b1;                                
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
                read_buf_mem_address <= 24'd0;
            end

                // if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
				// 	read_buf_mem_address <= read_buf_mem_address + 1'b1;

            //    if ((y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) | (x == 10'd799)) // or use the active_pixels signal
            //     begin
            //         if (readCounter == 8'd3)
            //         begin
            //             if ((readCounterOverall == 16'd639))
            //             begin
            //                 if (readLineCounter == 1'b1)
            //                 begin
            //                     read_buf_mem_address <= read_buf_mem_address + 1'b1;
            //                     readCounter <= 8'd0;
            //                     readLineCounter <= 1'b0;
            //                     readCounterOverall <= 16'd0;
            //                     if (read_buf_mem_address == 24'd19199)
            //                     begin
			// 		                read_buf_mem_address <= 24'd0;
            //                     end
            //                 end
            //                 else
            //                 begin
            //                     read_buf_mem_address <= read_buf_mem_address - 24'd159;
            //                     readCounter <= 8'd0;
            //                     readLineCounter <= 1'b1;  
            //                     readCounterOverall <= 16'd0;                              
            //                 end
                            
            //             end
            //             else
            //             begin
                            
            //                 readCounter <= 8'd0;
            //             end
            //             readCounterOverall <= readCounterOverall + 1'b1;
            //         end
            //         else
            //         begin
            //             if (readCounter == 8'd1)
            //             begin
            //                 readCounterOverall <= readCounterOverall + 1'b1;
            //                 readCounter <= readCounter + 1'b1;
            //             end
            //             else
            //             begin
            //                 if (readCounter == 8'd2)
            //                 begin
            //                     read_buf_mem_address <= read_buf_mem_address + 1'b1;
            //                 end
            //                 readCounter <= readCounter + 1'b1;
            //             end
            //         end
            //     end





                    if(character_done == 1'b1)
                    begin
                        //character_buf_mem_address <= character_buf_mem_address + 1'b1;
						current_character <= 100'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111110;
                        character_done <= 1'b0;
                    end
            end
		endcase
	end
end


reg [7:0] readCounter;
reg [99:0] current_character;
reg readLineCounter;
reg [15:0] readCounterOverall;

reg character_done;
reg [7:0] character_row_count;
reg [7:0] character_count;

/* -------------------------------- */
/* MEMORY to STORE a MINI framebuffer.  Problem is the FPGA's on-chip memory can't hold an entire frame 640*480 , so some
form of compression is needed.  I show a simple compress the image to 16 pixels or a 4 by 4, but this memory
could handle more */
reg [14:0] frame_buf_mem_address0;
reg [23:0] frame_buf_mem_data0;
reg frame_buf_mem_wren0;
wire [23:0]frame_buf_mem_q0;


reg [14:0] character_buf_mem_address;
reg [143:0] character_buf_mem_data;
reg character_buf_mem_wren;
wire [143:0]character_buf_mem_q;

// CharacterArray characters(
// character_buf_mem_address,
// character_buf_mem_data,
// character_buf_mem_wren,
// character_buf_mem_q
// );

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