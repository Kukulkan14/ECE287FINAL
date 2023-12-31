module FinalProject(// Clock Inputs
  input         CLOCK_50,    // 50MHz Input 1
  input         CLOCK2_50,   // 50MHz Input 2
  input         CLOCK3_50,   // 50MHz Input 3
  output        SMA_CLKOUT,  // External Clock Output
  input         SMA_CLKIN,   // External Clock Input

  // Push Button
  input  [3:0]  KEY,         // Pushbutton[3:0]

  // DPDT Switch
  input  [17:0] SW,          // Toggle Switch[17:0]

  // 7-SEG Display
  output [6:0]  HEX0,        // Seven Segment Digit 0
  output [6:0]  HEX1,        // Seven Segment Digit 1
  output [6:0]  HEX2,        // Seven Segment Digit 2
  output [6:0]  HEX3,        // Seven Segment Digit 3
  output [6:0]  HEX4,        // Seven Segment Digit 4
  output [6:0]  HEX5,        // Seven Segment Digit 5
  output [6:0]  HEX6,        // Seven Segment Digit 6
  output [6:0]  HEX7,        // Seven Segment Digit 7

  // LED
  output [8:0]  LEDG,        // LED Green[8:0]
  output [17:0] LEDR,        // LED Red[17:0]

  // UART
  output        UART_TXD,    // UART Transmitter
  input         UART_RXD,    // UART Receiver
  output        UART_CTS,    // UART Clear to Send
  input         UART_RTS,    // UART Reframe_buf_mem_quest to Send

  // IRDA
  input         IRDA_RXD,    // IRDA Receiver

  // SDRAM Interface
  inout  [31:0] DRAM_Dframe_buf_mem_q,     // SDRAM frame_buf_mem_data bus 32 Bits
  output [12:0] DRAM_ADDR,   // SDRAM frame_buf_mem_address bus 13 Bits
  output [1:0]  DRAM_BA,     // SDRAM Bank frame_buf_mem_address
  output [3:0]  DRAM_Dframe_buf_mem_qM,    // SDRAM Byte frame_buf_mem_data Mask 
  output        DRAM_RAS_N,  // SDRAM Row frame_buf_mem_address Strobe
  output        DRAM_CAS_N,  // SDRAM Column frame_buf_mem_address Strobe
  output        DRAM_CKE,    // SDRAM Clock Enable
  output        DRAM_CLK,    // SDRAM Clock
  output        DRAM_WE_N,   // SDRAM Write Enable
  output        DRAM_CS_N,   // SDRAM Chip Select

  // Flash Interface
  inout  [7:0]  FL_Dframe_buf_mem_q,       // FLASH frame_buf_mem_data bus 8 Bits
  output [22:0] FL_ADDR,     // FLASH frame_buf_mem_address bus 23 Bits
  output        FL_WE_N,     // FLASH Write Enable
  output        FL_WP_N,     // FLASH Write Protect / Programming Acceleration
  output        FL_RST_N,    // FLASH Reset
  output        FL_OE_N,     // FLASH Output Enable
  output        FL_CE_N,     // FLASH Chip Enable
  input         FL_RY,       // FLASH Ready/Busy output

  // SRAM Interface
  inout  [15:0] SRAM_Dframe_buf_mem_q,     // SRAM frame_buf_mem_data bus 16 Bits
  output [19:0] SRAM_ADDR,   // SRAM frame_buf_mem_address bus 20 Bits
  output        SRAM_OE_N,   // SRAM Output Enable
  output        SRAM_WE_N,   // SRAM Write Enable
  output        SRAM_CE_N,   // SRAM Chip Enable
  output        SRAM_UB_N,   // SRAM High-byte frame_buf_mem_data Mask 
  output        SRAM_LB_N,   // SRAM Low-byte frame_buf_mem_data Mask 

  // ISP1362 Interface
  inout  [15:0] OTG_frame_buf_mem_data,    // ISP1362 frame_buf_mem_data bus 16 Bits
  output [1:0]  OTG_ADDR,    // ISP1362 frame_buf_mem_address 2 Bits
  output        OTG_CS_N,    // ISP1362 Chip Select
  output        OTG_RD_N,    // ISP1362 Write
  output        OTG_WR_N,    // ISP1362 Read
  output        OTG_RST_N,   // ISP1362 Reset
  input  [1:0]  OTG_INT,     // ISP1362 Interrupts
  inout         OTG_FSPEED,  // USB Full Speed, 0 = Enable, Z = Disable
  inout         OTG_LSPEED,  // USB Low Speed,  0 = Enable, Z = Disable
  input  [1:0]  OTG_DREframe_buf_mem_q,    // ISP1362 DMA Reframe_buf_mem_quest
  output [1:0]  OTG_DACK_N,  // ISP1362 DMA Acknowledge

  // LCD Module 16X2
  inout  [7:0]  LCD_frame_buf_mem_data,    // LCD frame_buf_mem_data bus 8 bits
  output        LCD_ON,      // LCD Power ON/OFF
  output        LCD_BLON,    // LCD Back Light ON/OFF
  output        LCD_RW,      // LCD Read/Write Select, 0 = Write, 1 = Read
  output        LCD_EN,      // LCD Enable
  output        LCD_RS,      // LCD Command/frame_buf_mem_data Select, 0 = Command, 1 = frame_buf_mem_data

  // SD Card Interface
  inout  [3:0]  SD_DAT,      // SD Card frame_buf_mem_data
  inout         SD_CMD,      // SD Card Command Line
  output        SD_CLK,      // SD Card Clock
  input         SD_WP_N,     // SD Write Protect

  // EEPROM Interface
  output        EEP_I2C_SCLK, // EEPROM Clock
  inout         EEP_I2C_SDAT, // EEPROM frame_buf_mem_data

  // PS2
  inout         PS2_DAT,     // PS2 frame_buf_mem_data
  inout         PS2_CLK,     // PS2 Clock
  inout         PS2_DAT2,    // PS2 frame_buf_mem_data 2 (use for 2 devices and y-cable)
  inout         PS2_CLK2,    // PS2 Clock 2 (use for 2 devices and y-cable)

  // I2C  
  inout         I2C_SDAT,    // I2C frame_buf_mem_data
  output        I2C_SCLK,    // I2C Clock

  // Audio CODEC
  inout         AUD_ADCLRCK, // Audio CODEC ADC LR Clock
  input         AUD_ADCDAT,  // Audio CODEC ADC frame_buf_mem_data
  inout         AUD_DACLRCK, // Audio CODEC DAC LR Clock
  output        AUD_DACDAT,  // Audio CODEC DAC frame_buf_mem_data
  inout         AUD_BCLK,    // Audio CODEC Bit-Stream Clock
  output        AUD_XCK,     // Audio CODEC Chip Clock

  // Ethernet Interface (88E1111)
  input         ENETCLK_25,    // Ethernet clock source

  output        ENET0_GTX_CLK, // GMII Transmit Clock 1
  input         ENET0_INT_N,   // Interrupt open drain output 1
  input         ENET0_LINK100, // Parallel LED output of 100BASE-TX link 1
  output        ENET0_MDC,     // Management frame_buf_mem_data clock ref 1
  inout         ENET0_MDIO,    // Management frame_buf_mem_data 1
  output        ENET0_RST_N,   // Hardware Reset Signal 1
  input         ENET0_RX_CLK,  // GMII and MII receive clock 1
  input         ENET0_RX_COL,  // GMII and MII collision 1
  input         ENET0_RX_CRS,  // GMII and MII carrier sense 1
  input   [3:0] ENET0_RX_frame_buf_mem_data, // GMII and MII receive frame_buf_mem_data 1
  input         ENET0_RX_DV,   // GMII and MII receive frame_buf_mem_data valid 1
  input         ENET0_RX_ER,   // GMII and MII receive error 1
  input         ENET0_TX_CLK,  // MII Transmit clock 1
  output  [3:0] ENET0_TX_frame_buf_mem_data, // MII Transmit frame_buf_mem_data 1
  output        ENET0_TX_EN,   // GMII and MII transmit enable 1
  output        ENET0_TX_ER,   // GMII and MII transmit error 1

  output        ENET1_GTX_CLK, // GMII Transmit Clock 1
  input         ENET1_INT_N,   // Interrupt open drain output 1
  input         ENET1_LINK100, // Parallel LED output of 100BASE-TX link 1
  output        ENET1_MDC,     // Management frame_buf_mem_data clock ref 1
  inout         ENET1_MDIO,    // Management frame_buf_mem_data 1
  output        ENET1_RST_N,   // Hardware Reset Signal 1
  input         ENET1_RX_CLK,  // GMII and MII receive clock 1
  input         ENET1_RX_COL,  // GMII and MII collision 1
  input         ENET1_RX_CRS,  // GMII and MII carrier sense 1
  input   [3:0] ENET1_RX_frame_buf_mem_data, // GMII and MII receive frame_buf_mem_data 1
  input         ENET1_RX_DV,   // GMII and MII receive frame_buf_mem_data valid 1
  input         ENET1_RX_ER,   // GMII and MII receive error 1
  input         ENET1_TX_CLK,  // MII Transmit clock 1
  output  [3:0] ENET1_TX_frame_buf_mem_data, // MII Transmit frame_buf_mem_data 1
  output        ENET1_TX_EN,   // GMII and MII transmit enable 1
  output        ENET1_TX_ER,   // GMII and MII transmit error 1

  // Expansion Header
  inout   [6:0] EX_IO,       // 14-pin GPIO Header
  inout  [35:0] GPIO,        // 40-pin Expansion header

  // TV Decoder
  input  [8:0]  TD_frame_buf_mem_data,     // TV Decoder frame_buf_mem_data
  input         TD_CLK27,    // TV Decoder Clock Input
  input         TD_HS,       // TV Decoder H_SYNC
  input         TD_VS,       // TV Decoder V_SYNC
  output        TD_RESET_N,  // TV Decoder Reset

  // VGA
  output        VGA_CLK,     // VGA Clock
  output        VGA_HS,      // VGA H_SYNC
  output        VGA_VS,      // VGA V_SYNC
  output        VGA_BLANK_N, // VGA BLANK
  output        VGA_SYNC_N,  // VGA SYNC
  output [7:0]  VGA_R,       // VGA Red[9:0]
  output [7:0]  VGA_G,       // VGA Green[9:0]
  output [7:0]  VGA_B       // VGA Blue[9:0]
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

    // frameData connects the frameData variable in the game module to the vga_driver_memory_double_buf module.
    wire [1343:0] frameData;
    vga_driver_memory_double_buf vga_output(CLOCK_50, CLOCK2_50, CLOCK3_50, SMA_CLKOUT, SMA_CLKIN, SW, frameData, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B);
    game gameEngine(CLOCK_50, SW[17], KEY, frameData);
endmodule