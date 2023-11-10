module FinalProject(input CLOCK_50, 
input [17:0]SW, 
output VGA_HS, 
output VGA_VS, 
output [7:0]VGA_R, 
output [7:0]VGA_G, 
output [7:0]VGA_B, 
output reg [17:0]LEDR,
output VGA_SYNC,
output VGA_CLK);
    VGA doathing(CLOCK_50, SW[17], VGA_HS, VGA_VS, VGA_R, VGA_G, VGA_B, VGA_SYNC, VGA_CLK);
    always@(*)
	 begin
	    LEDR[0] = SW[0];
	 end
endmodule