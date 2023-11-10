module FinalProject(input VGA_CLK, input [17:0]SW, output VGA_HS, output VGA_VS, output [7:0]VGA_R, output [7:0]VGA_G, output [7:0]VGA_B, output reg [17:0]LEDR);
    VGA doathing(VGA_CLK, SW[17], VGA_HS, VGA_VS, VGA_R, VGA_G, VGA_B);
    always@(*)
	 begin
	 LEDR[0] = SW[0];
	 end
endmodule