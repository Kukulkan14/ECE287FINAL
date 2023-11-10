module VGA(input clk, input rst, output reg VGA_HS, output reg VGA_VS, output reg [7:0]VGAR, output reg [7:0]VGAG, output reg [7:0]VGAB);
// Create FSM to control VGA states. Parameters Credit to V. Hunter Adams
    // Horizontal parameters (measured in clock cycles)
    parameter [9:0] H_ACTIVE  =  10'd_639 ;
    parameter [9:0] H_FRONT   =  10'd_15 ;
    parameter [9:0] H_PULSE   =  10'd_95 ;
    parameter [9:0] H_BACK    =  10'd_47 ;

    // Vertical parameters (measured in lines)
    parameter [9:0] V_ACTIVE   =  10'd_479 ;
    parameter [9:0] V_FRONT    =  10'd_9 ;
    parameter [9:0] V_PULSE =  10'd_1 ;
    parameter [9:0] V_BACK  =  10'd_32 ;

    // Parameters for readability
    parameter   LOW     = 1'b_0 ;
    parameter   HIGH    = 1'b_1 ;

    // States (more readable)
    parameter   [7:0]   H_ACTIVE_STATE    = 8'd_0;
    parameter   [7:0]   H_FRONT_STATE     = 8'd_1;
    parameter   [7:0]   H_PULSE_STATE   = 8'd_2;
    parameter   [7:0]   H_BACK_STATE     = 8'd_3;

    parameter   [7:0]   V_ACTIVE_STATE    = 8'd_0 ;
    parameter   [7:0]   V_FRONT_STATE    = 8'd_1 ;
    parameter   [7:0]   V_PULSE_STATE   = 8'd_2 ;
    parameter   [7:0]   V_BACK_STATE     = 8'd_3 ;

    // Declare State Variables
    reg [3:0]S;
    reg [3:0]NS;

    // Initialization Always Block
    always@(posedge clk or negedge rst)
    begin
        if (rst == 1'b0)
        begin
            S <= H_ACTIVE_STATE;
        end
        else
        begin
            S <= NS;
        end
    end

    // NS Always Block
    always@(*)
    begin
        case(S)
            H_ACTIVE_STATE:
            begin
                if (counterHActive < H_ACTIVE)
                    NS <= H_ACTIVE_STATE;
                else
                    NS <= H_FRONT_STATE;
            end
            H_FRONT_STATE: 
            begin
                if (counterHFront < H_FRONT)
                    NS <= H_FRONT_STATE;
                else
                    NS <= H_PULSE_STATE;
            end
            H_PULSE_STATE: 
            begin
                if (counterHPulse < H_PULSE)
                    NS <= H_PULSE_STATE;
                else
                    NS <= H_BACK_STATE;
            end
            H_BACK_STATE:
            begin
                if (counterHBack < H_BACK)
                    NS <= H_BACK_STATE;
                else
                begin
                    if (counterVActive < V_ACTIVE)
                    begin
                        NS <= H_ACTIVE_STATE;
                    end
                    else
                    begin
                        NS <= V_FRONT_STATE;
                    end
                end
            end
        endcase
    end

//declare counters
reg [9:0] counterHActive;
reg [9:0] counterHFront;
reg [9:0] counterHPulse;
reg [9:0] counterHBack;
reg [9:0] counterVActive;
reg [9:0] counterVFront;
reg [9:0] counterVPulse;
reg [9:0] counterVBack;

    // Functionality Always Block
    always@(posedge clk or negedge rst)
	 begin
    if (rst == 1'b0)
    begin
        // initialize counters so that 1 is true at a time for VS
        counterVActive <= 10'd0;
        counterVFront <= V_FRONT + 1'b1;
        counterVPulse <= V_PULSE + 1'b1;
        counterVBack <= V_BACK + 1'b1;
        counterHActive <= 10'd0;
        counterHFront <= 10'd0;
        counterHPulse <= 10'd0;
        counterHBack <= 10'd0;
    end
        case(S)
            H_ACTIVE_STATE:
            begin
                // Turn VGA_HS on. Determine state of VGA_VS. Assign pixels to white or turn off depending on position.
                VGA_HS <= HIGH;
                if (counterVFront <= V_FRONT)
                begin
                    VGA_VS <= HIGH;
                end
                else if (counterVPulse <= V_PULSE)
                begin
                    VGA_VS <= LOW;
                end
                else if (counterVBack <= V_BACK)
                begin
                    VGA_VS <= HIGH;
                end
                if (counterVActive <= V_ACTIVE)
                begin
                    VGA_VS <= HIGH;
                    VGAR <= 8'b11111111;
                    VGAG <= 8'b11111111;
                    VGAB <= 8'b11111111;
                end
                else
                begin
                    VGAR <= 8'b00000000;
                    VGAG <= 8'b00000000;
                    VGAB <= 8'b00000000;
                end
                counterHActive <= counterHActive + 1'b1;
                if (counterHActive == H_ACTIVE)
                begin
                    counterHFront <= 10'd0;
                end
            end
            H_FRONT_STATE: 
            begin
                // Turn pixels off while H in frontporch area.
                VGAR <= 8'b00000000;
                VGAG <= 8'b00000000;
                VGAB <= 8'b00000000;
                counterHFront <= counterHFront + 1'b1;
                if (counterHFront == H_FRONT)
                begin
                    counterHPulse <= 10'd0;
                end
            end
            H_PULSE_STATE:
            begin
                // Turn HS off during pulse area.
                VGA_HS <= LOW;
                counterHPulse <= counterHPulse + 1'b1;
                if (counterHPulse == H_PULSE)
                begin
                    counterHBack <= 10'd0;
                end
            end
            H_BACK_STATE:
            begin
                // Turn HS on during back area. Iterate counters for VS.
                VGA_HS <= HIGH;
                counterHBack <= counterHBack + 1'b1;
                if (counterHBack == H_BACK)
                begin
                    counterHActive <= 10'd0;
                    if (counterVActive <= V_ACTIVE)
                    begin
                        if (counterVActive == V_ACTIVE)
                        begin
                            counterVFront <= 10'd0;
                        end
                        counterVActive <= counterVActive + 1'b1;
                    end
                    else if (counterVFront <= V_FRONT)
                    begin
                        if (counterVFront == V_FRONT)
                        begin
                            counterVPulse <= 10'd0;
                        end
                        counterVFront <= counterVFront + 1'b1;
                    end
                    else if (counterVPulse <= V_PULSE)
                    begin
                        if (counterVPulse == V_PULSE)
                        begin
                            counterVBack <= 10'd0;
                        end
                        counterVPulse <= counterVPulse + 1'b1;
                    end
                    else if (counterVBack <= V_BACK)
                    begin
                        if (counterVBack == V_BACK)
                        begin
                            counterVActive <= 10'd0;
                        end
                        counterVBack <= counterVBack + 1'b1;
                    end
                end
            end
        endcase
    end
endmodule
