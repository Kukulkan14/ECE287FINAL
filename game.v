module game(clk, rst, input1, input2, input3, input4);

[2:0]S;
[2:0]NS;
[2:0]LS;

always@(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin
        S <= START;
        LS <= START;
    end
    else
        S <= NS;
end

parameter START = 3'd0,
          firstChoice = 3'd1,
          secondChoice = 3'd2,
          thirdChoice = 3'd3,
          fourthChoice = 3'd4,
          ending       = 3'd5,
          confirmInput = 3'd6;

always@(*)
begin
    case(S)
        START: 
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = START;
        end
        firstChoice:
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = firstChoice;
        end
        secondChoice:
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = secondChoice;
        end
        thirdChoice:
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = thirdChoice;
        end
        fourthChoice:
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = fourthChoice;
        end
        ending:
        begin
            if (userInput == 1'b1)
                NS = confirmInput;
            else    
                NS = ending;
        end
        confirmInput:
        begin
            if (userInput == 1'b0)
            begin
                case(LS)
                    START: NS = firstChoice;
                    firstChoice: NS = secondChoice;
                    secondChoice: NS = thirdChoice;
                    thirdChoice: NS = fourthChoice;
                    fourthChoice: NS = ending;
                    ending: NS = START;
                endcase
            end
            else
                NS = confirmInput;
        end
    endcase
end

always@(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin

    end
    else
    begin
        case(S)
            START:
            begin
                
            end
        endcase
    end
end

endmodule