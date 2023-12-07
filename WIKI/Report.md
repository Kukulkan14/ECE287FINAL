# Introduction
This text-based, choose-your-own-adventure game was created by Michael Brown and Carl Porter, III on December 2nd, 2023. The project was created to entertain Dr. Jamieson and our fellow students as the game places the user in the shoes of a student taking an ECE 287 exam. The game is not serious and involves the user succeeding or failing in different comical ways. There are twenty four game states with nine endings. We greatly enjoyed creating this game and hope that everyone that plays it enjoys it.
# Functionality
## Control
- Switch 17 is our reset signal and resets the system when it is in the off position.
- The push button keys allow the user to make an input. The inputs are four to one from left to right.
## Game States
Within the game are 24 different game states plus a state for confirming user input. The confirm input state was made to prevent the user from flying through the game if they hold the button for too long. The game states are a mix of option screens, continue screens, and ending screens. The 24 screens are alphabetically as follows.
### ALIVE
- Your iPad is alive. You fly through the exam thinking how easy it is. While you are right, more time would have been better.
  - You passed! A-
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a win for the user as they pass the exam with an A-.
### ALT_NOW
- You go to get a new exam. As you do, you see your printer. Do you print another copy?
  - 1 Yes, I learned nothing!
  - 2 No, not again
  - 3 Yes, that was fun!
  - 4 No, go digital
- An option screen. The user is presented with four options. If buttons 1 or 3 are pressed, "PRINT" will be displayed. If buttons 2 or 4 are pressed, "NO_PRINT" will be displayed.
### CRUMBLE
- Despite what you think, you crumble under pressure. It could have been worse though. Maybe start sooner next time.
  - You passed! D+
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a win for the user as they pass the exam with a D+.
### DEAD
- Your iPad is dead. You wait for it to charge and then start. The extra time benefitted you.
  - You passed! A
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a win for the user as they pass the exam with an A.
### EXAM
- You are playing counterstrike 2 when you receive an Email from Dr. Jamieson. It is the 287 exam!
  - 1 Do it now
  - 2 Do it later
  - 3 Do it never
- An option screen. This is the official start of the game. The user is presented with three options. If button 1 is pressed, "NOW" will be displayed. If button 2 is pressed, "TWO_DAYS" will be displayed. If button 3 is pressed, "NEVER" will be displayed. When button 4 is pressed, there will be no change to the display.
### FIRE
- The fire travels across the very flammable grass. The university has no chance. You immediately graduate with a degree in engineering.
  - You passed! A+
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a win for the user as they pass the exam with an A+.
### FOOD
- You go to make food, but accidentally set the exam on fire! What do you do?
  - 1 Panic!
  - 2 Stay calm!
  - 3 Throw the exam in the sink!
  - 4 Put it out!
- An option screen. The user is presented with four options. If buttons 1 or 3 are pressed, "SINK" will be displayed. If buttons 2 or 4 are pressed, "PUT_OUT" will be displayed.
### GAMER
- You never took the exam, but became a professional CS2 player with all the free time you had.
  - You failed the exam, but passed at life. F+
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is up to the user for interpretation as they fail the exam but pass at life with an F+.
### GIVE_UP
- You think you made a good effort after almost burning your kitchen down. Dr. Jamieson does not agree.
  - You failed. F
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a lose for the user as they fail the exam with an F.
### NEVER
- You never take the exam.
  - You failed. F
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a lose for the user as they fail the exam with an F.
### NO_FOOD
- You ignore your hunger and take the exam on an empty stomach. As Snickers would put it, you get a little stupid when you're hungry.
  - You barely pass! D-
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is considered a win for the user as they barely pass the exam with a D-.
### NO_PRINT
- You go digital with your exam. You get your iPad, but did you charge it?
  - 1 Of course!
  - 2 No, I always forget
  - 3 No, it's dead
  - 4 It should be charged
- An option screen. The user is presented with four options. If buttons 1 or 3 are pressed, "DEAD" will be displayed. If buttons 2 or 4 are pressed, "ALIVE" will be displayed.
### NOW
- You decide to be responsible and start the exam. As you load the file, you see your printer. It has been a while since it's last use.
- A continue screen. When any button is pressed, "NOW_2" will be displayed.
### NOW_2
- Do you print out the exam?
  - 1 No, I'm not a caveman!
  - 2 Yes, the printer must be used.
  - 3 No, ink is too expensive!
  - 4 Yes, kill some trees!
- An option screen. The user is presented with four options. If buttons 1 or 3 are pressed, "NO_PRINT" will be displayed. If buttons 2 or 4 are pressed, "PRINT" will be displayed.
### ONE_DAY
- You ignore the exam for a day. There is now 1 day remaining. What do you do?
  - 1 Do it later
  - 2 Do it now
- An option screen. The user is presented with two options. If button 1 is pressed, "ZERO_DAYS" will be displayed. If button 2 is pressed, "NOW" will be displayed. If buttons 3 or 4 are pressed, there will be no change to the display.
### PRESSURE
- You go to start the exam, but can't remember if you work well under pressure. Do you?
  - 1 Hold my beer
  - 2 One way to find out
- An option screen. The user is presented with two options. If button 1 is pressed, "CRUMBLE" will be displayed. If button 2 is pressed, "SURPRISE" will be displayed. If buttons 3 or 4 are pressed, there will be no change to the display.
### PRINT
- You print out the exam, but are soon reminded of your hunger. Do you make food?
  - 1 Yes, with cheese.
  - 2 No, skip food
  - 3 Yes, without cheese
  - 4 No time!
- An option screen. The user is presented with four options. If buttons 1 or 3 are pressed, "FOOD" will be displayed. If buttons 2 or 4 are pressed, "NO_FOOD" will be displayed.
### PUT_OUT
- You grab the extinguisher and put out the fire. You need a new exam, but can't help but question if it's worth it.
  - 1 Get new exam
  - 2 Not worth it
- An option screen. The user is presented with two options. If button 1 is pressed, "ALT_NOW" will be displayed. If button 2 is pressed, "GIVE_UP" will be displayed. If buttons 3 or 4 are pressed, there will be no change to the display.
### SINK
- You throw the exam in the sink thinking it will land in water. You are wrong. It lands in a pot of hot oil instead. Your kitchen goes up in flames and the fire quickly spreads.
- A continue screen. When any button is pressed, "FIRE" will be displayed.
### START
- Welcome! When you see ^ press any button to continue.
- A continue screen. The first screen of the game. It informs the user of the symbol used to indicate a continue screen. When the user presses any button, "START_2" will be displayed.
### START_2
- Use the buttons on the board to make your selection. The order of the buttons is as follows from left to right 4 3 2 1
- A continue screen. This screen is the second part of the starting screens and informs the user of the correlation between the buttons on the DE2-115 board and the options presented. When the user presses any button, "EXAM" will be displayed.
### SURPRISE
- You do work well under pressure! You get the whole thing done with 2 minutes to spare!
  - You passed! B+
- An ending screen. The user will have to flip the reset switch if they want to play again. This ending is a win for the user as they pass the exam with a B+.
### TWO_DAYS
- You ignore the exam for a day. There are now 2 days remaining. What do you do?
  - 1 Do it later
  - 2 Do it now
- An option screen. The user is presented with two options. If button 1 is pressed, "ONE_DAY" will be displayed. If button 2 is pressed, "NOW" will be displayed. If buttons 3 or 4 are pressed, there will be no change to the display.
### ZERO_DAYS
- You ignore the exam for a day. There are now 0 days remaining. What do you do?
  - 1 Do it later
  - 2 Do it now
- An option screen. The user is presented with two options. If button 1 is pressed, "GAMER" will be displayed. If button 2 is pressed, "PRESSURE" will be displayed. If buttons 3 or 4 are pressed, there will be no change to the display.
## VGA
### VGA States
NOTE: All states transition after one clock cycle in the order listed unless otherwise noted.

- START initializes all variables to a known state.
- W2M_INIT initializes the variables used in writing data to the buffers to zero and enables writing to the first buffer. Finally, character_buf_mem_address is assigned the first seven-bit address of frame as it takes a clock cycle for the frame variable to update. Thus, character_buf_mem_address must be assigned here to be accurate and not in START.
- W2M_COND waits for the writing to be complete by waiting for the counter i to reach the size of the memory. It transitions to W2M_INC if there is more data to write or W2M_DONE if there is no more data to write.
- W2M_INC increments the counter i and the memory address while writing a solid color to the display depending on switches sixteen to fourteen. This is irrelevant as it will only be displayed for a frame before the game data is displayed from the later write and read logic. This state is included so the VGA display will be in a known state before the game begins.
- W2M_DONE turns off writing to the buffer and initializes the read_buf_mem_address to zero. Finally, it initializes current_character to the data output by the CharacterArray.v module as this data was not accurate in the START state or the W2M_INIT state as it takes multiple clock cycles for the data output from the CharacterArray.v module to be accurate once character_buf_mem_address is updated. This state loops before transitioning to RFM_INIT_START when frame_done is equal to one. This signal is one when the vga_driver.v module finishes displaying the data on the first buffer.
- RFM_INIT_START resets all the write variables to zero and assigns current_current character again. This is the state where frame is updated to display new data or redisplay the current data. completedFirstCycle is assigned zero so the data being written to the buffer not being read from can be initialized in the next state. The wr_id variable is assigned here which controls which buffer is currently being read from and which buffer is being written to. Finally, the first instance of the read logic is included. See the read structure section for more information.
- RFM_INIT_WAIT contains the read structure and a structure to initialize the data to be written to the buffer not being read from. This structure depends on the completedFirstCycle variable being zero so that it only executes once. In this structure, data is written to write_buf_mem_address location zero so that location has accurate data and is not written with data from the end of the last write cycle. In this structure, current_character, frame, character_buf_mem_address, and counterPixels go through their first incrementation. See the Data Writing section for more information. RFM_INIT_WAIT transitions to RFM_DRAWING if frame_done is equal to zero which means the vga_driver.v module is ready to draw the next frame. Otherwise, it loops.
- RFM_DRAWING contains the write logic and the read logic structures as well as the data structure. See the respective sections for more information.
### VGA Buffer Writing Structure
The VGA buffer writing logic works as follows: 
- counterPixels, counterLines, and character_count are counters to control timing and the value of write_buf_mem_address.
- counterPixels counts how many addressable pixels (only a 160x120 frame can be stored in the buffers while the display has a resolution of 640x480 so addressable pixels are groups of actual pixels) have passed so counterLines can be incremented by one and write_buf_mem_address can be incremented by 151 after the end of each line in the ten by ten pixel boxes that each character is written into. counterPixels is then reset to zero.
- counterLines counts how many lines in each of our ten by ten pixel boxes have been written. Once this value reaches nine, we reset counterLines and counterPixels, as both must be at nine before for this condition to be reached, and we subtract 1439 from the write_buf_mem_address to move the writing to the next ten by ten pixel box. Finally, we increment character_count by one.
- character_count counts how many ten by ten pixel boxes we have finished writing. When counterPixels and counterLines are both equal to nine and character_count is equal to fifteen, we reset all of the counters to zero. Then, we increment write_buf_mem_address by one to bring writing to the next line of ten by ten pixel boxes. 
- Finally, when counterPixels and counterLines are both equal to nine, character_count is equal to fifteen, and write_buf_mem_address is greater than or equal to 19199 (the final memory location for the frame), we reset all of the counters and write_buf_memory address. Further, character_done, which is part of the data writing logic, is set to zero and character_buf_mem_address is set to the first address in frame which was reset before this. Finally, writing to the buffer is disabled by assigning write_buf_mem_wren to zero;
- This whole structure and the data writing structure are dependent on write_buf_mem_wren being one. If it is not, then all of this is ignored and write_buf_mem_wren is kept zero, character_buf_mem_address is assigned the first address in frame, and frame is reset.
### VGA Data Writing Structure
The VGA buffer data writing logic functions as follows:
- current_character contains the data to draw one character. All of the encodings for the characters are stored in the CharacterArray.v Quartus IP memory module which updates current_character. If current_character's least significant bit is equal to one, then white is written to memory. If not, blue is written. current_character is shifted to the right by one in both cases so that the encoded data in the variable can be read sequentially.
- The above logic is executed if character_done is equal to zero which is true until the final line of each character box at the second to last location as counted by counterPixels. It takes a clock cycle to update the character_done variable; thus, the following logic is executed at the final location in each character box.
- When character_done is one, the final value is written to the character box using the same logic as above. In addition, character_buf_mem_address is assigned the seven most significant bits of frame and frame is shifted to the left by seven bits to prepare the next value. character_buf_mem_address is always ahead of current_character by one character which ensure the data being output to current_character is accurate when it is assigned in this area. The same is true for frame with respect to character_buf_mem_address. Finally, if the final character of the frame has been written, then frame is set to the value being input from the game.v module. Otherwise, frame is shifted left by seven bits. 
### VGA Read Structure
The VGA buffer reading logic is as follows:
- readCounter, readCounterOverall, and readLineCounter are used to control read_buf_mem_address which updates read_buf_mem_q which outputs data to the vga_driver.v module that writes to the display.
- readCounter counts clock cycles to increment readCounterOverall and control read_buf_mem_address. It increments readCounterOverall on odd clock cycles so readCounterOverall matches the value of the x coordinate of the display as it changes every even value. readCounter increments read_buf_mem_address every seventh clock cycle so read_buf_mem_data can be accurate for the next four pixels which are read over eight clock cycles. On the eighth clock cycle, readCounter is reset.
- readCounterOverall counts how many pixels have been written to in each line to increment readLineCounter. When readcounter is equal to six and readCounterOverall is equal to 639, read_buf_mem_address is reset to reread the same line of data. When readcounter is equal to seven and readCounterOverall is equal to 639, readLineCounter is incremented by one.
- readLineCounter counts how many times the same line of memory data has been read. To downscale the resolution, each line of memory must be read four times to complete the vertical component of each addressable pixel. When readcounter is equal to six, readCounterOverall is equal to 639, and readLineCounter is greater than or equal to three, then read_buf_mem_address is incremented by one to read the next line of data. When readcounter is equal to seven, readCounterOverall is equal to 639, and readLineCounter is greater than or equal to three, then they are all reset to zero.
### VGA Data Variables
- The frame variable is updated in the game.v module depending on the state and is passed to the vga_driver_memory_double_buf.v module through the FinalProject.v module.
- The frame variable contains 192 7-bit addresses that are used to reference the data stored in the CharacterArray.v Quartus IP memory module.
- character_buf_mem_address in the vga_driver_memory_double_buf.v module updates current_character with the data to draw each character.
# Citations
Dr. Peter Jamieson provided the vga_driver_memory_double_buf.v module and subsidiary files that were modified to work with our project.
