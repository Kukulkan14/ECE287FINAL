import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Scanner;

public class Mif {

	public static void main(String[] args) {
		File f = new File("char.txt");
		Scanner fileReader = null;
		PrintWriter pw = null;
		int i = 0;
		String temp = "";
		
		try {
			fileReader = new Scanner(f);
			pw = new PrintWriter(new File("characters.txt"));
			System.out.println("Hello");
			pw.print("-- Copyright (C) 2019  Intel Corporation. All rights reserved.\n"
					+ "-- Your use of Intel Corporation's design tools, logic functions \n"
					+ "-- and other software and tools, and any partner logic \n"
					+ "-- functions, and any output files from any of the foregoing \n"
					+ "-- (including device programming or simulation files), and any \n"
					+ "-- associated documentation or information are expressly subject \n"
					+ "-- to the terms and conditions of the Intel Program License \n"
					+ "-- Subscription Agreement, the Intel Quartus Prime License Agreement,\n"
					+ "-- the Intel FPGA IP License Agreement, or other applicable license\n"
					+ "-- agreement, including, without limitation, that your use is for\n"
					+ "-- the sole purpose of programming logic devices manufactured by\n"
					+ "-- Intel and sold by Intel or its authorized distributors.  Please\n"
					+ "-- refer to the applicable agreement for further details, at\n"
					+ "-- https://fpgasoftware.intel.com/eula.\n"
					+ "\n"
					+ "-- Quartus Prime generated Memory Initialization File (.mif)\n"
					+ "\n"
					+ "WIDTH=128;\n"
					+ "DEPTH=100;\n"
					+ "\n"
					+ "ADDRESS_RADIX=UNS;\n"
					+ "DATA_RADIX=UNS;\n"
					+ "\n"
					+ "CONTENT BEGIN\n"
					+ "\t\t");
			while(fileReader.hasNextLine()) {
				temp = fileReader.nextLine();
				pw.print(i + "   :   " + temp + ";\n\t\t");
				i++;
			}
			pw.print("[" + i + "..127]  :   " + temp + ";\n"
					+ "END;\n");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			try {
				fileReader.close();
				pw.close();
			} catch(Exception d) {
				d.printStackTrace();
			}
		}
	}
}
