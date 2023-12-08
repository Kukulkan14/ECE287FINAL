import java.util.Scanner;

public class ScreenEncoder {
    public static void main(String[] args) {

        Scanner keys = new Scanner(System.in);
        String stringToEncode = keys.nextLine();
        while (stringToEncode.length() > 192) {
            System.out.println("The input is too long. Here is the string that will fit: " + stringToEncode.substring(0,193));
            System.out.println("Would you like to input a new string y/n?");
            if (keys.next().charAt(0) == 'y')
                stringToEncode = keys.nextLine();
            else
                stringToEncode = stringToEncode.substring(0,193);
        }
        String result = "";
        for (int i = 0; i < stringToEncode.length(); i++) {
            if ((int)stringToEncode.charAt(i) > 96) {
                if ((int) (stringToEncode.charAt(i) - 97) > 9)
                    result += "0" + (int) (stringToEncode.charAt(i) - 97);
                else  
                    result += "00" + (int) (stringToEncode.charAt(i) - 97);
            }
            
            else if ((int)stringToEncode.charAt(i) > 64)
                if ((int) (stringToEncode.charAt(i) - 39) > 9)
                    result += "0" + (int) (stringToEncode.charAt(i) - 39);
                else  
                    result += "00" + (int) (stringToEncode.charAt(i) - 39);
            else if ((int)stringToEncode.charAt(i) > 47 && (int)stringToEncode.charAt(i) <= 57) {
                result += "0" + (int) (stringToEncode.charAt(i) + 4);
            }
            else if ((int)stringToEncode.charAt(i) == 33) // !
                result += "063";
            else if ((int)stringToEncode.charAt(i) == 46) // .
                result += "062";
            else if ((int)stringToEncode.charAt(i) == 44) // ,
                result += "066";
            else if ((int)stringToEncode.charAt(i) == 63) // ?
                result += "064";
            else if ((int)stringToEncode.charAt(i) == 39) // '
                result += "065";
            else if ((int)stringToEncode.charAt(i) == 45) // -
                result += "067";
            else if ((int)stringToEncode.charAt(i) == 43) // +
                result += "068";
            else if ((int)stringToEncode.charAt(i) == 32) // ' '
                result += "070";
            else if ((int)stringToEncode.charAt(i) == 42) // ' '
                result += "070";
            else if ((int)stringToEncode.charAt(i) == 36) // Special
                result += "069";
            
        }
        String binaryResult = "";
        int i;
        for (i = 0; (i + 4) < result.length(); i+=3) {
            String current = String.format("%7s", Integer.toBinaryString(Integer.parseInt(result.substring(i, i + 3)))).replaceAll(" ", "0"); 
            binaryResult += current;
        }
        String current = String.format("%7s", Integer.toBinaryString(Integer.parseInt(result.substring(i)))).replaceAll(" ", "0"); 
        binaryResult += current;
        if ((binaryResult.length() / 7) < 192) {

            for (int j = (binaryResult.length() / 7); j < 192; j++) {
                binaryResult += "1000110";
            }

        }
        System.out.println(binaryResult);
    }

    
}
