package serial_arduino;

import java.io.InputStream;
import java.io.OutputStream;

import com.fazecast.jSerialComm.SerialPort;

public class Serial_arduino {
	public static void main(String[] args) {
        
		SerialPort comPort = SerialPort.getCommPort("COM3");
		comPort.openPort();
		comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);
		InputStream in = comPort.getInputStream();
		OutputStream out = comPort.getOutputStream();
		try
		{
		   while(true) {
		      char read=(char) in.read();
		      System.out.println((char)read);
		      if(read=='1'){
		    	  break;
		      }
		   }
		   in.close();
		   for(int i=0;i<(15360-2);i++) {
			   if(i==0)
				   out.write((byte) (180));
			   else {
				   
				   if((i%2)==1)
					   out.write((byte) (100));
				   else
					   out.write((byte) (100));
				   //out.write((byte) (i%150)+75);
				  // System.out.println(((i%150)+75));
				   }
		   }
		   out.close();
		} catch (Exception e) { e.printStackTrace(); }
		comPort.closePort();
	      System.out.println("fim ");
	   }
}
