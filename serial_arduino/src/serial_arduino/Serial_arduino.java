package serial_arduino;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;

import com.fazecast.jSerialComm.SerialPort;


public class Serial_arduino {
	public static void main(String[] args) throws IOException {


		Boolean fim=false;
		Boolean buffered_delta=false;
		int delta=0;
		int last_read=0;
		int[] buffer = new int[1];
		int extra_delta=0;

		SerialPort comPort = SerialPort.getCommPort("COM3");
		comPort.openPort();
		comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);
		InputStream in = comPort.getInputStream();
		OutputStream out = comPort.getOutputStream();
		PrintWriter texto = new PrintWriter("filename.txt");
		try
		{


			WavFile wavFile = WavFile.openWavFile(new File("C:\\Users\\PC\\Music\\seno.wav"));
			wavFile.display();
			int numChannels = wavFile.getNumChannels();
			if(numChannels >1) {
				System.out.println("Ficheiro nao momo");
				return;
			}
			
			while(true) {


				while(true) {
					char read=(char) in.read();
					
					if(read=='1'){
						System.out.println("inicio");
						break;
					}
				}

					for(int i=0;i<(15360-2);i++) {

						texto.println(i);
						System.out.println(i);
						if(buffered_delta==false)
							if(fim==false)
								if((wavFile.readFrames(buffer,1))==-1)//nao leu uma amostra
									fim=true;
								else {
									buffer[0]=(int)Math.round(((float)buffer[0])/((float) 64));//ajustar a 10 bits
									texto.print((int)buffer[0] +"--> ");
								}
						if(fim==false) {
							if(buffered_delta==false){
								delta=buffer[0]-last_read;
								last_read=buffer[0];
								if(Math.abs(delta)>126) {
									if(delta>0) {
										extra_delta+=delta-126;
										delta=126;
									}
									else {
										extra_delta+=delta+126;
										delta=-126;
									}
								}
								else if(extra_delta!=0) {
									if(extra_delta>0)
										while(delta<126 && extra_delta>0) {
											delta++;
											extra_delta--;
										}
									if(extra_delta<0)
										while(delta>-126 && extra_delta<0) {
											delta--;
											extra_delta++;
										}
								}
							}
							if(buffered_delta==true)
								buffered_delta=false;
						}
						if(fim==true)
							delta=0;


						int envios=0;
						if(Math.abs(delta)>63)
							envios=3;
						else
							envios=2;
						if((15360-2)-i<envios) {//nao há espaco suficiente
							buffered_delta=true;
							out.write((byte) (180));//sinal de fim
							texto.print((int) (180) + "  ");
							texto.println((byte) (180) + " / ");
							continue;
						}
						else {   
							texto.print(delta+ "-->");
							if(delta>0) {
								out.write((byte) (75));//positivo
								texto.print( (75) + " ");
							}
							else {
								out.write((byte) (84));//negativo
								texto.print( (84)+ " ");
							}
							if(envios==2) {
								out.write((byte) ((Math.abs(delta))+96));//entre 0 e 63
								texto.print(((int)(Math.abs(delta)+96))+"  ");
								texto.println(((byte)(Math.abs(delta)+96))+" / ");
								i++;
							}
							if(envios==3) {
								out.write((byte) (63+96));//64
								texto.print((int) (63+96)+ "  ");
								texto.print((byte) (63+96)+ " / ");
								
								i++;
								out.write((byte) ((Math.abs(delta))+(-63+96)));//+ resto
								texto.print(((int) Math.abs(delta) + (-63+96))+ "  ");
								texto.println((byte) ((Math.abs(delta))+(-63+96))+ " / ");
								i++;
							}


						}
					}
					System.out.println("fim ");
					texto.flush();
				}

			} 

			
		
		catch (Exception e) { 
			e.printStackTrace();
		}
		in.close();


		out.close();
		comPort.closePort();
		System.out.println("fim ");
		texto.flush();
	}
}

