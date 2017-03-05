package IO;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {
	private static Logger instance = null;
	private String path = "/home/lvuser/";
	private File file, posFile;
    private BufferedWriter out, posOut;
	public Logger(){
		try{
			Date date = new Date() ;
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss") ;
			file = new File(path + "Log.txt");
			posFile = new File(path + "Position.txt");
			out = new BufferedWriter(new FileWriter(file,true));
			posOut = new BufferedWriter(new FileWriter(posFile,true));
			out.write("NEW BOOT");
			out.newLine();
			out.flush();
			
			posOut.write("}{");
			posOut.newLine();
			posOut.flush();
		}catch (Exception e) {
	        // TODO Auto-generated catch block
	        System.out.println(e);
	    }
	}
	public void writeToLog(String item){
		try {			
			out.append(item);
			out.newLine();
			out.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.print(e);
		}
	}
	public void writePosition(double X, double Y){
		try {
			posOut.append(",("+Double.toString(Math.floor(X))+","+Double.toString(Math.floor(Y))+","+Long.toString(System.currentTimeMillis())+")");
			posOut.newLine();
			posOut.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.print(e);
		}
	}
	public static Logger getInstance(){
		if( instance == null )
            instance = new Logger();
        return instance;
	}
}