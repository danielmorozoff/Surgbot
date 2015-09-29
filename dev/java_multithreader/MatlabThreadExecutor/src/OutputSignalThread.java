


import com.mathworks.jmi.*;
public class OutputSignalThread extends Thread {

	public Matlab matlabEngine = null;

	
	public OutputSignalThread(){
	    
		matlabEngine = new Matlab();
		Matlab.whenMatlabReady(this);
	}
	
	@Override 
    public void run(){
		 //Create a proxy, which we will use to control MATLAB -- this is an APU wrapper.
			runMatMeth("pool= parpool('local', 2)");
			
	    	runMatMeth("a  = Autodrill('output')");
	    	runMatMeth("batch('a.sendDAQProbePulseTimed(100,100)')");
	    	
	    	runMatMeth("delete(pool)");
//	    	System.out.println("This is the class of Autodrill that the  returns: "+obj.getClass()+"\n");
	    
    }
	
	private void runMatMeth(String command){
	    try
	    {
			String[] cmd_ar = new String [1];
				cmd_ar[0] = command;
			 matlabEngine.mtFevalConsoleOutput("eval", cmd_ar, 0);
		} catch (Exception e) {
	    	System.out.println("***ERROR OCCURED IN: OutputSignalThread***");
	        dumpStack();
	    }
	}
}