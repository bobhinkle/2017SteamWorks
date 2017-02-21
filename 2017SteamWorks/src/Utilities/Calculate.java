/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
/*
   The Calculate class is a static class that defines multiple utility calculation methods
*/
package Utilities;

	//wraps a given angle between -180 to 180 degrees
	public class Calculate {
    public static double wrapAngle(double angle) {
        return angle - 360 * Math.floor((angle + 180)/ 360);
    }
    public static double saturate(double input, double max, double min){//limit the minimum and maximum values of an input
        if(input>max)
            input = max;
        if(input<min)
            input = min;
        return input;
    }
    
    //wraps a given angle between 0 and 360 degrees
    public static double wrapAngle360(double angle){
        return angle%360;
    }
    
    //returns the hypotenuse a triangle with given side lengths
    public static double hypot(double x, double y){
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
    }
    
    
    public static double[] addAndShift(double[] arr,double a){//add most current value to an array and shift to the right
        for(int i=0; i<arr.length-1; i++)
        {
            arr[i]=arr[i+1];
        }
        arr[arr.length-1]=a;
        return arr;
    }
    
    public static boolean checkTolerance(double[] error, double tolerance)//check if a tolerance is being met
    {
        int counter=0;
        for (int i =0; i<error.length; i++)
        {
            if (Math.abs(error[i])<tolerance)
                counter++;
        }
        if(counter==error.length)
            return true;
        else
            return false;
    }
}
