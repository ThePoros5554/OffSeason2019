package poroslib.util;

public class MathHelper 
{	
	public static double mapRange(double oldMin, double oldMax, double newMin , double newMax, double value)
	{
		System.out.println("oldMin: " + oldMin);
		System.out.println("oldMax: " + oldMax);
		System.out.println("newMin: " + newMin);
		System.out.println("newMax: " + newMax);

		double newSlope = (newMax - newMin) / (oldMax - oldMin);
		System.out.println("newSlope: " + newSlope);
		double newB = newMax - (newSlope*oldMax);
		System.out.println("newB: " + newB);
		System.out.println("value: " + value);

		double output = (newSlope * value) + newB;
		
		return output;
	}
	
    public static double handleDeadband(double value, double deadband) 
    {
    	double newValue = 0;
    	
    	if(Math.abs(value) > Math.abs(deadband))
    	{
    		newValue = value;
    	}
    	
        return newValue;
	}
	
}

