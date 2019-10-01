package poroslib.commands;

import edu.wpi.first.wpilibj.command.Command;
import poroslib.subsystems.MechDrivetrain;
import poroslib.triggers.SmartJoystick;

/**
 *
 */
public class MechanumDrive extends Command{
	
	private MechDrivetrain drivetrain;
	private SmartJoystick joy;
	
	private double maxOutput = 1;

    public MechanumDrive(MechDrivetrain drivetrain, SmartJoystick joy)
    {   
    	this.drivetrain = drivetrain;
    	this.joy = joy;
    	requires(this.drivetrain);
    }

    
    protected void initialize()
    {
    	
    }

    
    protected void execute()
    {
    	double speedValue = this.joy.getSpeedAxis();
    	double rotateValue = this.joy.getRotateAxis();
    	double twistValue =  this.joy.getTwistAxis();
    	
    	
    	if(this.drivetrain.IsRanged())
    	{
    		this.maxOutput = this.joy.getSlider();
    	}
    	else
    	{
    		this.maxOutput = 1;
    	}
    	
    	this.drivetrain.MechanumDrive(speedValue, rotateValue, twistValue, 0, this.maxOutput);
    }

    
    protected boolean isFinished()
    {
        return false;
    }

    
    protected void end()
    {
    	this.drivetrain.MechanumDrive(0, 0, 0, 0, this.maxOutput);
    }

    
    protected void interrupted()
    {
    	end();
    }
}
