/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX master;


  public Elevator()
  {
    master = new WPI_TalonSRX(RobotMap.kElevatorPort);
    
    master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    master.setNeutralMode(NeutralMode.Brake);

    master.set(ControlMode.PercentOutput, 0);

    master.setSensorPhase(true);
    master.setInverted(InvertType.InvertMotorOutput);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void set(ControlMode mode, double value)
  {
    master.set(mode, value);
  }
}
