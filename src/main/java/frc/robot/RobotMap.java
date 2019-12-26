/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int kFrontLeftPort = 0; // Talon SRX with encoder
  public static final int kFrontRightPort = 1; // Talon SRX with encoder
  public static final int kMiddleLeftPort = 5; // Talon SRX
  public static final int kMiddleRightPort = 6; // Talon SRX
  public static final int kRearLeftPort = 1; // Victor SPX
  public static final int kRearRightPort = 3; // Victor SPX
  
  public static final int kDriverJoyPort = 0;
  public static final int kOperateJoyPort = 1;
  public static final int kArmPort = 3;
  public static final int kElevatorPort = 2;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
