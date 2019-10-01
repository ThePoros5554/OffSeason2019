/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ActivateArm;
import frc.robot.commands.ActivateElevator;
import poroslib.triggers.JoyAxis;
import poroslib.triggers.JoyAxisPart;
import poroslib.triggers.SmartJoystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
////////////////////////ports///////////////////////////////
  private static final int kWristAxis = 1; // L
    private static final int kElevatorUpAxis = 3; // RT
    private static final int kElevatorDownAxis = 2;//LT
    ///////////////////////////////////////////////////////////

  public static SmartJoystick driverJoy;
  public static SmartJoystick operatorJoy;
  


  /////////////////////////Commands/////////////////////////
  //////////////////////////////////////////////////////////

 
  public OI()
  {
    driverJoy = new SmartJoystick(RobotMap.kDriverJoyPort);
    operatorJoy = new SmartJoystick(RobotMap.kOperateJoyPort);
    driverJoy.setSpeedAxis(1);
    driverJoy.setRotateAxis(4);

    ////////////////////////////////Activators////////////////
    JoyAxisPart wristUpAxis = new JoyAxisPart(operatorJoy, kWristAxis, -1, 1, 1, -1, 0.2, 1);
    JoyAxisPart wristDownAxis = new JoyAxisPart(operatorJoy, kWristAxis, -1, 1, 1, -1, -1, -0.2);

    JoyAxis elevatorUpAxis = new JoyAxis(operatorJoy, kElevatorDownAxis, 0, 1, -1, 0);
    JoyAxis elevatorDownAxis = new JoyAxis(operatorJoy, kElevatorUpAxis, 0, -1, -1, 0);
  ///////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////Commands///////
    ActivateArm wristDown = new ActivateArm(wristDownAxis);
    ActivateArm wristUp = new ActivateArm(wristUpAxis);

    ActivateElevator elevatorDown = new ActivateElevator(elevatorDownAxis);
    ActivateElevator elevatorUp = new ActivateElevator(elevatorUpAxis);
    /////////////////////////////////////////////////////



    wristDownAxis.whileActive(wristDown);
    wristUpAxis.whileActive(wristUp);
    
    elevatorUpAxis.whileActive(elevatorUp);
    elevatorDownAxis.whileActive(elevatorDown);
  }
}
