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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX master;
  private int targetPosition;
  private boolean isStoping;

  //motion values
  private final int kMaxAcceleration = 500;//acceleration limit
  private final int kMaxVelocity = 500;//velocity limit

  //PID VALUES
  private final int kMagicSlot = 0;
  private final double kMagicP = 0.3;
  private final double kMagicI = 0.0001;
  private final double kMagicD = 0;
  private final double kMagicF = 2.046;

  //limit
  private int ticksAtTop = 2800;



  public enum ElevatorPos
  {
    FLOOR(0),
    COLLECT_CARGO(4780),
    COLLECT_CARGO_FEEDER(21300),
    COLLECT_HATCH(1500), 
    LOW_HATCH(6200),
    LOW_CARGO(8960),
    MIDDLE_HATCH(27570),
    MIDDLE_CARGO(30280),
    CARGO_SHIP(31500),
    HIGH_HATCH(45850),
    HIGH_CARGO(47500),
    LIFT(17000);
    
    private final int position;

    private ElevatorPos(int position)
    {
        this.position = position;
    }

    public int getPosition()
    {
      return position;
    }
  }

  public Elevator()
  {
    //config motor
    master = new WPI_TalonSRX(RobotMap.kElevatorPort);
    master.setNeutralMode(NeutralMode.Brake);
    
    //config encoder
    master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    master.setSensorPhase(true);
    master.setInverted(InvertType.InvertMotorOutput);

    //limits
    master.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, Robot.driveTrain.GetEleSwitchDeviceId(), 0);
    master.overrideLimitSwitchesEnable(true);
    
    master.set(ControlMode.PercentOutput, 0);

    //config motion magic
    master.configMotionAcceleration(kMaxAcceleration);
    master.configMotionCruiseVelocity(kMaxVelocity);

    configProfileSlot(kMagicSlot, kMagicP, kMagicI, kMagicD, kMagicF);

    isStoping = false;
  }

  public void configProfileSlot(int profileSlot, double kP, double kI, double kD, double kF)
  {
    master.config_kP(profileSlot, kP, 10);
    master.config_kI(profileSlot, kI, 10);
    master.config_kD(profileSlot, kD, 10);
    master.config_kF(profileSlot, kF, 10);
  }

  public void stop(){
    isStoping = true;

    targetPosition = getCurrentPosition();
    double maxGravityFF = 0.236; 

    master.set(ControlMode.PercentOutput, maxGravityFF);

    System.out.println("stop");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public int getCurrentPosition(){
    return this.master.getSelectedSensorPosition();
  }

  public void set(ControlMode mode, double value)
  {
    master.set(mode, value);
  }

  public void setPercentOutput(double power) {
    isStoping = false;

    master.set(ControlMode.PercentOutput, power);
    targetPosition = getCurrentPosition() + (int)(master.getSelectedSensorVelocity()/100 * 1000  * 0.02);
  }

  public void goTo(int pos){
    isStoping = false;

    targetPosition = pos;
    master.selectProfileSlot(kMagicSlot, 0);
    master.set(ControlMode.MotionMagic, pos);
  }

  public double getOutputCurrent() {
    return master.getOutputCurrent();
    
  }

  public boolean atTarget() {
    return Math.abs(getCurrentPosition() - targetPosition) < 8;
  }

  @Override
  public void periodic() {
    if(isStoping) {
      stop();
    }

    if (Robot.driveTrain.getIsElevatorLimit()) {
        this.master.setSelectedSensorPosition(0);
    }
  }
}
