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
public class Arm extends Subsystem {
  private WPI_TalonSRX master;
  private int targetPosition;
  private boolean isStoping;

  //motion values
  private final int kMaxAcceleration = 500;//acceleration limit
  private final int kMaxVelocity = 500;//velocity limit

  //PID VALUES
  private final int kMagicSlot = 0;
  private final double kMagicP = 1;
  private final double kMagicI = 0;
  private final double kMagicD = 0.1;
  private final double kMagicF = 2.046;

  //kF consts
  private int ticksAtHorizontal = 2800;
  private double ticksPerDegree = 4096/360;


  public enum WristPos
  {
    UP(1780),
    DOWN(2775),
    INSIDE(1000),
    COLLECT_CARGO(3100),
    HIGH_CARGO(2450);
    
    private final int position;

    private WristPos(int position)
    {
        this.position = position;
    }

    public int getPosition()
    {
      return position;
    }
  }

  public Arm()
  {
    master = new WPI_TalonSRX(RobotMap.kArmPort);
    
    master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    master.setNeutralMode(NeutralMode.Brake);

    master.set(ControlMode.PercentOutput, 0);

    master.setSensorPhase(true);
    master.setInverted(InvertType.InvertMotorOutput);

    master.configMotionAcceleration(kMaxAcceleration);
    master.configMotionCruiseVelocity(kMaxVelocity);

    configProfileSlot(kMagicSlot, kMagicP, kMagicI, kMagicD, kMagicF);

    isStoping = true;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void configProfileSlot(int profileSlot, double kP, double kI, double kD, double kF)
  {
    master.config_kP(profileSlot, kP, 10);
    master.config_kI(profileSlot, kI, 10);
    master.config_kD(profileSlot, kD, 10);
    master.config_kF(profileSlot, kF, 10);
  }

  public void setPercentOutput(double power) {
    isStoping = false;
    master.set(ControlMode.PercentOutput, power);
    targetPosition = getCurrentPosition() + (int)(master.getSelectedSensorVelocity()/100 * 1000  * 0.02);
  }

  public void stop(){
    isStoping = true;
    targetPosition = getCurrentPosition();

    double degrees = (targetPosition - ticksAtHorizontal)/ticksPerDegree;
    double radians = Math.toRadians(degrees);
    double cosineScalar = Math.cos(radians);
    double maxGravityFF = -0.11; 

    master.set(ControlMode.PercentOutput, maxGravityFF * cosineScalar);

  }

  public void goTo(int pos){
    isStoping = false;
    targetPosition = pos;
    master.selectProfileSlot(kMagicSlot, 0);
    master.set(ControlMode.MotionMagic, pos);
  }

  public int getCurrentPosition(){
    return this.master.getSelectedSensorPosition();
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
  }

}
