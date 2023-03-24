// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveStraight extends CommandBase {
  public static final double SPEED_M_S = 1;
  public static final double kPDriveVal = 0.6;public static final double kIDriveVal = 0.4;public static final double kDDriveVal = .03;
  public static final double kOnRampCrosser = 2;
  public static final double kPivotCrosser = 1;
  int time;
//.4 .6 .03
  private DriveSubsystem m_drive;

  //private MedianFilter
  private PIDController m_left = new PIDController(kPDriveVal, kIDriveVal,kDDriveVal);
  private PIDController m_right = new PIDController(kPDriveVal, kIDriveVal, kDDriveVal);
  private LinearFilter m_gyroFilter = LinearFilter.movingAverage(20);
  private double m_xAccel;
  private Autostate m_autostate = Autostate.OFFRAMP;
  private boolean monitor = true;
  private Timer m_timer;
  private ArmSubsystem m_arm;

  private enum Autostate{
      OFFRAMP,ONRAMP,PIVOT
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraight(DriveSubsystem subsystem, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies
    m_drive = subsystem;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  //resets everything 
  @Override
  public void initialize() {
    m_autostate = Autostate.OFFRAMP;
    SmartDashboard.setDefaultNumber("DriveStraight/PVal", kPDriveVal);
    SmartDashboard.setPersistent("DriveStraight/PVal");
    SmartDashboard.setDefaultNumber("DriveStraight/Setpoint", SPEED_M_S);
    SmartDashboard.setPersistent("DriveStraight/Setpoint");

    m_arm.setOveride(true);
    m_arm.setBreak(true);
    m_arm.setArmPivotSetpoint(90);
    //m_left = new PIDController(SmartDashboard.getNumber("DriveStraight/PVal", kPDriveVal), kIDriveVal, kDDriveVal);
    m_left = new PIDController(kPDriveVal, kIDriveVal, kDDriveVal);
    //m_right = new PIDController(SmartDashboard.getNumber("DriveStraight/PVal", kPDriveVal), kIDriveVal, kDDriveVal);
    m_right = new PIDController(kPDriveVal,kIDriveVal, kDDriveVal);
    m_left.reset();
    m_right.reset();
    m_left.setSetpoint(SPEED_M_S);
    m_right.setSetpoint(SPEED_M_S);
    m_drive.resetEncoders();
    m_drive.zeroHeading();
    m_gyroFilter.reset();
    m_xAccel = 0;
    m_drive.zeroHeading();
    time = 0;
    m_drive.setAutoDrive(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1){
      m_arm.setBreak(false);
      m_arm.setOveride(false);
    }
    //Sets initial values
    SmartDashboard.putNumber("DriveStraight/read setpoint", m_left.getSetpoint());
    //SmartDashboard.putString("State:", m_autostate.toString());
    var wheelSpeeds = m_drive.getWheelSpeeds();
    SmartDashboard.putNumber("DriveStraight/Wheelspeeds", wheelSpeeds.leftMetersPerSecond);
    var leftVelocity = m_left.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightVelocity = m_right.calculate(wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("DriveStraight/leftVelocity", leftVelocity);
    double rawYAngle = m_drive.getYAngle();
    
    double overrideYAngle = SmartDashboard.getNumber("DriveStraight/Gyro Override", 0);
    rawYAngle = overrideYAngle != 0? overrideYAngle : rawYAngle;
    //Reduces Y angle to a range from -180-180
    if (rawYAngle > 180)
    {
      rawYAngle -= 360;
    }
    //filters the yAngle to a moving average 
    double yAngle = m_gyroFilter.calculate(rawYAngle);
    SmartDashboard.putNumber("DriveStraight/raw y angle", rawYAngle);
    SmartDashboard.putNumber("DriveStraight/y angle", yAngle);
    time++;
    //forces the robot to move forward 20 u to avoid angle spikes
    if (time > 20)
    {
      //When off the ramp the robot moves constantly forward
      if (m_autostate.equals(Autostate.OFFRAMP))
      {
        m_drive.autoDrive(leftVelocity, rightVelocity);
        //Checks for when the robot makes it on the ramp. Looking for a angle spike of 15
        if (yAngle > 15)
        {
          //changes the state of the robot to on ramp
          //starts a timer to stop balance from going early
          m_timer = new Timer();
          m_timer.start();
          m_timer.reset();
          m_autostate = Autostate.ONRAMP;
        }
      }
      //puts timer on Network Table
      if(m_timer != null)
        SmartDashboard.putNumber("Timer", m_timer.get());
      //triggers after the robot is on the ramp and the timer is over deadtime
      if (m_autostate.equals(Autostate.ONRAMP) && m_timer.get() > 0.5)//SmartDashboard.getNumber("DriveStraight/DeadTime", 0.4)
      {
        //checks for whether ramp is level once it is it will trigger balance command
        if (yAngle > -2.5)
        {
          //scales speed to an inverse function of angle drives forward
          m_left.setSetpoint(SPEED_M_S/((Math.abs(yAngle)/30)*9+1));
          m_right.setSetpoint(SPEED_M_S/((Math.abs(yAngle)/30)*9+1));
          m_drive.autoDrive(leftVelocity, rightVelocity);
        }
        // else if (yAngle < -2.5)
        // {
        //   //scales speed to an inverse function of angle drives backward
        //   System.out.println("BACKWARDS");
        //   m_left.setSetpoint(-(SmartDashboard.getNumber("DriveStraight/Setpoint", SPEED_M_S)/((Math.abs(yAngle)/30)*9+1)));
        //   m_right.setSetpoint(-(SmartDashboard.getNumber("DriveStraight/Setpoint", SPEED_M_S)/((Math.abs(yAngle)/30)*9+1)));
        //   System.out.println(m_left.getSetpoint() + " " + m_right.getSetpoint());
        //   m_drive.autoDrive(leftVelocity, rightVelocity);
        // }
        else 
        {
          //ends command and triggers balance command
          System.out.println("pro");
          m_autostate = Autostate.PIVOT;
        }
      }
    }
    else
    {
      //drives the robot forward for 20u
      m_drive.autoDrive(leftVelocity, rightVelocity);
    }
    

    // m_xAccel = m_gyroFilter.calculate(m_drive.getAccelX());
    // SmartDashboard.putNumber("Smooth X accel: ", m_xAccel);
    // SmartDashboard.putString("Autostate: ", m_autostate.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAutoDrive(false);
    if(m_timer != null)
      m_timer.stop();
    //starts balance com
    SmartDashboard.putString("DriveStraight/State", m_autostate.toString());
    if (interrupted) return;
    // Balance balance = new Balance(m_drive);
    // balance.andThen(() -> m_drive.tankDriveVolts(0, 0)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // switch(m_autostate){
    //   case OFFRAMP: 
    //     if(m_xAccel > kOnRampCrosser)
    //       m_autostate = Autostate.ONRAMP;
    //     break;
    //   case ONRAMP:
    //     if(m_xAccel<kPivotCrosser)
    //       m_autostate = Autostate.PIVOT;
    //     break;
    //   case PIVOT:
    //     return true;
    // }
    // return false;

    //ends command
    return m_autostate.equals(Autostate.PIVOT);
  }
}
