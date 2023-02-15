// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveStraight extends CommandBase {
  public static double SPEED_M_S = .5;
  public static final double kPDriveVal = .4;
  public static final double kIDriveVal = .6;
  public static final double kDDriveVal = .03;
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
  private double m_yAngle;
  private boolean hasReset = false;


  private enum Autostate{
      OFFRAMP,ONRAMP,PIVOT
  }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraight(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    m_drive = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SPEED_M_S = 0.5;
    m_autostate = Autostate.OFFRAMP;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("State:", m_autostate.toString());
    var wheelSpeeds = m_drive.getWheelSpeeds();
    var leftVelocity = m_left.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightVelocity = m_right.calculate(wheelSpeeds.rightMetersPerSecond);
    double rawYAngle = m_drive.getYAngle();
    if(hasReset)
      m_autostate = Autostate.ONRAMP;
    if (rawYAngle > 180)
    {
      rawYAngle -= 360;
    }
    double yAngle = m_gyroFilter.calculate(rawYAngle);
    SmartDashboard.putNumber("raw y angle", rawYAngle);
    SmartDashboard.putNumber("y angle", yAngle);
    time++;
    if (time > 20)
    {
      
      if (m_autostate.equals(Autostate.OFFRAMP))
      {
        m_drive.tankDrive(leftVelocity, rightVelocity);
        if (yAngle > 15)
        {
          m_autostate = Autostate.ONRAMP;
        }
      }
      if (m_autostate.equals(Autostate.ONRAMP))
      {
        if (yAngle > 2.5)
        {
          if (!monitor){
            monitor = true;
            SPEED_M_S/=2;
          }
          m_left.setSetpoint(SPEED_M_S/((yAngle/30)*9+1));
          m_right.setSetpoint(SPEED_M_S/((yAngle/30)*9+1));
          System.out.println("FORWARD");
          m_drive.tankDrive(leftVelocity, rightVelocity);
        }
        else if (yAngle < -2.5)
        {
          if (monitor){
            monitor = false;
            SPEED_M_S/=2;
          }
          m_left.setSetpoint(-SPEED_M_S/((yAngle/30)*9+1));
          m_right.setSetpoint(-SPEED_M_S/((yAngle/30)*9+1));
          System.out.println("BACKWARD");
          m_drive.tankDrive(leftVelocity, rightVelocity);
        }
        else 
        {
          System.out.println("pro");
          m_drive.tankDrive(0, 0);
          m_autostate = Autostate.PIVOT;
        }
      }
    }
    else
    {
      m_drive.tankDrive(leftVelocity, rightVelocity);
    }
    

    // m_xAccel = m_gyroFilter.calculate(m_drive.getAccelX());
    // SmartDashboard.putNumber("Smooth X accel: ", m_xAccel);
    // SmartDashboard.putString("Autostate: ", m_autostate.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hasReset = false;
    double rawYAngle = m_drive.getYAngle();
    if (rawYAngle > 180)
    {
      rawYAngle -= 360;
    }
    SmartDashboard.putNumber("End Y", rawYAngle);
    if(rawYAngle > 2.5 || rawYAngle < -2.5)
    {
      m_autostate = Autostate.ONRAMP;
      SmartDashboard.putString("State", m_autostate.toString());
      hasReset = true;
      
    }
    SmartDashboard.putString("State:", m_autostate.toString());
    m_drive.tankDrive(0, 0);
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
    return m_autostate.equals(Autostate.PIVOT);
  }
}
