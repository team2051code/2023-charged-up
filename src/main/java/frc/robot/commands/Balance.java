// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import javax.swing.text.BadLocationException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public static double SPEED_M_S = .5;
  public static final double kPDriveVal = 1;public static final double kIDriveVal = .6;public static final double kDDriveVal = .03;
  public static final double kOnRampCrosser = 2;
  public static final double kPivotCrosser = 1;
  private PIDController m_left = new PIDController(kPDriveVal, kIDriveVal,kDDriveVal);
  private PIDController m_right = new PIDController(kPDriveVal, kIDriveVal, kDDriveVal);
  private DriveSubsystem m_drive;
  private LinearFilter m_gyroFilter = LinearFilter.movingAverage(20);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Balance(DriveSubsystem subsystem) {
    m_drive = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Balance reached");
    SmartDashboard.setDefaultNumber("Balance/BPVal", 1.2);
    SmartDashboard.setPersistent("Balance/BPVal");
    SmartDashboard.setDefaultNumber("Balance/BSetpoint", 0.5);
    SmartDashboard.setPersistent("Balance/BSetpoint");;
    m_left = new PIDController(SmartDashboard.getNumber("Balance/BPVal", 1.2), kIDriveVal, kDDriveVal);
    m_right = new PIDController(SmartDashboard.getNumber("Balance/BPVal", 1.2), kIDriveVal, kDDriveVal);
    SPEED_M_S = 0.5;
    m_left.reset();
    m_right.reset();
    m_left.setSetpoint(SmartDashboard.getNumber("Balance/BSetpoint",0.5));
    m_right.setSetpoint(SmartDashboard.getNumber("Balance/BSetpoint",0.5));
    m_drive.resetEncoders();
    m_drive.zeroHeading();
    m_gyroFilter.reset();
    m_drive.zeroHeading();
    m_drive.setAutoDrive(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Balance/Balancing", true);
    SmartDashboard.putNumber("Balance/Left Setpoint", m_left.getSetpoint());
    var wheelSpeeds = m_drive.getWheelSpeeds();
    SmartDashboard.putNumber("Balance/Wheelspeeds", wheelSpeeds.leftMetersPerSecond);
    var leftVelocity = m_left.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightVelocity = m_right.calculate(wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Balance/PIDPower", leftVelocity);
    double rawYAngle = m_drive.getYAngle();

    double overrideYAngle = SmartDashboard.getNumber("Balance/Gyro Override", 0);
    rawYAngle = overrideYAngle != 0? overrideYAngle : rawYAngle;
    
    //converts angle to a range of -180 to 180
    if (rawYAngle > 180)
    {
      rawYAngle -= 360;
    }
    double yAngle = m_gyroFilter.calculate(rawYAngle);
    SmartDashboard.putNumber("Balance/raw y angle", rawYAngle);
    SmartDashboard.putNumber("Balance/y angle", yAngle);
    //checks for whether ramp is level
    if (rawYAngle > 2.5)
    {
      //scales speed to an inverse function of angle drives forward
      SmartDashboard.putBoolean("Balance/Finished", false);
      m_left.setSetpoint(SmartDashboard.getNumber("Balance/BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1));
      m_right.setSetpoint(SmartDashboard.getNumber("Balance/BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1));
      m_drive.autoDrive(leftVelocity, rightVelocity);
    }else if (rawYAngle < -2.5)
    {
      //scales speed to an inverse function of angle drives backward
      SmartDashboard.putBoolean("Balance/Finished", false);
      m_left.setSetpoint(-(SmartDashboard.getNumber("Balance/BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1)));
      m_right.setSetpoint(-(SmartDashboard.getNumber("Balance/BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1)));
      m_drive.autoDrive(leftVelocity, rightVelocity);
    }else
    {
      //stops robot
      SmartDashboard.putBoolean("Balance/Finished", true);
      // m_left.setSetpoint(0);
      // m_right.setSetpoint(0);
      m_drive.autoDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAutoDrive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
