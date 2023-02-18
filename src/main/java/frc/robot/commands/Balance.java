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
  public static final double kPDriveVal = 1;
  public static final double kIDriveVal = .6;
  public static final double kDDriveVal = .03;
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
    m_left = new PIDController(SmartDashboard.getNumber("BPVal", kPDriveVal), kIDriveVal, kDDriveVal);
    m_right = new PIDController(SmartDashboard.getNumber("BPVal", kPDriveVal), kIDriveVal, kDDriveVal);
    SPEED_M_S = 0.5;
    m_left.reset();
    m_right.reset();
    m_left.setSetpoint(SmartDashboard.getNumber("BSetpoint",SPEED_M_S));
    m_right.setSetpoint(SmartDashboard.getNumber("BSetpoint",SPEED_M_S));
    m_drive.resetEncoders();
    m_drive.zeroHeading();
    m_gyroFilter.reset();
    m_drive.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Balance", true);
    SmartDashboard.putNumber("Setpoint: ", m_left.getSetpoint());
    var wheelSpeeds = m_drive.getWheelSpeeds();
    SmartDashboard.putNumber("Wheelspeeds: ", wheelSpeeds.leftMetersPerSecond);
    var leftVelocity = m_left.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightVelocity = m_right.calculate(wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("PIDPower", leftVelocity);
    double rawYAngle = m_drive.getYAngle();
    if (rawYAngle > 180)
    {
      rawYAngle -= 360;
    }
    double yAngle = m_gyroFilter.calculate(rawYAngle);
    SmartDashboard.putNumber("raw y angle", rawYAngle);
    SmartDashboard.putNumber("y angle", yAngle);
    if (rawYAngle > 2.5)
    {
      SmartDashboard.putBoolean("Finished", false);
      m_left.setSetpoint(SmartDashboard.getNumber("BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1));
      m_right.setSetpoint(SmartDashboard.getNumber("BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1));
      m_drive.tankDrive(leftVelocity, rightVelocity);
    }else if (rawYAngle < -2.5)
    {
      SmartDashboard.putBoolean("Finished", false);
      m_left.setSetpoint(-(SmartDashboard.getNumber("BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1)));
      m_right.setSetpoint(-(SmartDashboard.getNumber("BSetpoint",SPEED_M_S)/(((30-Math.abs(yAngle))/30)*4+1)));
      m_drive.tankDrive(leftVelocity, rightVelocity);
    }else
    {
      SmartDashboard.putBoolean("Finished", true);
      // m_left.setSetpoint(0);
      // m_right.setSetpoint(0);
      m_drive.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
