// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveStraight extends CommandBase {
  public static final double SPEED_M_S = 1;
  public static final double kPDriveVal = .4;
  public static final double kIDriveVal = .6;
  public static final double kDDriveVal = .03;
  public static final double kOnRampCrosser = 2;
  public static final double kPivotCrosser = 1;
//.4 .6 .03
  private DriveSubsystem m_drive;
  private PIDController m_left = new PIDController(kPDriveVal, kIDriveVal,kDDriveVal);
  private PIDController m_right = new PIDController(kPDriveVal, kIDriveVal, kDDriveVal);
  private LinearFilter m_gyroFilter = LinearFilter.movingAverage(10);
  private double m_xAccel;
  private Autostate m_autostate = Autostate.OFFRAMP;

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
    m_autostate = Autostate.OFFRAMP;
    m_left.reset();
    m_right.reset();
    m_left.setSetpoint(SPEED_M_S / 2);
    m_right.setSetpoint(SPEED_M_S / 2);
    m_drive.resetEncoders();
    m_drive.zeroHeading();
    m_gyroFilter.reset();
    m_xAccel = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var wheelSpeeds = m_drive.getWheelSpeeds();
    var leftPower = m_left.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightPower = m_right.calculate(wheelSpeeds.rightMetersPerSecond);
    System.out.println("Speeds: " + wheelSpeeds.leftMetersPerSecond + ", " + wheelSpeeds.rightMetersPerSecond);
    System.out.println("Power: " + leftPower + ", "+ rightPower);
    m_drive.tankDrive(leftPower, rightPower);
    m_xAccel = m_gyroFilter.calculate(m_drive.getAccelX());
    SmartDashboard.putNumber("Smooth X accel: ", m_xAccel);
    SmartDashboard.putString("Autostate: ", m_autostate.toString());

    SmartDashboard.putString("testAud", m_autostate.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(m_autostate){
      case OFFRAMP: 
        if(m_xAccel > kOnRampCrosser)
          m_autostate = Autostate.ONRAMP;
        break;
      case ONRAMP:
        if(m_xAccel<kPivotCrosser)
          m_autostate = Autostate.PIVOT;
        break;
      case PIVOT:
        return true;
    }

    return false;
  }
}
