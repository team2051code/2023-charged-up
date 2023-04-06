// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import frc.robot.commands.DriveLinear;
import frc.robot.commands.DriveToScore.Offset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class OnRamp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private double m_offset;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OnRamp(DriveSubsystem subsystem) {
    m_drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Commands/OnRamp", true);
    m_drive.setAutoDrive(true);
    m_drive.autoBrake(true);
    if(DriverStation.getAlliance() == Alliance.Red)//if the field is red
      m_offset = 0.4;
    else
      m_offset = 0.4;
    SmartDashboard.putNumber("BalanceOffset", m_offset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.autoDrive(0.3, 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Commands/OnRamp", false);
    m_drive.setAutoDrive(false);
    m_drive.autoBrake(false);
    Command seekBalance = 
    new SequentialCommandGroup(
      new SeekBalance(m_drive, m_drive.getLeftEncoder()-m_offset),
      new HoldPosition(m_drive)
    );
    CommandScheduler.getInstance().schedule(seekBalance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getFilteredY() < 8;
  }
}
