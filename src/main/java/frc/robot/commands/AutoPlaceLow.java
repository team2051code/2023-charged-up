// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmSubsystem.IntakeMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class AutoPlaceLow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private final DriveSubsystem m_drive;
  private static final double TIME_OVERRIDE_SECS = 1.0;
  private Timer m_timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoPlaceLow(ArmSubsystem subsystem,DriveSubsystem drive) {
    m_arm = subsystem;
    m_drive = drive;
    m_timer.reset();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setOverride(true);
    m_arm.openBrake(true);
    m_arm.setArmPivotSetpoint(270);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setIntakeMode(IntakeMode.FORWARD);;
    m_arm.setOverride(false);
    m_arm.openBrake(false);
    Command Drive = new DriveLinear(Units.feetToMeters(5), m_drive);
    CommandScheduler.getInstance().schedule(Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > TIME_OVERRIDE_SECS) || (Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1);
  }
}
