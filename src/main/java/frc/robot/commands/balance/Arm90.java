// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Arm90 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private static final double TIME_OVERRIDE_SECS = 3.0;
  private Timer m_timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Arm90(ArmSubsystem subsystem) {
    m_arm = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setOverride(true);
    m_arm.openBrake(true);
    m_arm.setArmPivotSetpoint(90);
    // Start a timer to hold the command to a few-second window
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_arm.setGripperPivotSetpoint(180);
    m_arm.openBrake(false);
    m_arm.setOverride(false);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // Command ends when reaching target or operational window expires.
    return (m_timer.get() > TIME_OVERRIDE_SECS) || 
    (Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1);
  }
}
