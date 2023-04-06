// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import frc.robot.commands.Delay;
import frc.robot.commands.DriveLinear;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class SeekBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private final double m_target;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SeekBalance(DriveSubsystem subsystem,double target) {
    m_drive = subsystem;
    m_target = target;
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Commands/SeekBalance", true);
    SmartDashboard.putNumber("Commands/SeekBalanceTarget", m_target);
    m_drive.setAutoDrive(true);
    m_drive.autoBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Commands/SeekBalanceEnc", m_drive.getLeftEncoder());
    if(m_drive.getLeftEncoder() > m_target+.25)
      m_drive.autoDrive(-0.3,-0.3);
    else if(m_drive.getLeftEncoder() < m_target-.25)
      m_drive.autoDrive(0.3, 0.3);
    else
      m_drive.autoDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new Delay(1));
    if(!interrupted) //&& !(Math.abs(m_target-m_drive.getLeftEncoder())<.25)
    {
      Command balAgain = new SequentialCommandGroup(
        new SeekBalance(m_drive, m_target)
        //new HoldPosition(m_drive)
      );
      CommandScheduler.getInstance().schedule(balAgain);
    }
    SmartDashboard.putBoolean("Commands/SeekBalance", false);
    m_drive.setAutoDrive(false);
    m_drive.autoBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_target-m_drive.getLeftEncoder())<.25;
  }
}
