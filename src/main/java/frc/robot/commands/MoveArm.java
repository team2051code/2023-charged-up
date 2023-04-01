// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.lang.annotation.Target;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

/** An example command that uses an example subsystem. */
public class MoveArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private double m_target;
  private TrapezoidProfile m_trajectory;
  private final Timer m_timer = new Timer();
  private double m_totalTrajectoryTime;
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(180, 360);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveArm(ArmSubsystem subsystem,double target) {
    m_arm = subsystem;
    m_target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setBreak(true);
    m_arm.setOveride(true);
    m_timer.start();
    m_timer.reset();
    m_trajectory = new TrapezoidProfile(CONSTRAINTS, 
    new TrapezoidProfile.State(m_target,0),
    new TrapezoidProfile.State(m_arm.getArmPivotAbs(), 0));
    m_totalTrajectoryTime = m_trajectory.totalTime()+0.25;
    SmartDashboard.putNumber("Commands/MoveArm/TrajectoryTime", m_totalTrajectoryTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var armSetpoint = m_trajectory.calculate(m_timer.get());
    m_arm.setArmPivotSetpoint(armSetpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setBreak(false);
    m_arm.setOveride(false);
    m_timer.stop();
  }

  public static Command moveArm(ArmSubsystem arm, double target){
    Command move = new MoveArm(arm, target);
    CommandScheduler.getInstance().schedule(move);
    return move;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get()>=m_totalTrajectoryTime ;
  }
}
