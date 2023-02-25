// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Place extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private final int level;
  private final boolean isCube;
  private boolean finished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Place(ArmSubsystem subsystem,int level,boolean isCube) {
    m_subsystem = subsystem;
    this.level = level;
    this.isCube = isCube;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isCube)
    {
      if(level==0)
      {
        
      }
      else if(level == 1)
      {

      }
      else if(level == 2)
      {

      }
      else
      {
        finished = true;
      }
    }else
    {
      if(level==0)
      {

      }
      else if(level == 1)
      {

      }
      else if(level == 2)
      {

      }
      else
      {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
