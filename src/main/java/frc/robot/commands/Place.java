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
  // The level we want to place, from 0 (low) to 2 (high).
  private final DriveToScore.Level level;
  // Whether the claw is currently holding a cube.
  private final boolean isCube;
  // Whether the robot is moving with "front" as the bigger side.
  private final boolean frontSide;
  private boolean finished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Place(ArmSubsystem subsystem, DriveToScore.Level level,boolean isCube,boolean side) {
    m_subsystem = subsystem;
    this.level = level;
    this.isCube = isCube;
    frontSide = side;
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
    if(frontSide)
      if(isCube)
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          m_subsystem.setArmPivotSetpoint(45);
          m_subsystem.setExtenderSetpoint(40);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {
          m_subsystem.setArmPivotSetpoint(90);
          m_subsystem.setExtenderSetpoint(40);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.TOP)
        {
          m_subsystem.setArmPivotSetpoint(135);
          m_subsystem.setExtenderSetpoint(40);
          m_subsystem.toggleGripper();
        }
        else
        {
        }
      }else
      {
        if(level == DriveToScore.Level.BOTTOM)
        {

        }
        else if(level == DriveToScore.Level.MIDDLE)
        {

        }
        else if(level == DriveToScore.Level.TOP)
        {

        }
        else
        {
          finished = true;
        }
      }
    else 
      if(isCube)
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {

        }
        else if(level == DriveToScore.Level.TOP)
        {

        }
        else
        {
          finished = true;
        }
      }else
      {
        if(level == DriveToScore.Level.BOTTOM)
        {

        }
        else if(level == DriveToScore.Level.MIDDLE)
        {

        }
        else if(level == DriveToScore.Level.TOP)
        {

        }
        else
        {
          finished = true;
        }
      }
    finished = true;
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
