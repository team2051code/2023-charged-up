// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.util.Units;
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
  private final double distance; //distance from front of nodes

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Place(ArmSubsystem subsystem, DriveToScore.Level level,boolean isCube,boolean frontSide, double distance) {
    m_subsystem = subsystem;
    this.level = level;
    this.isCube = isCube;
    this.frontSide = frontSide;
    this.distance = distance;
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
    double theta = 0;
    if(frontSide)
      if(isCube)//when holding cube on front
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0));
          m_subsystem.setArmPivotSetpoint(theta);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
          m_subsystem.setGripperPivotSetpoint(theta-90);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {
          theta = Units.radiansToDegrees(Math.atan(6/(16+distance)));
          m_subsystem.setArmPivotSetpoint(theta+90);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(6,2)+Math.pow((16+distance), 2))-28));
          m_subsystem.setGripperPivotSetpoint(180+theta);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.TOP)
        {
          theta = Units.radiansToDegrees(Math.atan(18/(16+distance)));
          m_subsystem.setArmPivotSetpoint(theta+90);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(18,2)+Math.pow((16+distance), 2))-28));
          m_subsystem.setGripperPivotSetpoint(180+theta);
        }
      }else//when holding cone on front
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0));
          m_subsystem.setArmPivotSetpoint(theta);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
          m_subsystem.setGripperPivotSetpoint(theta-90);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {
          theta = Units.radiansToDegrees(Math.atan(16/(16+distance)));
          m_subsystem.setArmPivotSetpoint(theta+90);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(16,2)+Math.pow((16+distance), 2))-28));
          m_subsystem.setGripperPivotSetpoint(180+theta);
          m_subsystem.toggleGripper();
        }
        else if(level == DriveToScore.Level.TOP)
        {
          theta = Units.radiansToDegrees(Math.atan(28/(16+distance)));
          m_subsystem.setArmPivotSetpoint(theta+90);
          m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(28,2)+Math.pow((16+distance), 2))-28));
          m_subsystem.setGripperPivotSetpoint(180+theta);
          m_subsystem.toggleGripper();
        }
      }
    else //when faceing back
    if(isCube)//when holding cube on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0))+180;
        m_subsystem.setArmPivotSetpoint(theta);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
        m_subsystem.setGripperPivotSetpoint(theta-90);
        m_subsystem.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.MIDDLE)
      {
        theta = Units.radiansToDegrees(Math.atan(6/(16+distance)))+180;
        m_subsystem.setArmPivotSetpoint(theta+90);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(6,2)+Math.pow((16+distance), 2))-28));
        m_subsystem.setGripperPivotSetpoint(180+theta);
        m_subsystem.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.TOP)
      {
        theta = Units.radiansToDegrees(Math.atan(18/(16+distance)))+180;
        m_subsystem.setArmPivotSetpoint(theta+90);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(18,2)+Math.pow((16+distance), 2))-28));
        m_subsystem.setGripperPivotSetpoint(180+theta);
        finished = true;
      }
    }else//when holding cone on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0))+180;
        m_subsystem.setArmPivotSetpoint(theta);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
        m_subsystem.setGripperPivotSetpoint(theta-90);
        m_subsystem.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.MIDDLE)
      {
        theta = Units.radiansToDegrees(Math.atan(16/(16+distance)))+180;
        m_subsystem.setArmPivotSetpoint(theta+90);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(16,2)+Math.pow((16+distance), 2))-28));
        m_subsystem.setGripperPivotSetpoint(180+theta);
        m_subsystem.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.TOP)
      {
        theta = Units.radiansToDegrees(Math.atan(28/(16+distance)))+180;
        m_subsystem.setArmPivotSetpoint(theta+90);
        m_subsystem.setExtenderSetpoint(Math.sqrt((Math.pow(28,2)+Math.pow((16+distance), 2))-28));
        m_subsystem.setGripperPivotSetpoint(180+theta);
        m_subsystem.toggleGripper();
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
