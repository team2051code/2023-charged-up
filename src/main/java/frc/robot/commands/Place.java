// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class Place extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
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
    m_arm = subsystem;
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
    m_arm.setOveride(true);
    SmartDashboard.putBoolean("Place", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theta = 0;
    if(frontSide)//when placing on front side
      if(isCube)//when holding cube on front
      {
        if(level == DriveToScore.Level.BOTTOM) //scores on the bottom level
        {
          theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0)); //finds angle needed to score imagining
          //0 is vertically down
          m_arm.setArmPivotSetpoint(theta);//sets arm to go to calced angle
          m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);//calculates
          //and sets the arm to the right length to score
          m_arm.setGripperPivotSetpoint(90+theta);//sets the gripper parallel to the arm
          m_arm.toggleGripper();//opens gripper to let out cube
        }
        else if(level == DriveToScore.Level.MIDDLE) //scores on the middle level
        {
          theta = Units.radiansToDegrees(Math.atan(6/(16+distance+14.25)));//finds angle need to score imagining 0 is
          //horizontally front
          m_arm.setArmPivotSetpoint(theta+90);//sets arm to go to calced angle
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(6,2)+Math.pow((16+distance+14.25), 2)))-28));//calculates
          //and sets the arm to the right length to score
          m_arm.setGripperPivotSetpoint(180+theta);//sets the gripper parallel to the arm
          m_arm.toggleGripper();//opens gripper to let out
        }
        else if(level == DriveToScore.Level.TOP) //scores on the top level
        {
          theta = Units.radiansToDegrees(Math.atan(18/(16+distance+24+7.75)));
          m_arm.setArmPivotSetpoint(theta+90);
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(18,2)+Math.pow((16+distance+24+7.75), 2)))-28));
          m_arm.setGripperPivotSetpoint(180+theta);
        }
      }else//when holding cone on front
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0));
          m_arm.setArmPivotSetpoint(theta);
          m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
          m_arm.setGripperPivotSetpoint(theta-90);
          m_arm.toggleGripper();
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {
          theta = Units.radiansToDegrees(Math.atan(16/(16+distance+12+10.75)));
          m_arm.setArmPivotSetpoint(theta+90);
          m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(16,2)+Math.pow((16+distance+12+10.75), 2))-28));
          m_arm.setGripperPivotSetpoint(180+theta);
          m_arm.toggleGripper();
        }
        else if(level == DriveToScore.Level.TOP)
        {
          theta = Units.radiansToDegrees(Math.atan(28/(16+distance+36+3.75)));
          m_arm.setArmPivotSetpoint(theta+90);
          m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(28,2)+Math.pow((16+distance+36+3.75), 2))-28));
          m_arm.setGripperPivotSetpoint(180+theta);
          m_arm.toggleGripper();
        }
      }
    else //when faceing back
    if(isCube)//when holding cube on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        theta = 360-Units.radiansToDegrees(Math.atan((16+distance)/24.0));
        m_arm.setArmPivotSetpoint(theta);
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
        m_arm.setGripperPivotSetpoint(theta-90);
        m_arm.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.MIDDLE)//cube back middle
      {
          theta = 90 -Units.radiansToDegrees(Math.atan(6/(16+distance+14.25)));
          m_arm.setArmPivotSetpoint(theta+180);
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(6,2)+Math.pow((16+distance+14.25), 2)))-28));
        m_arm.setGripperPivotSetpoint(theta+90);
        m_arm.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.TOP)
      {
        theta = 360-Units.radiansToDegrees(Math.atan(18/(16+distance+24+7.75)));
        m_arm.setArmPivotSetpoint(theta-90);
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(18,2)+Math.pow((16+distance+24+7.75), 2))-28));
        m_arm.setGripperPivotSetpoint(theta-180);
        finished = true;
      }
    }else//when holding cone on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0))+180;
        m_arm.setArmPivotSetpoint(theta);
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
        m_arm.setGripperPivotSetpoint(theta-90);
        m_arm.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.MIDDLE)
      {
        theta = Units.radiansToDegrees(Math.atan(16/(16+distance+12+10.75)))+180;
        m_arm.setArmPivotSetpoint(theta+90);
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(16,2)+Math.pow((16+distance+12+10.75), 2))-28));
        m_arm.setGripperPivotSetpoint(180+theta);
        m_arm.toggleGripper();
        finished = true;
      }
      else if(level == DriveToScore.Level.TOP)
      {
        theta = Units.radiansToDegrees(Math.atan(28/(16+distance+36+3.75)))+180;
        m_arm.setArmPivotSetpoint(theta+90);
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(28,2)+Math.pow((16+distance+36+3.75), 2))-28));
        m_arm.setGripperPivotSetpoint(180+theta);
        m_arm.toggleGripper();
      }
    }
    finished = true;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setBreak(false);
    m_arm.setOveride(false);
    // Retract retract = new Retract(m_subsystem);
    // CommandScheduler.getInstance().schedule(retract);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
