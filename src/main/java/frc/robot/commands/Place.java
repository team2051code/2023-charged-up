// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
  private static final double TIME_OVERRIDE_SECS = 3.0;
  //private boolean finished;
  private final double distance; //distance from front of nodes
  private Timer m_timer = new Timer();
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
    //finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setOveride(true);
    m_arm.setBreak(true);
    SmartDashboard.putBoolean("Place", true);
    double theta = 0;
    m_timer.reset();
    m_timer.start();
    if(frontSide)//when placing on front side
      if(isCube)//when holding cube on front
      {
        if(level == DriveToScore.Level.BOTTOM) //scores on the bottom level
        {
          //theta = Units.radiansToDegrees(Math.atan((16+30)/24.0)); //finds angle needed to score imagining
          //0 is vertically down
          m_arm.setExtenderSetpoint(21.7);//calculates
          //and sets the arm to the right length to score
          
          //m_arm.setArmPivotSetpoint(67.5);//sets arm to go to calced angle
          MoveArm.moveArm(m_arm, 67.5);
          m_arm.setGripperPivotSetpoint(217);//sets the gripper parallel to the arm
          //m_arm.toggleGripper();//opens gripper to let out cube
        }
        else if(level == DriveToScore.Level.MIDDLE) //scores on the middle level
        {
          theta = Units.radiansToDegrees(Math.atan(2/(16+distance+14.25)));//finds angle need to score imagining 0 is
          //horizontally front
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(2,2)+Math.pow((16+distance+14.25), 2)))-28));//calculates
          //and sets the arm to the right length to score
          m_arm.setArmPivotSetpoint(theta+90);//sets arm to go to calced angle
          m_arm.setGripperPivotSetpoint(180+theta);//sets the gripper parallel to the arm
          //m_arm.toggleGripper();//opens gripper to let out
        }
        else if(level == DriveToScore.Level.TOP) //scores on the top level
        {
          theta = Units.radiansToDegrees(Math.atan(14/(16+distance+24+7.75)));
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(14,2)+Math.pow((16+distance+24+7.75), 2)))-28));
          
          m_arm.setArmPivotSetpoint(theta+90);
          m_arm.setGripperPivotSetpoint(180+theta);
        }
      }else//when holding cone on front
      {
        if(level == DriveToScore.Level.BOTTOM)
        {
          //theta = Units.radiansToDegrees(Math.atan((16+30)/24.0));
          m_arm.setExtenderSetpoint(12.1);
          
         //m_arm.setArmPivotSetpoint(67.5);
         MoveArm.moveArm(m_arm, 60);
          m_arm.setGripperPivotSetpoint(201.3);
          //m_arm.toggleGripper();
        }
        else if(level == DriveToScore.Level.MIDDLE)
        {
          //theta = Units.radiansToDegrees(Math.atan(12/(16+distance+12+10.75)));
          m_arm.setExtenderSetpoint(24.3);
          
          //m_arm.setArmPivotSetpoint(121.8);
          MoveArm.moveArm(m_arm, 121.8);
          m_arm.setGripperPivotSetpoint(154.2);
          //m_arm.toggleGripper();
        }
        else if(level == DriveToScore.Level.TOP)
        {
          //theta = Units.radiansToDegrees(Math.atan(24/(16+distance+36+3.75)));
          m_arm.setExtenderSetpoint(39);
          
          //m_arm.setArmPivotSetpoint(128);
          MoveArm.moveArm(m_arm, 128);
          m_arm.setGripperPivotSetpoint(164);
          //m_arm.toggleGripper();
        }
      }
    else //when faceing back
    if(isCube)//when holding cube on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        //theta = 360-Units.radiansToDegrees(Math.atan((16+30)/24.0));
        m_arm.setExtenderSetpoint(21.7);
        m_arm.setArmPivotSetpoint(360-67.5);
        m_arm.setGripperPivotSetpoint(360-217);
        //m_arm.toggleGripper();
      }
      else if(level == DriveToScore.Level.MIDDLE)//cube back middle
      {
          theta = 90 -Units.radiansToDegrees(Math.atan(2/(16+distance+14.25)));
          m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(2,2)+Math.pow((16+distance+14.25), 2)))-28));
          
          m_arm.setArmPivotSetpoint(theta+180);
          m_arm.setGripperPivotSetpoint(theta+90);
        //m_arm.toggleGripper();
      }
      else if(level == DriveToScore.Level.TOP)
      {
        theta = 360-Units.radiansToDegrees(Math.atan(14/(16+distance+24+7.75)));
        m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(14,2)+Math.pow((16+distance+24+7.75), 2))-28));
        m_arm.setArmPivotSetpoint(theta-90);
        m_arm.setGripperPivotSetpoint(theta-180);
      }
    }else//when holding cone on back
    {
      if(level == DriveToScore.Level.BOTTOM)
      {
        //theta = 360-Units.radiansToDegrees(Math.atan((16+30)/24.0));
        m_arm.setExtenderSetpoint(12.1);
       // m_arm.setArmPivotSetpoint(360-67.5);
       MoveArm.moveArm(m_arm, 360-60); 
       m_arm.setGripperPivotSetpoint(360-201.3);
        //m_arm.toggleGripper();
      }
      else if(level == DriveToScore.Level.MIDDLE)
      {
        //theta = Units.radiansToDegrees(Math.atan(12/(16+distance+12+10.75)))+180;
        m_arm.setExtenderSetpoint(24.3);
       // m_arm.setArmPivotSetpoint(360-121.8);
        MoveArm.moveArm(m_arm, 360-121.8);
        m_arm.setGripperPivotSetpoint(260-154.2);
        //m_arm.toggleGripper();
      }
      else if(level == DriveToScore.Level.TOP)
      {
        //theta = Units.radiansToDegrees(Math.atan(24/(16+distance+36+3.75)))+180;
        m_arm.setExtenderSetpoint(39);
       // m_arm.setArmPivotSetpoint(360-128);
        MoveArm.moveArm(m_arm, 360-128);
        m_arm.setGripperPivotSetpoint(360-164);
        //m_arm.toggleGripper();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double theta = 0;
    // if(frontSide)//when placing on front side
    //   if(isCube)//when holding cube on front
    //   {
    //     if(level == DriveToScore.Level.BOTTOM) //scores on the bottom level
    //     {
    //       theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0)); //finds angle needed to score imagining
    //       //0 is vertically down
    //       m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);//calculates
    //       //and sets the arm to the right length to score
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta);//sets arm to go to calced angle
    //       m_arm.setGripperPivotSetpoint(90+theta);//sets the gripper parallel to the arm
    //       //m_arm.toggleGripper();//opens gripper to let out cube
    //     }
    //     else if(level == DriveToScore.Level.MIDDLE) //scores on the middle level
    //     {
    //       theta = Units.radiansToDegrees(Math.atan(2/(16+distance+14.25)));//finds angle need to score imagining 0 is
    //       //horizontally front
    //       m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(2,2)+Math.pow((16+distance+14.25), 2)))-28));//calculates
    //       //and sets the arm to the right length to score
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //        return;
    //       m_arm.setArmPivotSetpoint(theta+90);//sets arm to go to calced angle
    //       m_arm.setGripperPivotSetpoint(180+theta);//sets the gripper parallel to the arm
    //       //m_arm.toggleGripper();//opens gripper to let out
    //     }
    //     else if(level == DriveToScore.Level.TOP) //scores on the top level
    //     {
    //       theta = Units.radiansToDegrees(Math.atan(14/(16+distance+24+7.75)));
    //       m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(14,2)+Math.pow((16+distance+24+7.75), 2)))-28));
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta+90);
    //       m_arm.setGripperPivotSetpoint(180+theta);
    //     }
    //   }else//when holding cone on front
    //   {
    //     if(level == DriveToScore.Level.BOTTOM)
    //     {
    //       theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0));
    //       m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta);
    //       m_arm.setGripperPivotSetpoint(theta-90);
    //       //m_arm.toggleGripper();
    //     }
    //     else if(level == DriveToScore.Level.MIDDLE)
    //     {
    //       theta = Units.radiansToDegrees(Math.atan(12/(16+distance+12+10.75)));
    //       m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(12,2)+Math.pow((16+distance+12+10.75), 2))-28));
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta+90);
    //       m_arm.setGripperPivotSetpoint(180+theta);
    //       //m_arm.toggleGripper();
    //     }
    //     else if(level == DriveToScore.Level.TOP)
    //     {
    //       theta = Units.radiansToDegrees(Math.atan(24/(16+distance+36+3.75)));
    //       m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(24,2)+Math.pow((16+distance+36+3.75), 2))-28));
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta+90);
    //       m_arm.setGripperPivotSetpoint(180+theta);
    //       //m_arm.toggleGripper();
    //     }
    //   }
    // else //when faceing back
    // if(isCube)//when holding cube on back
    // {
    //   if(level == DriveToScore.Level.BOTTOM)
    //   {
    //     theta = 360-Units.radiansToDegrees(Math.atan((16+distance)/24.0));
    //     m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
    //     if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //       return;
    //     m_arm.setArmPivotSetpoint(theta);
    //     m_arm.setGripperPivotSetpoint(theta-90);
    //     //m_arm.toggleGripper();
    //   }
    //   else if(level == DriveToScore.Level.MIDDLE)//cube back middle
    //   {
    //       theta = 90 -Units.radiansToDegrees(Math.atan(2/(16+distance+14.25)));
    //       m_arm.setExtenderSetpoint(Math.abs(Math.sqrt((Math.pow(2,2)+Math.pow((16+distance+14.25), 2)))-28));
    //       if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //         return;
    //       m_arm.setArmPivotSetpoint(theta+180);
    //       m_arm.setGripperPivotSetpoint(theta+90);
    //     //m_arm.toggleGripper();
    //   }
    //   else if(level == DriveToScore.Level.TOP)
    //   {
    //     theta = 360-Units.radiansToDegrees(Math.atan(14/(16+distance+24+7.75)));
    //     m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(14,2)+Math.pow((16+distance+24+7.75), 2))-28));
    //     if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //       return;
    //     m_arm.setArmPivotSetpoint(theta-90);
    //     m_arm.setGripperPivotSetpoint(theta-180);
    //   }
    // }else//when holding cone on back
    // {
    //   if(level == DriveToScore.Level.BOTTOM)
    //   {
    //     theta = Units.radiansToDegrees(Math.atan((16+distance)/24.0))+180;
    //     m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((distance+16), 2)+Math.pow(24, 2)))-28);
    //     if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //       return;
    //     m_arm.setArmPivotSetpoint(theta);
    //     m_arm.setGripperPivotSetpoint(theta-90);
    //     //m_arm.toggleGripper();
    //   }
    //   else if(level == DriveToScore.Level.MIDDLE)
    //   {
    //     theta = Units.radiansToDegrees(Math.atan(12/(16+distance+12+10.75)))+180;
    //     m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(12,2)+Math.pow((16+distance+12+10.75), 2))-28));
    //     if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //       return;
    //     m_arm.setArmPivotSetpoint(theta+90);
    //     m_arm.setGripperPivotSetpoint(180+theta);
    //     //m_arm.toggleGripper();
    //   }
    //   else if(level == DriveToScore.Level.TOP)
    //   {
    //     theta = Units.radiansToDegrees(Math.atan(24/(16+distance+36+3.75)))+180;
    //     m_arm.setExtenderSetpoint(Math.sqrt((Math.pow(24,2)+Math.pow((16+distance+36+3.75), 2))-28));
    //     if(!(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1)) 
    //       return;
    //     m_arm.setArmPivotSetpoint(theta+90);
    //     m_arm.setGripperPivotSetpoint(180+theta);
    //     //m_arm.toggleGripper();
    //   }
    // }
    //finished = Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1;
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
    return (m_timer.get() > TIME_OVERRIDE_SECS) ||((Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1)&&(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1));
  }
}
