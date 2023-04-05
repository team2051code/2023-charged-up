// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class Grab extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private boolean m_frontSide;
  //private double m_distance;
  //private boolean finished;
  private static final double TIME_OVERRIDE_SECS = 3.0;
  private Timer m_timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Grab(ArmSubsystem subsystem, boolean frontSide) {
    m_arm = subsystem;
    m_frontSide = frontSide;
    //finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setOverride(true);
    m_arm.openBrake(true);
    //double theta = 0;
    m_timer.reset();
    m_timer.start();
    if(m_frontSide)//pick up from frontside
    {
      //m_arm.toggleGripper();
      //theta = Units.radiansToDegrees(Math.atan((37.25-24)/(16+m_distance)));
      //m_arm.setArmPivotSetpoint(134.5);
      MoveArm.moveArm(m_arm, 127.6);
      m_arm.setExtenderSetpoint(9.34);
      m_arm.setGripperPivotSetpoint(144.5);
      //m_arm.toggleGripper();
    }else//pick up from backside
    {
      //m_arm.toggleGripper();
      //theta = 360-Units.radiansToDegrees(Math.atan((37.25-24)/(16+m_distance)));
      //m_arm.setArmPivotSetpoint(360-134.5);
      MoveArm.moveArm(m_arm, 360-127.6);
      m_arm.setExtenderSetpoint(9.34);
      m_arm.setGripperPivotSetpoint(360-144.5);
      //m_arm.toggleGripper();
    }
    //finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setOverride(false);
    m_arm.openBrake(false);
    // Retract retract = new Retract(m_arm);
    // CommandScheduler.getInstance().schedule(retract);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > TIME_OVERRIDE_SECS) ||((Math.abs(m_arm.getArmPivotAbs()-m_arm.getArmPivotSetpoint())<1)&&(Math.abs(m_arm.getExtendorAbs()-m_arm.getExtenderSetpoint())<1));
  }
}
