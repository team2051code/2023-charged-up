// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class Grab extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private boolean m_frontSide;
  private double m_distance;
  private boolean finished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Grab(ArmSubsystem subsystem, boolean frontSide, double distance) {
    m_arm = subsystem;
    m_frontSide = frontSide;
    m_distance = distance;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setOveride(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theta = 0;
    if(m_frontSide)//pick up from frontside
    {
      m_arm.toggleGripper();
      theta = Units.radiansToDegrees(Math.atan((37.25-24)/(16+m_distance)));
      m_arm.setArmPivotSetpoint(theta+90);
      m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((37.25-24),2)+Math.pow((16+m_distance), 2))-28));
      m_arm.setGripperPivotSetpoint(180+theta);
      m_arm.toggleGripper();
    }else//pick up from backside
    {
      m_arm.toggleGripper();
      theta = 360-Units.radiansToDegrees(Math.atan((37.25-24)/(16+m_distance)));
      m_arm.setArmPivotSetpoint(theta-90);
      m_arm.setExtenderSetpoint(Math.sqrt((Math.pow((37.25-24),2)+Math.pow((16+m_distance), 2))-28));
      m_arm.setGripperPivotSetpoint(theta-180);
      m_arm.toggleGripper();
    }
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setOveride(false);
    m_arm.setBreak(false);
    // Retract retract = new Retract(m_arm);
    // CommandScheduler.getInstance().schedule(retract);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
