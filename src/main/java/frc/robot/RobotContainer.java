// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveLinear;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.Place;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompetitionDriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
  private DriveSubsystem m_robotDrive;
  private ArmSubsystem m_robotArm;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  public RobotContainer(MotorControllerGroup m_leftMotors, MotorControllerGroup m_rightMotors, RelativeEncoder m_leftEncoder, RelativeEncoder m_RightEncoder)
  {
    configureButtonBindings();
    m_robotDrive = new DriveSubsystem(m_leftMotors, m_rightMotors, m_leftEncoder, m_RightEncoder);
    m_robotArm = new ArmSubsystem();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void resetOdometry()
  {
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public Command getBalanceCommand() {
   
    DriveStraight item = new DriveStraight(m_robotDrive);
    Balance balancer = new Balance(m_robotDrive);
    //DriveLinear test = new DriveLinear(1, m_robotDrive);
    return item.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
  public void arcadeDrive(double speed, double rotation)
  {
    SmartDashboard.putNumber("left encoder", m_robotDrive.getLeftEncoder());
    SmartDashboard.putNumber("right encoder", m_robotDrive.getRightEncoder());
    m_robotDrive.arcadeDrive(speed, rotation);
  }
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_robotDrive.tankDrive(leftSpeed, rightSpeed);
  }

public ArmSubsystem getArmSubsystem(){
  return m_robotArm;
}

}
