// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.components.LimitedMotor;
import frc.robot.subsystems.DriveConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
 {
  private static final double POWER_LIMIT = 0.25;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PhotonCamera m_camera;
  private PhotonCamera m_cameraB;
  private ArrayList<PhotonCamera> cameraList;
  private final CANSparkMax m_rightFront = new LimitedMotor(DriveConstants.kRightMotor1Port, MotorType.kBrushless, POWER_LIMIT);
  private final CANSparkMax m_LeftFront = new LimitedMotor(DriveConstants.kLeftMotor1Port, MotorType.kBrushless, POWER_LIMIT);
  private final CANSparkMax m_RightBack = new LimitedMotor(DriveConstants.kRightMotor2Port, MotorType.kBrushless, POWER_LIMIT);
  private final CANSparkMax m_LeftBack = new LimitedMotor(DriveConstants.kLeftMotor2Port, MotorType.kBrushless, POWER_LIMIT);
  private final RelativeEncoder m_leftEncoder = m_LeftFront.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();
  private final XboxController m_driverController = new XboxController(1);
  // private final CANSparkMax m_intakeRight = new CANSparkMax(5, MotorType.kBrushed);
  // private final CANSparkMax m_intakeLeft = new CANSparkMax(6, MotorType.kBrushed);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_LeftFront, m_LeftBack);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_RightBack);
  public final double ksVolts = 0.16985;
  public final double kvVoltSecondsPerMeter = 0.12945;
  public final double kaVoltSecondsSquaredPerMeter = 0.025994;
  public final double kPDriveVel = 0.17824;
  public final double kTrackwidthMeters = 0.55245;
  public  final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public final double kMaxSpeedMetersPerSecond = 2.5;
  public final double kRamseteB = 2;
  public final double kRamseteZeta = 0.7;


 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_LeftBack.restoreFactoryDefaults();
    m_LeftFront.restoreFactoryDefaults();
    m_RightBack.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();

    m_RightBack.setInverted(true);
    m_rightFront.setInverted(true);

    m_LeftBack.setIdleMode(IdleMode.kBrake);
    m_LeftFront.setIdleMode(IdleMode.kBrake);
    m_RightBack.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);

    m_robotContainer = new RobotContainer(m_left, m_right, m_leftEncoder, m_rightEncoder);
    // m_camera = new PhotonCamera("Camera_A");
    // m_cameraB = new  PhotonCamera("Camera_B");
    // cameraList = new ArrayList<PhotonCamera>();
    // cameraList.add(m_camera);
    // cameraList.add(m_cameraB);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // var result = m_camera.getLatestResult();
    // boolean hasTargets = result.hasTargets();
    // List<PhotonTrackedTarget> targets = result.getTargets();
    // PhotonTrackedTarget target = result.getBestTarget();
    //  for (PhotonCamera camera: cameraList)
    // {
    //    SmartDashboard.putString("active", "yes");
    //    var result = camera.getLatestResult();
    //    double latency = result.getLatencyMillis();
    //    List<PhotonTrackedTarget> targets = result.getTargets();
    //    System.out.print(camera.getName() + "(" + latency + ")" +  ": ");
    //    System.out.println(targets.size() + "targets found");
    //    for (PhotonTrackedTarget target: targets)
    //    {
    //      double yaw = target.getYaw();
    //      double pitch = target.getPitch();
    //      double area = target.getArea();
    //     double skew = target.getSkew();
    //     double ID = target.getFiducialId();
    //      SmartDashboard.putNumber("AprilTag ID: " ,  ID);
    //      SmartDashboard.putNumber("Yaw", yaw);
    //      SmartDashboard.putNumber("Pitch", pitch);
    //      SmartDashboard.putNumber("Area" , area);
    //     SmartDashboard.putNumber("skew", skew);
    //      Transform3d pose = target.getBestCameraToTarget();
    //      double x = pose.getX();
    //      double y = pose.getY();
    //      double z = pose.getZ();
    //      SmartDashboard.putNumber("Pose X Value", x);
    //     SmartDashboard.putNumber("Pose Y Value", y);
    //     SmartDashboard.putNumber("Pose Z value", z);
    //      if (ID >= -1)
    //     {
    //     if (ID == 1)
    //     {
    //      m_intakeLeft.set(1.0);
    //      m_intakeRight.set(-1.0);
        
    //      }


    //   }
    //  }
    //  }
    // if (target != null)
    // {
      // double yaw = target.getYaw();
      // double pitch = target.getPitch();
      // double area = target.getArea();
      // double skew = target.getSkew();
    //   System.out.println("" + yaw + ", " + pitch + ", " + area + ", " + skew);

    //   SmartDashboard.putNumber("Yaw", yaw);
    //   SmartDashboard.putNumber("Pitch", pitch);
    //   SmartDashboard.putNumber("Area", area);
    //   SmartDashboard.putNumber("skew", skew);

    // }
    // else
    // {
    //   System.out.println("No targets locked.");

    //   SmartDashboard.putNumber("Yaw", 0);
    //   SmartDashboard.putNumber("Pitch", 0);
    //   SmartDashboard.putNumber("Area", 0);
    //   SmartDashboard.putNumber("skew", 0); 
    // }
    SmartDashboard.putNumber("right motor speed:", m_rightFront.get());
    CommandScheduler.getInstance().run();
    // System.out.println(targets.size() + "targets found: ");
    // if (targets.size() > 0)
    // {
    //   for (PhotonTrackedTarget targety: targets)
    //   {
    //     double yaw = targety.getYaw();
    //     double pitch = targety.getPitch();
    //     double area = targety.getArea();
    //     double skew = targety.getSkew();
    //     int id = targety.getFiducialId();
    //     System.out.println("ID " + id +  ": " + yaw + ", " + pitch + ", " + area + ", " + skew);
    //   }
    // }

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
   {
    //m_robotContainer.arcadeDrive(m_driverController.getLeftX()/1.5, m_driverController.getLeftY()/1.5);
    //m_myRobot.arcadeDrive(-m_driverController.getLeftY()/1.5, -m_driverController.getLeftX()/1.5);
    if(m_driverController.getLeftY()!=1 && m_driverController.getLeftY() != -1)
      if (m_driverController.getLeftY()>0){
        m_robotContainer.arcadeDrive(-m_driverController.getLeftY()/2,m_driverController.getRightX()/2);
      }
      else{
        m_robotContainer.arcadeDrive(-m_driverController.getLeftY()/2,-m_driverController.getRightX()/2);
      }
    else
      if (m_driverController.getLeftY()>0){
        m_robotContainer.arcadeDrive(-m_driverController.getLeftY()/2,m_driverController.getRightX()/1.5);
      }
      else{
        m_robotContainer.arcadeDrive(-m_driverController.getLeftY()/2,-m_driverController.getRightX()/1.5);
      }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
