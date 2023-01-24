// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PhotonCamera m_camera;
  private PhotonCamera m_cameraB;
  private ArrayList<PhotonCamera> cameraList;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_camera = new PhotonCamera("Camera_A");
    m_cameraB = new PhotonCamera("Camera_B");
    cameraList = new ArrayList<PhotonCamera>();
    cameraList.add(m_camera);
    cameraList.add(m_cameraB);


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
    for (PhotonCamera camera: cameraList)
    {
      SmartDashboard.putString("active", "yes");
      var result = camera.getLatestResult();
      double latency = result.getLatencyMillis();
      List<PhotonTrackedTarget> targets = result.getTargets();
      System.out.print(camera.getName() + "(" + latency + ")" +  ": ");
      System.out.println(targets.size() + "targets found");
      for (PhotonTrackedTarget target: targets)
      {
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        double ID = target.getFiducialId();
        SmartDashboard.putNumber("AprilTag ID: " ,  ID);
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Area" , area);
        SmartDashboard.putNumber("skew", skew);
        Transform3d pose = target.getBestCameraToTarget();
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        SmartDashboard.putNumber("Pose X Value", x);
        SmartDashboard.putNumber("Pose Y Value", y);
        SmartDashboard.putNumber("Pose Z value", z);
      }

      
    }
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
    // CommandScheduler.getInstance().run();
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
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
