// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveToScore;
import frc.robot.commands.Place;
import frc.robot.commands.DriveToScore.Level;
import frc.robot.commands.DriveToScore.Offset;
import frc.robot.components.LimitedMotor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompetitionDriveConstants;
import frc.robot.subsystems.ArmSubsystem.IntakeMode;
import frc.robot.subsystems.simulated.CANSparkMaxSimulated;
import frc.robot.subsystems.simulated.SimulatedEncoder;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.util.ElementScanner14;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double POWER_LIMIT = 1;
  private Command m_balanceCommand;
  private Command m_trajectory;
  private RobotContainer m_robotContainer;
  private PhotonCamera m_camera;
  private PhotonCamera m_cameraB;
  private ArrayList<PhotonCamera> cameraList;
  private CANSparkMax m_RightFront;
  private CANSparkMax m_LeftFront;
  private final CANSparkMax m_RightBack = new LimitedMotor(CompetitionDriveConstants.kRightMotor2Port,
      MotorType.kBrushless, POWER_LIMIT);
  private final CANSparkMax m_LeftBack = new LimitedMotor(CompetitionDriveConstants.kLeftMotor2Port,
      MotorType.kBrushless, POWER_LIMIT);
  
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private final XboxController m_ArmController = new XboxController(CompetitionDriveConstants.XboxArmPort);
  private final XboxController m_DriveController = new XboxController(CompetitionDriveConstants.XboxDrivePort);
  private final Joystick m_joystickController = new Joystick(CompetitionDriveConstants.joyStickPort);
  // private final CANSparkMax m_intakeRight = new CANSparkMax(5,
  // MotorType.kBrushed);
  // private final CANSparkMax m_intakeLeft = new CANSparkMax(6,
  // MotorType.kBrushed);
  private MotorControllerGroup m_left;
  private MotorControllerGroup m_right;
  public final double ksVolts = 0.16985;
  public final double kvVoltSecondsPerMeter = 0.12945;
  public final double kaVoltSecondsSquaredPerMeter = 0.025994;
  public final double kPDriveVel = 0.17824;
  public final double kTrackwidthMeters = 0.55245;
  public final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public final double kMaxSpeedMetersPerSecond = 2.5;
  public final double kRamseteB = 2;
  public final double kRamseteZeta = 0.7;
  public boolean useButtonBoard = true;
  public ArmSubsystem m_arm;
  private int m_lastBoardButtonValue;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    CANSparkMaxSimulated simulatedLeft = null;
    CANSparkMaxSimulated simulatedRight = null;
    if (RobotBase.isSimulation())
    {
      m_leftEncoder = new SimulatedEncoder();
      m_rightEncoder = new SimulatedEncoder();
      simulatedLeft = new CANSparkMaxSimulated(CompetitionDriveConstants.kLeftMotor1Port, MotorType.kBrushless);
      simulatedRight = new CANSparkMaxSimulated(CompetitionDriveConstants.kRightMotor1Port, MotorType.kBrushless);
      m_RightFront = simulatedRight;
      m_LeftFront = simulatedLeft;
    }
    else
    {
      m_RightFront = new LimitedMotor(CompetitionDriveConstants.kRightMotor1Port,
      MotorType.kBrushless, POWER_LIMIT);
      m_LeftFront = new LimitedMotor(CompetitionDriveConstants.kLeftMotor1Port,
      MotorType.kBrushless, POWER_LIMIT);
      m_leftEncoder = m_LeftFront.getEncoder();
      m_rightEncoder = m_RightFront.getEncoder();
    }
    m_left = new MotorControllerGroup(m_LeftFront, m_LeftBack);
    m_right = new MotorControllerGroup(m_RightFront, m_RightBack);
    SmartDashboard.putNumber("Distance", 0);
    SmartDashboard.putNumber("Dead Time", 0);
    SmartDashboard.putNumber("Setpoint", 0);
    SmartDashboard.putNumber("PVal", 0);
    SmartDashboard.putNumber("BPVal", 0);
    SmartDashboard.putNumber("BSetpoint", 0);

    SmartDashboard.putNumber("Gyro Override", 0);

    SmartDashboard.putNumber("boardButton", 0);

    m_LeftBack.restoreFactoryDefaults();
    m_LeftFront.restoreFactoryDefaults();
    m_RightBack.restoreFactoryDefaults();
    m_RightFront.restoreFactoryDefaults();

    m_LeftFront.setInverted(CompetitionDriveConstants.kLeftMotorsReversed);
    m_LeftBack.setInverted(CompetitionDriveConstants.kLeftMotorsReversed);
    m_RightBack.setInverted(CompetitionDriveConstants.kRightMotorsReversed);
    m_RightFront.setInverted(CompetitionDriveConstants.kRightMotorsReversed);

    m_LeftBack.setIdleMode(IdleMode.kBrake);
    m_LeftFront.setIdleMode(IdleMode.kBrake);
    m_RightBack.setIdleMode(IdleMode.kBrake);
    m_RightFront.setIdleMode(IdleMode.kBrake);

    m_robotContainer = new RobotContainer(m_left, m_right, m_leftEncoder, m_rightEncoder, simulatedLeft, simulatedRight);
    m_arm = m_robotContainer.getArmSubsystem();
    // m_camera = new PhotonCamera("Camera_A");
    // m_cameraB = new PhotonCamera("Camera_B");
    // cameraList = new ArrayList<PhotonCamera>();
    // cameraList.add(m_camera);
    // cameraList.add(m_cameraB);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    // var result = m_camera.getLatestResult();
    // boolean hasTargets = result.hasTargets();
    // List<PhotonTrackedTarget> targets = result.getTargets();
    // PhotonTrackedTarget target = result.getBestTarget();
    // for (PhotonCamera camera: cameraList)
    // {
    // SmartDashboard.putString("active", "yes");
    // var result = camera.getLatestResult();
    // double latency = result.getLatencyMillis();
    // List<PhotonTrackedTarget> targets = result.getTargets();
    // System.out.print(camera.getName() + "(" + latency + ")" + ": ");
    // System.out.println(targets.size() + "targets found");
    // for (PhotonTrackedTarget target: targets)
    // {
    // double yaw = target.getYaw();
    // double pitch = target.getPitch();
    // double area = target.getArea();
    // double skew = target.getSkew();
    // double ID = target.getFiducialId();
    // SmartDashboard.putNumber("AprilTag ID: " , ID);
    // SmartDashboard.putNumber("Yaw", yaw);
    // SmartDashboard.putNumber("Pitch", pitch);
    // SmartDashboard.putNumber("Area" , area);
    // SmartDashboard.putNumber("skew", skew);
    // Transform3d pose = target.getBestCameraToTarget();
    // double x = pose.getX();
    // double y = pose.getY();
    // double z = pose.getZ();
    // SmartDashboard.putNumber("Pose X Value", x);
    // SmartDashboard.putNumber("Pose Y Value", y);
    // SmartDashboard.putNumber("Pose Z value", z);
    // if (ID >= -1)
    // {
    // if (ID == 1)
    // {
    // m_intakeLeft.set(1.0);
    // m_intakeRight.set(-1.0);

    // }

    // }
    // }
    // }
    // if (target != null)
    // {
    // double yaw = target.getYaw();
    // double pitch = target.getPitch();
    // double area = target.getArea();
    // double skew = target.getSkew();
    // System.out.println("" + yaw + ", " + pitch + ", " + area + ", " + skew);

    // SmartDashboard.putNumber("Yaw", yaw);
    // SmartDashboard.putNumber("Pitch", pitch);
    // SmartDashboard.putNumber("Area", area);
    // SmartDashboard.putNumber("skew", skew);

    // }
    // else
    // {
    // System.out.println("No targets locked.");

    // SmartDashboard.putNumber("Yaw", 0);
    // SmartDashboard.putNumber("Pitch", 0);
    // SmartDashboard.putNumber("Area", 0);
    // SmartDashboard.putNumber("skew", 0);
    // }
    SmartDashboard.putNumber("right motor speed:", m_RightFront.get());
    SmartDashboard.putNumber("leftStick", m_DriveController.getLeftX());

    CommandScheduler.getInstance().run();
    // System.out.println(targets.size() + "targets found: ");
    // if (targets.size() > 0)
    // {
    // for (PhotonTrackedTarget targety: targets)
    // {
    // double yaw = targety.getYaw();
    // double pitch = targety.getPitch();
    // double area = targety.getArea();
    // double skew = targety.getSkew();
    // int id = targety.getFiducialId();
    // System.out.println("ID " + id + ": " + yaw + ", " + pitch + ", " + area + ",
    // " + skew);
    // }
    // }

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_trajectory = m_robotContainer.getTrajectories();
    m_balanceCommand = m_robotContainer.getBalanceCommand();
    var m_ramseteCommand = m_robotContainer.getRamseteCommand();
    // Command drive = m_robotContainer.getTrajectories();
    // schedule the autonomous command (example)
    // if(m_trajectory != null)
    // m_trajectory.schedule();
    if (m_ramseteCommand != null) {
      m_ramseteCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   SmartDashboard.putData("command", CommandScheduler.getInstance());
  }
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_balanceCommand != null) {
      m_balanceCommand.cancel();
    }
    if (m_trajectory != null) {
      m_trajectory.cancel();
    }
    m_robotContainer.resetOdometry();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    handleButtonBoard();

    // m_robotContainer.arcadeDrive(m_driverController.getLeftX()/1.5,
    // m_driverController.getLeftY()/1.5);
    // m_myRobot.arcadeDrive(-m_driverController.getLeftY()/1.5,
    // -m_driverController.getLeftX()/1.5);
    if (m_DriveController.getLeftY() > 0) {
      m_robotContainer.arcadeDrive(-m_DriveController.getLeftY(), m_DriveController.getRightX());
    } else {
      m_robotContainer.arcadeDrive(-m_DriveController.getLeftY(), -m_DriveController.getRightX());
    }
    //gripper pivot controller
    if (m_ArmController.getXButton()) {
      m_arm.incrementGripperPivotSetpoint(-20);
    }
    if (m_ArmController.getYButton()) {
      m_arm.incrementGripperPivotSetpoint(20);
    }
    //gripper rotate controller
    if (m_ArmController.getAButton()) {
      m_arm.incrementGripperRotatorSetpoint(180);
    }
    if (m_ArmController.getBButton()) {
      m_arm.incrementGripperRotatorSetpoint(-180);
    }
    if (m_ArmController.getRightStickButton()) {

    }
    if (m_ArmController.getLeftStickButton()) {

    }
    if (m_ArmController.getBackButton()) {

    }
    if (m_ArmController.getStartButton()) {

    }

    //intake controller
    if (m_ArmController.getRightBumper()) {
      m_arm.setIntakeMode(IntakeMode.BACKWARD);
    }
    else if (m_ArmController.getLeftBumper()) {
      m_arm.setIntakeMode(IntakeMode.FORWARD);
    }
    else
    {
      m_arm.setIntakeMode(IntakeMode.OFF);
    }

    m_arm.incrementArmPivotSetpoint(-m_ArmController.getLeftY() * 60);
    m_arm.incrementExtenderSetpoint(-m_ArmController.getRawAxis(3) * 10);

    //from bottom left: down-up left-right
    if (m_joystickController.getRawButtonPressed(3))
    {

    }
    if (m_joystickController.getRawButtonPressed(4))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(5))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(6))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(7))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(8))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(9))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(10))
    {
      
    }
    if (m_joystickController.getRawButtonPressed(11))
    {
      
    }

  }
  private void handleButtonBoard(){
    int boardButton = (int) SmartDashboard.getNumber("boardButton", 0);
    if (boardButton == m_lastBoardButtonValue){
      return;
    }
    m_lastBoardButtonValue = boardButton;

    if (boardButton == 1){
      scorePiece(Level.TOP, Offset.LEFT);
    }
    if (boardButton == 2){
      scorePiece(Level.TOP, Offset.CENTER);
    }
    if (boardButton == 3){
      scorePiece(Level.TOP, Offset.RIGHT);
    }
    if (boardButton == 4){
      scorePiece(Level.MIDDLE, Offset.LEFT);
    }
    if (boardButton == 5){
      scorePiece(Level.MIDDLE, Offset.CENTER);
    }
    if (boardButton == 6){
      scorePiece(Level.MIDDLE, Offset.RIGHT);
    }
    if (boardButton == 7){
      scorePiece(Level.BOTTOM, Offset.LEFT);
    }
    if (boardButton == 8){
      scorePiece(Level.BOTTOM, Offset.CENTER);
    }
    if (boardButton == 9){
      scorePiece(Level.BOTTOM, Offset.RIGHT);
    }
        
  }

  private void scorePiece(DriveToScore.Level level, DriveToScore.Offset offset){
    Place highPlace = new Place(m_arm, level, true, true, SmartDashboard.getNumber("Distance", 10)); //is a cube and facing forwards //p = 1 and d = 0.5 works well
    CommandScheduler.getInstance().schedule(highPlace);
  }

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}