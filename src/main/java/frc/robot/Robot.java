// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveLinear;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveToScore;
import frc.robot.commands.Grab;
import frc.robot.commands.Place;
import frc.robot.commands.Retract;
import frc.robot.commands.DriveToScore.Level;
import frc.robot.commands.DriveToScore.Offset;
import frc.robot.components.LimitedMotor;
import frc.robot.controls.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompetitionDriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.IntakeMode;
import frc.robot.subsystems.simulated.CANSparkMaxSimulated;
import frc.robot.subsystems.simulated.SimulatedEncoder;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
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
  //private final XboxController m_ArmController = new XboxController(CompetitionDriveConstants.XboxArmPort);
  private final Joystick m_ArmController = new Joystick(CompetitionDriveConstants.XboxArmPort);
  private final XboxController m_DriveController = new XboxController(CompetitionDriveConstants.XboxDrivePort);
  private final Joystick m_buttonPanel = new Joystick(CompetitionDriveConstants.joyStickPort);

  private static final int[] BUTTON_PANEL_MAP = {
    -1,-1,-1,-1,
   7, 4, 1,
   8, 5, 2,
   9, 3, 6
  };
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
  private boolean m_gripperPivotButtonPressed;
  private boolean m_gripperRotatorButtonPressed;
  private boolean m_autoBalanceButtonPressed;
  private boolean m_gearButtonPressed;
  private boolean m_intakeButtonPressed;
  private DriveSubsystem m_drive;
  private Command m_teleopAutoBalance = null;
  private TeleopDrive m_filteredDriveController;
  public boolean dropOffMode = true;

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
    m_gripperPivotButtonPressed = false;
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

    m_LeftBack.setSmartCurrentLimit(40);
    m_LeftFront.setSmartCurrentLimit(40);
    m_RightBack.setSmartCurrentLimit(40);
    m_RightFront.setSmartCurrentLimit(40);

    m_LeftBack.setIdleMode(IdleMode.kBrake);
    m_LeftFront.setIdleMode(IdleMode.kBrake);
    m_RightBack.setIdleMode(IdleMode.kBrake);
    m_RightFront.setIdleMode(IdleMode.kBrake);

    m_LeftBack.burnFlash();
    m_LeftFront.burnFlash();
    m_RightBack.burnFlash();
    m_RightFront.burnFlash();

    m_robotContainer = new RobotContainer(m_left, m_right, m_leftEncoder, m_rightEncoder, simulatedLeft, simulatedRight);
    m_arm = m_robotContainer.getArmSubsystem();
    m_drive = m_robotContainer.getDriveSubsystem();
    // m_camera = new PhotonCamera("Camera_A");
    // m_cameraB = new PhotonCamera("Camera_B");
    // cameraList = new ArrayList<PhotonCamera>();
    // cameraList.add(m_camera);
    // cameraList.add(m_cameraB);

    m_arm.resetEncoders();
    m_arm.setArmPivotSetpoint(180);
    m_arm.setExtenderSetpoint(3);
    m_arm.setGripperPivotSetpoint(180);

    var modebutton = new JoystickButton(m_ArmController, 11);
    modebutton.onTrue(Commands.runOnce(() -> {
      dropOffMode = !dropOffMode;
    }));
    
    SmartDashboard.putNumber("autoname", 2);

    m_filteredDriveController = new TeleopDrive(m_drive, m_DriveController);

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
    if(m_buttonPanel.getRawButton(6)){
      SmartDashboard.putNumber("autoname", 1);
    }
    if(m_buttonPanel.getRawButton(9)){
      SmartDashboard.putNumber("autoname", 2);
    }
    if(m_buttonPanel.getRawButton(11)){
      SmartDashboard.putNumber("autoname", 3);
    }
    SmartDashboard.putNumber("right motor speed:", m_RightFront.get());
    SmartDashboard.putNumber("leftStick", m_DriveController.getLeftX());
    SmartDashboard.putBoolean("drop off mode", dropOffMode);
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());


    
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
    Command autoprogram = null;
    // m_trajectory = m_robotContainer.getTrajectories();
    //m_balanceCommand = m_robotContainer.getBalanceCommand();
    autoprogram = m_robotContainer.getRamseteCommand();
    // Command drive = m_robotContainer.getTrajectories();
    // schedule the autonomous command (example)
    // if(m_trajectory != null)
    // m_trajectory.schedule();
    var autoname = SmartDashboard.getNumber("autoname", 2);

    System.out.println("Auto to be scheduled is "  + autoname);
    
    if (autoname == 1 /* driveforward */){
      System.out.println("Drive forward scheduled");
      autoprogram = new DriveLinear(Units.feetToMeters(3), m_drive);
    }    else if (autoname == 3 /* autobalance */){
      System.out.println("Autobalance scheduled");
      autoprogram = m_robotContainer.getBalanceCommand();
    }

    if (autoprogram != null) {
      CommandScheduler.getInstance().schedule(autoprogram);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   SmartDashboard.putData("command", CommandScheduler.getInstance());
  }
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
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

    m_filteredDriveController.update();

    SmartDashboard.putBoolean("Gear", m_drive.getGear());

      if(m_DriveController.getXButton() && !m_gearButtonPressed){
        m_drive.toggleGear();
      }

      m_gearButtonPressed = m_DriveController.getXButton();
    
    //gripper pivot controller
    // if (m_ArmController.getXButton()) {
    //   m_arm.incrementGripperPivotSetpoint(-20);
    // }
    // if (m_ArmController.getYButton()) {
    //   m_arm.incrementGripperPivotSetpoint(20);
    // }
    // gripper rotate controller
    
    // if (m_ArmController.getAButton()&&!pressed){
    //   m_arm.toggleGripper();
    // } 
    //  pressed = m_ArmController.getAButton();

    // if (m_ArmController.getXButton()){
    //   m_arm.setIntakeMode(IntakeMode.FORWARD);
    // }
    // else if (m_ArmController.getYButton()){
    //   m_arm.setIntakeMode(IntakeMode.BACKWARD);
    // }
    // else{
    //   m_arm.setIntakeMode(IntakeMode.OFF);
    // }

    // if (m_ArmController.getLeftBumper()){
    //   m_arm.incrementGripperPivotSetpoint(10);
    // }

    if (m_ArmController.getRawButton(2)&&!m_gripperPivotButtonPressed){
      m_arm.toggleGripper();
    } 
    m_gripperPivotButtonPressed = m_ArmController.getRawButton(2);


    // if(m_ArmController.getRawButton(7)&&!m_intakeButtonPressed){
    //    if(m_arm.getIntakeMode() !=IntakeMode.SLOW)
    //     m_arm.setIntakeMode(IntakeMode.SLOW);
    //   else
    //     m_arm.setIntakeMode(IntakeMode.OFF);
    // }else{
      if (m_ArmController.getRawButton(4)){
        m_arm.setIntakeMode(IntakeMode.FORWARD);
      }
      else if (m_ArmController.getRawButton(5)){
        m_arm.setIntakeMode(IntakeMode.BACKWARD);
      }
      else { //if(!m_arm.getIntakeMode().equals(IntakeMode.SLOW))
        m_arm.setIntakeMode(IntakeMode.SLOW);
      }
    //}

    m_intakeButtonPressed = m_ArmController.getRawButton(7);
    if(!m_arm.getOveride())
     m_arm.setGripperPivotSetpoint(-m_ArmController.getRawAxis(2)*45 + 180);
    if(m_ArmController.getRawButton(8)&&!m_autoBalanceButtonPressed){
      if(m_teleopAutoBalance == null){
        m_teleopAutoBalance = new Balance(m_drive);
        CommandScheduler.getInstance().schedule(m_teleopAutoBalance);
      }else{
        m_teleopAutoBalance.cancel();
        m_teleopAutoBalance = null;
      }

    }
    m_autoBalanceButtonPressed = m_ArmController.getRawButton(8);
    SmartDashboard.putBoolean("TeleopBalance", m_teleopAutoBalance == null);


    if(m_ArmController.getRawButton(6) && !m_gripperRotatorButtonPressed)
      m_arm.toggleGripperRotator();
    
    m_gripperRotatorButtonPressed = m_ArmController.getRawButton(6);
    // if (m_ArmController.getBButton()) {
    //   m_arm.toggleGripper();
    // }
    // if (m_ArmController.getRightStickButton()) {

    // }
    // if (m_ArmController.getLeftStickButton()) {

    // }
    // if (m_ArmController.getBackButton()) {

    // }
    // if (m_ArmController.getStartButton()) {

    // }

    //intake controlle


    // if(!m_arm.getOveride()){
    //   if(-m_ArmController.getLeftY()<0.25 && -m_ArmController.getLeftY()>-0.25)
    //     m_arm.setBreak(false);
    //   else{
    //     m_arm.setBreak(true);
    //     m_arm.incrementArmPivotSetpoint(-m_ArmController.getLeftY() * 50);
    //   }
    //   if(!(-m_ArmController.getRightY()<0.25 && -m_ArmController.getRightY()>-0.25))
    //     m_arm.incrementExtenderSetpoint(-m_ArmController.getRightY()*5);
    // }

    if(!m_arm.getOveride()){
      if(-m_ArmController.getRawAxis(1)<0.25 && -m_ArmController.getRawAxis(1)>-0.25)
        m_arm.setBreak(false);
      else{
        m_arm.setBreak(true);
        m_arm.incrementArmPivotSetpoint(-m_ArmController.getRawAxis(1) * 60);
      }
      if(m_ArmController.getRawButton(1)){
        m_arm.incrementExtenderSetpoint(15);
      }else if(m_ArmController.getRawButton(3))
        m_arm.incrementExtenderSetpoint(-15);
    }

    //from bottom left: down-up left-right
  }
  private void handleButtonBoard(){

    int physicalBoardButton = getPressedBoardButton();

    int boardButton = (int) SmartDashboard.getNumber("boardButton", 0);
    boardButton = boardButton > 0 ? boardButton:physicalBoardButton; 
    if (boardButton == m_lastBoardButtonValue){
      return;
    }
    m_lastBoardButtonValue = boardButton;

    System.out.println(boardButton);
    if (boardButton == 1){
      scorePiece(Level.TOP, Offset.RIGHT,true);
    }
    if (boardButton == 2){
      Retract retract = new Retract(m_arm);
      CommandScheduler.getInstance().schedule(retract);
    }
    if (boardButton == 3){
      scorePiece(Level.TOP, Offset.RIGHT,false);
    }
    if (boardButton == 4){
      scorePiece(Level.MIDDLE, Offset.CENTER,true);
    }
    if (boardButton == 5){
      grabPiece(false);
    }
    if (boardButton == 6){
      scorePiece(Level.MIDDLE, Offset.CENTER,false);
    }
    if (boardButton == 7){
      scorePiece(Level.BOTTOM, Offset.LEFT,true);
    }
    if (boardButton == 8){
      grabPiece(true);
    }
    if (boardButton == 9){
      scorePiece(Level.BOTTOM, Offset.LEFT,false);
    }
        
  }
  
  private void grabPiece(boolean frontSide){
    Grab grab = new Grab(m_arm, frontSide, SmartDashboard.getNumber("Distance", 10));
    CommandScheduler.getInstance().schedule(grab);
  }

  private int getPressedBoardButton() {
    int buttonPressed = 0;
    for(int i = 1; i<13; i++){
      if(m_buttonPanel.getRawButton(i)){
        buttonPressed = BUTTON_PANEL_MAP[i];
      } 
    }
    return buttonPressed;
  }

  private void scorePiece(DriveToScore.Level level, DriveToScore.Offset offset,boolean frontSide){
    Place highPlace = new Place(m_arm, level, true, frontSide, SmartDashboard.getNumber("Distance", 10)); //is a cube and facing forwards //p = 1 and d = 0.5 works well
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