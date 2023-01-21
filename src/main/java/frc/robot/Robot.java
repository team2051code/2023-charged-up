/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController controller;
  private static final int leftDeviceID = 1; 
  private static final int leftDeviceID2 = 2;
  private static final int rightDeviceID2 = 4;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private static final int rightDeviceID = 3;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;
  private MotorControllerGroup m_leftSide;
  private MotorControllerGroup m_rightSide;
  private RelativeEncoder m_leftMotorRE;
  private RelativeEncoder m_rightMotorRE;

  // @Override 
  // public void robotPeriodic(){

  //   double m_leftMotorREPosition = m_leftMotorRE.getPosition();
  //   double m_rightMotorREPosition = m_rightMotorRE.getPosition();


  //   SmartDashboard.putNumber("LeftMotor", m_leftMotorREPosition);
  //   SmartDashboard.putNumber("RightMotor", m_rightMotorREPosition);
  // }

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    controller = new XboxController(0);

    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushless);
    m_leftSide = new MotorControllerGroup(m_leftMotor, m_leftMotor2);
    //m_leftSide.setInverted(true);
    m_rightSide = new MotorControllerGroup(m_rightMotor, m_rightMotor2);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftSide, m_rightSide);

    //m_leftMotorRE = m_leftMotor.getEncoder();    
    //m_rightMotorRE = m_rightMotor.getEncoder();
    //m_leftMotorRE.setInverted(true);
    
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(controller.getLeftY(), controller.getRightY());
  }

  public void autonomousInit(){
    
  }

  

}