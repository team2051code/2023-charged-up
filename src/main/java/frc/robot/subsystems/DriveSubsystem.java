package frc.robot.subsystems;

import java.security.Key;

import javax.crypto.spec.PBEParameterSpec;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.simulated.CANSparkMaxSimulated;
import frc.robot.subsystems.simulated.PoseEstimator;
import frc.robot.subsystems.simulated.SimpleSimulatedChassis;
import frc.robot.subsystems.simulated.SimulatedEncoder;
import frc.robot.subsystems.simulated.SimulatedGyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class DriveSubsystem extends SubsystemBase {

  public enum Gear{LOW, HIGH};
  // The motors on the left side of the drive.

  private final MotorControllerGroup m_leftMotors;
  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors;
  private final DifferentialDrive m_drive;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final Field2d m_field = new Field2d();
  private final Solenoid m_gearSolenoid;
  private SimpleSimulatedChassis simulatedChassis;
  private CANSparkMaxSimulated leftSimulated;
  private CANSparkMaxSimulated rightSimulated;
  private PoseEstimator poseEstimator;
  private Field2d simPose;
  // The robot's drive

  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(MotorControllerGroup left, MotorControllerGroup right, RelativeEncoder leftEncode, RelativeEncoder rightEncode, CANSparkMaxSimulated leftSimulatedMotor, CANSparkMaxSimulated rightSimulatedMotor) {
    if (RobotBase.isSimulation())
    {
      m_gyro = new SimulatedGyro();
      simulatedChassis = new SimpleSimulatedChassis((SimulatedGyro) m_gyro, (SimulatedEncoder) leftEncode, (SimulatedEncoder) rightEncode);
      poseEstimator = new PoseEstimator(this);
    }
    leftSimulated = leftSimulatedMotor;
    rightSimulated = rightSimulatedMotor;
    m_leftEncoder = leftEncode;
    m_rightEncoder = rightEncode;
    m_leftEncoder.setPositionConversionFactor(CompetitionDriveConstants.kLinearDistanceConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(CompetitionDriveConstants.kLinearDistanceConversionFactor / 60);
    m_rightEncoder.setPositionConversionFactor(CompetitionDriveConstants.kLinearDistanceConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(CompetitionDriveConstants.kLinearDistanceConversionFactor / 60);
    m_leftMotors = left;
    m_rightMotors = right;
    m_gearSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CompetitionDriveConstants.kGearShiftSolenoid);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    SmartDashboard.putData("Field: ", m_field);
    simPose = new Field2d();


    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(degToRad(m_gyro.getAngle())), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    if (RobotBase.isSimulation())
    {
      poseEstimator.initPose();
      simPose.setRobotPose(poseEstimator.getPose());
      SmartDashboard.putData("Simulation Field:", simPose);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        new Rotation2d(degToRad(m_gyro.getAngle())), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("left distance", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("right distance", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("angle", getHeading());
    SmartDashboard.putNumber("x accel", m_gyro.getAccelX());
    SmartDashboard.putNumber("y accel", m_gyro.getAccelY());
    SmartDashboard.putNumber("z accel", m_gyro.getAccelZ());
  }
  @Override
  public void simulationPeriodic()
  {
    simulatedChassis.updateSimulation(leftSimulated, rightSimulated);
    poseEstimator.updatePose();
    simPose.setRobotPose(poseEstimator.getPose());
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
   return m_odometry.getPoseMeters();
  }
  private double degToRad(double degrees)
  {
    return degrees * Math.PI / 180;
  }
  private double radToDeg(double radians)
  {
    return radians * 180 / Math.PI;
  }

  public double getAccelX(){
    return m_gyro.getAccelX();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The curjrent wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        new Rotation2d(degToRad(m_gyro.getAngle())), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
     m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
  
  

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return m_gyro.getRotation2d().getDegrees();
    return m_gyro.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
 
  public double getRightEncoder(){
    return m_rightEncoder.getPosition();
  }

  public double getLeftEncoder(){
    return m_leftEncoder.getPosition();
  }
  public double getAngle()
  {
    return m_gyro.getAngle();
  }
  public double getXAngle()
  {
    return m_gyro.getXFilteredAccelAngle();
  }
  public double getYAngle()
  {
    return m_gyro.getYFilteredAccelAngle();
  }
  public void toggleGear(){
    m_gearSolenoid.toggle();
  }

  public boolean getGear(){
    return m_gearSolenoid.get();
  }
}
  
  
  