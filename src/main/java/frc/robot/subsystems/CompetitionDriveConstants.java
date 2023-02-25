package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class CompetitionDriveConstants {
    public static final double ksVolts = 0.11995;
    public static final double kvVoltSecondsPerMeter = 5.4667;
    public static final double kaVoltSecondsSquaredPerMeter = 0.22833;
    public static final double kPDriveVel = 3.828E-06;
    public static final double kTrackwidthMeters = 0.56515;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 2; //set higher later
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;
    public static final int kGripperRotatorMotorPort = 7;
    public static final int kGripperPivotMotorPort = 10;
    public static final int kArmPivotMotorPort1 = 6;
    public static final int kArmPivotMotorPort2 = 8;
    public static final int kExtenderMotorPort = 5;
    public static final int kIntakeLeft = 9;
    public static final int kIntakeRight = 11;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;
    public static final boolean kLeftMotorsReversed = false;
    public static final boolean kRightMotorsReversed = true;
    public static final double kGearRatio = 20.95;
    public static final double kWheelRadiusInches = 3;
    public static final double kLinearDistanceConversionFactor = 1 / kGearRatio *  2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches);
    public static final int XboxArmPort = 0;
    public static final int XboxDrivePort = 1;
    public static final int joyStickPort = 2;
}
