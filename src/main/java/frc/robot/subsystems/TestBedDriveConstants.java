package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class TestBedDriveConstants {
    public static final double ksVolts = 0.19702;
    public static final double kvVoltSecondsPerMeter = 2.4203;
    public static final double kaVoltSecondsSquaredPerMeter = 0.32792;
    public static final double kPDriveVel = 0.024649;
    public static final double kTrackwidthMeters = 0.55245;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 0.75; //set higher later
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;
    public static final Integer[] kLeftEncoderPorts = {1, 2};
    public static final Integer[] kRightEncoderPorts = {1, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;
    public static final double kGearRatio = 8.45;
    public static final double kWheelRadiusInches = 3;
    public static final double kLinearDistanceConversionFactor = 1 / kGearRatio *  2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches);
}
