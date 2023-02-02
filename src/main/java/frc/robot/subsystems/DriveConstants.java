package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConstants {
    public static final double ksVolts = 0.16985;
    public static final double kvVoltSecondsPerMeter = 0.12945;
    public static final double kaVoltSecondsSquaredPerMeter = 0.025994;
    public static final double kPDriveVel = 0.17824;
    public static final double kTrackwidthMeters = 0.55245;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 3;
    public static final Integer[] kLeftEncoderPorts = {2, 4};
    public static final Integer[] kRightEncoderPorts = {1, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;
    public static final double kEncoderDistancePerPulse = .25;
}
