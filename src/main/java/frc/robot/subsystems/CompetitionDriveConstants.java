package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class CompetitionDriveConstants {
    public static final double ksVolts = 0.12427;
    public static final double kvVoltSecondsPerMeter = 35.422;
    public static final double kaVoltSecondsSquaredPerMeter = 1.2637;
    public static final double kPDriveVel = 2.3773E-06;
    public static final double kTrackwidthMeters = 0.56515;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 0.75; //set higher later
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 12;
    public static final int kRightMotor2Port = 11;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;
    public static final double kGearRatio = 20.95;
    public static final double kWheelRadiusInches = 3;
    public static final double kLinearDistanceConversionFactor = 1 / kGearRatio *  2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches);
}
