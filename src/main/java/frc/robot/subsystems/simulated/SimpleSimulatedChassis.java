package frc.robot.subsystems.simulated;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.SimVisionTarget;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;

/**
 * A simulation of the robot's chassis, converting input drive voltages to
 * changes to a robot pose
 */
public class SimpleSimulatedChassis {
    private SimulatedGyro m_gyro;
    private SimulatedEncoder m_leftEncoder;
    private SimulatedEncoder m_rightEncoder;
    private double m_leftEncoderValue = 0;
    private double m_rightEncoderValue = 0;
    private double m_rotation = 0;
    private double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    private double camPitch = 0; // degrees
    private double camHeightOffGround = .25; // meters
    private double maxLEDRange = 20; // meters
    private int camResolutionWidth = 640; // pixels
    private int camResolutionHeight = 480; // pixels
    private double minTargetArea = 10; // square pixels
    SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    camDiagFOV,
                    new Transform3d(
                            new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);
    private double targetWidth = .1524;
    private double targetHeight = .1524;
    private double targetXPos = 6;
    private double targetYPos = 6;
    Pose3d farTargetPose =
    new Pose3d(
            new Translation3d(targetXPos, targetYPos, 61.913),
            new Rotation3d(0.0, 0.0, 0.0));
    private SimVisionTarget target = new SimVisionTarget(farTargetPose, targetWidth, targetHeight, camResolutionHeight);


    /**
     * Speed at which robot drives forward in meters per second at max velocity
     */
    private static final double FORWARD_SPEED_MS = 1.0;

    /**
     * Number of encoder counts per full wheel revolution
     */
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1000.0;

    /**
     * Circumference of wheel in meters
     */
    private static final double WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI;

    /**
     * Ticks per second at full speed
     */
    private static final double TICKS_PER_SECOND = FORWARD_SPEED_MS / WHEEL_CIRCUMFERENCE_METERS * ENCODER_TICKS_PER_REVOLUTION;

    /**
     * Rotations per second at full forward + reverse
     * 
     * We could probably do some trigonometry to estimate this by considering how far
     * one wheel or the other turns and conclude what the rotation must be as a result,
     * but easier to make up a number
     */
    private static final double ROTATIONS_PER_SECOND = 2.0;

    private double m_lastUpdateTime;

    public SimpleSimulatedChassis (SimulatedGyro gyro, SimulatedEncoder leftEncoder, SimulatedEncoder rightEncoder) {
        m_gyro = gyro;
        m_leftEncoder = leftEncoder;
        m_rightEncoder = rightEncoder;
        m_lastUpdateTime = Timer.getFPGATimestamp();
    }

    /**
     * Update the pose state based on the current speed controller outputs and time elapsed
     * @param leftMotor Output to left motor
     * @param rightMotor Output to right motor
     */
    public void updateSimulation(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        /* Get the power commands sent to the right and left motors to drive the robot forward.
         * On the ROMI chassis, the right motor is flipped (i.e. positive power values drive the
         * robot backward), so invert that input.
         */
        double leftPower = leftMotor.get();
        double rightPower = rightMotor.get();


        /* Compute a delta and update m_lastUpdateTime. The delta makes the simulation
         * realtime-independent (i.e. if the robot runs slower or faster, we should
         * get close to the same answers).
         */
        double newTimestamp = Timer.getFPGATimestamp();
        double delta = newTimestamp - m_lastUpdateTime;
        m_lastUpdateTime = newTimestamp;

        /* Very simplified physics model used below:
         * We assume motion is completely described as 'linear' and 'turn' motion.
         * Turn is in the range ROTATIONS_PER_SECOND to -ROTATIONS_PER_SECOND, and is
         * determined by the difference between speed controller voltages / 2.
         * 
         * A real robot has several phenomena (momentum, friction, the chance to
         * skid) that we don't model here, but this is a good start.
         */ 
        // calculate so that right-motor-forward translates to counterclockwise (positive radians) motion
        double turn = (rightPower - leftPower) / 2;

        /* Now figure out how far the robot went this blip of time.
         * We'll pretend wheel motion scales completely linearly to speed controller
         * input. We multiply by delta to scale the whole computation by the fraction of
         * time that has passed, then add to previous encoder values.
         * 
         * Note: negative power applied to the right motor spins the encoder in the positive
         * direction, so we flip the power value here
         */
        m_leftEncoderValue += TICKS_PER_SECOND * leftPower * delta;
        m_rightEncoderValue += TICKS_PER_SECOND * rightPower * delta;

        /* Rotation is just scaling the max speed by how fast we're turning,
         * multiplying the result by delta to scale by the faction of time passed, and
         * adding it to the current rotation
         */
        m_rotation += ROTATIONS_PER_SECOND * turn * delta;

        /* Override encoders wtih simulated value */
        m_leftEncoder.set(m_leftEncoderValue);
        m_rightEncoder.set(m_rightEncoderValue);

        /* Override read gyro value with simulated value
         * Note: angle on gyro is flipped from right-hand rule; counterclockwise
         * chassis rotation makes angle more negative 
         */
       m_gyro.setAngle(m_rotation);
    }
    public SimVisionSystem getVision()
    {
        return simVision;
    }
    public SimVisionTarget getTarget()
    {
        return target;
    }
}