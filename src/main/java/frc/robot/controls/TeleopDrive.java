package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive {
    public static final double SLEW_RATE_LIMIT = 10;
    private DriveSubsystem m_drive;
    private SlewRateLimiter m_leftLimiter;
    private SlewRateLimiter m_rightLimiter;
    private double m_lastRight;
    private double m_lastLeft;
    private XboxController m_joystick;


    public TeleopDrive (DriveSubsystem subsystem,XboxController joystick) {
        m_drive = subsystem;
        m_leftLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
        m_rightLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
        m_joystick = joystick;
    }

    public void update(){
        var rightY = MathUtil.applyDeadband(-m_joystick.getRightY(),0.25);
        var leftY = MathUtil.applyDeadband(-m_joystick.getLeftY(),0.25);

        double leftMotorOut = computeMotorOut(leftY,m_lastLeft,m_leftLimiter);
        double rightMotorOut = computeMotorOut(rightY,m_lastRight,m_rightLimiter);

        m_lastLeft = leftY;
        m_lastRight = rightY;

        m_drive.tankDrive(rightMotorOut, leftMotorOut);
    }

    private double computeMotorOut(double current, double previous, SlewRateLimiter limiter) {
        if(current == 0 || signsDiffer(current,previous)){
            limiter.reset(0);
        }
        return limiter.calculate(current);
    }

    private boolean signsDiffer(double current, double previous) { 
        return Math.signum(previous) != Math.signum(current);
    }

    
}