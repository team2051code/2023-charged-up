package frc.robot.controls;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickTeleopDrive {
    public static final double SLEW_RATE_LIMIT = 10;
    private DriveSubsystem m_drive;
    private SlewRateLimiter m_leftLimiter;
    private SlewRateLimiter m_rightLimiter;
    private double m_lastRight;
    private double m_lastLeft;
    private Joystick m_joystick;
    private ButtonLatch m_lowSpeedButton;
    private boolean m_lowSpeed = false;


    public JoystickTeleopDrive (DriveSubsystem subsystem, Joystick joystick) {
        m_drive = subsystem;
        m_leftLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
        m_rightLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
        m_joystick = joystick;
        m_lowSpeedButton = new ButtonLatch(() -> m_joystick.getRawButton(3));
    }

    public void update(){
        var rightY = MathUtil.applyDeadband(m_joystick.getRawAxis(1),0.25);
        var leftY = MathUtil.applyDeadband(-m_joystick.getRawAxis(0),0.25);
        SmartDashboard.putNumber("JoystickTeleopDrive/rightY", rightY);
        SmartDashboard.putNumber("JoystickTeleopDrive/rightY", leftY);
        
        if(m_joystick.getRawButton(1))
            m_drive.setGear(true);
        else
            m_drive.setGear(false);

        if (m_lowSpeedButton.wasPressed()) {
            m_lowSpeed = !m_lowSpeed;
        }


        if(m_lowSpeed) {
            rightY /= 2;
            leftY /= 2;
        }

        SmartDashboard.putBoolean("TeleopDrive/lowSpeed", m_lowSpeed);
        double leftMotorOut = computeMotorOut(leftY,m_lastLeft,m_leftLimiter);
        double rightMotorOut = computeMotorOut(rightY,m_lastRight,m_rightLimiter);

        m_lastLeft = leftY;
        m_lastRight = rightY;

        m_drive.arcadeDrive(rightMotorOut,leftMotorOut);
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