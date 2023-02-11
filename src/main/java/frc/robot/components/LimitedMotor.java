package frc.robot.components;

import com.revrobotics.CANSparkMax;

// Motor controller that limits maximum voltage output
public class LimitedMotor extends CANSparkMax {
    private static final double MAX_VOLTAGE = 12;
    // ratio, between 0 and 1, to limit power
    private double m_limit = 1;

    // Creates a CANSparkMax that is limited
    // limit is a value between 0 and 1 that limits the max power of the motor
    public LimitedMotor(int deviceId, MotorType type, double limit) {
        super(deviceId, type);
        m_limit = limit;
    }

    @Override
    public void set(double speed) {
        super.set(clamp(-m_limit, m_limit, speed));
    }

    @Override
    public void setVoltage(double outputVolts) {
        var voltageLimit = m_limit / MAX_VOLTAGE;

        super.setVoltage(clamp(-voltageLimit, voltageLimit, outputVolts));
    }

    private double clamp(double min, double max, double val) {
        return Math.max(min, Math.min(max, val));
    }

}
