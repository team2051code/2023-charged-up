package frc.robot.subsystems.simulated;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * CANSparkMax that plumbs setVoltage through the `set` method so that
 * the `get` method works properly.
 */
public class CANSparkMaxSimulated extends CANSparkMax {
    double m_dutyCycle = 0;
    public CANSparkMaxSimulated(int channel, MotorType type) {
        super(channel, type);
    }

    @Override
    public double get() {
        return m_dutyCycle;
    }

    @Override
    public void set(double dutyCycle) {
        m_dutyCycle = dutyCycle;
        super.set(dutyCycle);
    }

    @Override
    public void setVoltage(double voltage) {
        var dutyCycle = MathUtil.clamp(voltage / 12.0, -1.0, 1.0);
        set(dutyCycle);
    }
    
}
