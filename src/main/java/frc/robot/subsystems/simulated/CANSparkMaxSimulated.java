package frc.robot.subsystems.simulated;

import com.revrobotics.CANSparkMax;

/**
 * CANSparkMax that plumbs setVoltage through the `set` method so that
 * the `get` method works properly.
 */
public class CANSparkMaxSimulated extends CANSparkMax {
    public CANSparkMaxSimulated(int channel, MotorType type) {
        super(channel, type);
    }

    @Override
    public void setVoltage(double voltage) {
        set(voltage / 12.0);
    }
    
}
