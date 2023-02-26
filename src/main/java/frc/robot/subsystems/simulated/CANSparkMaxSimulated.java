package frc.robot.subsystems.simulated;

import com.revrobotics.CANSparkMax;

public class CANSparkMaxSimulated extends CANSparkMax {
    public CANSparkMaxSimulated(int channel, MotorType type) {
        super(channel, type);
    }

    @Override
    public void setVoltage(double voltage) {
        set(voltage / 12.0);
    }
    
}
