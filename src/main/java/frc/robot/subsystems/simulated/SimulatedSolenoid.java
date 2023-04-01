package frc.robot.subsystems.simulated;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;


/*
 * A wrapper for a solenoid that echoes its settings back along get().
 */
public class SimulatedSolenoid extends Solenoid {
    private boolean m_on = false;
    public SimulatedSolenoid(final PneumaticsModuleType moduleType, final int channel) {
        super(moduleType, channel);
    }

    @Override
    public void set(boolean on) {
        m_on = on;
        super.set(on);
    }

    @Override
    public boolean get() {
        return m_on;
    }
}
