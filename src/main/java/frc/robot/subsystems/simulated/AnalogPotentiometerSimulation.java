package frc.robot.subsystems.simulated;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/*
 * An analog potentiometer that lets you set a value to override the one that is read
 */
public class AnalogPotentiometerSimulation extends AnalogPotentiometer{
    private double m_value;

    public AnalogPotentiometerSimulation(int channel, double fullRange, double offset) {
        super(channel, fullRange, offset);
    }

    @Override
    public double get() {
        return m_value;
    }

    public void set(double value) {
        m_value = value;
    }
    
}
