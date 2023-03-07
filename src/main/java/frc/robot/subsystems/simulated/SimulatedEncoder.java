package frc.robot.subsystems.simulated;

import java.nio.channels.UnsupportedAddressTypeException;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.CounterBase;

/**
 * An "encoder" that emits values injected into it from a simulator framework
 * (such as SimpleSimulatedChassis)
 */
public class SimulatedEncoder implements RelativeEncoder {
    private int m_encoderZeroPoint = 0;
    private int m_encoderValue = 0;

    @Override
    public double getPosition() {
        return m_encoderValue - m_encoderZeroPoint;
    }

    @Override
    public REVLibError setPosition(double start) {
        m_encoderZeroPoint = m_encoderValue;
        return REVLibError.kOk;
    }

    /**
     * Set the encoder
     * @param value New encoder value
     */
    public void set(int value) {
        m_encoderValue = value;
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getPositionConversionFactor() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getVelocityConversionFactor() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getAverageDepth() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getMeasurementPeriod() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getCountsPerRevolution() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return false;
    }
}
