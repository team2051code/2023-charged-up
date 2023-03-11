package frc.robot.subsystems.simulated;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class SimulatedGyro extends ADIS16470_IMU{
    double angle;
    @Override
    public synchronized double getAngle()
    {
        return angle;
    }
    public void setAngle(double newAngle)
    {
        angle = newAngle;
    }
}
