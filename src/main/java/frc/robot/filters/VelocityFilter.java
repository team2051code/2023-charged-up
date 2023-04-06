package frc.robot.filters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VelocityFilter {
    private double m_prevPos = 0;
    private boolean m_primed = false;
    private double m_pos = 0;
    private double m_velocity;

    public double calculate(double pos){
        double velocity = 0;
        if(m_primed)
            velocity = (m_pos-m_prevPos)*50;
        m_prevPos = pos;
        m_primed = true;
        m_velocity = velocity;
        SmartDashboard.putNumber("armVelocity", velocity);
        return velocity;
    }

    public double getVelocity(){
        return m_velocity;
    }
}
