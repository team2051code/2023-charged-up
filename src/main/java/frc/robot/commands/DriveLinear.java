
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveLinear extends CommandBase {

    private double distance_meters;
    private double m_targetPower;
    private double m_encoderTarget;
    private Timer newTimer = new Timer();
    private boolean frontSide;
    private DriveSubsystem m_drive;

    public DriveLinear(double distance_meters, DriveSubsystem drive, double targetPower){
        this.distance_meters = distance_meters;
        m_targetPower = targetPower;
        m_drive = drive;
        if(distance_meters < 0)
            frontSide = false;
        else
            frontSide = true;
    }

    public DriveLinear(double distance_meters, DriveSubsystem drive){
       this(distance_meters,drive,0.5);
    }
    
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Commands/DriveLinear", true);
        newTimer.start();
        m_drive.setAutoDrive(true);
        m_encoderTarget = getEncoder() + distance_meters;
        SmartDashboard.putNumber("DriveLinear/Encoder Target", m_encoderTarget);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("DriveLinear/Distance", getEncoder());
        if(frontSide)
            m_drive.autoDrive(m_targetPower, m_targetPower);
        else
            m_drive.autoDrive(-m_targetPower, -m_targetPower);
    }

    public double getEncoder(){
        return m_drive.getRightEncoder()*((Math.PI*6)/53.213);
    }
    @Override
    public boolean isFinished(){
        SmartDashboard.putNumber("DriveLinear/Distance Off", getEncoder()-m_encoderTarget);
        boolean finished = false;
        if(frontSide)
            finished = getEncoder()>=m_encoderTarget;
        else
            finished = getEncoder()<=m_encoderTarget;
        return finished;
    }

    @Override
    public void end(boolean isInterrupted){
        newTimer.stop();
        m_drive.autoDrive(0,0);
        m_drive.setAutoDrive(false);
        SmartDashboard.putBoolean("Commands/DriveLinear", false);
    }
}