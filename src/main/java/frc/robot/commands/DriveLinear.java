
package frc.robot.commands;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompetitionDriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveLinear extends CommandBase {

    private double distance;
    private double targetPower;
    private Timer newTimer = new Timer();
   
    private DriveSubsystem m_drive;

    public DriveLinear(double distance, DriveSubsystem drive){
        this.distance = distance;
        targetPower = 0.5;
        m_drive = drive;
    }
    
    @Override
    public void initialize(){
        newTimer.start();
        m_drive.resetEncoders();
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Distance", m_drive.getRightEncoder());

        if (getDistance()>distance){
            m_drive.tankDrive(targetPower, targetPower);
        }
    }

    public double getDistance(){
        return m_drive.getRightEncoder()*((Math.PI*6)/39.37);
    }
    @Override
    public boolean isFinished(){
        return getDistance()>=distance;
    }

    @Override
    public void end(boolean isInterrupted){
        newTimer.stop();
        m_drive.tankDrive(0,0);
    }

    // private double calculateRunDuration(){
    //     return distance/targetSpeed;
    // }

    // private double calculateRatioSpeed(){
    //     return targetSpeed/realSpeed;
    // }

    // private double modifyDistance(){
    //     return distance += (distance * (5.5/36)) + 1/12;
    // }

//Neo rpm-5676
//Neo rps-(5676/60)*(1/8.71)
//d = v*t
//diameter = 6 in
//circumference = pi*6 in

//36/circumference - rotations/yd
// rps*circumference/36 - yd/s
//(((5676/60)*(1/8.71))*(pi*6))/36 - v in yd/s
//speed- 5.650778978902421 yd/s



//2sec per yd
//2/speed


  //targetSpeed is the amount of yards you want to cover in one second. 
  //ratioSpeed is the percent of power exerted per repetition needed to reach target speed by wheels in one repetition
  //does not account for friction or any other real factors. 
//   public void linearDrive(double currentTime, double distance,double targetSpeed){


}