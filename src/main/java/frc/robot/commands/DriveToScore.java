package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
public class DriveToScore extends CommandBase{
    DriveSubsystem m_drive;
    double xOffset = -.25;
    double yOffset;
    PhotonCamera camera1;
    boolean finished = false;
    boolean perpendicular = false;
    double pidP = 0;
    double pidI = 0;
    double pidD = 0;
    PIDController leftPID;
    PIDController rightPID;
    Pod pod;
    int id = 0;
    public DriveToScore(DriveSubsystem subsystem, PhotonCamera camera)
    {
        m_drive = subsystem;
        camera1 = camera;
    }
    public DriveToScore(DriveSubsystem subsystem, PhotonCamera camera,  Pod podNum, Offset offset, Level level, String team)
    {
        m_drive = subsystem;
        camera1 = camera;
        pod = podNum;
        if (team.equals("red"))
        {
            if (pod == Pod.LEFT)
            {
                id = 3;
            }
            else if (pod == Pod.CENTER)
            {
                id = 2;
            }
            else
            {
                id = 1;
            }
        }
        else if (team.equals("blue"))
        {
            if (pod == Pod.LEFT)
            {
                id = 8;
            }
            else if (pod == Pod.CENTER)
            {
                id = 7;
            }
            else
            {
                id = 6;
            }
        }
    }
    public enum Pod {LEFT, CENTER, RIGHT}
    public enum Offset{LEFT, CENTER, RIGHT}
    public enum Level{TOP, MIDDLE, BOTTOM}
    @Override
    public void initialize() {
        leftPID = new PIDController(pidP, pidI, pidD);
        rightPID = new PIDController(pidP, pidI, pidD);
        PhotonPipelineResult result = camera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
    }
    @Override
    public void execute()
    {
        PhotonPipelineResult result = camera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        boolean parallel = false;
        for (PhotonTrackedTarget target: targets)
        {
            int targetId = target.getFiducialId();
            if (targetId == 0)
            {
                Transform3d transform = target.getBestCameraToTarget();
                double xValue = transform.getX();
                double yValue = transform.getY();
                double angleDeg = Math.atan(yValue / xValue) * 180 / Math.PI;
                perpendicular = goPerpendicular(angleDeg);
                double horizontalDistance = target.getYaw();
                if (perpendicular && yValue >= 0)
                {
                    m_drive.tankDrive(1, 1);
                }
                else if (yValue < 0)
                {
                   parallel = goParrallel(angleDeg);
                }
                if (parallel && xValue > xOffset)
                {
                    m_drive.tankDriveVolts(5, 5);
                }
                else
                {
                    m_drive.tankDriveVolts(0, 0);
                }
            }
        double leftVelocity = leftPID.calculate(m_drive.getWheelSpeeds().leftMetersPerSecond);
        double rightVelocity = rightPID.calculate(m_drive.getWheelSpeeds().rightMetersPerSecond);
        m_drive.tankDrive(leftVelocity, rightVelocity);
        }
    }
    public boolean goPerpendicular(double angleDeg)
    {
        if (angleDeg > 0 && angleDeg < 89)
        {
            m_drive.tankDriveVolts(5, 0);
            return false;
        }
        else if (angleDeg < 0 && angleDeg > -89)
        {
            m_drive.tankDriveVolts(0, 5);
            return false;
        }
        return true;
    }
    public boolean goParrallel(double angleDeg)
    {
        if (angleDeg > 1)
        {
            m_drive.tankDriveVolts(5, 0);
            return false;
        }
        if (angleDeg < -1)
        {
            m_drive.tankDriveVolts(0, 5);
            return false;
        }
        return true;
    }
    @Override
    public boolean isFinished() {
      return finished;
    }
}

