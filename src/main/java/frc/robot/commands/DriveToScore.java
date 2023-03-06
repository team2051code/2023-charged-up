package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class DriveToScore extends CommandBase{
    DriveSubsystem m_drive;
    double xOffset = -.25;
    double yOffset;
    PhotonCamera camera1;
    boolean finished = false;
    boolean scheduled = false;
    Pod pod;
    int id = 0;
    public DriveToScore(DriveSubsystem subsystem, PhotonCamera camera, Pod podNum, Offset offset, Level level, String team)
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
        FollowRamsete ramsete = new FollowRamsete(m_drive);
        PhotonPipelineResult result = camera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target: targets)
        {
            int targetId = target.getFiducialId();
            if (targetId == id)
            {
                Transform3d transform = target.getBestCameraToTarget();
                double xValue = transform.getX();
                double yValue = transform.getY();
                Command command = ramsete.ramsetePose(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(xValue + xOffset, yValue + yOffset, new Rotation2d(-target.getSkew())));
                command.andThen(()-> finished = true);
                scheduled = true;
                command.schedule();
            }
        }
    }
    @Override
    public void execute()
    {
        if (!scheduled)
        {
        FollowRamsete ramsete = new FollowRamsete(m_drive);
        PhotonPipelineResult result = camera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target: targets)
        {
            int targetId = target.getFiducialId();
            if (targetId == id)
            {
                Transform3d transform = target.getBestCameraToTarget();
                double xValue = transform.getX();
                double yValue = transform.getY();
                Command command = ramsete.ramsetePose(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(xValue + xOffset, yValue + yOffset, new Rotation2d(-target.getSkew())));
                command.andThen(()-> finished = true);
                scheduled = true;
                command.schedule();
            }
        }
        }
    }
    @Override
    public boolean isFinished() {
      return finished;
    }
}

