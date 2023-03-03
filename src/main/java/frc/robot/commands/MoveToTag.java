package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.DriveSubsystem;

public class MoveToTag {
    private DriveSubsystem m_drive;
    private PhotonCamera camera1;
    private double xOffset;
    private double yOffset;
    private FollowRamsete ramseteCommand;
    
    public MoveToTag(DriveSubsystem driveSubsystem, PhotonCamera camera, double xOffset, double yOffset)
    {
        m_drive = driveSubsystem;
        camera1 = camera;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        ramseteCommand = new FollowRamsete(m_drive);
    }
    public Command moveToTagID(int id)
    {
        PhotonPipelineResult result = camera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target: targets)
        {
            int targetId = target.getFiducialId();
            if (targetId == id)
            {
                Transform3d movement = target.getBestCameraToTarget();
                double x = movement.getX() + xOffset;
                double y = movement.getY() + yOffset;
                Rotation3d rotation = movement.getRotation();
                return ramseteCommand.ramsetePose(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(x, y, new Rotation2d(-rotation.getAngle() * 180 / Math.PI)));
            }
        }
        return null;
    }
}
