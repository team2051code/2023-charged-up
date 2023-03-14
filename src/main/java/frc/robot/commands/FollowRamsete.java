package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.CompetitionDriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowRamsete {
    private DriveSubsystem m_robotDrive;
    public FollowRamsete(DriveSubsystem driveSubsystem)
    {
        m_robotDrive = driveSubsystem;
    }
    public Command ramseteFile(String fileName)
    {
  
      // An ExampleCommand will run in autonomous
      m_robotDrive.resetEncoders();
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(
      CompetitionDriveConstants.ksVolts, CompetitionDriveConstants.kvVoltSecondsPerMeter), CompetitionDriveConstants.kDriveKinematics, 10);
      var TrajectoryConfig = new TrajectoryConfig(CompetitionDriveConstants.kMaxSpeedMetersPerSecond,
      CompetitionDriveConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(CompetitionDriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
      Trajectory trajectory;
      try {
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
        return null;
      }
      System.out.println(trajectory);
      System.out.println("Robot will finish in" + trajectory.getTotalTimeSeconds());
      RamseteCommand ramseteCommand =
      new RamseteCommand(
          trajectory,
          m_robotDrive::getPose,
          new RamseteController(CompetitionDriveConstants.kRamseteB, CompetitionDriveConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              CompetitionDriveConstants.ksVolts,
              CompetitionDriveConstants.kvVoltSecondsPerMeter,
              CompetitionDriveConstants.kaVoltSecondsSquaredPerMeter),
          CompetitionDriveConstants.kDriveKinematics,
          m_robotDrive::getWheelSpeeds,
          new PIDController(CompetitionDriveConstants.kPDriveVel, 0, 0),
          new PIDController(CompetitionDriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_robotDrive::tankDriveVolts,
          m_robotDrive);
      m_robotDrive.resetOdometry(trajectory.getInitialPose());
      m_robotDrive.resetEncoders();
      return ramseteCommand.andThen(() -> System.out.println("Finished running RAMSETE"))
      .andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
    public Command ramsetePose(Pose2d initialPose, List<Translation2d> poseList, Pose2d endingPose)
    {
  
      // An ExampleCommand will run in autonomous
      m_robotDrive.resetEncoders();
      m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(
      CompetitionDriveConstants.ksVolts, CompetitionDriveConstants.kvVoltSecondsPerMeter), CompetitionDriveConstants.kDriveKinematics, 10);
      var TrajectoryConfig = new TrajectoryConfig(CompetitionDriveConstants.kMaxSpeedMetersPerSecond,
      CompetitionDriveConstants.kMaxAccelerationMetersPerSecondSquared);
      TrajectoryConfig.setKinematics(CompetitionDriveConstants.kDriveKinematics).addConstraint(autoVoltageConstraint);
      Trajectory trajectory;
      trajectory = TrajectoryGenerator.generateTrajectory(initialPose, poseList, endingPose, TrajectoryConfig);
  
      
      System.out.println(trajectory);
      System.out.println("Robot will finish in" + trajectory.getTotalTimeSeconds());
        
      RamseteCommand ramseteCommand =
      new RamseteCommand(
          trajectory,
          m_robotDrive::getPose,
          new RamseteController(CompetitionDriveConstants.kRamseteB, CompetitionDriveConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              CompetitionDriveConstants.ksVolts,
              CompetitionDriveConstants.kvVoltSecondsPerMeter,
              CompetitionDriveConstants.kaVoltSecondsSquaredPerMeter),
          CompetitionDriveConstants.kDriveKinematics,
          m_robotDrive::getWheelSpeeds,
          new PIDController(CompetitionDriveConstants.kPDriveVel, 0, 0),
          new PIDController(CompetitionDriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_robotDrive::tankDriveVolts,
          m_robotDrive);
      m_robotDrive.resetOdometry(trajectory.getInitialPose());
      m_robotDrive.resetEncoders();
      return ramseteCommand.andThen(() -> System.out.println("Finished running RAMSETE"))
      .andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
}
