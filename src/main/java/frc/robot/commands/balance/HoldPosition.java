package frc.robot.commands.balance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class HoldPosition extends CommandBase{
    private DriveSubsystem m_drive;

    HoldPosition(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Commands/HoldPosition", true);
        m_drive.setAutoDrive(true);
        m_drive.autoBrake(true);
    }

    @Override
    public void execute() {
        m_drive.autoDrive(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Commands/HoldPosition", false);
        m_drive.setAutoDrive(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
