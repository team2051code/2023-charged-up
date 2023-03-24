package frc.robot.commands.balance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveLinear;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceFactory {
    public static final Command balance(DriveSubsystem drive){
        Command offRamp = new SequentialCommandGroup(
        new DriveLinear(0.25, drive),
        new OffRamp(drive),
        new DriveLinear(0.5, drive),
        new OnRamp(drive)
        // OnRamp will schedule SeekBalance with its own calculation once done
        );
        return offRamp;
    }
}