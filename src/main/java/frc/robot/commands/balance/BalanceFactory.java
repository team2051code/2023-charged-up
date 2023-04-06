package frc.robot.commands.balance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveLinear;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceFactory {
    public static final Command balance(DriveSubsystem drive, ArmSubsystem arm){
        Command armMoves = new SequentialCommandGroup(new MoveArm(arm,225));
        Command offRamp = new SequentialCommandGroup(
        new DriveLinear(0.25, drive),
        new OffRamp(drive),
        new DriveLinear(0.5, drive),
        new OnRamp(drive)
        // OnRamp will schedule SeekBalance with its own calculation once done
        );
        Command balance = new ParallelCommandGroup(armMoves,offRamp);
        return balance;
    }
}
