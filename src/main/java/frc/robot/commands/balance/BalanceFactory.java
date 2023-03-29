package frc.robot.commands.balance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveLinear;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceFactory {
    public static final Command balance(DriveSubsystem drive, ArmSubsystem arm){
        Command armMoves = new SequentialCommandGroup(new Arm90(arm));
        Command offRamp = new SequentialCommandGroup(
        new DriveLinear(0.25, drive),
        new OffRamp(drive),
        new DriveLinear(0.5, drive),
        new OnRamp(drive),
        new DriveLinear(-0.08, drive,0.3),
        new HoldPosition(drive)
        // OnRamp will schedule SeekBalance with its own calculation once done
        );
        //Command balance = new ParallelCommandGroup(armMoves,offRamp);
        return offRamp;
    }
}
