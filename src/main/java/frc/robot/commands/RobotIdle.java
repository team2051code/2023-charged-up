package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class RobotIdle {
    public static Command idle(ArmSubsystem arm){
        Command idle = new SequentialCommandGroup(
            new Delay(0.5),
            new MoveArm(arm, 180+120),
            new Delay(0.5),
            new MoveArm(arm, 180-60),
            new Delay(0.5),
            new MoveArm(arm, 180+60),
            new Delay(0.5),
            new MoveArm(arm, 180-120)
        );
        return idle;
    }
}
