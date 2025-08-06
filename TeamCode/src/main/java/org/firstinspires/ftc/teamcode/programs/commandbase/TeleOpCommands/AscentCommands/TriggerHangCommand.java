package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands.SetHangStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class TriggerHangCommand extends SequentialCommandGroup {
    public TriggerHangCommand(){
        super(
                new SetHangStateCommand(Hang.HangState.TRIGGERED),
                new InstantCommand(Globals::isHangingLevel3),
                new InstantCommand(MecanumDriveTrain::resetEncoders),
                new InstantCommand(MecanumDriveTrain::setRotation)
        );

    }
}
