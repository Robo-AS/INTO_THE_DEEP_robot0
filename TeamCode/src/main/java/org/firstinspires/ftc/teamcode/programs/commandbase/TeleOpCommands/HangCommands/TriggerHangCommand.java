package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.HangCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class TriggerHangCommand extends SequentialCommandGroup {
    public TriggerHangCommand(){
        super(
                new SetHangStateCommand(Hang.HangState.TRIGGERED)
                //new InstantCommand(Globals::isHanging),
                //new InstantCommand(MecanumDriveTrain::resetEncoders)
        );

    }
}
