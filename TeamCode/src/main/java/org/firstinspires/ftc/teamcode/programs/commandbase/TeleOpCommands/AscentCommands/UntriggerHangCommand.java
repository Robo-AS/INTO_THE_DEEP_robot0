package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class UntriggerHangCommand extends SequentialCommandGroup {
    public UntriggerHangCommand(){
        super(
                //new SetHangStateCommand(Hang.HangState.IDLE),
                new InstantCommand(Globals::isNotHanging)
        );

    }
}
