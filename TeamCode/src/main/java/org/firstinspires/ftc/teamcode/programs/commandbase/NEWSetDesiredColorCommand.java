package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWSetDesiredColorCommand extends InstantCommand {
    public NEWSetDesiredColorCommand(Intake.DesiredSampleColor color){
        super(
                () -> Intake.getInstance().updateDesiredSampleColor(color)
        );
    }
}
