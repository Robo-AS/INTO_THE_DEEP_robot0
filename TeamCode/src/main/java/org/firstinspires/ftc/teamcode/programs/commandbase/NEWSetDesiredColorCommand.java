package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWSetDesiredColorCommand extends InstantCommand {
    public NEWSetDesiredColorCommand(Intake.DesiredSampleColor color){
        super(
                () -> Intake.getInstance().updateDesiredSampleColor(color)
        );
    }
}
