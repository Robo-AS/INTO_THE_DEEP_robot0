package org.firstinspires.ftc.teamcode.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SetDesiredColorCommand extends InstantCommand {
    public SetDesiredColorCommand(Intake intake, Intake.DesiredSampleColor color){
        super(
                () -> intake.setDesiredSampleColor(color)
        );
    }
}
