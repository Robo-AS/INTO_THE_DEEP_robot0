package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class SetDesiredColorCommandTEST extends InstantCommand {
    public SetDesiredColorCommandTEST(Brush.DesiredSampleColor color, Brush brush){
        super(
                () -> brush.updateDesiredSampleColor(color)
        );
    }
}

