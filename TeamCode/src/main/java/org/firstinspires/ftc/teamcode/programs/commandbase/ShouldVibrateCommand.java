package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class ShouldVibrateCommand extends InstantCommand {
    public ShouldVibrateCommand(){
        super(
                () -> Brush.getInstance().updateShouldVibrate()
        );
    }
}
