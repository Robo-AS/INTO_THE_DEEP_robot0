package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import android.provider.Settings;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class SetBrushAngleCommand extends InstantCommand {
    public SetBrushAngleCommand(double position){
        super(
                () -> Brush.getInstance().brushAngleServo.setPosition(position)
        );
    }
}
