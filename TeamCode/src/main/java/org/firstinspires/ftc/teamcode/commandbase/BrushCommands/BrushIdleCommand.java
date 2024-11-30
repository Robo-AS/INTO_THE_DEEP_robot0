package org.firstinspires.ftc.teamcode.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Brush;

public class BrushIdleCommand extends SequentialCommandGroup{
    public BrushIdleCommand(){
        super(
                new BrushCommand(0, 0.5),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );
    }
}
