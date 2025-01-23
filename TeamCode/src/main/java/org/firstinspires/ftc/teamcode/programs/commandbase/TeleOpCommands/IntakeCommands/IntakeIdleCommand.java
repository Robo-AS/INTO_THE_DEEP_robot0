package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class IntakeIdleCommand extends ParallelCommandGroup {
    public IntakeIdleCommand(){
        super(
                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );

    }
}
