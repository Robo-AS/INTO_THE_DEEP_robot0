package org.firstinspires.ftc.teamcode.tests.dontneedanymore;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushIntakeCommand extends ParallelCommandGroup {
    public BrushIntakeCommand(){
        super(
                new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING),
                new SetBrushStateCommand(Brush.BrushState.INTAKING)
        );
    }
}
