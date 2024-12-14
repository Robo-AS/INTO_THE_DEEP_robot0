package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class BrushThrowingCommandTEST extends ParallelCommandGroup {
    public BrushThrowingCommandTEST(Brush brush){
        super(
                new BrushCommandTEST(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_THROWING, brush),
                new SetBrushStateCommandTEST(Brush.BrushState.THROWING, brush)
        );
    }
}