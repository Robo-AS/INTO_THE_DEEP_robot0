package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushIntakeCommandTEST extends ParallelCommandGroup {
    public BrushIntakeCommandTEST(Brush brush){
        super(
                new BrushCommandTEST(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING, brush),
                new SetBrushStateCommandTEST(Brush.BrushState.INTAKING, brush)
        );
    }
}
