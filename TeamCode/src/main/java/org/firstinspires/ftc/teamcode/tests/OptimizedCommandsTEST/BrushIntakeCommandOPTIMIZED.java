package org.firstinspires.ftc.teamcode.tests.OptimizedCommandsTEST;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class BrushIntakeCommandOPTIMIZED extends ParallelCommandGroup {
    public BrushIntakeCommandOPTIMIZED(){
        super(
                new InstantCommand(()-> Brush.getInstance().updateSampleState()),
                new InstantCommand(()-> Brush.getInstance().updateIntakedSampleColor()),
                new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING),
                new SetBrushStateCommand(Brush.BrushState.INTAKING)
        );
    }
}
