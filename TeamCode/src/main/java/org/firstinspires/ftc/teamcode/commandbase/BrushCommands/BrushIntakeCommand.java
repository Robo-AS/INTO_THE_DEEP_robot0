package org.firstinspires.ftc.teamcode.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Brush;

public class BrushIntakeCommand extends SequentialCommandGroup {
    public BrushIntakeCommand(){
        super(
                new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED),
                new SetBrushStateCommand(Brush.BrushState.INTAKING)
        );
    }
}
