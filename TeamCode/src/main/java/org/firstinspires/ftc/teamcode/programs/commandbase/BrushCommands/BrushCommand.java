package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushCommand extends ParallelCommandGroup {
    public BrushCommand(double motorPower, double servoPower){
        super(
                new InstantCommand(() -> Brush.getInstance().brushMotor.setPower(motorPower)),
                new InstantCommand(() -> Brush.getInstance().brushSampleServo.setPosition(servoPower))
        );
    }
}
