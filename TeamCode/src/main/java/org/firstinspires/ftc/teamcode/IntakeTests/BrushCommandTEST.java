package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushCommandTEST extends ParallelCommandGroup  {
    public BrushCommandTEST(double motorPower, double servoPower, Brush brush){
        super(
                new InstantCommand(() -> brush.brushMotor.setPower(motorPower)),
                new InstantCommand(() -> brush.brushSampleServo.setPosition(servoPower))

        );
    }
}