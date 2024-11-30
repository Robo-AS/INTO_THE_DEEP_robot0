package org.firstinspires.ftc.teamcode.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Brush;

public class BrushCommand extends SequentialCommandGroup {
    public BrushCommand(double motorPower, double servoPower){
        super(
                new InstantCommand(() -> Brush.getInstance().brushMotor.setPower(motorPower)),
                new InstantCommand(() -> Brush.getInstance().brushSampleServo.setPosition(servoPower))
        );
    }
}
