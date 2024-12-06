package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushCommand extends SequentialCommandGroup {
    public BrushCommand(double motorPower, double servoPower){
        super(
                new RunCommand(() -> Brush.getInstance().brushMotor.setPower(motorPower)),
                new RunCommand(() -> Brush.getInstance().brushSampleServo.setPosition(servoPower))

        );
    }
}
