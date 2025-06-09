package org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class SetIntakeAngleCommand extends InstantCommand {
    public SetIntakeAngleCommand(Intake.IntakeAngle angle){
        super(
                () -> Intake.getInstance().updateAngle(angle)
        );
    }
}
