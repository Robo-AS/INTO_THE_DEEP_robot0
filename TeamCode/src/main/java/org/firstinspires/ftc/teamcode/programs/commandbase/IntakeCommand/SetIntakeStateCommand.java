package org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


public class SetIntakeStateCommand extends InstantCommand {
    public SetIntakeStateCommand(Intake.IntakeState state){
        super(
                () -> Intake.getInstance().updateIntakeState(state)
        );
    }
}
