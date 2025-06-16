package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeThrowingCommandAuto extends SequentialCommandGroup {
    public IntakeThrowingCommandAuto(){
        super(
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetIntakeStateCommand(Intake.IntakeState.THROWING),
                new WaitUntilCommand(Intake.getInstance()::canStopThrowingWrongSample_AUTO),
//                new InstantCommand(()-> Intake.getInstance().setSampleState(Intake.SampleState.ISNOT)),
                new InstantCommand(()-> Intake.getInstance().setSampleColor(Intake.IntakedSampleColor.NOTHING)),

//                new WaitCommand(4000),
                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
        );
    }
}
