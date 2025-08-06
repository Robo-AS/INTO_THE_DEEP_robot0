package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeThrowingCommand extends SequentialCommandGroup {
    public IntakeThrowingCommand(){
        super(
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetIntakeStateCommand(Intake.IntakeState.THROWING),
                new WaitUntilCommand(Intake.getInstance()::canStopThrowingWrongSample_TELEOP),
                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
        );
    }
}
