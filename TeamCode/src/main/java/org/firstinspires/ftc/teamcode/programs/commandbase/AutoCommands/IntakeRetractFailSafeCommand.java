package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeRetractFailSafeCommand extends SequentialCommandGroup {
    public IntakeRetractFailSafeCommand() {
        super(
                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACT_AUTO_FAIL_SAFE)

        );
    }
}
