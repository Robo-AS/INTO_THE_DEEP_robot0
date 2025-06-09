package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class NEWIntakeRetractCommand extends SequentialCommandGroup {
    public NEWIntakeRetractCommand(){
        super(
                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                new WaitCommand(200),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_UP),
                new SetIntakeStateCommand(Intake.IntakeState.SPITTING),
                new WaitCommand(100),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
        );
    }
}
