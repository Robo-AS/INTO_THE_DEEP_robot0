package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class NEWIntakeIdleCommand extends ParallelCommandGroup {
    public NEWIntakeIdleCommand(){
        super(
                new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_UP),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
        );

    }
}
