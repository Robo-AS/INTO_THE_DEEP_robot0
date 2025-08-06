package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;


public class NEWIntakeIntakingCommand extends ParallelCommandGroup {
    public NEWIntakeIntakingCommand(){
        super(
                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new InstantCommand(() -> Intake.getInstance().resetTime()),
                new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_DOWN),
                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
        );

    }
}
