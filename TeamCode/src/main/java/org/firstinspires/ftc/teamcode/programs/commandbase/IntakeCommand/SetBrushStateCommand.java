package org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


public class SetBrushStateCommand extends InstantCommand {
    public SetBrushStateCommand(Intake.BrushState state){
        super(
                () -> Intake.getInstance().updateBrushState(state)
        );
    }
}
