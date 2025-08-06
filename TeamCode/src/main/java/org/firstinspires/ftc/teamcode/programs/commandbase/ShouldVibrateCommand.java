package org.firstinspires.ftc.teamcode.programs.commandbase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class ShouldVibrateCommand extends InstantCommand {
    public ShouldVibrateCommand(){
        super(
                () -> Intake.getInstance().updateShouldVibrate()
        );
    }
}
