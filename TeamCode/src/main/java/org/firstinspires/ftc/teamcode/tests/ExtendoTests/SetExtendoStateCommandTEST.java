package org.firstinspires.ftc.teamcode.tests.ExtendoTests;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class SetExtendoStateCommandTEST extends InstantCommand {
    public SetExtendoStateCommandTEST(Extendo.ExtendoState state){
        super(
                () -> Extendo.getInstance().update(state)
        );
    }
}
