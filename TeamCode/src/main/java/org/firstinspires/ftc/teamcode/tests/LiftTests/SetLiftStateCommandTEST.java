package org.firstinspires.ftc.teamcode.tests.LiftTests;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class SetLiftStateCommandTEST extends InstantCommand {
    public SetLiftStateCommandTEST(Lift.LiftState state){
        super(
                () -> Lift.getInstance().update(state)
        );
    }
}
