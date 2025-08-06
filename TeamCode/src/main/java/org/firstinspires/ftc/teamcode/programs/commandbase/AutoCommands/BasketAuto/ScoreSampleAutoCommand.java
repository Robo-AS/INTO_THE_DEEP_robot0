package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class ScoreSampleAutoCommand extends SequentialCommandGroup {
    public ScoreSampleAutoCommand(){
        super(
                new WaitUntilCommand(Lift.getInstance()::canOpenClaw),
                new WaitCommand(20),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new WaitCommand(150)//300
//                new SetArmStateCommand(Arm.ArmState.INIT),
//                new SetClawStateCommand(Arm.ClawState.OPEN),
//                new SetWristStateCommand(Arm.WristState.INIT),
//                new SetLiftStateCommand(Lift.LiftState.IDLE)

        );
    }
}
