package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OuttakeGoLowBasketCommand extends SequentialCommandGroup {
    public OuttakeGoLowBasketCommand(){
        super(
                new SetLiftStateCommand(Lift.LiftState.LOW_BASKET),
                new WaitUntilCommand(Lift.getInstance()::canRotateWrist),
                new SetWristStateCommand(Arm.WristState.HIGH_BASKET),
                new SetArmStateCommand(Arm.ArmState.HIGH_BASKET)
        );
    }
}
