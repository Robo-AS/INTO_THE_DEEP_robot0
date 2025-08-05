package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OuttakeGoHighBasketAutoCommand extends SequentialCommandGroup {
    public OuttakeGoHighBasketAutoCommand(){
        super(
                new SetLiftStateCommand(Lift.LiftState.HIGH_BASKET),
                new WaitUntilCommand(Lift.getInstance()::canRotateArmHighBasket),
                new SetWristStateCommand(Arm.WristState.HIGH_BASKET),//HIGH_BASKET
                new SetArmStateCommand(Arm.ArmState.HIGH_BASKET)
        );
    }
}
