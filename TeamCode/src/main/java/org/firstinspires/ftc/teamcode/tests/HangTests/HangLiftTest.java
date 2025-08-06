package org.firstinspires.ftc.teamcode.tests.HangTests;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

//@TeleOp(name = "HangTest", group = "Tests")
public class HangLiftTest extends CommandOpMode {
    private final Lift lift = Lift.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);
        lift.initializeHardware(hardwareMap);
        lift.initialize();
    }

    @Override
    public void run(){
        lift.hangTestLoop(gamepad1.left_stick_y);


        telemetry.addData("LIFT MOTOR CURRENT", lift.getCurrentLiftMotor());
        telemetry.addData("FOLLOWER MOTOR CURRENT", lift.getCurrentFollowerMotor());
        telemetry.update();
    }
}
