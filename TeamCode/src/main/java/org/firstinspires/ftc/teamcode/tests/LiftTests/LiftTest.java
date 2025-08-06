package org.firstinspires.ftc.teamcode.tests.LiftTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

@TeleOp(name = "LiftTest", group = "Tests")
public class LiftTest extends CommandOpMode {
    private final Lift lift = Lift.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        lift.initializeHardware(hardwareMap);
        lift.initialize();

//        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(
//                        () -> CommandScheduler.getInstance().schedule(
//                                new ConditionalCommand(
//                                        new SetLiftStateCommand(Lift.LiftState.HIGH_RUNG),
//                                        new SetLiftStateCommand(Lift.LiftState.IDLE),
//                                        () -> lift.liftState == Lift.LiftState.IDLE
//                                )
//                        )
//
//                );



    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


//        lift.loopAuto();
//        lift.loop();


        telemetry.addData("Current Position", lift.liftMotor.getCurrentPosition());
        telemetry.addData("Target Position", lift.getTargetPosition());
        telemetry.update();

    }
}
