package org.firstinspires.ftc.teamcode.tests.ExtendoTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


@TeleOp(name = "ExtendoTest", group = "Tests")
public class ExtendoTest extends CommandOpMode {
    private final Extendo extendo = Extendo.getInstance();
    private final Intake intake = Intake.getInstance();
    private GamepadEx gamepadEx;
    public double currentPosition = 0;


    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        extendo.initializeHardware(hardwareMap);
        extendo.initialize();

        intake.initializeHardware(hardwareMap);
        intake.initialize();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        //extendo.loopAuto(gamepadEx.gamepad.left_stick_y);
        extendo.testPID();
//        extendo.loop(gamepadEx.getLeftY());




        // Add to on-screen telemetry (optional)
        telemetry.addData("Current Position", extendo.extendoMotor.getCurrentPosition());
        telemetry.addData("Target Position", extendo.getTargetPosition());
        telemetry.update();

    }
}
