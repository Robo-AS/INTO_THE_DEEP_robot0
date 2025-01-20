package org.firstinspires.ftc.teamcode.tests.ArmTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

@TeleOp(name = "ArmTest", group = "Tests")
public class ArmTest extends CommandOpMode {
    private final Arm arm = Arm.getInstance();
    private GamepadEx gamepadEx;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        arm.initializeHardware(hardwareMap);
        arm.initialize();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        arm.loop();

        // Add to on-screen telemetry (optional)
//        telemetry.addData("Current Position", extendo.extendoMotor.getCurrentPosition());
//        telemetry.addData("Target Position", extendo.getTargetPosition());
        telemetry.update();

    }
}
