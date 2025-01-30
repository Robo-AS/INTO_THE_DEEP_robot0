package org.firstinspires.ftc.teamcode.tests.LiftTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

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

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetLiftStateCommandTEST(Lift.LiftState.HIGH_RUNG),
                                        new SetLiftStateCommandTEST(Lift.LiftState.IDLE),
                                        () -> lift.liftState == Lift.LiftState.IDLE
                                )
                        )

                );



    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


//        lift.loop();
        lift.testLoop();

        // Create a telemetry packet for FTCDashboard
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Current Position", lift.liftMotor.getCurrentPosition());
//        packet.put("Target Position", lift.getTargetPosition());

        // Send the packet to the dashboard
//        dashboard.sendTelemetryPacket(packet);
//        telemetry.update();


        // Add to on-screen telemetry (optional)
        telemetry.addData("Current Position", lift.liftMotor.getCurrentPosition());
        telemetry.addData("Target Position", lift.getTargetPosition());
        telemetry.update();

    }
}
