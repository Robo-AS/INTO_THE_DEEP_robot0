package org.firstinspires.ftc.teamcode.tests.ExtendoTests;

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
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

//@TeleOp(name = "ExtendoTest", group = "Tests")
public class ExtendoTest extends CommandOpMode {
    private final Extendo extendo = Extendo.getInstance();
    private final Brush brush = Brush.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        extendo.initializeHardware(hardwareMap);
        extendo.initialize();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        //extendo.loopAuto(gamepadEx.gamepad.left_stick_y);




        // Add to on-screen telemetry (optional)
        telemetry.addData("Current Position", extendo.extendoMotor.getCurrentPosition());
        telemetry.addData("Target Position", extendo.getTargetPosition());
        telemetry.update();

    }
}
