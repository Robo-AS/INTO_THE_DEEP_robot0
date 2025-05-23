package org.firstinspires.ftc.teamcode.tests.ArmTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;

//@TeleOp(name = "SweeperTest", group = "Tests")
public class SweeperTest extends CommandOpMode {
    private final Sweeper sweeper = Sweeper.getInstance();
    private GamepadEx gamepadEx;
    private ElapsedTime elapsedtime;
    private double loopTime = 0;


    @Override
    public void initialize(){
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        sweeper.initializeHardware(hardwareMap);
        sweeper.testInit();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        sweeper.testInit();


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
