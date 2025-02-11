package org.firstinspires.ftc.teamcode.tests.HangTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;

@Config
@TeleOp(name = "HangServosTest")
public class HangServosTest extends CommandOpMode {
    private final Hang hang = Hang.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);
        hang.initializeHardware(hardwareMap);
        hang.initialize();
    }


    @Override
    public void run(){

        hang.testServosLopp();


        telemetry.update();
    }
}
