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

    public static double POSITION_HANG_LEFT = 0.5;
    public static double POSITION_HANG_RIGHT = 0.5;
    public static double POSITION_SAFETY_LEFT = 0.5;
    public static double POSITION_SAFETY_RIGHT = 0.5;

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);
        hang.initializeHardware(hardwareMap);
        //hang.initialize();
    }


    @Override
    public void run(){


        hang.hangServoLeft.setPosition(POSITION_HANG_LEFT);
        hang.hangServoRight.setPosition(POSITION_HANG_RIGHT);

        hang.servoSafetyLeft.setPosition(POSITION_SAFETY_LEFT);
        hang.servoSafetyRight.setPosition(POSITION_SAFETY_RIGHT);

        telemetry.update();
    }
}
