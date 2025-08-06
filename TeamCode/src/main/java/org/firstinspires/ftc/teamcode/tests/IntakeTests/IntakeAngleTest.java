package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


@Disabled
@Config
@TeleOp(name = "IntakeAngleTest", group = "Tests")
public class IntakeAngleTest extends CommandOpMode {
    private final Intake intake = Intake.getInstance();
    public static double ANGLE = 0.5;

    @Override
    public void initialize(){
        intake.initializeHardware(hardwareMap);

    }

    @Override
    public void run(){
        intake.angleServo.setPosition(ANGLE);
    }
}
