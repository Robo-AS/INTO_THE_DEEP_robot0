package org.firstinspires.ftc.teamcode.tests.IntakeTests;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

@Disabled
@TeleOp(name = "AxonTest", group = "Tests")
public class AxonTest extends CommandOpMode {
    private final Intake intake = Intake.getInstance();
    public GamepadEx gamepadEx;
    double initial, previous, current;
    double total = 0;
    boolean firstRead = true;
    int biggest = Integer.MIN_VALUE;
    int rotations = 0;

    @Override
    public void initialize() {
        intake.initializeHardware(hardwareMap);
        intake.initialize();
        total = 0;
        rotations = 0;
//        telemetry.addData("FIRSTREAD", firstRead);
//        telemetry.update();
//
//
//        telemetry.addData("CURRENT", current);
//        telemetry.addData("PREVIOUS", previous);
//        telemetry.update();
    }

    @Override
    public void run(){
         current = intake.analogInput.getVoltage() / 3.3 * 360;


        if(gamepad1.dpad_up)
            intake.rollersServo.setPosition(1);
        if(gamepad1.dpad_left)
            intake.rollersServo.setPosition(0.5);
        if(gamepad1.dpad_down)
            intake.rollersServo.setPosition(0);

        if (firstRead) {
            initial = current;
            previous = current;
            firstRead = false;
            return;
        }


        double delta = current - previous;

        if(delta > 180)
            rotations --;
        else if (delta < -180)
            rotations ++;

        previous = current;
        total = (rotations * 360) + (current);





        telemetry.addData("Current:", current);
        telemetry.addData("Previous:", previous);
        telemetry.addData("Total:", total);
        telemetry.addData("Rotations", rotations);

        telemetry.update();
    }
}
