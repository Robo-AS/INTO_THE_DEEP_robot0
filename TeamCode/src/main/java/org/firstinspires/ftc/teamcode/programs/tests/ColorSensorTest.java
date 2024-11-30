package org.firstinspires.ftc.teamcode.programs.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {
    public RevColorSensorV3 colorSensor0;

    @Override
    public void runOpMode(){
        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor0");

        waitForStart();


        while(opModeIsActive()){
            int red = colorSensor0.red();
            int green = colorSensor0.green();
            int blue = colorSensor0.blue();
            int alpha = colorSensor0.alpha(); // Combined light intensity
            int argb = colorSensor0.argb();   // Packed ARGB color
            double distance = colorSensor0.getDistance(DistanceUnit.CM);

            if(distance < 3)
                telemetry.addData("There is piece", true);
            else telemetry.addData("There is piece", false);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("ARGB", argb);
            telemetry.update();
        }
    }

}


