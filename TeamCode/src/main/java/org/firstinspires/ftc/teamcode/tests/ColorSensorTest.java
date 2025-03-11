package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensorTest", group = "Tests")
public class ColorSensorTest extends LinearOpMode {
    public RevColorSensorV3 colorSensor0;//declare
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor"); //mapping

        waitForStart();


        while(opModeIsActive()){
            //Some funtion example(you can find all looking int the RecColorSensorV3 class)
            int red = colorSensor0.red();
            int green = colorSensor0.green();
            int blue = colorSensor0.blue();
//            int alpha = colorSensor0.alpha(); // Combined light intensity
//            int argb = colorSensor0.argb();   // Packed ARGB color
//            double distance = colorSensor0.getDistance(DistanceUnit.CM);

//            if(distance < 3)
//                telemetry.addData("There is piece", true);
//            else telemetry.addData("There is piece", false);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
//            telemetry.addData("Alpha", alpha);
//            telemetry.addData("ARGB", argb);


            if(blue > 400)
                telemetry.addData("Blue", "BLUE");
            else if(red > 400 && red < 600)
                telemetry.addData("RED", "RED");
            else if(red > 600)
                telemetry.addData("YELLOW", "YELLOW");
            else telemetry.addData("NOTHING", "NOTHING");

            telemetry.update();
        }


    }

}


