package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    public RevColorSensorV3 colorSensor0;

    public CachingServo brushSampleServo;

    enum SampleColor{
        RED,
        BLUE,
        YELLOW;
    }

    public double position = 0.8;
    public boolean isSample;
    SampleColor sampleColor;

    @Override
    public void runOpMode(){
        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor0");

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));


        waitForStart();


        while(opModeIsActive()){
            int red = colorSensor0.red();
            int green = colorSensor0.green();
            int blue = colorSensor0.blue();
            double distance = colorSensor0.getDistance(DistanceUnit.CM);

            brushSampleServo.setPosition(0.7);

//            if(distance < 3) {
//                if(blue > 400)
//                    sampleColor = SampleColor.BLUE;
//                else if(red > 400 && red < 600)
//                    sampleColor = SampleColor.RED;
//                else if(red > 600)
//                    sampleColor = SampleColor.YELLOW;
//
//                if(sampleColor == SampleColor.BLUE || sampleColor == SampleColor.YELLOW){
//                    brushSampleServo.setPosition(0);
//                    sleep(50);
//                    brushSampleServo.setPosition(0.5);
//                    sleep(900000000);
//                }
//
//            }

            if(distance < 3){
                brushSampleServo.setPosition(0);
                sleep(50);
                brushSampleServo.setPosition(0.5);
                sleep(900000000);
            }


//            if(distance < 3){
//                if(blue > 400)
//                    sampleColor = SampleColor.BLUE;
//                else if(red > 400 && red < 600)
//                    sampleColor = SampleColor.RED;
//                else if(red > 600)
//                    sampleColor = SampleColor.YELLOW;
//
//                if(sampleColor == SampleColor.BLUE || sampleColor == SampleColor.YELLOW){
//                    position = 0.5;
//                }
//                else position=1;
//            }






            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();
        }
    }

}


