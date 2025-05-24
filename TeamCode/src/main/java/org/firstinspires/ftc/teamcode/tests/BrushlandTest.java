package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@TeleOp(name = "BrushlandTest", group = "Tests")
public class BrushlandTest extends LinearOpMode {
    DigitalChannel pin0;
    DigitalChannel pin1;
    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode(){
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();


        while(opModeIsActive()){
            telemetry.addData("digital 0", pin0.getState());
            telemetry.addData("digital 1", pin1.getState());
            telemetry.update();
        }


    }

}


