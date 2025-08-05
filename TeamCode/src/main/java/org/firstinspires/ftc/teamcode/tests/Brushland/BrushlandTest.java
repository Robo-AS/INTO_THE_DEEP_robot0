package org.firstinspires.ftc.teamcode.tests.Brushland;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name = "BrushlandTest", group = "Tests")
public class BrushlandTest extends LinearOpMode {
    DigitalChannel pin0;
    DigitalChannel pin1;
    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean pin0_value;
    private boolean pin1_value;
    private int yellow_counter=0;

    @Override
    public void runOpMode(){
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        yellow_counter = 0;
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();


        while(opModeIsActive()){
            pin0_value = pin0.getState();
            pin1_value = pin1.getState();

            if(pin0_value && pin1_value){
                yellow_counter++;
            }
            telemetry.addData("digital 0", pin0_value);
            telemetry.addData("digital 1", pin1_value);
            telemetry.addData("IS YELLOW", yellow_counter);


            telemetry.update();
        }


    }

}


