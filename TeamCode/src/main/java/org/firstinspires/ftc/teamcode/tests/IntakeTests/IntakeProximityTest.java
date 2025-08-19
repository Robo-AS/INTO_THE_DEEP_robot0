package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


//@Disabled
@TeleOp(name="Intake_PROXIMITY_Test", group = "Tests")
public class IntakeProximityTest extends CommandOpMode {
    private GamepadEx gamepadEx;
    private final Intake intake = Intake.getInstance();

    private boolean state;
    private int counter = 0;
    @Override
    public void initialize(){
        state = true;
        counter = 0;
        intake.initializeHardware(hardwareMap);
//        CommandScheduler.getInstance().reset();

    }

    @Override
    public void run(){
        //CommandScheduler.getInstance().run();
//        intake.updateSampleColor();
//        intake.updateSampleStateDigital();
//
//        telemetry.addData("Color:", intake.intakedSampleColor);
//        telemetry.addData("State:", intake.sampleState);

        state = intake.proximitySensor.getState();
        if(state == true)
            counter++;

        telemetry.addData("Proximity:", state);
        telemetry.addData("Counter:", counter);
        telemetry.update();



    }
}
