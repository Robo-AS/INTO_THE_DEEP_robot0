package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.util.Robot;

@TeleOp(name = "🚨EMERGENCY🚨")
public class EMERGENCY extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    public GamepadEx gamepadEx;


    @Override
    public void initialize() {
        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();
    }



    @Override
    public void run(){
        double powerLift = Math.max(-0.5, Math.min(0.5, gamepad1.left_stick_y));
        robot.lift.liftMotor.setPower(powerLift);
        robot.lift.followerMotor.setPower(powerLift);

        double powerExtendo = Math.max(-0.5, Math.min(0.5, gamepad1.right_stick_y));
        robot.extendo.extendoMotor.setPower(powerExtendo);



        telemetry.addData("left_stick", powerLift);
        telemetry.addData("right_stick", powerExtendo);
        telemetry.update();
    }
}
