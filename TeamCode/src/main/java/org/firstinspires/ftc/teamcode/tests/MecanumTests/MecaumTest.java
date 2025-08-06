package org.firstinspires.ftc.teamcode.tests.MecanumTests;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.geometry.PoseRR;


//@TeleOp(name = "MECANUM_TEST", group = "Tests")
public class MecaumTest extends CommandOpMode {
    private final MecanumDriveTrain mecanumDriveTrain = MecanumDriveTrain.getInstance();
    private GamepadEx gamepadEx;

    @Override
    public void initialize() {
        //CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);

        mecanumDriveTrain.initializeHardware(hardwareMap);
    }


    @Override
    public void run(){
        //CommandScheduler.getInstance().run();

        double turnSpeed = -gamepad1.left_stick_x;
        PoseRR drive = new PoseRR(-gamepad1.right_stick_x, -gamepad1.right_stick_y, turnSpeed);
        mecanumDriveTrain.set(drive, 0);
    }
}
