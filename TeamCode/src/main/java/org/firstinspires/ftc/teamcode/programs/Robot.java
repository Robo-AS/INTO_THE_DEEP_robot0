package org.firstinspires.ftc.teamcode.programs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;



public class Robot {

    public Intake intake = new Intake();


    public void initializeRobotHadrware(){
        intake.initializeHardware();
    }

    public void initializeRobot(){
        intake.initialize();
    }

}
