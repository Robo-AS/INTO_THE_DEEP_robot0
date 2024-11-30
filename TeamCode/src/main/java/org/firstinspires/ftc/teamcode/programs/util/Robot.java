package org.firstinspires.ftc.teamcode.programs.util;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;



public class Robot {

    public Intake intake = new Intake();


    public void initializeRobotHadrware(){
        intake.initializeHardware();
    }

    public void initializeRobot(){
        intake.initialize();
    }

}
