package org.firstinspires.ftc.teamcode.programs.util;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class Robot {
    private static Robot instance = null;
    private HardwareMap hardwareMap;

    public Brush brush;
    public Extendo extendo;

    private Robot(){
        brush = Brush.getInstance();
        extendo = Extendo.getInstance();
    }

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        brush.initializeHardware(hardwareMap);
        extendo.initializeHardware(hardwareMap);

    }

    public void initializeRobot() {
        brush.initialize();
        extendo.initialize();
    }





}
