package org.firstinspires.ftc.teamcode.programs.util;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;

public class Robot {
    private static Robot instance = null;
    private HardwareMap hardwareMap;

    public Brush brush;
    public Extendo extendo;
    public Lift lift;
    public MecanumDriveTrain mecanumDriveTrain;
    public Arm arm;


//    public double sensor;
    public LynxModule chub;

    private Robot(){
        brush = Brush.getInstance();
        extendo = Extendo.getInstance();
        lift = Lift.getInstance();
        mecanumDriveTrain = MecanumDriveTrain.getInstance();
        arm = Arm.getInstance();
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
        lift.initializeHardware(hardwareMap);
        mecanumDriveTrain.initializeHardware(hardwareMap);
        arm.initializeHardware(hardwareMap);
//        sensor = hardwareMap.voltageSensor.iterator().next().getVoltage();

        chub = hardwareMap.get(LynxModule.class, "Control Hub");
        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }

    public void initializeRobot() {
        brush.initialize();
        extendo.initialize();
        lift.initialize();
        arm.initialize();
    }


//    public double getVoltage(){
//        return sensor;
//    }


    public void loop(){
        chub.clearBulkCache();
    }



}
