package org.firstinspires.ftc.teamcode.programs.util;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;

import java.util.List;

public class NEWRobot {
    private static NEWRobot instance = null;
    private HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    public Intake intake;
    public Extendo extendo;
    public Lift lift;
    public MecanumDriveTrain mecanumDriveTrain;
    public Arm arm;
    public Hang hang;
    public Sweeper sweeper;
    public Limelight limelightCamera;






    private NEWRobot(){
        intake = Intake.getInstance();
        extendo = Extendo.getInstance();
        lift = Lift.getInstance();
        mecanumDriveTrain = MecanumDriveTrain.getInstance();
        arm = Arm.getInstance();
        hang = Hang.getInstance();
        sweeper = Sweeper.getInstance();
        limelightCamera = Limelight.getInstance();
    }

    public static NEWRobot getInstance() {
        if (instance == null) {
            instance = new NEWRobot();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        intake.initializeHardware(hardwareMap);
        extendo.initializeHardware(hardwareMap);
        lift.initializeHardware(hardwareMap);
        mecanumDriveTrain.initializeHardware(hardwareMap);
        arm.initializeHardware(hardwareMap);
        hang.initializeHardware(hardwareMap);
        sweeper.initializeHardware(hardwareMap);
//        limelightCamera.initializeHardware(hardwareMap);


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void initializeRobot() {
        intake.initialize();
        extendo.initialize();
        lift.initialize();
        mecanumDriveTrain.initialize();
        arm.initialize();
        hang.initialize();
        sweeper.initialize();
        //limelightCamera.initialize();
    }


    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }


}

