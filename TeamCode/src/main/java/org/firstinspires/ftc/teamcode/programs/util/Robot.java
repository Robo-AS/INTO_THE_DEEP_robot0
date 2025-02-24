package org.firstinspires.ftc.teamcode.programs.util;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;

import java.util.List;

public class Robot {
    private static Robot instance = null;
    private HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    public Brush brush;
    public Extendo extendo;
    public Lift lift;
    public MecanumDriveTrain mecanumDriveTrain;
    public Arm arm;
    public Hang hang;
    public Sweeper sweeper;




    private Robot(){
        brush = Brush.getInstance();
        extendo = Extendo.getInstance();
        lift = Lift.getInstance();
        mecanumDriveTrain = MecanumDriveTrain.getInstance();
        arm = Arm.getInstance();
        hang = Hang.getInstance();
        sweeper = Sweeper.getInstance();
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
        hang.initializeHardware(hardwareMap);
        sweeper.initializeHardware(hardwareMap);


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public void initializeRobot() {
        brush.initialize();
        extendo.initialize();
        lift.initialize();
        arm.initialize();
        hang.initialize();
        sweeper.initialize();
    }




    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isSpecimenBlocked(){
        CurrentUnit currentUnit = CurrentUnit.AMPS;
        return brush.brushMotor.getCurrent(currentUnit) > 5;
    }



}
