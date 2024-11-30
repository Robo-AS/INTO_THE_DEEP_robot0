package org.firstinspires.ftc.teamcode.programs.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake extends SubsystemBase {
    private HardwareMap hardwareMap;
    public Extendo extendo = new Extendo();
    public Brush brush = new Brush();






    public void initializeHardware(){
        extendo.initializeHarware(hardwareMap);
        brush.initializeHardware(hardwareMap);
    }

    public void initialize() {

    }









}
