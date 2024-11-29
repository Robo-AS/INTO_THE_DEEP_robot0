package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake extends SubsystemBase {
    private HardwareMap hardwareMap;
    private Extendo extendo = new Extendo();
    private Brush brush = new Brush();


    public enum IntakedSampleColor {
        RED,
        BLUE,
        YELLOW;
    }

    public enum DesiredSampleColor {
        YELLOW,
        BLUE,
        RED,
        BOTH;

    }

    public DesiredSampleColor desiredSampleColor = DesiredSampleColor.BOTH;;
    public Brush.BrushAngle brushAngle = Brush.BrushAngle.UP;;
    public Brush.BrushState brushState = Brush.BrushState.IDLE;;



    public void initializeHardware(){
        extendo.initializeHarware(hardwareMap);
        brush.initializeHardware(hardwareMap);
    }

    public void initialize() {

    }

    public void setDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
    }

    public DesiredSampleColor getDesiredSampleColor(){
        return desiredSampleColor;
    }







}
