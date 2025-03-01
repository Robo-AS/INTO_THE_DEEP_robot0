package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.utils.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.WSubsystem;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector2D;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class MecanumDriveTrain extends WSubsystem implements Drivetrain {
    private static MecanumDriveTrain instance = null;
    public static CachingDcMotorEx dtFrontLeftMotor, dtFrontRightMotor, dtBackLeftMotor, dtBackRightMotor;
    public double sensor;

    double[] ws = new double[4];
    private final int frontLeft = 3, frontRight = 1, backLeft = 2, backRight = 0;


    public double ks = 0;


    //PIDs for Hang:
    private final PIDController left_pid, right_pid;
    public static double p = 0.01, i = 0, d = 0;
    public static double targetPositionLeft = 0, targetPositionRight = 0;
    public static double currentPositionLeft = 0, currentPositionRight = 0;
    public static double joystickConstantLeft = 10, joystickConstantRight = 10;
    public static double minPositionLeft = 0, maxPositionLeft = 2650;
    public static double minPositionRight = 0, maxPositionRight = 2650;

    public MecanumDriveTrain(){
        left_pid = new PIDController(p, i, d);
        right_pid = new PIDController(p, i, d);
    }


    public static MecanumDriveTrain getInstance(){
        if (instance == null) {
            instance = new MecanumDriveTrain();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        dtBackLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeftMotor"));
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtFrontLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeftMotor"));
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtBackRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRightMotor"));
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtFrontRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRightMotor"));
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        sensor = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }

    public void initialize(){
        left_pid.reset();
        right_pid.reset();
        targetPositionLeft = 0;
        targetPositionRight = 0;
        currentPositionLeft = 0;
        currentPositionRight = 0;
    }


    @Override
    public void set(Pose pose) {
        set(pose, 0);
    }

    public void set(double forwardSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {

        Vector2D input = new Vector2D(forwardSpeed, strafeSpeed).rotate(-gyroAngle);
        double actualks = ks; // *12/getVoltage();

        if(!Globals.HANGING_LEVEL_3){
            forwardSpeed = Range.clip(input.x, -1, 1);
            strafeSpeed = Range.clip(input.y, -1, 1);
            turnSpeed = Range.clip(turnSpeed, -1, 1);

            double[] wheelSpeeds = new double[4];

            wheelSpeeds[frontLeft] = (forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed - strafeSpeed + turnSpeed));
            wheelSpeeds[frontRight] = (forwardSpeed + strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed + turnSpeed);
            wheelSpeeds[backLeft] = (forwardSpeed + strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed - turnSpeed);
            wheelSpeeds[backRight] = (forwardSpeed - strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed - strafeSpeed - turnSpeed);  //(forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed - strafeSpeed + turnSpeed));

            double max = 1;
            for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


            if (max > 1) {
                wheelSpeeds[frontLeft] /= max;
                wheelSpeeds[frontRight] /= max;
                wheelSpeeds[backLeft] /= max;
                wheelSpeeds[backRight] /= max;
            }

            ws[frontLeft] = wheelSpeeds[frontLeft];
            ws[frontRight] = wheelSpeeds[frontRight];
            ws[backLeft] = wheelSpeeds[backLeft];
            ws[backRight] = wheelSpeeds[backRight];

            dtFrontLeftMotor.setPower(ws[frontLeft]);
            dtFrontRightMotor.setPower(ws[frontRight]);
            dtBackLeftMotor.setPower(ws[backLeft]);
            dtBackRightMotor.setPower(ws[backRight]);
        }

        else{
            currentPositionLeft = dtFrontLeftMotor.getCurrentPosition();
            currentPositionRight = dtFrontRightMotor.getCurrentPosition();

            left_pid.setPID(p, i, d);
            right_pid.setPID(p, i, d);
            double powerLeft = left_pid.calculate(currentPositionLeft, targetPositionLeft);
            double powerRight = right_pid.calculate(currentPositionRight, -targetPositionRight);

            dtFrontLeftMotor.setPower(powerLeft);
            dtFrontRightMotor.setPower(powerRight);

            dtBackLeftMotor.setPower(-(double) 80/100*powerLeft);
            dtBackRightMotor.setPower(-(double) 80/100*powerRight);
        }


    }


    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }


    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }


    public double getVoltage(){
        return sensor;
    }


    public static void resetEncoders(){
        dtBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dtFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dtBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dtFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    

    public double getCurrentPotionLeft(){
        return currentPositionLeft;
    }

    public double getCurrentPositionRight(){
        return currentPositionRight;
    }


    public double getTargetPositionLeft(){
        return targetPositionLeft;
    }

    public void updateTargetPositionHang(double joystickYCoord){
        targetPositionLeft += (joystickYCoord * joystickConstantLeft);
        targetPositionLeft = Math.max(minPositionLeft, Math.min(maxPositionLeft, targetPositionLeft)); //limita de prosti

        targetPositionRight += (joystickYCoord *joystickConstantRight);
        targetPositionRight = Math.max(minPositionRight, Math.min(maxPositionRight, targetPositionRight)); //limita de prosti

    }

    public void resetTargetPositions(){
        targetPositionRight = 0;
        targetPositionLeft = 0;
    }



}
