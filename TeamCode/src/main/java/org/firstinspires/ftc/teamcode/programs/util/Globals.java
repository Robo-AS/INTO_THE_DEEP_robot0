package org.firstinspires.ftc.teamcode.programs.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static double BRUSH_MOTOR_SPEED = 1;//0.75
    public static double BRUSH_SAMPLE_SERVO_SPEED_INTAKING= 0.57; //0.8
    public static double BRUSH_SAMPLE_SERVO_SPEED_THROWING = 1;
    public static double BRUSH_POSITION_UP = 0.72;
    public static double BRUSH_POSITION_DOWN = 0.05;


    // Mecanum
    public static double DECREASE_TURN_SPEED_CONSTANT = 2;

    //Extendo
    public static double EXTENDO_JOYSTICK_CONSTANT_UP = 20;
    public static double EXTENDO_JOYSTICK_CONSTANT_DOWN = 50;


    //Gamepad
    public static double rumble1 = 1, rumble2 = 1;
    public static int durationMs = 200;
    public static boolean shouldVibrate = false;


    //Hang
    public static boolean HANGING_LEVEL_2 = false;
    public static boolean HANGING_LEVEL_3 = false;


    public static void isHangingLevel2(){
        HANGING_LEVEL_2 = true;
    }
    public static void isHangingLevel3(){
        HANGING_LEVEL_3 = true;

    }

    public static void isNotHanging(){
        HANGING_LEVEL_3 = false;

    }


}
