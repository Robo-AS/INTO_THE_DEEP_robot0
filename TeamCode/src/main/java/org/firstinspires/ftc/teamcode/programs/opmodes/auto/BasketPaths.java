package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
public class BasketPaths {
    public boolean GRAB_1_COMPLETED = false;
    public boolean GRAB_2_COMPLETED = false;
    public boolean GRAB_3_COMPLETED = false;

    public boolean SCORE_1_COMPLETED = false;
    public boolean SCORE_2_COMPLETED = false;
    public boolean SCORE_3_COMPLETED = false;

    public void resetTrajectoryes(){
        GRAB_1_COMPLETED = false;
        GRAB_2_COMPLETED = false;
        GRAB_3_COMPLETED = false;
    }

    public void setGrab2Completed(){ GRAB_2_COMPLETED = true; }
    public void setGrab3Completed(){ GRAB_3_COMPLETED = true; }
    public void setScore1Completed(){ SCORE_1_COMPLETED = true; }
    public void setScore2Completed(){ SCORE_2_COMPLETED = true; }
    public void setScore3Completed(){ SCORE_3_COMPLETED = true; }


    public boolean getscore1(){ return SCORE_1_COMPLETED; }
    public boolean getscore2(){ return SCORE_2_COMPLETED; }
    public boolean getscore3(){ return SCORE_3_COMPLETED; }


}
