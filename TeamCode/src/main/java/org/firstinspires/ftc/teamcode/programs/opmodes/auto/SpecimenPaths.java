package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpecimenPaths {
    public boolean BRING_3_TAKE_1COMPLETED = false;
    public boolean SCORE_1_COMPLETED = false;
    public boolean SCORE_1_SMALL_COMPLETED = false;
    public boolean TAKE_2_COMPLETED = false;
    public boolean SCORE_2_COMPLETED = false;
    public boolean SCORE_2_SMALL_COMPLETED = false;
    public boolean TAKE_3_COMPLETED = false;
    public boolean SCORE_3_COMPLETED = false;
    public boolean SCORE_3_SMALL_COMPLETED = false;
    public boolean TAKE_4_COMPLETED = false;
    public boolean SCORE_4_COMPLETED = false;
    public boolean SCORE_4_SMALL_COMPLETED = false;
    public boolean SCORE_5_SMALL_COMPLETED = false;
    public boolean SCORE_6_SMALL_COMPLETED = false;
    public boolean SCORE_7_SMALL_COMPLETED = false;
    public boolean SCORE_8_SMALL_COMPLETED = false;
    public boolean SCORE_9_SMALL_COMPLETED = false;


    public void resetTrajectoryes(){
        BRING_3_TAKE_1COMPLETED = false;
        SCORE_1_COMPLETED = false;
        SCORE_1_SMALL_COMPLETED = false;
        TAKE_2_COMPLETED = false;
        SCORE_2_COMPLETED = false;
        SCORE_2_SMALL_COMPLETED = false;
        TAKE_3_COMPLETED = false;
        SCORE_3_COMPLETED = false;
        SCORE_3_SMALL_COMPLETED = false;
        TAKE_4_COMPLETED = false;
        SCORE_4_COMPLETED = false;
        SCORE_4_SMALL_COMPLETED = false;
        SCORE_5_SMALL_COMPLETED = false;
        SCORE_6_SMALL_COMPLETED = false;
        SCORE_7_SMALL_COMPLETED = false;
        SCORE_8_SMALL_COMPLETED = false;
        SCORE_9_SMALL_COMPLETED = false;
    }


    public void setBring3Take1Completed(){ BRING_3_TAKE_1COMPLETED = true;}
    public void setScore1Completed(){ SCORE_1_COMPLETED = true;}
    public void setScore1SmallCompleted(){ SCORE_1_SMALL_COMPLETED = true;}
    public void setTake2Completed(){ TAKE_2_COMPLETED = true;}
    public void setScore2Completed(){ SCORE_2_COMPLETED = true;}
    public void setScore2SmallCompleted(){ SCORE_2_SMALL_COMPLETED = true;}
    public void setTake3Completed(){ TAKE_3_COMPLETED = true;}
    public void setScore3Completed(){ SCORE_3_COMPLETED = true;}
    public void setScore3SmallCompleted(){ SCORE_3_SMALL_COMPLETED = true;}
    public void setTake4Completed(){ TAKE_4_COMPLETED = true;}
    public void setScore4Completed(){ SCORE_4_COMPLETED = true;}
    public void setScore4SmallCompleted(){ SCORE_4_SMALL_COMPLETED = true;}
    public void setScore5SmallCompleted(){ SCORE_5_SMALL_COMPLETED = true;}
    public void setScore6SmallCompleted(){ SCORE_6_SMALL_COMPLETED = true;}
    public void setScore7SmallCompleted(){ SCORE_7_SMALL_COMPLETED = true;}
    public void setScore8SmallCompleted(){ SCORE_8_SMALL_COMPLETED = true;}
    public void setScore9SmallCompleted(){ SCORE_9_SMALL_COMPLETED = true;}



    public boolean getBring3Take1Completed(){ return BRING_3_TAKE_1COMPLETED; }
    public boolean getScore1Completed(){ return SCORE_1_COMPLETED;}
    public boolean getScore1SmallCompleted(){ return SCORE_1_SMALL_COMPLETED;}
    public boolean getTake2Completed(){ return  TAKE_2_COMPLETED;}
    public boolean getScore2Completed(){ return SCORE_2_COMPLETED;}
    public boolean getScore2SmallCompleted(){ return SCORE_2_SMALL_COMPLETED;}
    public boolean getTake3Completed(){ return TAKE_3_COMPLETED;}
    public boolean getScore3Completed(){ return SCORE_3_COMPLETED;}
    public boolean getScore3SmallCompleted(){ return SCORE_3_SMALL_COMPLETED;}
    public boolean getTake4Completed(){ return TAKE_4_COMPLETED;}
    public boolean getScore4Completed(){ return SCORE_4_COMPLETED;}
    public boolean getScore4SmallCompleted(){ return SCORE_4_SMALL_COMPLETED;}
    public boolean getScore5SmallCompleted(){ return SCORE_5_SMALL_COMPLETED;}
    public boolean getScore6SmallCompleted(){ return SCORE_6_SMALL_COMPLETED;}
    public boolean getScore7SmallCompleted(){ return SCORE_7_SMALL_COMPLETED;}
    public boolean getScore8SmallCompleted(){ return SCORE_8_SMALL_COMPLETED;}
    public boolean getScore9SmallCompleted(){ return SCORE_9_SMALL_COMPLETED;}


    public boolean allTrajectoriesCompleted(){
        return BRING_3_TAKE_1COMPLETED && SCORE_1_COMPLETED && TAKE_2_COMPLETED && SCORE_2_COMPLETED &&
                TAKE_3_COMPLETED && SCORE_3_COMPLETED && TAKE_4_COMPLETED && SCORE_4_COMPLETED;
    }


}
