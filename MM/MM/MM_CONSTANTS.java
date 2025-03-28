package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.config.Config;


public class MM_CONSTANTS {
    @Config
    public static class ASCENT_CONSTANTS{
        public static double ASCENT_LVL_1 = .1; //2-7-2025 was .6
        public static double FOLD_POSITION = .95;
        public static double TICK_INCREMENT = 100;
        public static int OPERATION = 1;
    }

    @Config
    public static class DRIVE_CONSTANTS{
        public static double DRIVE_P_COEFF = 0.04; //prev 0.03125
        public static double BASE_ROTATE_FACTOR = 0.036;
        public static double TANGENT_THRESHOLD = 0.5;
        public static double HEADING_ERROR_THRESHOLD = 2;
        public static  double DRIVE_ERROR_THRESHOLD = 4;
    }

    @Config
    public static class COLLECT_CONSTANTS{
        public static double COLLECT_BASE_POWER = 1; // 2-7-2025 (and 2-15) was .37
        public static int SCORE_POWER = 1;
        public static double COLLECT_POWER_EFFECTOR = .4;
        public static double GRAB_POS = .78;
        public static double SPEC_OPEN_POS = .5;

    }
}
