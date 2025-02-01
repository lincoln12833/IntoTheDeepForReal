package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.config.Config;


public class MM_CONSTANTS {
    @Config
    public static class ASCENT_CONSTANTS{
        public static double ASCENT_LVL_1 = 0.6;
        public static double FOLD_POSITION = .95;
    }

    @Config
    public static class DRIVE_CONSTANTS{
        public static double BASE_ROTATE_FACTOR = 0.036;
        public static double TANGENT_THRESHOLD = 0.5;
        public static double HEADING_ERROR_THRESHOLD = 2;
        public static  double DRIVE_ERROR_THRESHOLD = 0;
    }

    @Config
    public static class COLLECT_CONSTANTS{
        public static double COLLECT_BASE_POWER = .4;
        public static int SCORE_POWER = 1;
        public static double COLLECT_POWER_EFFECTOR = .4;
    }

    @Config
    public static class PID_CONSTANTS{
        public static double P_CO_EFF = .4;
        public static double I_CO_EFF = 0;
        public static double D_CO_EFF = 0;
    }
}
