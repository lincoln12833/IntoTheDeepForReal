package org.firstinspires.ftc.teamcode.MM.MM;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MM_CONSTANTS {
    public static class ASCENT_CONSTANTS{
        public static double ASCENT_LVL_1 = .7;
        public static double FOLD_POSITION = 0;
    }

    public static class DRIVE_CONSTANTS{
        public static double BASE_ROTATE_FACTOR = 0.036;
        public static double TANGENT_THRESHOLD = 0.5;
        public static double HEADING_ERROR_THRESHOLD = 2;
        public static  double DRIVE_ERROR_THRESHOLD = .35;
    }

    public static class COLLECT_CONSTANTS{
        public static double COLLECT_BASE_POWER = .6;
        public static int SCORE_POWER = 1;
        public static double COLLECT_POWER_EFFECTOR = .4;
    }
}
