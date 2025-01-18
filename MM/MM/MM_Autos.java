package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autos", group = "mm")
public class MM_Autos extends MM_OpMode {

    @Override
    public void runProcedures() {
        if (goal.equals(CHAMBER)) {
            chamber();
        } else {
            robot.navigation.setPosition(-36, -60, 60);
            basket();
        }
    }

    public void chamber() {
        //    robot.drivetrain.driveInches(24, 0);

//        robot.drivetrain.driveInches(4, 0);
//        robot.drivetrain.rotateToAngle(179);
//        robot.drivetrain.driveToDistance(2.2, 90, 20, false,  false);
//        robot.transport.slide.setTargetInches(36);
//        robot.drivetrain.driveInches(12, 179);
//        robot.transport.home();
//        robot.drivetrain.driveInches(25, 179);
//        robot.drivetrain.strafeInches(38, 179);
        //TODO TEST
    }

    public void basket() {
        driveToBasketAndScore();

        //collect
        robot.drivetrain.driveToPosition(-46.6, -42.6, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
        driveToBasketAndScore();

        //second collect
        robot.drivetrain.driveToPosition(-56.6, -42.6, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
        driveToBasketAndScore();

        //third collect
        //robot.drivetrain.driveToPosition(-56, -40.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 6, false, true);
        driveToBasketAndScore();
        //pre-penfield code (before inch change)

//        //robot.drivetrain.doEverything(5, -8, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
//        robot.drivetrain.strafeInches(-8, -45, 90, robot.transport.slide.MAX_TICKS, true, false); //TODO test to see if doEverything() works then delete these lines
//        robot.drivetrain.driveInches(-5, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
////        sleep(30000); hihihihihihi
//        robot.collector.score();
//        robot.drivetrain.driveInches(6, -45, 90, 14, false, false);
//        robot.drivetrain.driveInches(7, 0, -13, 7.5, false, true);
//        if(robot.collector.haveSample()) {
//            robot.drivetrain.driveInches(-13, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
//            robot.collector.score();
//            robot.drivetrain.driveInches(13, 0, -5, 5, false, false);
//        } else {
//            //robot.drivetrain.driveInches(-2, 0, -5, 8.5, false, false);
//            robot.drivetrain.strafeInches(-2, 0);
//        }
//        robot.drivetrain.strafeInches(-7.5, 0, -5, 8.5, false, false);
//        robot.drivetrain.strafeInches(-.1, 0, -12, 8.5, false, true);
//        robot.drivetrain.strafeInches(6.5, 0, 90, robot.transport.slide.MAX_TICKS, true, false);
//        robot.drivetrain.driveInches(-11.5, -50);
//        robot.collector.score();
//        robot.drivetrain.driveInches(13, 29, 90, 11, false, false);
//        robot.drivetrain.driveInches(.2, 29, -12, 8, false, true);
////        robot.drivetrain.rotateToAngle(-90);
//        robot.drivetrain.strafeInches(46, -90);
//        robot.drivetrain.driveToDistance(1, 90, 8, false, false);
        //TODO TEST
    }

    public void driveToBasketAndScore() {
        if (robot.collector.haveSample()) {
            robot.drivetrain.driveToPosition(-54.7, -57, 32, MM_TestTrigNav.ROTATE_FACTOR, 93, 52, false, false);
            robot.collector.score();
        }
    }

}
