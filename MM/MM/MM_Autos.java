package org.firstinspires.ftc.teamcode.MM.MM;

import static org.firstinspires.ftc.teamcode.MM.MM.MM_CONSTANTS.DRIVE_CONSTANTS.BASE_ROTATE_FACTOR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autos", group = "mm")
public class MM_Autos extends MM_OpMode {
    public ElapsedTime aprilTagTime = new ElapsedTime();


    @Override
    public void runProcedures() {
        if (goal.equals(CHAMBER)) {
            chamber();
        } else {
            //robot.navigation.setPosition(-36, -60, 60);
            basket();
        }
    }

    public void chamber() {
        robot.drivetrain.driveToPosition(33 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038,  -1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(40 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038, -1,93, 1, false, false);
        robot.drivetrain.driveToPosition(40 * alliance, -55 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(40 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(50 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(50 * alliance, -55 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(50 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(55 * alliance, -12.58 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);
        robot.drivetrain.driveToPosition(55 * alliance, -55 * alliance, .5, alliance == 1 ? 0 : 180, .038,-1, 93, 1, false, false);


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

        lookAtAprilTag();

        multipleTelemetry.addData("Status", "Trying to Collect");
        robot.drivetrain.driveToPosition(-46.6 * alliance, -39.5 * alliance, .2, alliance == 1 ? 91.5 : 91.5 + 180, BASE_ROTATE_FACTOR, .2, -13.2, 5.5, false, true);
        //multipleTelemetry.addData("Status", "re-align collect");

        //robot.drivetrain.driveToPosition(-46.6, -39.5, 91.5, MM_TestTrigNav.ROTATE_FACTOR, -13.2, 5.5, false, true);
        driveToBasketAndScore();

        lookAtAprilTag();

        multipleTelemetry.addData("Status", "Trying to Collect");
        robot.drivetrain.driveToPosition(-56.6 * alliance, -39.5 * alliance, .4, alliance == 1 ? 91.5 : 91.5 + 180, BASE_ROTATE_FACTOR,.2,  -13.2, 5.5, false, true);

        driveToBasketAndScore();
        robot.drivetrain.driveToPosition(-35.78 * alliance, -7 * alliance, .5, alliance == 1 ? 180 : 180 + 180, BASE_ROTATE_FACTOR, .2, 90, 1, false, false);

    }

    private void lookAtAprilTag() {
        multipleTelemetry.addData("Status", "Driving to April Tag");
        robot.drivetrain.driveToPosition(-47 * alliance, -47 * alliance, .5, alliance == 1 ? 90 : 90 + 180, .043, -1, 0, 5.5, false, false);

        aprilTagTime.reset();
        while (opModeIsActive() && aprilTagTime.milliseconds() <= 100) {
            multipleTelemetry.addData("Status", "looking, at apriltag");
            robot.navigation.updatePosition(true);
            multipleTelemetry.update();
        }
    }

    public void driveToBasketAndScore() {
        if (robot.collector.haveSample()) {
            multipleTelemetry.addData("Status", "Trying to Score");
            robot.drivetrain.driveToPosition(-53.7 * alliance, -55 * alliance, .5, alliance == 1 ? 32 : 32 + 180, BASE_ROTATE_FACTOR, -1, 93, 52, false, false);
            robot.collector.score();
        }
    }

}
