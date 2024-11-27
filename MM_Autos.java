package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autos", group="mm")
public class MM_Autos extends MM_OpMode{

    @Override
    public void runProcedures(){
        if (goal.equals(CHAMBER)){
            chamber();
        } else {
            basket();
        }
    }

    public void chamber(){
        robot.drivetrain.driveToDistance(2, 90, 20, false,  false);
        robot.transport.slide.setTargetInches(36);
        robot.drivetrain.driveInches(12, 179);
        robot.transport.home();
        robot.drivetrain.driveInches(25, 179);
        robot.drivetrain.strafeInches(38, 179);
        //TODO TEST
    }

    public void basket() {
        robot.drivetrain.strafeInches(-14.5, -45, 90, robot.transport.slide.MAX_TICKS, true, false);
        robot.collector.score();
        robot.transport.home();
        robot.drivetrain.rotateToAngle(-90);
        robot.drivetrain.strafeInches(46, -90);
        robot.drivetrain.driveToDistance(1, 90, 8, false, false);
        //TODO TEST
    }

}
