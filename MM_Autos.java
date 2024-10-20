package org.firstinspires.ftc.teamcode.MM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autos", group="mm")
public class MM_Autos extends MM_OpMode{

    @Override
    public void runProcedures(){

    }

    public void chamber(){
        robot.drivetrain.driveInches(-30, 0);
        //TODO add the rest of chamber code
    }

    public void basket() {


    }
}
