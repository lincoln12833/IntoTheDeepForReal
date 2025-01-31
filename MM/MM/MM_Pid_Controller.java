package org.firstinspires.ftc.teamcode.MM.MM;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Pid_Controller {
    private final MM_OpMode opMode;

    private final ElapsedTime pidTime = new ElapsedTime();
    private double driveIntegral;
    private double strafeIntegral;
    private double driveDerivative;
    private double strafeDerivative;
    private double lastDrivePower;
    private double lastStrafepower;
    double[] powers = new double[2];

    public MM_Pid_Controller(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public double[] calculatePowers(double driveIn, double strafeIn){
//        driveIntegral += driveIn / pidTime.milliseconds();
//        strafeIntegral += strafeIn / pidTime.milliseconds();
        driveDerivative = (driveIn - lastDrivePower) / pidTime.milliseconds();
        strafeDerivative = (strafeIn - lastStrafepower) / pidTime.milliseconds();
        pidTime.reset();

        lastDrivePower = driveIn;
        lastStrafepower = strafeIn;


        powers[0] = driveIn * MM_CONSTANTS.PID_CONSTANTS.P_CO_EFF; //+ driveIntegral + driveDerivative;
        powers[1] = strafeIn * MM_CONSTANTS.PID_CONSTANTS.P_CO_EFF; //+ strafeIntegral + strafeDerivative;

        return powers;
    }
}
