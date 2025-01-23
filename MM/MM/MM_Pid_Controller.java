package org.firstinspires.ftc.teamcode.MM.MM;

public class MM_Pid_Controller {
    private final MM_OpMode opMode;

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
        driveIntegral += driveIn;
        driveDerivative = driveIn - lastDrivePower;
        strafeDerivative = strafeIn - lastStrafepower;

        lastDrivePower = driveIn;
        lastStrafepower = strafeIn;

        powers[0] = driveIn + driveIntegral + driveDerivative;
        powers[1] = strafeIn + strafeIntegral + driveDerivative;

        return powers;
    }
}
