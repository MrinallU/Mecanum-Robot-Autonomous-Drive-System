package org.firstinspires.ftc.teamcode.State_Championship_2022;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;
import org.firstinspires.ftc.teamcode.Utils.RoadRunner.drive.Mecanum;

import java.util.List;
import java.util.Locale;

public abstract class Base extends LinearOpMode {
    // Lynx Modules
    List<LynxModule> allHubs;

    // Motors
    protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;

    // Servos

    // Servo Positions


    // Testing Mode
    boolean testing = false;

    // Sleep Times
    ElapsedTime matchTime = new ElapsedTime();
    ElapsedTime pollTime, cTime;
    int testingSleep = 200, normalSleep = 10;

    // Gyro and Angles
    public BNO055IMU gyro;
    double startAngle;

    // Constants and Conversions

    // Other
    boolean isRunning;

    //Modules
    Mecanum driveTrain;

    public void initHardware(int type){
        // Setting up Hubs (Manual Caching)
        allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //  Motor
        fLeftMotor = new Motor(hardwareMap, "fLeft");
        bLeftMotor = new Motor(hardwareMap, "bLeft");
        fRightMotor = new Motor(hardwareMap, "fRight");
        bRightMotor = new Motor(hardwareMap, "bRight");

        // Servos

        // Misc
        isRunning = true;

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        driveTrain = new Mecanum(hardwareMap);
    }

    public void initServos(){

    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double getAngle(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //ZYX is Original
        return angles.firstAngle;
    }

    // MULTI-USE FUNCTIONS
    public void driveFieldCentric(double baseAngle, double drive, double turn, double strafe){
        double fRightPow, bRightPow, fLeftPow, bLeftPow;

        double bLeftAngle = Math.toRadians(baseAngle + 135);
        double fLeftAngle = Math.toRadians(baseAngle + 45);
        double bRightAngle = Math.toRadians(baseAngle + 225);
        double fRightAngle = Math.toRadians(baseAngle + 315);

        fRightPow = (drive * Math.sin(fRightAngle) + strafe * Math.cos(fRightAngle)) / Math.sqrt(1) + turn;
        bRightPow = (drive * Math.sin(bRightAngle) + strafe * Math.cos(bRightAngle)) / Math.sqrt(1) + turn;
        fLeftPow = (drive * Math.sin(fLeftAngle) + strafe * Math.cos(fLeftAngle)) / Math.sqrt(1) + turn;
        bLeftPow = (drive * Math.sin(bLeftAngle) + strafe * Math.cos(bLeftAngle)) / Math.sqrt(1) + turn;

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];

        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    }

    public void driveRobotCentric(double drive, double turn, double strafe){
        double fRightPow = 0, bRightPow = 0, fLeftPow = 0, bLeftPow = 0;

        fLeftPow = -drive + turn - strafe;
        bLeftPow = -drive + turn + strafe;
        fRightPow = drive + turn - strafe;
        bRightPow = drive + turn + strafe;

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];

        setDrivePowers(bLeftPow, fLeftPow, bRightPow,fRightPow);
    }

    public double[] scalePowers (double bLeftPow, double fLeftPow, double bRightPow, double fRightPow){
        double maxPow = Math.max(Math.max(Math.abs(fLeftPow), Math.abs(bLeftPow)), Math.max(Math.abs(fRightPow), Math.abs(bRightPow)));
        if (maxPow > 1) {
            fLeftPow /= maxPow;
            bLeftPow /= maxPow;
            fRightPow /= maxPow;
            bRightPow /= maxPow;
        }

        return new double[] {fLeftPow, bLeftPow, fRightPow, bRightPow};
    }

    public void setDrivePowers(double bLeftPow, double fLeftPow, double bRightPow, double fRightPow){
        bLeftMotor.setPower(bLeftPow);
        fLeftMotor.setPower(fLeftPow);
        bRightMotor.setPower(bRightPow);
        fRightMotor.setPower(fRightPow);
    }

    // AUTONOMOUS/TESTING FUNCTIONS
    public void rest(){
        rest(testing ? testingSleep : normalSleep);
    }

    public void rest(int time){
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void turnTo(double targetAngle){
        driveTrain.turn(targetAngle);
    }

    // ODOMETRY FUNCTIONS
    private final double distanceToPowerScale = 0.03;
    private final double angleToPowerScale = 0.01;



    private final double max_i = 0.01;

    // Spline PID Coefficients
    private final double k_p_s = 0.075;
    private final double k_d_s = 0;
    private final double k_i_s = 0;

    // Normal moveToPosition PID Coefficients
    private final double k_p = 0.055;
    private final double k_d = 0;
    private final double k_i = 0;

    public Trajectory moveToPosition(double targetXPos, double targetYPos, double targetAngle){
        Pose2d start = driveTrain.getPoseEstimate();
        Trajectory traj = driveTrain.trajectoryBuilder(start)
                .lineToSplineHeading(new Pose2d(targetXPos, targetYPos, Math.toRadians(targetAngle)))
                .build();
        driveTrain.followTrajectory(traj);
        return traj;
    }

    public void splineTo(Point [] wp){
        Pose2d currP = driveTrain.getPoseEstimate();
        for (Point p: wp
             ) {
            Trajectory traj = driveTrain.trajectoryBuilder(currP)
                    .lineToSplineHeading(new Pose2d(p.xP, p.xP, Math.toRadians(p.ang)))
                    .build();
            driveTrain.followTrajectory(traj);
            currP = traj.end();
        }
    }

    public double distanceToPower(double dist) {
        return dist * distanceToPowerScale;
    }

    public double angleToPower(double angle) {
        return angle * angleToPowerScale;
    }

    // BULK-READING FUNCTIONS
    public void resetCache(){
        // Clears cache of all hubs
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }


    // Other Functions
    public double normalizeThreeDigits(double d){
        return (int)(d * 1000) / 1000.;
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
