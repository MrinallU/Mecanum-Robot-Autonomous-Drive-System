package org.firstinspires.ftc.teamcode.NewRobot.Modules;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.T2_2022.Base;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;

public class Drive extends Base {

  protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;
  protected Motor odoL, odoN;
  protected SlideSystem slideSystem;
  protected IMU gyro;
  protected Odometry odometry;
  protected OpMode opMode;
  protected List<LynxModule> allHubs;

  Telemetry output;
  public boolean raise1 = false, raise2 = false, dropSlides3 = false, dropSlides4 = false;

  public Drive(
      Motor fLeftMotor,
      Motor bLeftMotor,
      Motor fRightMotor,
      Motor bRightMotor,
      Motor odoL,
      Motor odoN,
      IMU gyro,
      OpMode m,
      int xPos,
      int yPos,
      int angle,
      List<LynxModule> allHubs,
      Telemetry t,
      SlideSystem slideSystem) {
    this.fLeftMotor = fLeftMotor;
    this.fRightMotor = fRightMotor;
    this.bLeftMotor = bLeftMotor;
    this.bRightMotor = bRightMotor;
    this.odoL = odoL;
    this.odoN = odoN;
    this.gyro = gyro;
    this.opMode = m;
    this.allHubs = allHubs;
    this.output = t;
    this.slideSystem = slideSystem;
    odometry = new Odometry(xPos, yPos, angle);
  }


    public void ChaseTheCarrot(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout, double powerCap) {
        ElapsedTime time = new ElapsedTime();
        resetCache();
        odo.update();
        Pose2D pos = odo.getPosition();


        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(pos.getX(DistanceUnit.INCH) - wp.get(wp.size() - 1).xP) > error
                || Math.abs(pos.getY(DistanceUnit.INCH) - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout && opModeIsActive()) {

            odo.update();
            pos = odo.getPosition();
            resetCache();
          
            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double theta = pos.getHeading(AngleUnit.DEGREES);

            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                odo.update();

                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            telemetry.addData("target: ", destPt);
            telemetry.update();
            driveFieldCentric(-xPow, turnPow, yPow, powerCap);
        }
        stopDrive();
    }

  public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference) {
    // GM0
    double currAngle = odometry.getAngle();
    ElapsedTime time = new ElapsedTime();
    while (Math.abs(currAngle - targetAngle) > minDifference
        && time.milliseconds() < timeout
        && ((LinearOpMode) opMode).opModeIsActive()) {
      resetCache();
      updatePosition();
      currAngle = odometry.getAngle();
      double angleDiff = Angle.normalize(currAngle - targetAngle);
      double calcP = Range.clip(angleDiff * 0.01, -powerCap, powerCap);
      driveFieldCentric(0, calcP, 0);
    }

    stopDrive();
  }

  public void updatePosition() {
    odometry.updatePosition(
        -odoL.encoderReading(),
        -odoN.encoderReading(),
        getAngleImu()); // forward shold be neg perp, right neg parallel.
  }

  public double getX() {
    return getCurrentPosition().xP;
  }

  public double getY() {
    return getCurrentPosition().yP;
  }

  public double getAngleOdo() {
    return getCurrentPosition().ang;
  }

  public Point getCurrentPosition() {
    updatePosition();
    return new Point(odometry.getX(), odometry.getY(), odometry.getAngle());
  }

  public double getAngleImu() {
    return Angle.normalize(
        getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + initAng); // add init angle
  }

  public YawPitchRollAngles getRobotYawPitchRollAngles() {
    return gyro.getRobotYawPitchRollAngles();
  }

  // Driving
  public void driveFieldCentric(double drive, double turn, double strafe) {
    // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
    double fRightPow, bRightPow, fLeftPow, bLeftPow;
    double botHeading = -Math.toRadians(getAngleImu());
    System.out.println(drive + " " + turn + " " + strafe);

    double rotX = drive * Math.cos(botHeading) - strafe * Math.sin(botHeading);
    double rotY = drive * Math.sin(botHeading) + strafe * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);
    fLeftPow = (rotY + rotX + turn) / denominator;
    bLeftPow = (rotY - rotX + turn) / denominator;
    fRightPow = (rotY - rotX - turn) / denominator;
    bRightPow = (rotY + rotX - turn) / denominator;

    setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
  }

  public void driveFieldCentricAuto(double drive, double turn, double strafe) {
    // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
    double fRightPow, bRightPow, fLeftPow, bLeftPow;
    double botHeading = -Math.toRadians(getAngleOdo());
    System.out.println(drive + " " + turn + " " + strafe);

    double rotX = drive * Math.cos(botHeading) - strafe * Math.sin(botHeading);
    double rotY = drive * Math.sin(botHeading) + strafe * Math.cos(botHeading);

    double denominator = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);
    fLeftPow = (rotY + rotX + turn) / denominator;
    bLeftPow = (rotY - rotX + turn) / denominator;
    fRightPow = (rotY - rotX - turn) / denominator;
    bRightPow = (rotY + rotX - turn) / denominator;

    setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
  }

  public void driveRobotCentric(double drive, double turn, double strafe) {
    // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code

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

    setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
  }

  public void setDrivePowers(double bLeftPow, double fLeftPow, double bRightPow, double fRightPow) {
    //    if(gyro.getAngularOrientation().thirdAngle > 5){ // anti-tip
    //      dt.stopDrive();
    //      dt.setDrivePowers(-0.5, 0, -0.5, 0);
    //      return;
    //    }

    bLeftMotor.setPower(bLeftPow);
    fLeftMotor.setPower(fLeftPow);
    bRightMotor.setPower(bRightPow);
    fRightMotor.setPower(fRightPow);
  }

  public void stopDrive() {
    setDrivePowers(0, 0, 0, 0);
  }

  public String getTicks() {
    return odoL.encoderReading() + " x " + odoN.encoderReading();
  }

  public double[] scalePowers(
      double bLeftPow, double fLeftPow, double bRightPow, double fRightPow) {
    double maxPow =
        Math.max(
            Math.max(Math.abs(fLeftPow), Math.abs(bLeftPow)),
            Math.max(Math.abs(fRightPow), Math.abs(bRightPow)));
    if (maxPow > 1) {
      fLeftPow /= maxPow;
      bLeftPow /= maxPow;
      fRightPow /= maxPow;
      bRightPow /= maxPow;
    }

    return new double[] {fLeftPow, bLeftPow, fRightPow, bRightPow};
  }

  // Misc. Functions / Overloaded Method Storage

  public double getRobotDistanceFromPoint(Point p2) {
    return Math.sqrt((p2.yP - getY()) * (p2.yP - getY()) + (p2.xP - getX()) * (p2.xP - getX()));
  }

  public void update() {
    resetCache();
    updatePosition();
  }
  // BULK-READING FUNCTIONS
  public void resetCache() {
    // Clears cache of all hubs
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }

  @Override
  public void runOpMode() throws InterruptedException {}
}
