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

  // Very similar to the carrot chasing algo (https://arxiv.org/abs/2012.13227)
  public void ChaseTheCarrot(
      ArrayList<Point> wp,
      double switchTolerance,
      double heading,
      double error,
      double angleError,
      double normalMovementConstant,
      double finalMovementConstant,
      double turnConstant,
      double maxSpeed,
      double movementD,
      double turnD,
      double timeout) {
    ElapsedTime time = new ElapsedTime();
    double prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, dx, dy, dtheta;
    int pt = 0;
    time.reset();
    while ((pt < wp.size() - 1
            || (Math.abs(getX() - wp.get(wp.size() - 1).xP) > error
                || Math.abs(getY() - wp.get(wp.size() - 1).yP) > error
                || (heading == Double.MAX_VALUE
                    ? Math.abs(wp.get(wp.size() - 1).ang - odometry.getAngle()) > angleError
                    : Math.abs(heading - odometry.getAngle()) > angleError)))
        && time.milliseconds() < timeout) {
      update();
      double x = getX();
      double y = getY();
      double theta = odometry.getAngle();
      if (raise1 && time.milliseconds() > 2500) {
        slideSystem.extendVerticalSlides();
      }
      if (raise2 && time.milliseconds() > 1000) {
        slideSystem.extendVerticalSlides();
      }
      Point destPt;
      while (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance
          && pt != wp.size() - 1
          && time.milliseconds() <= timeout) {
        update();
        pt++;
        x = getX();
        y = getY();
        prevXDiff = 0;
        prevYDiff = 0;
        prevAngleDiff = 0;
      }

      /*
            splineAngle = Math.atan2(yDiff, xDiff);
            double dist = getRobotDistanceFromPoint(nxtP); // mtp 2.0
      double relAngToP =
          Angle.normalizeRadians(
              splineAngle - (Math.toRadians(theta) - Math.toRadians(90))); // mtp 2.0
      double relX = Math.sin(relAngToP) * dist, relY = Math.cos(relAngToP) * dist;
      double xPow = (relX / (Math.abs(relY) + Math.abs(relX))) * driveSpeedCap,
          yPow = (relY / (Math.abs(relX) + Math.abs(relY))) * driveSpeedCap;
       */
      destPt = wp.get(pt);
      double xDiff = destPt.xP - x;
      double yDiff = destPt.yP - y;
      double angleDiff =
          heading == Double.MAX_VALUE ? wp.get(pt).ang - odometry.getAngle() : heading - theta;

      double xPow = 0, yPow = 0, turnPow = 0;
      xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
      yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
      turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);

      turnPow += angleDiff * turnConstant;
      if (pt == wp.size() - 1) {
        xPow += xDiff * finalMovementConstant;
        yPow += yDiff * finalMovementConstant;
      } else {
        xPow += xDiff * normalMovementConstant;
        yPow += yDiff * normalMovementConstant;
      }

      if (xPow < 0) {
        if (xPow < -maxSpeed) {
          xPow = -maxSpeed;
        }
      } else {
        if (xPow > maxSpeed) {
          xPow = maxSpeed;
        }
      }

      if (yPow < 0) {
        if (yPow < -maxSpeed) {
          yPow = -maxSpeed;
        }
      } else {
        if (yPow > maxSpeed) {
          yPow = maxSpeed;
        }
      }

      prevTime = time.seconds();
      prevXDiff = xDiff;
      prevYDiff = yDiff;
      prevAngleDiff = angleDiff;
      driveFieldCentricAuto(-yPow, -turnPow, xPow);
    }

    stopDrive();
  }

  // Kinda Like:
  // https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
  public void traversePath(
      ArrayList<Point> wp,
      double heading,
      double driveSpeedCap,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    ElapsedTime time = new ElapsedTime();
    int lastLhInd = 0;
    time.reset();
    while ((lastLhInd < wp.size() - 1
            || (Math.abs(getX() - wp.get(wp.size() - 1).xP) > xError
                || Math.abs(getY() - wp.get(wp.size() - 1).yP) > yError
                || (heading == Double.MAX_VALUE
                    ? Math.abs(wp.get(wp.size() - 1).ang - getAngleImu()) > angleError
                    : Math.abs(heading - getAngleImu()) > angleError)))
        && time.milliseconds() < timeout) {
      resetCache();
      updatePosition();
      double x = getX();
      double y = getY();
      double theta = getAngleImu();

      // find point which fits the look ahead criteria
      Point nxtP = null;
      int i = 0, cnt = 0, possInd = -1;
      double maxDist = -1;

      for (int j = lastLhInd; j < wp.size(); j++) {
        Point p = wp.get(j);
        double ptDist = getRobotDistanceFromPoint(p);
        if (Math.abs(ptDist) <= lookAheadDist && i > lastLhInd && Math.abs(ptDist) > maxDist) {
          nxtP = p;
          possInd = i;
          maxDist = Math.abs(ptDist);
        }
        i++;
      }

      if (possInd == -1) {
        possInd = lastLhInd;
        nxtP = wp.get(lastLhInd);
      }
      if (nxtP == null) {
        stop();
        break;
      }

      // assign powers to follow the look-ahead point
      double xDiff = nxtP.xP - x;
      double yDiff = nxtP.yP - y;
      double angDiff, splineAngle;

      splineAngle = Math.atan2(yDiff, xDiff);
      if (heading == Double.MAX_VALUE) {
        angDiff = theta - nxtP.ang;
      } else {
        angDiff = theta - heading;
      }

      if (Math.abs(angDiff) < angleError) angDiff = 0;

      double dist = getRobotDistanceFromPoint(nxtP); // mtp 2.0
      double relAngToP =
          Angle.normalizeRadians(
              splineAngle - (Math.toRadians(theta) - Math.toRadians(90))); // mtp 2.0
      double relX = Math.sin(relAngToP) * dist, relY = Math.cos(relAngToP) * dist;
      double xPow = (relX / (Math.abs(relY) + Math.abs(relX))) * driveSpeedCap,
          yPow = (relY / (Math.abs(relX) + Math.abs(relY))) * driveSpeedCap;

      xPow = xDiff * 0.05;
      yPow = -yDiff * 0.05;

      if (Math.abs(getRobotDistanceFromPoint(wp.get(wp.size() - 1))) < 5) {
        xPow = xDiff * 0.1;
        yPow = -yDiff * 0.1;
      } else {
        xPow = xDiff * 0.1;
        yPow = -yDiff * 0.1;
      }

      if (Math.abs(xDiff) <= xError) {
        xPow = 0;
      }
      if (Math.abs(yDiff) <= yError) {
        yPow = 0;
      }

      System.out.println(wp.size());
      driveFieldCentric(yPow, 0.03 * angDiff, xPow);
      lastLhInd = possInd;
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
