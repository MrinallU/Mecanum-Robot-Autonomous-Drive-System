package org.firstinspires.ftc.teamcode.Mecanum_2.Modules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mecanum_2.Base;
import org.firstinspires.ftc.teamcode.Mecanum_2.PathGenerator;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;

import java.util.ArrayList;
import java.util.List;

public class Drive extends Base {

  protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;
  protected Motor odoR, odoL, odoN;
  protected BNO055IMU gyro;
  protected Odometry odometry;
  protected OpMode opMode;
  protected List<LynxModule> allHubs;

  public Drive(
      Motor fLeftMotor,
      Motor bLeftMotor,
      Motor fRightMotor,
      Motor bRightMotor,
      Motor odoL,
      Motor odoR,
      Motor odoN,
      BNO055IMU gyro,
      OpMode m,
      int xPos,
      int yPos,
      int angle,
      List<LynxModule> allHubs) {

    this.fLeftMotor = fLeftMotor;
    this.fRightMotor = fRightMotor;
    this.bLeftMotor = bLeftMotor;
    this.bRightMotor = bRightMotor;
    this.odoL = odoL;
    this.odoR = odoR;
    this.odoN = odoN;
    this.gyro = gyro;
    this.opMode = m;
    this.allHubs = allHubs;
    odometry = new Odometry(xPos, yPos, angle);
  }

  public void traversePath(
      ArrayList<Point> wp,
      double heading,
      double driveSpeedCap,
      boolean limitPower,
      double powerLowerBound,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    ElapsedTime time = new ElapsedTime();
    int lastLhInd = 0;
    time.reset();
    while ((lastLhInd < wp.size() - 1
            || (Math.abs(odometry.getX() - wp.get(wp.size() - 1).xP) > xError
                || Math.abs(odometry.getY() - wp.get(wp.size() - 1).yP) > yError
                || Math.abs(heading - odometry.getAngle()) > angleError))
        && time.milliseconds() < timeout) {
      resetCache();
      updateOdometry();
      double x = odometry.getX();
      double y = odometry.getY();
      double theta = odometry.getAngle();

      // find point which fits the look ahead criteria
      Point nxtP = null;
      int i = 0, cnt = 0, possInd = -1;
      double maxDist = -1;
      for (Point p : wp) {
        double ptDist = getRobotDistanceFromPoint(p);
        if (Math.abs(ptDist) <= lookAheadDist
            && i > lastLhInd
            && Math.abs(ptDist) > maxDist
            && Math.abs(i - lastLhInd) < 5) {
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
        angDiff = theta - Angle.normalize(Math.toDegrees(splineAngle));
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

      if (limitPower) {
        if (yPow < 0) {
          yPow = Math.min(-powerLowerBound, yPow);
        } else {
          yPow = Math.max(powerLowerBound, yPow);
        }

        if (xPow < 0) {
          xPow = Math.min(-powerLowerBound, xPow);
        } else {
          xPow = Math.max(powerLowerBound, xPow);
        }
      }
      driveFieldCentric(odometry.getAngle(), xPow, 0.05 * angDiff, yPow);
      lastLhInd = possInd;
    }
    stopDrive();
  }

  public void traversePath(
      ArrayList<Point> wp,
      double heading,
      double driveSpeedCap,
      double powLb,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    traversePath(
        wp,
        heading,
        driveSpeedCap,
        true,
        powLb,
        xError,
        yError,
        angleError,
        lookAheadDist,
        timeout);
  }

  public void traversePath(
      ArrayList<Point> wp,
      double heading,
      double xError,
      double yError,
      double angleError,
      int lookAheadDist,
      double timeout) {
    traversePath(wp, heading, 1, false, -1, xError, yError, angleError, lookAheadDist, timeout);
  }

  public void moveToPosition(
      double targetXPos,
      double targetYPos,
      double targetAngle,
      double xAccuracy,
      double yAccuracy,
      double angleAccuracy,
      double timeout) {
    ArrayList<Point> pt = new ArrayList<>();
    pt.add(getCurrentPosition());
    pt.add(new Point(targetXPos, targetYPos));
    ArrayList<Point> wps = PathGenerator.generateLinearSpline(pt);
    traversePath(wps, targetAngle, 1, 0.1, xAccuracy, yAccuracy, angleAccuracy, 10, timeout);
    stopDrive();
  }

  public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference) {
    double currAngle = getAngle();
    ElapsedTime time = new ElapsedTime();
    while (Math.abs(currAngle - targetAngle) > minDifference
        && time.milliseconds() < timeout
        && ((LinearOpMode) opMode).opModeIsActive()) {
      resetCache();
      updateOdometry();
      currAngle = getAngle();
      double angleDiff = Angle.normalize(currAngle - targetAngle);
      double calcP = Range.clip(angleDiff * 0.01, -powerCap, powerCap);
      setDrivePowers(calcP, calcP, calcP, calcP);
    }

    stopDrive();
  }

  // Positional Data
  public void updateOdometry() {
    odometry.updatePosition(
        odoL.encoderReading(), odoR.encoderReading(), odoN.encoderReading(), getAngle());
  }

  public Point getCurrentPosition() {
    updateOdometry();
    return new Point(odometry.getX(), odometry.getY(), odometry.getAngle());
  }

  public double getAngle() {
    Orientation angles =
        gyro.getAngularOrientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // ZYX is Original
    return Angle.normalize(angles.firstAngle + initAng);
  }

  // Driving
  public void driveFieldCentric(double baseAngle, double drive, double turn, double strafe) {
    double fRightPow, bRightPow, fLeftPow, bLeftPow;

    double bLeftAngle = Math.toRadians(baseAngle + 135);
    double fLeftAngle = Math.toRadians(baseAngle + 45);
    double bRightAngle = Math.toRadians(baseAngle + 225);
    double fRightAngle = Math.toRadians(baseAngle + 315);

    fRightPow =
        (drive * Math.sin(fRightAngle) + strafe * Math.cos(fRightAngle)) / Math.sqrt(1) + turn;
    bRightPow =
        (drive * Math.sin(bRightAngle) + strafe * Math.cos(bRightAngle)) / Math.sqrt(1) + turn;
    fLeftPow = (drive * Math.sin(fLeftAngle) + strafe * Math.cos(fLeftAngle)) / Math.sqrt(1) + turn;
    bLeftPow = (drive * Math.sin(bLeftAngle) + strafe * Math.cos(bLeftAngle)) / Math.sqrt(1) + turn;

    double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    fLeftPow = calculatedPower[0];
    bLeftPow = calculatedPower[1];
    fRightPow = calculatedPower[2];
    bRightPow = calculatedPower[3];

    setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
  }

  public void driveRobotCentric(double drive, double turn, double strafe) {
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
    bLeftMotor.setPower(bLeftPow);
    fLeftMotor.setPower(fLeftPow);
    bRightMotor.setPower(bRightPow);
    fRightMotor.setPower(fRightPow);
  }

  public void stopDrive() {
    setDrivePowers(0, 0, 0, 0);
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
    return Math.sqrt(
        (p2.yP - odometry.getY()) * (p2.yP - odometry.getY())
            + (p2.xP - odometry.getX()) * (p2.xP - odometry.getX()));
  }

  // BULK-READING FUNCTIONS
  public void resetCache() {
    // Clears cache of all hubs
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }

  //    private double getVoltage() {
  //        double voltage = Double.MIN_VALUE;
  //        for (LynxModule hub : allHubs) {
  //            voltage = Math.max(voltage, hub.getInputVoltage(VoltageUnit.VOLTS));
  //        }
  //
  //        return voltage;
  //    }

  @Override
  public void runOpMode() throws InterruptedException {}
}
