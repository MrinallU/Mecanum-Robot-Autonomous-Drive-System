package org.firstinspires.ftc.teamcode.Utils.RoadRunner.drive.Localizers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DistanceSensorLocalizer implements Localizer {
    ModernRoboticsI2cRangeSensor xSensor;
    ModernRoboticsI2cRangeSensor ySensor;
    private BNO055IMU imu;
    Pose2d currPose = new Pose2d();

    public DistanceSensorLocalizer(HardwareMap hwMap){
         xSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "xMeasurement");
         ySensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "yMeasurement");
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return currPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //ZYX is Original
        currPose = new Pose2d(xSensor.cmUltrasonic(), ySensor.cmUltrasonic(), angles.firstAngle);
    }
}
