package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;
import org.firstinspires.ftc.teamcode.Utils.Point;


@Autonomous(name="T265_Test", group="Autonomous")
public class T265_Test extends T3_Base {

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(2000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        wheelOdometry.updatePosition(
                leftDrive.encoderReading(),
                rightDrive.encoderReading(),
                getAngle());
        odometry.updatePosition();
        arm.moveToPosition(300);

        sleep(1000);

        xTo(20,4000, 0.2, 1, this, false, true);
        sleep(500);
    }
}