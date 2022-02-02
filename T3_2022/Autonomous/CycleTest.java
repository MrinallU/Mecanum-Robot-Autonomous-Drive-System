package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;
import org.firstinspires.ftc.teamcode.Utils.Point;


@Autonomous(name="Cycle_Test", group="Autonomous")
public class CycleTest extends T3_Base {
    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(4000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        wheelOdometry.updatePosition(
                leftDrive.encoderReading(),
                rightDrive.encoderReading(),
                getAngle());
        odometry.updatePosition();
        arm.moveToPosition(300);

        Point [] pts = {new Point(0, 0), new Point(-71, 0)};
        crossBarrier(-71, 0, 0.5, 5000, 0, false, true);
        sleep(1000);
        crossBarrier(0, 0, 0.5, 5000, 0, true, false);
        sleep(1000);
        crossBarrier(-71, 0, 0.5, 5000, 0, false, true);
        odometry.stopT265();
        sleep(500);

    }
}