package org.firstinspires.ftc.teamcode.Mecanum_2.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Mecanum_2.Base;
import org.firstinspires.ftc.teamcode.Utils.Point;
import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Test_Autonomous", group = "OdomBot")
public class Test_Autonomous extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(0, this);
        sleep(1000);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        matchTime.reset();
        dt.updateOdometry();
        dt.resetCache();

        ArrayList < Point > path1 = new ArrayList < > ();
        path1.add(new Point(9, 16));
        path1.add(new Point(22, 22));
        path1.addAll(new ArrayList < > (
                Arrays.asList(
                        new Point(41, 19), new Point(53, 8.7),
                        new Point(60, -51), new Point(38, -53), new Point(15, -45.1), new Point(0, 0)
                )
        ));

        sleep(500);
        SplinePathConstantHeading(path1, 30, 1, 0.5, 3, 3, 2, 10, 100000);

        while (opModeIsActive()) {
            // Updates
            dt.resetCache();
            dt.updateOdometry();
            telemetry.addData("Odometry Info", dt.getCurrentPosition());
            telemetry.update();
        }
    }
}