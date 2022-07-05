package org.firstinspires.ftc.teamcode.Mecanum_2.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Mecanum_2.Base;
import org.firstinspires.ftc.teamcode.Utils.Point;

@Autonomous(name="Test_Autonomous", group = "State")
public class Test_Autonomous extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(this);
        initServos();

        sleep(2500);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        resetCache();

        moveToPosition(new Point(10, 2), 4000);
        sleep(500);
    }
}
