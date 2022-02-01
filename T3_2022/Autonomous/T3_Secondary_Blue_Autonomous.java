package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Blue_Autonomous", group="Autonomous")
public class T3_Secondary_Blue_Autonomous extends T3_Base {
    int pos = 0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(2000);

        pos = camera.readBarcode("blueSecondary");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if(pos == 0){
            arm.moveBottom();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveTop();
        }

        xTo(-10, 3000, 0.3,  1, this, true );
        sleep(250);

        turnToV2(90, 3000, this);

        sleep(250);
        yTo(15, 3000, 0.3,  1, this, true );

        sleep(250);
        arm.dump();
        sleep(500);

        // replace with cycle code
        yTo(0, 3000, 0.3,  1, this, true );
        sleep(250);

        moveTicksBack(1500, 2000, 0.5, 40, this);
        sleep(250);
        container.dumpBlock();
        arm.sweepPos();
        sleep(500);

        turnToV2(-70, 3000, this);

        sleep(1000);
        sweeper.sweep();
        sleep(500);

        moveTicksFront(500, 2000, 0.3, 20, this);

        sleep(2000);
        sweeper.stop();
        container.sweepBlock();

        sweeper.dump();
        sleep(2000);
        odometry.stopT265();
    }
}
