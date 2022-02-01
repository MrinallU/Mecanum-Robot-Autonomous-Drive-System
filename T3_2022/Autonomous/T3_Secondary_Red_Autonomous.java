package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Red_Autonomous", group="Autonomous")
public class T3_Secondary_Red_Autonomous extends T3_Base {
    int pos =  0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(3000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        pos = camera.readBarcode("redSecondary");


        if(pos == 0){
            arm.moveBottomRedSecond();
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        }else if(pos == 1){
            arm.moveMid();
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        }else if(pos == 2){
            arm.moveTop();
            telemetry.addData("Wobble Level: ", "Top");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }

        moveTicksBack(300, 4000, 0.5, 20,this);
        sleep(250);


        turnToV2(86, 4000, this);
        sleep(250);


        if(pos == 0){
            moveTicksBack(730, 4000, 0.4, 20, this); //bottom deposit
        }else if(pos == 1){
            moveTicksBack(800, 4000, 0.4, 20, this); //mid and top deposit
        }else{
            moveTicksBack(900, 4000, 0.4, 20, this); //mid and top deposit
        }

        sleep(500);
        arm.dump();
        sleep(500);

        // move to barrier
        moveTicksFront(2500, 4000, 0.4, 20, this);
        sleep(250);
        container.dumpBlock();
        arm.sweepPos();
        turnToV2(60, 2000, this);
        sleep(250);


        sleep(250);
        sweeper.sweep();
        moveTicksFront(440, 4000, 0.4, 20, this);
        sleep(1000);
        sweeper.stop();
        container.sweepBlock();

        sweeper.dump();
        sleep(1000);
        sweeper.stop();

        // replace with cycle code
        moveTicksBack(400, 4000, 0.4, 20, this);

        turnToV2(90, 4000, this);
        sleep(250);

        moveTicksBack(500, 4000, 0.5, 20,this);
        sleep(250);
        arm.moveToPosition(500);


        moveTicksBack(2000, 4000, 0.5, 20,this);
        sleep(250);


        arm.moveTop();
        sleep(1500);

        arm.dump();
        sleep(500);
        container.dumpBlock();

        moveTicksFront(2750, 6000, 0.4, 20, this);
        arm.sweepPos();
        sleep(500);
        odometry.stopT265();
    }
}
