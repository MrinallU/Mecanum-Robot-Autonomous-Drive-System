package org.firstinspires.ftc.teamcode.State_Championship_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;
import org.firstinspires.ftc.teamcode.State_Championship_2022.Base;

@Autonomous(name="Primary_Blue_Autonomous", group = "State")
public class Primary_Blue_Autonomous extends Base {
    int pos = 0;
    String elementDiagram = "";
    int bottomOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        Camera camera = new Camera(hardwareMap);
        initOdometry();
        sleep(1000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        pos = camera.readBarcode("bluePrimary");
        if (pos == 0) {
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        } else if (pos == 1) {
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        } else if (pos == 2) {
            telemetry.addData("Wobble Level: ", "Top");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }

        arm.moveToPosition(300);

        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.update();

        moveTicksBack(450, 4000, 0.4, 20, this);
        sleep(500);
        turnToV2(93, 4000, this);
        sleep(500);


        if(pos == 0){
            arm.moveBottom();
            sleep(2500);
            moveTicksBack(600, 4000, 0.4, 20, this);
            bottomOffset = 60;
        }else if(pos == 1){
            arm.moveMidBluePrim();
            sleep(2500);
            moveTicksBack(660, 4000, 0.4, 20, this);
        }else{
            arm.moveTop();
            sleep(2500);
            moveTicksBack(600, 4000, 0.4, 20, this);
        }



        sleep(500);
        arm.dump();
        sleep(500);


        //600
        // change to odo...
        yTo(4, 9000, 0.1, 1, this, true); // @Parth tune this
//        moveTicksFront(550 + bottomOffset, 4000, 0.4, 20, this); <--- this is inconsistent compared to odo
        container.dumpBlock();
        sleep(500);
        arm.moveToPosition(300);
        sleep(1000);

        turnToV2(180, 4000, this);
        sleep(500);

        moveTicksBack(200, 3000, 0.4, 20, this);
        sleep(250);

        turnToV2(-88, 4000, this);
        sleep(500);


        moveTicksBack(1000, 6000, 0.2, 20, this); // @Parth tune this: move to carousel
        sleep(500);

        moveTicksBack(140, 6000, 0.03, 20, this); // @Parth tune this: move to carousel
//        sleep(250);
        startBlueCarousel();
        sleep(3000);
        stopCarousel();

        //manouver to avoid driving over capstone
        moveTicksFront(200, 4000, 0.4, 20, this);
        sleep(500);

        turnToV2(-35, 4000, this);

        moveTicksBack(1000, 4000, 0.4, 20, this);
        sleep(250);
        turnToV2(0, 4000, this);
        moveTicksBack(50, 4000, 0.2, 20, this);

        sleep(500);
    }
}
