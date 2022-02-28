package org.firstinspires.ftc.teamcode.State_Championship_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.State_Championship_2022.Modules.Camera;
import org.firstinspires.ftc.teamcode.State_Championship_2022.Base;

@Autonomous(name="Secondary_Red_Autonomous", group="State")
public class Secondary_Red_Autonomous extends Base {
    int pos =  1;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        Camera camera = new Camera(hardwareMap);
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
            arm.moveMidRedSecond();
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


        while(opModeIsActive()){
            resetCache();
            telemetry.addLine("imu angle " + getAngle());
        }


    }
}


