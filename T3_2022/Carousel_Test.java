package org.firstinspires.ftc.teamcode.T3_2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// todo: remove t265 in teleop
@TeleOp(name="Carousel_Test", group="T3")
public class Carousel_Test extends T3_Base {
    boolean carouselIsOn = false;
    boolean sweeperIsOn = false;
    boolean armIsOn = false;
    boolean aP;
    boolean aLP;
    boolean yP;
    boolean yLP;
    boolean y2P;
    boolean yL2P = y2P;
    boolean a2P;
    boolean aL2P;
    boolean dPadRight2 = false;
    boolean dPadRightLast2 = false;
    boolean dPadUp2 = false;
    boolean dPadUpLast2 = false;
    boolean dPadDown2 = false;
    boolean dPadDownLast2 = false;
    boolean x2P;
    boolean xL2P;
    boolean dPadLeft2 = false;
    boolean dPadLeftLast2 = false;
    boolean bL2P = false;
    boolean b2P = false;
    boolean slowToggle = true;
    boolean safeftyLock = true;

    int toggle1 = 1;
    int toggle2 = 1;
    int toggle3 = 1;
    ElapsedTime carouselTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double sweeperPow;
        double powerMult = 0.9;


        initCarousel();
        sleep(2000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        matchTime.reset();
        resetCache();

        while (opModeIsActive()) {
            resetCache();


            if(gamepad1.y){
                startCarousel(carouselTime.milliseconds());
            }else{
                stopCarousel();
                carouselTime.reset();
            }

            telemetry.addLine("cTime " + carouselTime.milliseconds());
            telemetry.update();
        }

        sleep(2000);
    }
}
