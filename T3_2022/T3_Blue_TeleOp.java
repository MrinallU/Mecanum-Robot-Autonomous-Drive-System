package org.firstinspires.ftc.teamcode.T3_2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// todo: remove t265 in teleop
@TeleOp(name="T3-Blue-TeleOp", group="T3")
public class T3_Blue_TeleOp extends T3_Base {
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


        init(0);
        initServos();
        initOdometry();
        sleep(2000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        matchTime.reset();
        resetCache();

        while (opModeIsActive()) {
            resetCache();

            odometry.updatePosition();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            aLP = aP;
            aP = gamepad1.a;
            if(aP && !aLP){
                powerMult = slowToggle ? 0.3 : 0.9;
                slowToggle = !slowToggle;
            }

            left *= powerMult;
            right *= powerMult;

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);
            rightDrive.setPower(right);
            backleftDrive.setPower(left);
            backrightDrive.setPower(right);

            yL2P = y2P;
            y2P = gamepad2.y;
            if(!yL2P && y2P){
                safeftyLock = !safeftyLock;
            }

            if (gamepad2.right_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
                container.sweepRelease();
                container.dumpBlock();
                sweeper.sweep();
                toggle2 = 1;
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) {
                container.sweepRelease();
                container.dumpBlock();
                sweeper.dump();
                toggle2 = 1;
            } else {
                container.sweepBlock();
                sweeper.stop();
            }


            // arm
            dPadRightLast2 = dPadRight2;
            dPadRight2 = gamepad2.dpad_right;

            dPadUpLast2 = dPadUp2;
            dPadUp2 = gamepad2.dpad_up;

            dPadDownLast2 = dPadDown2;
            dPadDown2 = gamepad2.dpad_down;

            dPadLeftLast2 = dPadLeft2;
            dPadLeft2 = gamepad2.dpad_left;

            if(gamepad2.dpad_up) {
                arm.motor1.setPower(0.25);
            }else if(gamepad2.dpad_down){
                arm.motor1.setPower(-0.25);
            }else{
                if(arm.motor1.retMotorEx().getCurrentPosition() < 25) {
                    arm.motor1.setPower(0);
                }else{
                    arm.motor1.setPower(-0.01);
                }
            }

            if(arm.motor1.retMotorEx().getCurrentPosition() <= 700 && safeftyLock){
                arm.container.dumpBlock(); // safety
                toggle1 = 2;
            }

            // manual blocker controls
            aL2P = a2P;
            a2P = gamepad2.a;
            if(!aL2P && a2P){
                if(toggle1 % 2 != 0) {
                    container.dumpBlock();
                }else{
                    container.dumpRelease();
                }
                toggle1++;
            }

            xL2P = x2P;
            x2P = gamepad2.x;
            if(!xL2P && x2P){
                if(toggle2 % 2 != 0) {
                    container.sweepBlock();
                }else{
                    container.sweepRelease();
                }
                toggle2++;
            }


            if(gamepad1.y){
                startCarouselBlue(carouselTime.milliseconds());
            }else{
                stopCarousel();
                carouselTime.reset();
            }




            // Send telemetry message to signify robot running;

            telemetry.addLine("Arm Safety Status: "  + safeftyLock);
            telemetry.addLine("odo pos " + wheelOdometry.displayPositions());
            telemetry.addLine("cam pos " + odometry.displayPositions());
            telemetry.addLine("cTime " + carouselTime.milliseconds());
            telemetry.addLine("imu angle " + getRelativeAngle());
            telemetry.update();
        }

        odometry.stopT265();
        sleep(2000);
    }
}
