package org.firstinspires.ftc.teamcode.T3_2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;

// todo: remove t265 in teleop
@TeleOp(name="DuckAimTest", group="T3")
public class DuckAimTest extends T3_Base {
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
    boolean switchArmPowerCurr, switchArmPowerLast;
    boolean sharedHubMode = false;

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
        T3_Camera camera = new T3_Camera(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        matchTime.reset();
        resetCache();

        while (opModeIsActive()) {
            resetCache();

//            odometry.updatePosition();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());
            double [] status = camera.scanForDuck();

            // arm
            switchArmPowerLast = switchArmPowerCurr;
            switchArmPowerCurr = gamepad2.b;

            if(!switchArmPowerLast && switchArmPowerCurr){
                sharedHubMode = !sharedHubMode;
            }
            dPadRightLast2 = dPadRight2;
            dPadRight2 = gamepad2.dpad_right;

            dPadUpLast2 = dPadUp2;
            dPadUp2 = gamepad2.dpad_up;

            dPadDownLast2 = dPadDown2;
            dPadDown2 = gamepad2.dpad_down;

            dPadLeftLast2 = dPadLeft2;
            dPadLeft2 = gamepad2.dpad_left;


            if(gamepad2.dpad_up) {
                if(!sharedHubMode){
                    arm.motor1.setPower(0.4);
                }else{
                    arm.motor1.setPower(0.2);
                }
            }else if(gamepad2.dpad_down){
                if(!sharedHubMode){
                    arm.motor1.setPower(-0.4);
                }else{
                    arm.motor1.setPower(-0.2);
                }
            }else{
                if(arm.motor1.retMotorEx().getCurrentPosition() < 25) {
                    arm.motor1.setPower(0);
                    arm.motor1.useEncoder();
                }else{
                    arm.motor1.setPower(-0.001);
                }
            }

            if(arm.motor1.retMotorEx().getCurrentPosition() <= 700 && safeftyLock){
                arm.container.dumpBlock(); // safety
                toggle1 = 2;
            }

            xL2P = x2P;
            x2P = gamepad2.x;
            if(!xL2P && x2P){
                if(status[0] == 1){
                    turnToV2(status[1], 5000, this);
                }
            }

            // Send telemetry message to signify robot running;
            telemetry.addLine("Duck detected + Ang " + (status[0] == 1 ? status[1]  : "False"));
            telemetry.addLine("imu angle " + getRelativeAngle());
            telemetry.update();
        }

        sleep(2000);
    }
}
