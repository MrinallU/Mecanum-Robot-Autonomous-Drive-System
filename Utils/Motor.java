package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Motor {
    DcMotor motor;
    int multiplier = 1;

    public Motor(HardwareMap hardwareMap, String name){
        this.motor = hardwareMap.dcMotor.get(name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try{
            resetEncoder(true);
        }
        catch (Exception e){
            noEncoder();
        }
    }

    public Motor(HardwareMap hardwareMap, String name, boolean useEncoder){
        this.motor = hardwareMap.dcMotor.get(name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try{
            resetEncoder(useEncoder);
        }
        catch (Exception e){

        }
    }

    public void setDirection(DcMotorSimple.Direction d){
        motor.setDirection(d);
    }

    public void coast(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void negateEncoder(){ multiplier = -1; }

    public int encoderReading() { return motor.getCurrentPosition() * multiplier; }
    public void setPower(double power){this.motor.setPower(power); }

    public void resetEncoder(boolean useEncoder){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(useEncoder){
            useEncoder();
        }
        else{
            noEncoder();
        }
    }

    public void setFloat() { this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); }

    public void noEncoder() { this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }


    public void flip(){
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void useEncoder(){
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTarget(double target){this.motor.setTargetPosition((int)target);}

    public void setPid(double p, double i, double d, double f){this.retMotorEx().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));}

    public void toPosition(){this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public boolean isBusy(){return this.motor.isBusy();}
    public DcMotorEx retMotorEx(){
        return (DcMotorEx)motor;
    }
}
