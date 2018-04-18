package org.firstinspires.ftc.teamcode.General;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawThread implements Runnable{
    public enum SLIDER_POSITION {
        DOWN, UP, UNKNOWN
    }

    private LinearOpMode opMode;

    private static final double slide_speed = 0.5;

    private static final double openClawLeft = 0.95;
    private static final double closeClawLeft = 0.83;
    private static final double openClawRight = 0.8;
    private static final double closeClawRight = 1;

    private static final double servoVel = 0.01;

    private double servoLeftPostion, servoRightPosition;

    private HardwareMap hardwareMap = null;
    private Gamepad gamepad1;

    private SLIDER_POSITION claw_pos = SLIDER_POSITION.DOWN;

    private DcMotor linearSlideMotor;
    private Servo claw_servo_left, claw_servo_right;

    private LynxI2cColorRangeSensor encoder_sensor;

    private Thread thread;
    private boolean killed = false;

    public boolean auto = false;

    private double constrain(double amt, double low, double high) {
        return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
    }

    public void run() {
        while (!killed) {
            if (gamepad1.y) linearSlideMotor.setPower(slide_speed);
            else if (gamepad1.a) linearSlideMotor.setPower(-slide_speed);
            else if (gamepad1.left_bumper) {
                servoLeftPostion += servoVel;
                servoRightPosition -= servoVel;

                servoLeftPostion   = constrain(servoLeftPostion  , Math.min(openClawLeft ,  closeClawLeft), Math.max(openClawLeft ,  closeClawLeft));
                servoRightPosition = constrain(servoRightPosition, Math.min(openClawRight, closeClawRight), Math.max(openClawRight, closeClawRight));

                claw_servo_left.setPosition(servoLeftPostion);
                claw_servo_right.setPosition(servoRightPosition);
            }
            else if (gamepad1.right_bumper) {
                servoLeftPostion -= servoVel;
                servoRightPosition += servoVel;

                servoLeftPostion   = constrain(servoLeftPostion  , Math.min(openClawLeft ,  closeClawLeft), Math.max(openClawLeft ,  closeClawLeft));
                servoRightPosition = constrain(servoRightPosition, Math.min(openClawRight, closeClawRight), Math.max(openClawRight, closeClawRight));

                claw_servo_left.setPosition(servoLeftPostion);
                claw_servo_right.setPosition(servoRightPosition);
            }
            else if (gamepad1.b) openClaw();
            else if (gamepad1.x) closeClaw();
            else if (auto) {
                slideUp(700);
                auto = false;
            }
            else {
                linearSlideMotor.setPower(0);
            }
        }
    }

    public void init (LinearOpMode opMode, HardwareMap hm, Gamepad gp) {
        this.opMode = opMode;

        thread = new Thread(this);

        hardwareMap = hm;
        gamepad1 = gp;

        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");

        claw_servo_left = hardwareMap.get(Servo.class, "claw_servo_l");
        claw_servo_left.setPosition(openClawLeft);

        claw_servo_right = hardwareMap.get(Servo.class, "claw_servo_r");
        claw_servo_right.setPosition(openClawRight);

        encoder_sensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "encoder_sensor");

        thread.start();
    }

    public void closeClaw() {
        claw_servo_left.setPosition(closeClawLeft);
        delay(150);
        claw_servo_right.setPosition(closeClawRight);

        servoLeftPostion = closeClawLeft;
        servoRightPosition = closeClawRight;
    }

    public void openClaw() {
        claw_servo_left.setPosition(openClawLeft);
        claw_servo_right.setPosition(openClawRight);

        servoLeftPostion = openClawLeft;
        servoRightPosition = openClawRight;
    }

    public void slideUp (int t) {
        linearSlideMotor.setPower(slide_speed);
        delay(t);
        linearSlideMotor.setPower(0);
    }

    public void slideDown (int t) {
        linearSlideMotor.setPower(-slide_speed);
        delay(t);
        linearSlideMotor.setPower(0);
    }

    public SLIDER_POSITION getSliderPosition() {
        float hue = getSliderColor()[0];
        if (hue >= 20 && hue <= 60) return SLIDER_POSITION.UP; // Yellow
        else if (hue >= 80 && hue <= 140) return SLIDER_POSITION.DOWN; // Green
        else return SLIDER_POSITION.UNKNOWN;
    }

    public float[] getSliderColor() {
        float hsv[] = {0, 0, 0};
        Color.colorToHSV(encoder_sensor.argb(), hsv);

        return hsv;
    }

    private void delay(int t) {
        try {
            thread.sleep(t);
        } catch (InterruptedException e) {
            thread.interrupt();
        }
    }

    public void kill () {
        killed = true;

//        try {
//            thread.sleep(200000000);
//        } catch (InterruptedException e) {
//            thread.interrupt();
//        }
    }
}
