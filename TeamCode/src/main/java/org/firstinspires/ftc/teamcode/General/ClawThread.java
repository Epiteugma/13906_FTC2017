package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawThread implements Runnable{
    private enum Claws {
        DOWN, UP
    }

    private static final double closedUpClaw = 0.1;
    private static final double openUpClaw = 0.22;

    private static final double closedDownClaw = 0.9;
    private static final double openDownClaw = 0.77;

    private static final double linearSlideUp = 90;
    private static final double linearSlideDown = 0;

    private boolean cubeOnTop = false;

    private HardwareMap hardwareMap = null;
    private Gamepad gamepad1;

    private DcServo linearSlideMotor = new DcServo();
    private Servo downClaw, upClaw;

    private Thread thread;
    private boolean killed = false;

    private ElapsedTime runTime = new ElapsedTime();
    private double previousBPress = 0;

    public void run() {
        while (!killed) {
            if (gamepad1.b && runTime.milliseconds() - previousBPress < 100) {
                previousBPress = runTime.milliseconds();
                getCube();
            }
        }
    }

    public void init (HardwareMap hm,
                      Gamepad gp,
                      String sm,
                      String sme,
                      String dc,
                      String uc) {

        thread = new Thread(this);

        hardwareMap = hm;
        gamepad1 = gp;

        linearSlideMotor.init(hardwareMap, sm, sme);
        downClaw = hardwareMap.get(Servo.class, dc);
        upClaw = hardwareMap.get(Servo.class, uc);

        downClaw.setPosition(openDownClaw);
        upClaw.setPosition(openUpClaw);

        thread.start();
    }

    public boolean getCubeOnTop () {return cubeOnTop;}

    public void getCube () {
        if (!cubeOnTop) {
            setServoPosition(Claws.DOWN, closedDownClaw);
            // Linear Motor UP
            setServoPosition(Claws.UP, closedUpClaw);
            setServoPosition(Claws.DOWN, openDownClaw);
            // Linear Motor DOWN
        }
    }

    private void setServoPosition (Claws cl, double position) {
        if (cl == Claws.DOWN) {
            downClaw.setPosition(position);
            try {
                thread.sleep(2000);
            } catch (InterruptedException e) {
                thread.interrupt();
            }
        } else if (cl == Claws.UP) {
            upClaw.setPosition(position);
            try {
                thread.sleep(2000);
            } catch (InterruptedException e) {
                thread.interrupt();
            }
        }
    }

    public void kill () {
        linearSlideMotor.kill();
        killed = true;

        try {
            thread.sleep(200000000);
        } catch (InterruptedException e) {
            thread.interrupt();
        }
    }
}
