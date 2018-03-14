package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JewelHandler {
    public enum Team {
        RED,
        BLUE,
        UNKNOWN
    }

    private static final double armUpPosition = 0.50;
    private static final double armDownPosition = 0;
    private static final double armStartPosition = 1;

    private HardwareMap hardwareMap = null;

    private Team team = Team.UNKNOWN;
    private Team jewelTeam = Team.UNKNOWN;

    private volatile boolean knockdownJewel = Boolean.parseBoolean(null);

    public Servo servo = null;
    private LynxI2cColorRangeSensor colorJewel = null;

    public void initTeleOp(HardwareMap hm) {
        hardwareMap = hm;
        servo = hardwareMap.get(Servo.class, "jewelServo");
        servo.setPosition(armUpPosition);
    }

    public void initAutonomous(HardwareMap hm, Team t) {
        hardwareMap = hm;

        servo = hardwareMap.get(Servo.class, "jewelServo");
        colorJewel = hardwareMap.get(LynxI2cColorRangeSensor.class, "colorJewel");
        servo.setPosition(armStartPosition);

        team = t;
    }

    public Team getTeamColor() {return team;}
    public Team getJewelTeam() {return jewelTeam;}
    public boolean getknockdownJewel () {return knockdownJewel;}

    // Compute the color (RED or BLUE) that the chosen color sensor sees
    public Team computeColor() {
        float hsv[] = {0, 0, 0};
        Color.colorToHSV(colorJewel.argb(), hsv);

        float hue = hsv[0];
        if (hue < 30 || hue > 350) return Team.RED;
        else return Team.BLUE;
    }

    // Computes which of the 2 jewels to knock down
    public boolean compute() {
        servo.setPosition(armDownPosition);
        delay(2000);

        jewelTeam = computeColor();

        knockdownJewel = getJewelTeam() == getTeamColor();
        return knockdownJewel;
    }

    // Get the arm back to the up position
    public void retractServo() {
        servo.setPosition(armUpPosition);
        delay(2000);
    }

    private void delay(int t) {
        try {
            Thread.sleep(t);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
