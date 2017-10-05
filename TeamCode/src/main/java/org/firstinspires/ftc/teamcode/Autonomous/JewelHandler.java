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

    public enum Sensor {
        BALANCING_STONE,
        JEWEL
    }

    private static final double armUpPosition = .5;
    private static final double armDownPosition = 1;

    private HardwareMap hardwareMap = null;

    private Team team = Team.UNKNOWN;
    private Team jewelTeam = Team.UNKNOWN;

    private volatile boolean knockdownJewel = Boolean.parseBoolean(null);

    public Servo servo = null;
    private LynxI2cColorRangeSensor colorBalancingStone, colorJewel = null;

    public void init(HardwareMap hm,
                     String cbs,
                     String cj,
                     String s) {
        hardwareMap = hm;

        servo = hardwareMap.get(Servo.class, s);
        colorBalancingStone = hardwareMap.get(LynxI2cColorRangeSensor.class, cbs);
        colorJewel = hardwareMap.get(LynxI2cColorRangeSensor.class, cj);

        // Make sure the servo is in the correct starting position
        servo.setPosition(armUpPosition);

    }

    public Team getTeamColor() {return team;}
    public Team getJewelTeam() {return jewelTeam;}
    public boolean getknockdownJewel () {return knockdownJewel;}

    // Compute the color (RED or BLUE) that the chosen color sensor sees
    public Team computeColor(Sensor s) {
        LynxI2cColorRangeSensor sensor = s == Sensor.BALANCING_STONE ? colorBalancingStone : colorJewel;

        float hsv[] = {0, 0, 0};
        Color.colorToHSV(sensor.argb(), hsv);

        float hue = hsv[0];
        if (hue < 30 || hue > 350) return Team.RED;
        else return Team.BLUE;
    }

    // Computes which of the 2 jewels to knock down
    public boolean compute() {
        team = computeColor(Sensor.BALANCING_STONE);

        servo.setPosition(armDownPosition);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        jewelTeam = computeColor(Sensor.JEWEL);

        knockdownJewel = getJewelTeam() == getTeamColor();
        return knockdownJewel;
    }

    // Get the arm back to the up position
    public void retractServo() {
        servo.setPosition(armUpPosition);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
