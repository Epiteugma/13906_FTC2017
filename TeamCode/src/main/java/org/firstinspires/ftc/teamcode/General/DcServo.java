package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcServo implements Runnable{
    private HardwareMap hardwareMap = null;

    private PID pidController = new PID(1, 0, 0, -1, 1);

    private DcMotor motor;
    private AnalogInput encoderPot;

    private Thread thread;
    private boolean running = false;
    private boolean killed = false;

    public void init(HardwareMap hm, String motorName, String potName) {
        thread = new Thread(this);

        hardwareMap = hm;

        motor = hardwareMap.get(DcMotor.class, motorName);
        encoderPot = hardwareMap.get(AnalogInput.class, potName);

        pidController.IConstrain(-.1, .1);

        thread.start();
        running = true;
    }
    
    public void run () {
        while (!killed) {
            while (running) {
                double output = pidController.Compute(getPotValue());
                motor.setPower(output);
            }
            motor.setPower(0);
        }
    }

    public void kill() {
        killed = true;
        running = false;
    }

    private void setPosition(double pos) {
        pidController.setSetpoint(pos);
    }

    private double getPotValue () {
        double voltage = encoderPot.getVoltage();
        return map(voltage, 0, 3.3, 0, 4096);
    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
