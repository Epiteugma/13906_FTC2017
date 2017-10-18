package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.util.ElapsedTime;

class PID {
    private static final double UNDEFINED = -99999999;

    private ElapsedTime runTime = new ElapsedTime();

    private double lastCompute = 0;

    private double sampleTime;

    private double kp = 0;
    private double ki = 0;
    private double kd = 0;

    private double outputMax;
    private double outputMin;

    private double I = 0;

    private double IMax = UNDEFINED;
    private double IMin = UNDEFINED;

    private double setpoint = 0;
    private double input = 0;
    private double output = 0;

    private double lastError = 0;
    private double error = 0;

    PID () {

    }

    PID (double p, double i, double d, double omx, double omn) {
        init(p, i, d, omn, omx);
    }

    public void init (double p, double i, double d, double omn, double omx) {
        kp = p;
        ki = i;
        kd = d;

        outputMax = omx;
        outputMin = omn;

        sampleTime = 5;

        runTime.reset();
    }

    public void setSampleTime(double st) {
        sampleTime = st;
    }

    public void IConstrain(double mx, double mn) {
        IMax = mx;
        IMin = mn;
    }

    public void setSetpoint(double sp) {
        setpoint = sp;
    }

    public double Compute(double in) {
        if (runTime.milliseconds() - lastCompute >= sampleTime) {
            input = in;
            error = setpoint - input;

            I += error;
            if (IMax != UNDEFINED) I = constrain(I, IMin, IMax);

            output = kp * error + ki * I + kd * (error - lastError);
            output = constrain(output, outputMin, outputMax);

            error = lastError;
        }
        return output;
    }
    private double constrain(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }


}
