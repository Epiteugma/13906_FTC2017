package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotDrive {
    public static final float UNCHANGED = -999;

    private HardwareMap hardwareMap = null;
    private DcMotor left_drive, right_drive;

    private boolean encoderInMove = false;

    public RobotDrive() {}

    public void init(HardwareMap hm,
                     String ld,
                     String rd) {
        init(hm, ld, rd, true);
    }

    // Initialize the process
    public void init(HardwareMap hm,
                     String ld,
                     String rd,
                     boolean eim) {

        hardwareMap = hm;

        encoderInMove = eim;

        left_drive = hardwareMap.get(DcMotor.class, ld);
        right_drive = hardwareMap.get(DcMotor.class, rd);

        // Setting the direction of the motors
        left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make sure the motor actively resists any external forces
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        move(0,0);
    }

    // Move the robot by giving it ONLY motor power
    public void move(double powL, double powR) {
        // For using the motors without the use of the encoders
        if (encoderInMove) {
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Setting the motors power
        if (powL != UNCHANGED) left_drive.setPower(powL);
        if (powR != UNCHANGED) right_drive.setPower(powR);
    }

    // Moving the robot by giving it a desired motor position
    public void incrementMotorPosition (int l, int r, double power, boolean waitForAction) {
        // For using the build in PID motor control
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the desire motor position
        left_drive.setTargetPosition(left_drive.getCurrentPosition() + l);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() + r);

        // Set the power that we want the motor to run at
        left_drive.setPower(power);
        right_drive.setPower(power);

        // Wait for the desired position to be reached
        if (waitForAction) {
            while(left_drive.isBusy() || right_drive.isBusy());
        }
    }
}
