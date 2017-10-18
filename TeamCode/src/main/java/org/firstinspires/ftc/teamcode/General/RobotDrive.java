package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotDrive {
    public enum Direction {
        LEFT, RIGHT
    }

    public static final float UNCHANGED = -999;
    private static final double P_DRIVE_COEFF = 0.1;
    private static final double P_TURN_COEFF = 0.1;

    private HardwareMap hardwareMap = null;
    private DcMotor left_drive, right_drive;
    private BNO055IMU imu;

    private PID p_controller_DRIVE = new PID();
    private PID p_controller_TURN = new PID();

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

        // Setup BNO055 built in IMU
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile  = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        // Initialize Motors
        left_drive = hardwareMap.get(DcMotor.class, ld);
        right_drive = hardwareMap.get(DcMotor.class, rd);

        // Setting the direction of the motors
        left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make sure the motor actively resists any external forces
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        p_controller_DRIVE.init(P_DRIVE_COEFF, 0, 0, -1, 1);
        p_controller_TURN.init(P_TURN_COEFF, 0, 0, -1, 1);
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

    // Straight Movement Using Gyro
    public void moveUsingGyro (int ticks, double power) {
        double target_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        p_controller_DRIVE.setSetpoint(target_angle);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_drive.setTargetPosition(left_drive.getCurrentPosition() + ticks);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() + ticks);

        left_drive.setPower(power);
        right_drive.setPower(power);

        while (left_drive.isBusy() || right_drive.isBusy()) {
            double current_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double steer_value = p_controller_DRIVE.Compute(current_angle);

            left_drive.setPower(power - steer_value);
            right_drive.setPower(power + steer_value);
        }
    }

    // Turning using the gyro
    public void gyroTurn (double degrees, double power, double threshold) {
        double current_angle =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double target_angle = current_angle + degrees;

        p_controller_TURN.setSetpoint(target_angle);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do {
            current_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double pow = power * p_controller_TURN.Compute(current_angle);
            left_drive.setPower(pow);
            right_drive.setPower(-pow);
        } while (Math.abs(current_angle - target_angle) <= threshold);
    }
}
