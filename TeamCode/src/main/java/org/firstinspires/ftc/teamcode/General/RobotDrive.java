package org.firstinspires.ftc.teamcode.General;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotDrive {
    public enum Direction {
        LEFT, RIGHT
    }

    public enum ServoPos {
        Down, Up
    }

    private ServoPos servoPos = ServoPos.Up;

    public static final float UNCHANGED = -999;
    private static final double P_DRIVE_COEFF = 0.1;
    private static final double P_TURN_COEFF = 0.02;
    private static final double I_TURN_COEFF = 0;
    private static final double D_TURN_COEFF = 0;

    private static final double servoUpPosition = 0.25;
    private static final double servoDownPosition = 0.61;

    private HardwareMap hardwareMap = null;
    public DcMotor left_drive, right_drive;

    private BNO055IMU imu;
    private BNO055IMU.Parameters params;

    private PID p_controller_TURN = new PID();

    private boolean encoderInMove = false;

    private int desired_en_left = 0;
    private int desired_en_right = 0;

    private LinearOpMode opMode;

    private Servo balancing_stone_servo;


    public RobotDrive() {}

    public void init(LinearOpMode opMode, HardwareMap hm) {
        init(opMode, hm, true);
    }

    // Initialize the process
    public void init(LinearOpMode opMode, HardwareMap hm, boolean eim) {

        hardwareMap = hm;
        this.opMode = opMode;

        encoderInMove = eim;

        // Setup BNO055 built in IMU
        params = new BNO055IMU.Parameters();
        params.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile  = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        // Initialize Motors
        left_drive = hardwareMap.get(DcMotor.class, "leftDrive");
        right_drive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Initialize Servo
        balancing_stone_servo = hardwareMap.get(Servo.class, "balancing_stone_servo");
        balancing_stone_servo.setPosition(servoUpPosition);

        // Setting the direction of the motors
        left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make sure the motor actively resists any external forces
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        p_controller_TURN.init(P_TURN_COEFF, I_TURN_COEFF, D_TURN_COEFF, -1, 1);
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

        if (powL != 0 && powR != 0) {
            desired_en_left = left_drive.getCurrentPosition();
            desired_en_right = right_drive.getCurrentPosition();
        }
        
        // Setting the motors power
        left_drive.setPower(powL);
        right_drive.setPower(powR);
    }

    // Moving the robot by giving it a desired motor position
    public void incrementMotorPosition (int l, int r, double power, boolean waitForAction) {
        if (!opMode.isStopRequested()) {
            // For using the build in PID motor control
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            desired_en_left -= l;
            desired_en_right -= r;

            // Set the desire motor position
            left_drive.setTargetPosition(desired_en_left);
            right_drive.setTargetPosition(desired_en_right);

            // Set the power that we want the motor to run at
            left_drive.setPower(power);
            right_drive.setPower(power);

            // Wait for the desired position to be reached
            if (waitForAction) {
                while ((left_drive.isBusy() || right_drive.isBusy()) && opMode.opModeIsActive()) ;
                left_drive.setPower(0);
                right_drive.setPower(0);
            }
        }
    }

    public void toggleBackServoPosition ( ) {
        servoPos = servoPos == ServoPos.Down ? ServoPos.Up : ServoPos.Down;
        balancing_stone_servo.setPosition(servoPos == ServoPos.Down ? servoDownPosition : servoUpPosition);
    }

    public void back_servo_up() {
        balancing_stone_servo.setPosition(servoUpPosition);
    }

    public void back_servo_down() {
        balancing_stone_servo.setPosition(servoDownPosition);
    }

    // Turning using the gyro
    // TODO fix delay due to reinitialization of the object.
    public void turn (double degrees, double power, double t, boolean useTime) {
        imu.initialize(params);

        ElapsedTime eTime = new ElapsedTime();
        double current_angle = 0;
        double target_angle = current_angle + degrees;

        p_controller_TURN.setSetpoint(target_angle);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eTime.reset();
        while (opMode.opModeIsActive()) {
            current_angle = getHeading();

            double pow = power * p_controller_TURN.Compute(current_angle);
            left_drive.setPower(pow);
            right_drive.setPower(-pow);

            boolean exitCondition;
            if (useTime)  exitCondition = eTime.milliseconds() > t;
            else exitCondition = Math.abs(current_angle - target_angle) <= t;

            if (exitCondition) break;
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    private double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}


