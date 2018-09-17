package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.JewelHandler;
import org.firstinspires.ftc.teamcode.General.ClawThread;
import org.firstinspires.ftc.teamcode.General.RobotDrive;
import org.firstinspires.ftc.teamcode.General.Slide;

import java.security.spec.EllipticCurve;


@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime lastBackServoToggle = new ElapsedTime();

    private RobotDrive robot = new RobotDrive();
    private ClawThread claw = new ClawThread();
    private Slide slide = new Slide();
    private JewelHandler jewelHandler = new JewelHandler();

    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        robot.init(this, hardwareMap);
        claw.init(this, hardwareMap, gamepad1);
        slide.init(hardwareMap, gamepad1);

        jewelHandler.initTeleOp(hardwareMap);

        waitForStart();

        runTime.reset();
        lastBackServoToggle.reset();

        while (opModeIsActive()) {
            float drive = gamepad1.left_stick_y;
            float turn  = -gamepad1.left_stick_x;
            float micro_turning = -gamepad1.right_stick_x/2;
            turn = turn + micro_turning;
            robot.move(drive + turn, drive - turn);

            if (gamepad1.dpad_up) {
                robot.back_servo_up();
            } else if (gamepad1.dpad_down) {
                robot.back_servo_down();
            }
            telemetry.addData("POSITION", claw.getSliderPosition());
            float hsv[] = claw.getSliderColor();
            telemetry.addData("HUE", hsv[0]);
            telemetry.addData("SATURATION", hsv[1]);
            telemetry.addData("VALUE", hsv[2]);
            telemetry.update();
        }
        claw.kill();
        slide.kill();
    }
}
