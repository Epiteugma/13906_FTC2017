package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.General.ClawThread;
import org.firstinspires.ftc.teamcode.General.RobotDrive;


@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {

    private ElapsedTime runTime = new ElapsedTime();

    private ClawThread claw = new ClawThread();
    private RobotDrive robot = new RobotDrive();

    private Servo upClaw;
    private Servo downClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        upClaw = hardwareMap.get(Servo.class, "upClaw");
        downClaw = hardwareMap.get(Servo.class, "downClaw");

        waitForStart();

        robot.init(
                hardwareMap,
                "leftDrive",
                "rightDrive"
        );
//        claw.init(
//                hardwareMap,
//                gamepad1,
//                "linearSlideMotor",
//                "linearSlideMotorEncoder",
//                "downClaw",
//                "upClaw"
//        );

        runTime.reset();

        while (opModeIsActive()) {

            float drive = -gamepad1.left_stick_y;
            float turn  =  gamepad1.left_stick_x;
            robot.move(drive + turn, drive - turn);

            if (gamepad1.x) downClaw.setPosition(0.9);
            else downClaw.setPosition(0.77);

            if (gamepad1.y) upClaw.setPosition(0.1);
            else upClaw.setPosition(0.22);
        }
    }
}
