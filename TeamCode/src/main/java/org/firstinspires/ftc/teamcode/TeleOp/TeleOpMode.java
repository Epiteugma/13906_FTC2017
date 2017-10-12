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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);


        waitForStart();

        robot.init(
                hardwareMap,
                "leftDrive",
                "rightDrive"
        );
        claw.init(
                hardwareMap,
                gamepad1,
                "linearSlideMotor",
                "linearSlideMotorEncoder",
                "downClaw",
                "upClaw"
        );

        runTime.reset();

        while (opModeIsActive()) {
            float drive = gamepad1.left_stick_y;
            float turn  = -gamepad1.left_stick_x;
            robot.move(drive + turn, drive - turn);
        }
    }
}
