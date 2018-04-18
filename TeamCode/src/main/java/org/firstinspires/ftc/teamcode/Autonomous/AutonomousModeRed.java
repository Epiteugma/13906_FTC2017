package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.JewelHandler.Team;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.General.ClawThread;
import org.firstinspires.ftc.teamcode.General.RobotDrive;

@Autonomous(name="Autonomous Mode Red", group="Revved Up")
public class AutonomousModeRed extends LinearOpMode {
    private static final double turningSpeed = 0.5;
    private static final double alignmentPower = 0.2;
    private static final double driveSpeed = 0.8;
    private static final double pictographAlignTime = 1000;

    private static final int turnTicks = 50;
    private static final int stepDownTicks = 300;

    private static final int balancingStoneToLeft = 1120;
    private static final int balancingStoneToCenter = 928;
    private static final int balancingStoneToRight = 734;


    private ElapsedTime runTime = new ElapsedTime();

    private JewelHandler jewelHandler = new JewelHandler();
    private PictographIdentification pictographIdentification = new PictographIdentification();
    private RobotDrive robot = new RobotDrive();
    private ClawThread claw = new ClawThread();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        /*
         Initializing the processes:
            Pictograph Identification
            Robot Drive Train
            Jewel Handler
         */
        pictographIdentification.init(hardwareMap);
        robot.init(this, hardwareMap);
        claw.init(this, hardwareMap, gamepad1);
        jewelHandler.initAutonomous(hardwareMap, Team.RED);

        pictographIdentification.start();

        claw.closeClaw();

        waitForStart();

        if (opModeIsActive()) {
            claw.auto = true;

            runTime.reset();

            // Decrypting the Pictograph
            while (opModeIsActive() && runTime.milliseconds() < 3000 && pictographIdentification.getCubePosition() == RelicRecoveryVuMark.UNKNOWN)
                pictographIdentification.checkForPictograph();
            if (runTime.milliseconds() > 2900)
                pictographIdentification.setCubePosition(RelicRecoveryVuMark.LEFT);

            telemetry.addData("Pictograph", pictographIdentification.getCubePosition());
            telemetry.update();

            /*
            Calculating and moving the robot in that direction in order to
            knock down the correct jewel
             */
            boolean knockdownJewel = jewelHandler.compute();
            robot.incrementMotorPosition(
                    knockdownJewel ? turnTicks : -turnTicks,
                    knockdownJewel ? -turnTicks : turnTicks,
                    0.5,
                    true
            );
            jewelHandler.retractServo();
            robot.incrementMotorPosition(
                    knockdownJewel ? -turnTicks : turnTicks,
                    knockdownJewel ? turnTicks : -turnTicks,
                    0.5,
                    true
            );

            // Step Down from Balancing Stone
            robot.incrementMotorPosition(
                    stepDownTicks,
                    stepDownTicks,
                    0.5,
                    true
            );

            // Move to the correct cryptobox
            RelicRecoveryVuMark glyph_pos = pictographIdentification.getCubePosition();
            int pos_offset = (glyph_pos == RelicRecoveryVuMark.LEFT ?
                    balancingStoneToLeft :
                    glyph_pos == RelicRecoveryVuMark.CENTER ?
                            balancingStoneToCenter : balancingStoneToRight) - stepDownTicks;
            robot.incrementMotorPosition(pos_offset, pos_offset, driveSpeed, true);

            // Turn towards the cryptobox
            robot.turn(-90, 0.3, 5, false);
            robot.move(0, 0);
            delay(100);

            robot.move(-0.5, -0.5);
            delay(2000);

            claw.openClaw();

            robot.move(0.5, 0.5);
            delay(1000);
        }
        claw.kill();
    }

    private void delay(double t) {
        double s = runTime.milliseconds();
        while (opModeIsActive() && runTime.milliseconds() - s < t);
    }

    private double constrain(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }
}
