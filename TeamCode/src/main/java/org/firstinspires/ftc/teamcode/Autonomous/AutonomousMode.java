package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.General.RobotDrive;

@Autonomous(name="Autonomous Mode", group="Revved Up")
public class AutonomousMode extends LinearOpMode {
    private static final double turningSpeed = 0.5;
    private static final double alignmentPower = 0.2;
    private static final double driveSpeed = 0.8;

    private static final int turnTicks = 50;
    private static final int stepDownTicks = 300;

    private static final int balancingStoneToLeft = 1120;
    private static final int balancingStoneToCenter = 928;
    private static final int balancingStoneToRight = 734;


    private ElapsedTime runTime = new ElapsedTime();

    private JewelHandler jewelHandler = new JewelHandler();
    private PictographIdentification pictographIdentification = new PictographIdentification();
    private RobotDrive robot = new RobotDrive();

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        /*
         Initializing the processes:
            Pictograph Identification
            Robot Drive Train
            Jewel Handler
         */
        pictographIdentification.init(hardwareMap);
        robot.init(hardwareMap, "leftDrive", "rightDrive");
        jewelHandler.init(
                hardwareMap,
                "colorBalancingStone",
                "colorJewel",
                "jewelServo"
        );

        waitForStart();
		
		pictographIdentification.start();

        runTime.reset();

        // Decrypting the Pictograph
        while (pictographIdentification.getCubePosition() == RelicRecoveryVuMark.UNKNOWN)
            pictographIdentification.checkForPictograph();

        /*
        Calculating and moving the robot in that direction in order to
        knock down the correct jewel
         */
        boolean knockdownJewel = jewelHandler.compute();
        robot.incrementMotorPosition(
                knockdownJewel ? turnTicks : -turnTicks,
                knockdownJewel ? -turnTicks : turnTicks,
                turningSpeed,
                true
        );
        jewelHandler.retractServo();
        robot.incrementMotorPosition(
                knockdownJewel ? -turnTicks : turnTicks,
                knockdownJewel ? turnTicks : -turnTicks,
                turningSpeed,
                true
        );

        // Step Down from Balancing Stone
        robot.incrementMotorPosition(
                stepDownTicks,
                stepDownTicks,
                0.8,
                true
        );

        // Align with Pictograph
        Orientation angles;
        do {
            angles = pictographIdentification.getPictographAngle();
            robot.move(
                    angles.secondAngle > 0 ? alignmentPower : -alignmentPower,
                    angles.secondAngle > 0 ? -alignmentPower : alignmentPower
            );
        } while (Math.abs(angles.secondAngle) > 0.1);
        robot.move(0, 0);

        // Move to the correct cryptobox
        switch (pictographIdentification.getCubePosition()) {
            case RIGHT:
                robot.moveUsingGyro(balancingStoneToRight - stepDownTicks, driveSpeed);
                break;
            case CENTER:
                robot.moveUsingGyro(balancingStoneToCenter - stepDownTicks, driveSpeed);
                break;
            case LEFT:
                robot.moveUsingGyro(balancingStoneToLeft - stepDownTicks, driveSpeed);
                break;
        }
    }
}
