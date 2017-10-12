package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.General.RobotDrive;

@Autonomous(name="Autonomous Mode", group="Revved Up")
public class AutonomousMode extends LinearOpMode {
    private static final double turningSpeed = 0.5;
    private static final int turnDistance = 50;

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
                knockdownJewel ? turnDistance : -turnDistance,
                knockdownJewel ? -turnDistance : turnDistance,
                turningSpeed,
                true
        );
        jewelHandler.retractServo();
        robot.incrementMotorPosition(
                knockdownJewel ? -turnDistance : turnDistance,
                knockdownJewel ? turnDistance : -turnDistance,
                turningSpeed,
                true
        );
    }
}
