package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Autonomous.JewelHandler;
import org.firstinspires.ftc.teamcode.Autonomous.JewelHandler.Team;
import org.firstinspires.ftc.teamcode.Autonomous.JewelHandler.Sensor;
import org.firstinspires.ftc.teamcode.Autonomous.PictographIdentification;
import org.firstinspires.ftc.teamcode.General.RobotDrive;

@Autonomous(name = "Autonomous Testing")
public class Autonomous_Testing extends LinearOpMode{

    PictographIdentification pictographIdentification = new PictographIdentification();
    JewelHandler jewelHandler = new JewelHandler();
    RobotDrive robot = new RobotDrive();

    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode () throws InterruptedException {
        telemetry.setAutoClear(false);

        Telemetry.Item pictographTele = telemetry.addData("Pictograph: ", RelicRecoveryVuMark.UNKNOWN);
        Telemetry.Item teamColor = telemetry.addData("Team: ", Team.UNKNOWN);
        Telemetry.Item angle1 = telemetry.addData("Angle1: ", 0);
        Telemetry.Item angle2 = telemetry.addData("Angle2: ", 0);
        Telemetry.Item angle3 = telemetry.addData("Angle3: ", 0);


        pictographIdentification.init(hardwareMap);
//        jewelHandler.init(
//                hardwareMap,
//                "colorBalancingStone",
//                "colorJewel",
//                "jewelServo"
//        );
//        robot.init(
//                hardwareMap,
//                "leftDrive",
//                "rightDrive"
//        );


        waitForStart();

        runTime.reset();

        pictographIdentification.start();

        telemetry.update();

        while (opModeIsActive()) {
            Orientation rot = pictographIdentification.getPictographAngle();
            if (rot != null) {
                angle1.setValue(rot.firstAngle);
                angle2.setValue(rot.secondAngle);
                angle3.setValue(rot.thirdAngle);
            }

            pictographIdentification.checkForPictograph();
            pictographTele.setValue(pictographIdentification.getCubePosition());
            telemetry.update();

//            teamColor.setValue(jewelHandler.computeColor(Sensor.BALANCING_STONE));
//            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
