package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Yaw rest (zero)")
public class basic extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.initialize(myIMUparameters);

        // This is the line that ends the init of the bot
        waitForStart();
        imu.resetYaw();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", Yaw);
            telemetry.addData("Pitch (X)", "%.2f Deg.", Pitch);
            telemetry.addData("Roll (Y)", "%.2f Deg.", Roll);

            // Sends it to the control hub
            telemetry.update();
        }
    }
}
