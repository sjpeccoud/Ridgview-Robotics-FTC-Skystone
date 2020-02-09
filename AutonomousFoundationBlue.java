package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "AutonomousFoundationBlue", group = "Linear Opmode")
public class AutonomousFoundationBlue extends SkyStone
{


    @Override
    public void runOpMode()
    {
        //Initialization Period
        hardware.init(hardwareMap);

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Initalizes The Arm Position
        capstoneInit();
        armInit();
        clampRelease();
        hooksUp();

        telemetry.addData(">:", "Ready For Start");
        telemetry.update();
        sleep(100);

        waitForStart();

        //////////////////Game Start/////////////////////////////////////////////////////////////////////////////////////

        while (opModeIsActive())
        {
            armHome();

            sleep(15000);
            driveEncoderPID(-29);
            straightForTime(-0.3, 0.3);
            capstoneHome();

            hooksDown();
            sleep(1000);
            driveEncoderPID(20);
            gyroTurnPID(-90);
            gyroTurnPID(170);
            driveEncoderPID(-10);
            hooksUp();
            gyroTurnPID(180);
            strafeEncoder(0.4, -60, 3);
        }
    }
}
