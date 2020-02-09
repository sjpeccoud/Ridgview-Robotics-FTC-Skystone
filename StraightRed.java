package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "StraightRed", group = "Linear Opmode")
public class StraightRed extends SkyStone
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

        while(opModeIsActive())
        {
            capstoneHome();
            armHome();
            sleep(1000);
            //sleep(18000);
            driveEncoderPID(25);
            strafeEncoder(0.3, 1, 1);
            break;
        }

    }
}
