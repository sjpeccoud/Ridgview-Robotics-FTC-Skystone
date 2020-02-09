package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="SkyStone TeleOp", group="Linear Opmode")
public class SkyStoneTeleOp extends SkyStone {

    @Override
    public void runOpMode()
    {

        //Initialization Period
        hardware.init(hardwareMap);

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Initalizes The Arm Position
        armInit();
        hooksUp();
        clampRelease();
        capstoneHome();
        //capstoneInit();
        autoArmIntakeInit();

        telemetry.addData(">:", "Ready For Start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        ///////////////GAME START////////////////////////////////

        autoArmIntakeHome();
        capstoneHome();
        armHome();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            //automated
            autoClamp();

            //gamepad1 and gamepad2
            mecanumControl(gamepad2.right_trigger);

            //gamepad1
            intakeControl();
            hooksControl(gamepad1.y);
            //autoStackBack(gamepad1.b);
            liftSystemManualControl(gamepad1.dpad_up, gamepad1.dpad_down);
            cardinalDirectionTurn(gamepad1.right_bumper);


            //gamepad2
            //Button A Already Cuts out of the auto stack
            levelControl();
            capstoneControl(gamepad2.y);
            //autoArmDropControl(gamepad2.b);

            //TEST ON MONDAY
            raiseToStack(gamepad2.b);
            placeAndReturn(gamepad1.b);



            telemetry.update();

        }
    }
}
