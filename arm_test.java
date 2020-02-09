package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "armTest", group = "Linear Opmode")
public class arm_test extends LinearOpMode
{

    DigitalChannel btn;

    Servo intake1 = null;
    Servo intake2 = null;

    @Override
    public void runOpMode() throws InterruptedException
    {

        intake1 = hardwareMap.get(Servo.class, "in1");
        intake2 = hardwareMap.get(Servo.class, "in2");

        // get a reference to our digitalTouch object.
        btn = hardwareMap.get(DigitalChannel.class, "btn");

        // set the digital channel to input.
        btn.setMode(DigitalChannel.Mode.INPUT);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.left_trigger > 0)
            {


                intake1.setPosition(1);
                intake2.setPosition(0);

            }

            if (gamepad1.right_trigger > 0)
            {
                intake1.setPosition(0);
                intake2.setPosition(1);
            } else
            {
                intake1.setPosition(0.5);
                intake2.setPosition(0.5);
            }

            if (!btn.getState())
            {
                intake1.setPosition(0.5);
                intake2.setPosition(0.5);
            }

        }
    }
}
