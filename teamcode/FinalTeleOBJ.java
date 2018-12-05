package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "FinalTeleOBJ")
public class FinalTeleOBJ extends LinearOpMode {
    public float x, y, z, w, pwr;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor PULLEY;
    private DigitalChannel TopSensor;
    private DigitalChannel BottomSensor;
    private DcMotor flipper;
    private DcMotor extendpulley;
    private DcMotor grabber;

    private Servo MARKER;

    private static final double ARM_UP = 0.0;
    private static final double ARM_DOWN = 1.0;

    private static final double PHONE_UP = 0.0;
    private static final double PHONE_DOWN = 1.0;




    @Override
    public void runOpMode ()  throws InterruptedException
    {
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        flipper = hardwareMap.dcMotor.get("flipper");
        extendpulley = hardwareMap.dcMotor.get("extendpulley");
        grabber = hardwareMap.dcMotor.get("grabber");
        PULLEY = hardwareMap.dcMotor.get("PULLEY");
        MARKER = hardwareMap.servo.get("MARKER");

        TopSensor = hardwareMap.get(DigitalChannel.class, "TopSensor");
        BottomSensor = hardwareMap.get(DigitalChannel.class,"BottomSensor" );
        TopSensor.setMode(DigitalChannel.Mode.INPUT);
        BottomSensor.setMode(DigitalChannel.Mode.INPUT);



        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        MARKER.setPosition(ARM_UP);
        pwr = y;        PULLEY.setPower(w);


        FR.setPower(Range.clip(pwr - x+z, -1, 1));
        BL.setPower(Range.clip(pwr - x-z, -1, 1));
        FL.setPower(Range.clip(pwr + x-z, -1, 1));
        BR.setPower(Range.clip(pwr + x+z, -1, 1));
        PULLEY.setPower(w);

        waitForStart();
        while (opModeIsActive()) {

            FL.setPower(-gamepad1.left_stick_y);
            FR.setPower(-gamepad1.right_stick_y);
            BL.setPower(-gamepad1.left_stick_y);
            BR.setPower(-gamepad1.right_stick_y);

            flipper.setPower(-gamepad2.left_stick_y);
            flipper.setPower(gamepad2.right_stick_y);


            FL.setPower(-gamepad1.left_stick_x);
            FR.setPower(gamepad1.right_stick_x);
            BL.setPower(gamepad1.right_stick_x);
            BR.setPower(-gamepad1.left_stick_x);

            if (gamepad2.left_trigger > .3)
            {
                grabber.setDirection(DcMotor.Direction.FORWARD);
                grabber.setPower(gamepad2.left_trigger);
            }
            else if (gamepad2.right_trigger > .3)
            {
                grabber.setDirection(DcMotor.Direction.REVERSE);
                grabber.setPower(gamepad2.right_trigger);
            } else{
                grabber.setPower(0);
            }

            if((TopSensor.getState()== true)) {
                PULLEY.setDirection(DcMotor.Direction.FORWARD);
                PULLEY.setPower(gamepad1.right_trigger);

                //PULLEY.setPower(0);
                telemetry.addData("Not Pressed", TopSensor.getState());
                telemetry.update();
            }
            else
            {
                telemetry.addData("Pressed", TopSensor.getState());
                PULLEY.setPower(-.5);
                telemetry.update();

            }

            if((BottomSensor.getState()== true)) {
                PULLEY.setDirection(DcMotor.Direction.FORWARD);
                PULLEY.setPower(-gamepad1.left_trigger);
            }
            else
            {
                //telemetry.addData("Pressed", TopSensor.getState());
                PULLEY.setPower(.5);
                //telemetry.update();

            }





            // if((BottomSensor.getState() == false)) {
            //PULLEY.setPower(0);
            //}






            // if((BottomSensor.getState() == false)) {
            //PULLEY.setPower(0);
            //}


            if(gamepad1.a)
            {
                MARKER.setPosition(ARM_UP);
            }
            if(gamepad1.b)
            {
                MARKER.setPosition(ARM_DOWN);
            }





        }





        idle();
    }
}
