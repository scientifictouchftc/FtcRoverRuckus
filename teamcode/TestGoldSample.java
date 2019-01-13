package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestGoldSample")
public class TestGoldSample extends LinearOpMode{
   /* private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null; */


    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    static final double     TICKS_PER_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.7 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException

    {
        //Initialize motors
      /*  FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        //pulley = hardwareMap.dcMotor.get("pulley");

        FR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(RunMode.STOP_AND_RESET_ENCODER);


        FR.setMode(RunMode.RUN_TO_POSITION);
        FL.setMode(RunMode.RUN_TO_POSITION);
        BL.setMode(RunMode.RUN_TO_POSITION);
        BR.setMode(RunMode.RUN_TO_POSITION);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE); */

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 600; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        //changed from 200 to 400 then 600
        // 800 does not work
        detector.alignPosOffset = 5; // How far from center frame to offset this alignment zone.
        detector.downscale = .6; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;



        detector.enable();
        telemetry.addData("Location" , "Running"); // Gold X pos.
        telemetry.update();


        /////////////////
        // Yellow Middle is 560
        // Yellow Left is 213

        waitForStart();

        /* SampleFinder SampleFinder = new SampleFinder();
        SampleFinder.init_detect(hardwareMap);*/
        while (opModeIsActive() && (runtime.seconds() < 31.0)) {

            String location= "" ;

            if( detector.getAligned() == true){
                if( detector.getXPosition() > 300 ) {
                    location="middle";
                } else {
                    location="left";
                }

            } else {
                location = "right";
            }
            // telemetry.addData("IsAligned", SampleFinder.init_detect(hardwareMap)); // Is the bot aligned with the gold mineral
            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
            telemetry.addData("Location2" , location); // Gold X pos.

            telemetry.update();
        }

        // put (-x) for inches
        // 9.5 of negative
        //encoderDrive(.3,6,6,10); Move robot forward
        //encoderDrive(.3,-6,-6,10); Move robot backward.
        //encoderDrive(.3,9.5,-9.5,10);  Turn Right 90 Degree       9.5 is the real number for the
        //encoderDrive(.3,-9.5,9.5,10);  Turn Left 90 Degree        turning of 90 Degree.

        // encoderDrive(.3,-5.3,5.3,10);
//encoderDrive(.6, 22, 22, 10);
        // encoderDrive(.4, 10, -10, 10);
        // encoderDrive(.3, 20, 20, 30);
        //encoderDrive(.6, -35, -35, 30);


        // Initialize Servo


    }
    // (power)0.5 (time) 1000 is a almost perfect 90 degree turn.
    // (power) 1 (time) 1275 is almost a perfect 180 degree turn.



    //public void runOpModethrows InterruptedException

       /* public void encoderDrive(double speedleft,double speedright,
                                 double leftInches, double rightInches,
                                 double timeoutS)*/

  /*  public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(RunMode.RUN_TO_POSITION);
            FL.setMode(RunMode.RUN_TO_POSITION);
            BR.setMode(RunMode.RUN_TO_POSITION);
            BL.setMode(RunMode.RUN_TO_POSITION);

           /* runtime.reset();
            BL.setPower(Math.abs(speedleft));
            BR.setPower(Math.abs(speedright));
            FL.setPower(Math.abs(speedleft));
            FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
        /*    runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 31.0) &&
                    (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();





            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);



            // Turn off RUN_TO_POSITION
            FR.setMode(RunMode.RUN_USING_ENCODER);
            FL.setMode(RunMode.RUN_USING_ENCODER);
            BR.setMode(RunMode.RUN_USING_ENCODER);
            BL.setMode(RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void DriveForwardDistance ( int ticks)

    {
        FR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(RunMode.STOP_AND_RESET_ENCODER);

    /*
            drive_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
            FR.setMode(DcMotor.RunMode.RESET_ENCODERS);
            drive_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
            FL.setMode(DcMotor.RunMode.RESET_ENCODERS);
    */
      /*  FL.setTargetPosition(ticks);
        FR.setTargetPosition(ticks);
        BL.setTargetPosition(ticks);
        BR.setTargetPosition(ticks);

        FR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(RunMode.STOP_AND_RESET_ENCODER);


    /*
            drive_left.setMode(RunMode.RUN_USING_ENCODER);
            FR.setMode(RunMode.RUN_USING_ENCODER);
            drive_right.setMode(RunMode.RUN_USING_ENCODER);
            FL.setMode(RunMode.RUN_USING_ENCODER);




            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(RunMode.RUN_TO_POSITION);

            while(drive_left.isBusy()&& FR.isBusy())
                while (FL.isBusy()&& drive_right.isBusy());


               /*  FR.setPower(distance);
            FL.setPower(distance);
            drive_left.setPower(distance);
            drive_right.setPower(distance);

            */



    }






