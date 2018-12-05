package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name = "BlueDepot")
public class BlueDepot extends LinearOpMode{

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor PULLEY = null;

    private Servo MARKER = null;
    private GoldAlignDetector detector;



    private ElapsedTime runtime = new ElapsedTime();

    static final double     TICKS_PER_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException


    {
        //Initialize motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        PULLEY = hardwareMap.dcMotor.get("PULLEY");
        MARKER = hardwareMap.servo.get("MARKER");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PULLEY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PULLEY.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        PULLEY.setDirection(DcMotor.Direction.REVERSE);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();


        // Optional Tuning
        detector.alignSize = 600; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        //changed from 200 to 400 then 600
        // 800 does not work
        detector.alignPosOffset = 5; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.6; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        telemetry.addData("Status" , "Running BlueDepot"); // Gold X pos.

        telemetry.update();

        waitForStart();
        pulleyDrive(.6,-83,30);
        strafeDrive(0.5,-3,3,30);

        String location = getMineralPos();
        telemetry.addData("MineralPosition",location);
        telemetry.addData("MineralPosition", "Running to %s : %s", location, location);
        telemetry.update();
        detector.disable();


        if ( location.equals("middle")) {
            encoderDrive(.6,10,10,30);// move forward
            strafeDrive(.5,5,-5,30);//strafe to mineral
            encoderDrive(.7,26,26,30);//hit mineral
            MARKER.setPosition(1); // drop marker
            sleep(1200);
            encoderDrive(.6,-23,-23,30);// go back
            encoderDrive(.5,-13,13,30); // turn to crater
            encoderDrive(.8,31,31,30);//drive to crater
            encoderDrive(.5,-4,4,30);// turn to crater
            encoderDrive(.8,12,12,30); //hit crater

        } else if(location.equals("left")) {
            encoderDrive(.7,9,9,30); // going forward
            encoderDrive(.5,-7,7,30); // turns to mineral
            encoderDrive(.7,20,20,30); // hit to mineral
            encoderDrive(.7,-17,-17,30); // going back
            encoderDrive(.5,-7,7,30);// turn left to crater
            encoderDrive(.7,24,24,30); // drive forward
            encoderDrive(.5,22,-22,30); // turn to depot
            strafeDrive(.5,-7,7,30);
            encoderDrive(.7,34,34,30); // drive to depot
            MARKER.setPosition(1);// marker drop
            sleep(1200); //wait
            encoderDrive(1,-50,-50,30); // driving back into crater






        } else { //moving to right
            encoderDrive(.6,6,6,30); // move forward
            encoderDrive(.5,7.5,-7.5,30); // turn to mineral
            encoderDrive(.8,21,21,30); // hit mineral
            encoderDrive(.8,-14,-14,30); //moving backward
            encoderDrive(.5,7,-7,30); // turn
            encoderDrive(.7,-30,-30,30); // drive forward
            encoderDrive(.5,-9,9,30); // turn to depot
            strafeDrive(.5,-5,5,30);//Strafe to wall
            encoderDrive(.7,34,34,30); // drive to depot
            MARKER.setPosition(1);// marker drop
            sleep(1200); //wait
            encoderDrive(1,-51,-51,30); // driving back into crater */



        }
        sleep(10000);

        // put (-x) for inches
        // 9.5 of negative
        //encoderDrive(.3,6,6,10); Move robot forward
        //encoderDrive(.3,-6,-6,10); Move robot backward.
        //encoderDrive(.3,9.5,-9.5,10);  Turn Right 90 Degree       9.5 is the real number for the
        //encoderDrive(.3,-9.5,9.5,10);  Turn Left 90 Degree        turning of 90 Degree.


// Positive goes down
        // pulleyDrive(.2, 26, 10);


        // strafeDrive(.3, 2, -2, 5);







        // Initialize Servo


    }
    // (power)0.5 (time) 1000 is a almost perfect 90 degree turn.
    // (power) 1 (time) 1275 is almost a perfect 180 degree turn.



    //public void runOpModethrows InterruptedException

      /* public void encoderDrive(double speedleft,double speedright,
                                double leftInches, double rightInches,
                                double timeoutS)*/

    public void encoderDrive(double speed,
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
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
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
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void pulleyDrive(double speed,
                            double Inches,
                            double timeoutS)
    {

        int PulleyDistance;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            PulleyDistance = PULLEY.getCurrentPosition() + (int)(Inches* COUNTS_PER_INCH);



            PULLEY.setTargetPosition(PulleyDistance);



            // Turn On RUN_TO_POSITION
            PULLEY.setMode(DcMotor.RunMode.RUN_TO_POSITION);


          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            PULLEY.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 10) &&
                    (PULLEY.isBusy() )) {

                // Display it for the driver.
                //  telemetry.addData("Path1",  "Running to %7d :%7d", PulleyDistance);
                //telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();





            }

            // Stop all motion;
            PULLEY.setPower(0);



            // Turn off RUN_TO_POSITION
            PULLEY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafeDrive(double speed,
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
            newLeftBackTarget = BL.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
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
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


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
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public String getMineralPos() {
        String location = "";
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {


            if (detector.getAligned() == true) {
                if (detector.getXPosition() > 300) {
                    location = "middle";
                } else {
                    location = "left";
                }

            } else {
                location = "right";
            }
            // telemetry.addData("IsAligned", SampleFinder.init_detect(hardwareMap)); // Is the bot aligned with the gold mineral
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
            telemetry.addData("Location", location); // Gold X pos.

            telemetry.update();
        }
        return location;


    }


}







