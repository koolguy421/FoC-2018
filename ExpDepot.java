/**
 *  Working branch paths (probably) + value tuning
 */

package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.OpenCV.AutoCVAlgorithm;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;


@Autonomous(name = "ExpDepot")
public class ExpDepot extends LinearOpMode {

    //All class components (soon to extend lowering motor/marker servo)
    public DcMotor motorL;
    public DcMotor motorR;
    public static BNO055IMU imu;
    public CRServo markerServo;
    public DcMotor latchM;

    //Encoder values, might need to account for gear train reduction
    private static double inchesPerRot = (228) / (5.5 * Math.PI);
    private static double target;
    private static int tarPosL;
    private static int tarPosR;

    //IMU setup and values
    static Orientation angles;
    static double heading = 0.0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //CV components
    private AutoCVAlgorithm c1 = new AutoCVAlgorithm();
    private static int xCor;
    private static int yCor;





    public void runOpMode() throws InterruptedException {


        //Hardware Maps
        motorL = hardwareMap.dcMotor.get("MotorLeft");
        motorR = hardwareMap.dcMotor.get("MotorRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        latchM = hardwareMap.get(DcMotor.class, "pivotMotor");
        markerServo = hardwareMap.get(CRServo.class,"crServo");

        //Set reverse on left motor to get motors to have same polarity
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Get OpenCV initialized
        imuSetUp();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();



        while (opModeIsActive()) {

            //Landing code
            latchM.setPower(0.8);
            Thread.sleep(2000);
            latchM.setPower(0);

            //Move foward and turn/turn to be out of latch
            imuTurnCounter(0.4,15);
            Thread.sleep(500);
            encoderMove(3,0.3,false);
            Thread.sleep(500);
            imuTurnClock(0.4,-15);


            //OpenCV detection of mineral
            c1.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            c1.setShowCountours(true);
            c1.enable();

            Thread.sleep(1000);

            List<MatOfPoint> contours = c1.getContours();
            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                xCor = ((boundingRect.x + boundingRect.width) / 2);
                yCor = ((boundingRect.y + boundingRect.height) / 2);
            }

            telemetry.addLine("yCor: " + yCor);
            telemetry.update();
            Thread.sleep(500);

            //Experimental
            c1.disable();


            if(yCor > 0 && yCor <= 100){

                //Turn counter-clockwise (positive) to hit ball and move past
                imuTurnCounter(0.3,21);
                target = 54;

                Thread.sleep(1000);

                //Move to hit ball
                encoderMove(target,0.4,false);
                Thread.sleep(1000);

                //Turn perpendicular to depot
                imuTurnClock(0.3,-40);
                target = 10;

                Thread.sleep(500);

                //Go forward to depot
                encoderMove(target,0.4,false);
                Thread.sleep(500);

                //run servo code
                markerServo.setPower(1);
                Thread.sleep(1500);
                markerServo.setPower(0);

                //Move to crater
                target = 45;
                encoderMove(target,0.4,true);


            } else if(yCor > 100 && yCor <= 200){

                //Move straight ahead to hit cube and get to depot
                target = 50;
                encoderMove(target,0.4,false);
                Thread.sleep(500);

                //Servo deposit code
                markerServo.setPower(1);
                Thread.sleep(1500);
                markerServo.setPower(0);

                //Turn parallel to depot
                imuTurnCounter(0.25,45);
                target = 6;
                Thread.sleep(500);

                //Move slightly downward to avoid hitting ball
                encoderMove(target,0.3,false);
                Thread.sleep(500);

                //Turn to be perpendicular to depot
                imuTurnCounter(0.25,135);
                Thread.sleep(500);

                //Move to crater
                target = 45;
                encoderMove(target,0.4,false);


            } else if(yCor > 200){

                //Turn clockwise (negative) to left
                imuTurnClock(0.3, -21);
                target = 50;

                Thread.sleep(500);

                //Start moving to hit cube given set target distance to cover
                encoderMove(target, 0.4, false);
                Thread.sleep(500);

                //Turn to be perpendicular to depot
                imuTurnCounter(0.25, 45);
                target = 18;
                Thread.sleep(500);

                //Move towards depot to deposit marker
                encoderMove(target,0.4, false);
                Thread.sleep(500);

                //servo code
                markerServo.setPower(1);
                Thread.sleep(1500);
                markerServo.setPower(0);

                //Move towards crater
                target = 45;
                encoderMove(target, 0.4, true);

            } else {
                altBranch();
            }


            Thread.sleep(10000);

        }




    }





    //Movement
    private void encoderMove(double inches,double speed,boolean movingBackwards){

        tarPosR = motorR.getCurrentPosition() + (int)(inches * inchesPerRot);
        tarPosL = motorL.getCurrentPosition() + (int)(inches * inchesPerRot);

        if(!movingBackwards) {
            motorR.setTargetPosition(tarPosR);
            motorL.setTargetPosition(tarPosL);
        } else {
            motorR.setTargetPosition(-tarPosR);
            motorL.setTargetPosition(-tarPosL);
        }

        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorR.setPower(speed);
        motorL.setPower(speed);

        while(motorR.getCurrentPosition() != motorR.getTargetPosition() && motorL.getCurrentPosition() != motorL.getTargetPosition()){
            idle();
        }

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorR.setPower(0);
        motorL.setPower(0);
    }

    //Turning Methods
    private void imuTurnCounter (double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle)) {
            telemetry.addData("A:",heading);
            telemetry.update();
        }
    }

    private void imuTurnClock (double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle)) {
            telemetry.addData("A:",heading);
            telemetry.update();
        }
    }

    private boolean onHeading(double speed, double angle) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorL.setPower(-speed);
        motorR.setPower(speed);

        if(heading > angle){
            onTarget = true;
            motorL.setPower(0);
            motorR.setPower(0);
        }

        return onTarget;
    }

    private boolean onHeadingNegative(double speed, double angle) {

        boolean onTarget = false ;

        returnAngles();

        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorL.setPower(speed);
        motorR.setPower(-speed);

        if(heading < angle){
            onTarget = true;
            motorR.setPower(0);
            motorL.setPower(0);
        }

        return onTarget;
    }

    private static String formatAngle(AngleUnit angleUnit, double angle) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private static void returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    private void imuSetUp() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    //Maintenance Methods
    private void altBranch() {

    }











}

