package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.vals;

@TeleOp(name = "Epsilon")

public class newTele extends LinearOpMode implements vals {

    /* Hardware Vars */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collector = null;
    private DcMotor extender = null;
    private DcMotor lifter = null;
    private DcMotor flipper = null;

    /* Drive Vars */
    int mode = 1;
    double drive = 0.0;

    /* Controls */
    private double ONE_LY ;
    private double ONE_RY ;
    private double TWO_LY ;
    private double TWO_RY ;
    private double TWO_RT ;
    private double TWO_LT ;



    public void initialize()
    {
        lifter = hardwareMap.get(DcMotor.class, "pivotMotor");
        leftDrive = hardwareMap.get(DcMotor.class,"MotorLeft");
        rightDrive = hardwareMap.get(DcMotor.class,"MotorRight");
        extender = hardwareMap.get(DcMotor.class,"extender");
        flipper = hardwareMap.get(DcMotor.class,"flipper");
        collector = hardwareMap.get(DcMotor.class,"Collector");

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        TWO_LT = gamepad2.left_trigger;
        TWO_RT = gamepad2.right_trigger;
        TWO_LY = gamepad2.left_stick_y;
        TWO_RY = gamepad2.right_stick_y;

        ONE_LY = gamepad1.left_stick_y;
        ONE_RY = gamepad1.right_stick_y;
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();


        while (opModeIsActive()) {


            /* Uses triggers to move rack/pinion */
            if(TWO_LT > 0 || TWO_RT > 0)
            {

                if(TWO_RT > TWO_LT)
                {
                    lifter.setPower(TWO_RT);

                } else if(TWO_RT < TWO_LT)
                {
                    lifter.setPower(-TWO_LT);
                }
            } else
            {
                lifter.setPower(0);
            }


            /* Collecting Mechanism using button */
            if(gamepad2.b)
            {
                collector.setPower(SPIN_SPEED);

            } else if(gamepad2.y)
            {
                collector.setPower(-SPIN_SPEED);

            } else
            {
                collector.setPower(0);
            }


            /* Two mechs to deposit */
            extender.setPower(TWO_LY * EXTEND_SPEED);
            flipper.setPower(TWO_RY * SLOW_SPEED);


            /* Toggle code */
            if(gamepad1.a){
                mode++;
            }

            if(mode % 2 == 0)
            {
                drive = ONE_RY * SLOW_SPEED;
            } else
            {
                drive = ONE_LY * FAST_SPEED;
            }

            /* Movement */
            leftDrive.setPower(powerFunc(drive));
            rightDrive.setPower(powerFunc(drive));


        }


    }


    private static double powerFunc(double speed)
    {
        double cubic = Math.pow(speed,3);
        double dividend = Math.abs(speed);

        return(cubic / dividend);
    }



}






