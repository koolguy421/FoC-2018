    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;

    import org.firstinspires.ftc.teamcode.vals;

    @TeleOp(name = "Theta")

    public class newTele extends LinearOpMode implements vals {

        /* Hardware Vars */
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private CRServo collector = null;
        private CRServo spinner = null;
        private DcMotor extender = null;
        private DcMotor lifter = null;
        private DcMotor flipper = null;

        /* Drive Vars */
        int mode = 1;
        double drive = 0.0;
        double drive1 = 0.0;


        public void initialize() {

            lifter = hardwareMap.get(DcMotor.class, "pivotMotor");
            leftDrive = hardwareMap.get(DcMotor.class, "MotorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "MotorRight");
            extender = hardwareMap.get(DcMotor.class, "extender");
            flipper = hardwareMap.get(DcMotor.class, "flipper");
            collector = hardwareMap.get(CRServo.class, "collector");
            spinner = hardwareMap.get(CRServo.class, "spinner");

            lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }


        @Override
        public void runOpMode() {

            initialize();
            waitForStart();


            while (opModeIsActive()) {


                /* Uses triggers to move rack/pinion */
                if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {

                    lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if (gamepad2.right_trigger > gamepad2.left_trigger) {
                        lifter.setPower(gamepad2.right_trigger);

                    }

                    if (gamepad2.right_trigger < gamepad2.left_trigger) {
                        lifter.setPower(-gamepad2.left_trigger);

                    }

                } else if(gamepad2.right_bumper) {

                    lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter.setTargetPosition(MAX_POS_LIFT);
                    lifter.setPower(0.8);

                } else if(gamepad2.left_bumper) {

                    lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter.setTargetPosition(0);
                    lifter.setPower(0.8);

                } else if(lifter.isBusy() != true){
                    lifter.setPower(0);
                }


                /* Collects Minerals */
                if (gamepad2.dpad_down) {
                    spinner.setPower(SPIN_SPEED);
                } else if (gamepad2.dpad_up) {
                    spinner.setPower(-SPIN_SPEED);
                } else if(gamepad2.dpad_left) {
                    spinner.setPower(SPIN_SPEED);
                } else {
                    spinner.setPower(SPIN_SPEED);
                }


                /* Flips Collecting Bucket */
                if (gamepad2.y) {
                    collector.setPower(SPIN_SPEED);
                } else if (gamepad2.a) {
                    collector.setPower(-SPIN_SPEED);
                } else {
                    collector.setPower(0);
                }


                extender.setPower(-gamepad2.left_stick_y);

                if(gamepad2.right_stick_y < 0){
                    flipper.setPower(gamepad2.right_stick_y * SLOW_SPEED);
                }else if(gamepad2.right_stick_y > 0) {
                    flipper.setPower(gamepad2.right_stick_y * 0.2);
                }else{
                    flipper.setPower(0);
                }


                /* Movement */
                if (gamepad1.y) {
                    mode = 2;
                }

                if(gamepad1.b){
                    mode = 3;
                }

                if(gamepad1.x){
                    mode = 1;
                }

                if (mode == 1) {
                    drive = gamepad1.left_stick_y * NORMAL_SPEED;
                    drive1 = gamepad1.right_stick_y * NORMAL_SPEED;
                } else if(mode == 2) {
                    drive = gamepad1.left_stick_y * FACT;
                    drive1 = gamepad1.right_stick_y * FACT;
                } else if(mode == 3){
                    drive = gamepad1.left_stick_y * FAST_SPEED;
                    drive1 = gamepad1.right_stick_y * FAST_SPEED;
                }

                leftDrive.setPower((drive1));
                rightDrive.setPower((drive));
            }
        }
    }




