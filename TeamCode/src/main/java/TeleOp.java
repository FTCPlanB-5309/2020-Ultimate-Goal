import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")

public class TeleOp extends LinearOpMode {

    RobotHardware robot = new RobotHardware();


    @Override
    public void runOpMode() {
        int stick_direction;
        double ly;
        double rx;
        double lx;
        boolean slow_mode;
        boolean normal_mode;
        boolean scoop = true;
        boolean fire = false;
//        boolean slow_mode;
//        boolean normal_mode;

        robot.teleopInit(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_CLOSED);
            }

            if (gamepad2.left_trigger > 0.8) {
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
            }

            if (Math.abs(gamepad2.left_stick_y) > robot.TELEOPDEADZONE) {
                robot.wobbleMotor.setPower(-gamepad2.left_stick_y);

            }
            else robot.wobbleMotor.setPower(0);


            if(gamepad2.right_bumper) {
                robot.liftServo.setPosition(robot.LAUNCHER_LOAD_ANGLE);
                robot.scoopServo.setPosition(robot.DROPPING_POSITION);
                scoop = true;
            }
            else if (scoop)
                robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            if (gamepad2.dpad_up) {
                scoop = false;
                robot.scoopServo.setPosition(robot.DRIVING_POSITION);
            }
            if (gamepad2.dpad_down) {
                robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            }

            if (gamepad2.right_trigger > 0.8)
                robot.launchServo.setPosition(robot.LAUNCHER_FIRE_POSITION);
            else
                robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);

            if(Math.abs(gamepad2.right_stick_y) > robot.TELEOPDEADZONE) {
                if(gamepad2.right_stick_y > 0)
                    stick_direction = 1;
                else stick_direction = -1;
                robot.liftServo.setPosition(robot.liftServo.getPosition() + 0.001 * stick_direction);
            }


            if(gamepad2.a){
                robot.leftLauncherMotor.setPower(1);
                robot.rightLauncherMotor.setPower(1);
            }

            if(gamepad2.b){
                robot.leftLauncherMotor.setPower(0);
                robot.rightLauncherMotor.setPower(0);
            }

            if(gamepad2.x){
                robot.liftServo.setPosition(robot.LAUNCHER_POWER_ANGLE);
            }

            if(gamepad2.y){
                robot.liftServo.setPosition(robot.LAUNCHER_HIGH_ANGLE);
            }

            slow_mode = false;
            normal_mode = false;

            //strafe and turn right slowly with dpad
            if (gamepad1.dpad_right ||
                    gamepad1.dpad_left ||
                    gamepad1.dpad_up ||
                    gamepad1.dpad_down) {
                slow_mode = true;
                normal_mode = false;


                if (gamepad1.dpad_right) {
                    //turn right slowly with dpad
                    if (gamepad1.b) {
                        robot.leftFrontDrive.setPower(0.05);
                        robot.rightFrontDrive.setPower(-0.05);
                        robot.leftRearDrive.setPower(0.05);
                        robot.rightRearDrive.setPower(-0.05);
                    }
                    //strafe right slowly with dpad
                    else {
                        robot.leftFrontDrive.setPower(0.05);
                        robot.rightFrontDrive.setPower(-0.05);
                        robot.leftRearDrive.setPower(-0.05);
                        robot.rightRearDrive.setPower(0.05);
                    }
                    //strafe and turn left slowly with dpad
                } else if (gamepad1.dpad_left) {
                    //turn left slowly with dpad
                    if (gamepad1.b) {
                        robot.leftFrontDrive.setPower(-0.05);
                        robot.rightFrontDrive.setPower(0.05);
                        robot.leftRearDrive.setPower(-0.05);
                        robot.rightRearDrive.setPower(0.05);
                    }
                    //strafe left slowly with dpad
                    else {
                        robot.leftFrontDrive.setPower(-0.05);
                        robot.rightFrontDrive.setPower(0.05);
                        robot.leftRearDrive.setPower(0.05);
                        robot.rightRearDrive.setPower(-0.05);
                    }
                    //drive forward slowly with dpad
                } else if (gamepad1.dpad_up) {
                    robot.leftFrontDrive.setPower(0.07);
                    robot.rightFrontDrive.setPower(0.07);
                    robot.leftRearDrive.setPower(0.07);
                    robot.rightRearDrive.setPower(0.07);
                    //drive backward slowly with dpad
                } else if (gamepad1.dpad_down) {
                    robot.leftFrontDrive.setPower(-0.07);
                    robot.rightFrontDrive.setPower(-0.07);
                    robot.leftRearDrive.setPower(-0.07);
                    robot.rightRearDrive.setPower(-0.07);
                }
            }
            // Normal mode
            else {
                ly = -gamepad1.left_stick_y; //drive forward
                lx = -gamepad1.left_stick_x; //strafe
                rx = -gamepad1.right_stick_x; //turn

                if (Math.abs(ly) > robot.TELEOPDEADZONE ||
                        Math.abs(lx) > robot.TELEOPDEADZONE ||
                        Math.abs(rx) > robot.TELEOPDEADZONE) {

                    normal_mode = true;
                    slow_mode = false;
                    // Compute the drive speed of each drive motor based on formula from redit
                    double FL_power_raw = ly + lx - (rx * .7f);
                    double FR_power_raw = ly - lx + (rx * .7f);
                    double RL_power_raw = ly - lx - (rx * .7f);
                    double RR_power_raw = ly + lx + (rx * .7f);

                    //Clip the values generated by the above formula so that they never go outisde of -1 to 1
                    double FL_power = Range.clip(FL_power_raw, -1, 1);
                    double FR_power = Range.clip(FR_power_raw, -1, 1);
                    double RL_power = Range.clip(RL_power_raw, -1, 1);
                    double RR_power = Range.clip(RR_power_raw, -1, 1);

                    if (!gamepad1.a) { //This is normal mode
                        FL_power = FL_power * robot.NOTTURBOFACTOR;
                        FR_power = FR_power * robot.NOTTURBOFACTOR;
                        RL_power = RL_power * robot.NOTTURBOFACTOR;
                        RR_power = RR_power * robot.NOTTURBOFACTOR;
                    }

                    robot.leftFrontDrive.setPower(FL_power);
                    robot.rightFrontDrive.setPower(FR_power);
                    robot.leftRearDrive.setPower(RL_power);
                    robot.rightRearDrive.setPower(RR_power);

                }
                if (slow_mode == false && normal_mode == false) {
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                }
            }
            telemetry.addData("FLD", robot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("FRD", robot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("RLD", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("RRD", robot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("liftServoPosition", robot.liftServo.getPosition());
            telemetry.update();

//                }
//                if (slow_mode == false && normal_mode == false) {
//                    robot.leftFrontDrive.setPower(0);
//                    robot.rightFrontDrive.setPower(0);
//                    robot.leftRearDrive.setPower(0);
//                    robot.rightRearDrive.setPower(0);
//                }
//            }
//
//
//            if (Math.abs(gamepad2.right_stick_y) > robot.TELEOPDEADZONE) { //Move the lift up and down
//                robot.liftMotor.setPower(Range.clip(gamepad2.right_stick_y, -1.0, 1.0));
//            } else {
//                robot.liftMotor.setPower(0);
//            }
//
//
//            if (gamepad1.left_trigger > 0.5) { // Grab the foundation
//                robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_DOWN);
//                robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_DOWN);
//            }
//
//            if (gamepad1.left_bumper) { // Let go of the foundation
//                robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
//                robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
//            }
//
//
//            if (gamepad2.y) {
//                robot.capStoneServo.setPosition(robot.CAPSTONE_SERVO_IN);
//            }
//
//            if (gamepad2.a) {
//                robot.capStoneServo.setPosition(robot.CAPSTONE_SERVO_OUT);
//            }
//
//            if (gamepad2.b && !gamepad2.start) {
//                robot.capStoneHolder.setPosition(robot.CAPSTONE_HOLDER_RELEASE);
//            }
//            if (gamepad2.x) {
//                robot.capStoneHolder.setPosition(robot.CAPSTONE_HOLDER_GRAB);
//            }
//
//
//            if (gamepad2.left_bumper) { //Let go of the block
//                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
//
//            } else if (gamepad2.left_trigger > 0.5) { //Grab a block
//                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
//            }
//
//            if (gamepad2.dpad_up) {
//                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
//            }
//
//            if (gamepad2.dpad_down) {
//                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
//            }
//
//
//            // Pace this loop so jaw action is reasonable speed.
//            sleep(50);
//
       }
    }
}