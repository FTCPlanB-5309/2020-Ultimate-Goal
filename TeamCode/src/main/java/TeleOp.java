import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")

public class TeleOp extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    RingArm ringArm = new RingArm(robot, telemetry, this);


    @Override
    public void runOpMode() throws InterruptedException {
        int stick_direction;
        autoLoadState load_state = autoLoadState.GRAB;
        boolean autoLoadToggle = false;
        long system_time_base = 0;
        double ly;
        double rx;
        double lx;
        boolean slow_mode;
        boolean normal_mode;
        boolean scoop = true;
        boolean fire = false;
        boolean resetLiftMotor = false;

        robot.teleopInit(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("voltage", robot.voltage);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
        robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
        robot.scoopMotor.setTargetPosition(robot.DRIVING_POSITION);
        robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftServo.setPosition(robot.LAUNCHER_LOAD_ANGLE);
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
        robot.wobbleServoTwoProng.setPosition(robot.WOBBLE_SERVO_TWO_PRONG_OPEN);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.scoopMotor.setPower(0.9);

            if (gamepad1.back) {
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.scoopMotor.setPower(-0.5);
                resetLiftMotor = true;
            }
            else if (resetLiftMotor) {
                robot.scoopMotor.setPower(0);
                robot.scoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                resetLiftMotor = false;
            }

            if (gamepad2.left_bumper) {
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_CLOSED);
                robot.wobbleServoTwoProng.setPosition(robot.WOBBLE_SERVO_TWO_PRONG_CLOSED);
            }

            if (gamepad2.left_trigger > 0.8) {
                robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
                robot.wobbleServoTwoProng.setPosition(robot.WOBBLE_SERVO_TWO_PRONG_OPEN);
            }

            if (Math.abs(gamepad2.left_stick_y) > robot.TELEOPDEADZONE) {
                robot.wobbleMotor.setPower(-gamepad2.left_stick_y);

            }
            else robot.wobbleMotor.setPower(0);


            if(gamepad2.right_bumper) {
                robot.liftServo.setPosition(robot.LAUNCHER_LOAD_ANGLE);
                robot.scoopMotor.setTargetPosition(robot.DROPPING_POSITION);
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                scoop = true;
            }
            else if (scoop && !autoLoadToggle && !resetLiftMotor) {
                robot.scoopMotor.setTargetPosition(robot.SCOOPING_POSITION);
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_up) {
                scoop = false;
                robot.scoopMotor.setTargetPosition(robot.DRIVING_POSITION);
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.dpad_down) {
                robot.scoopMotor.setTargetPosition(robot.SCOOPING_POSITION);
                robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_left){
                robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
                robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            }
            if (gamepad2.dpad_right){
                robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
                robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
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


            if(gamepad2.right_stick_button) {
                autoLoadToggle = true;
            }

            if(autoLoadToggle) {
                switch(load_state) {
                    case GRAB:
                        if (system_time_base == 0)
                            system_time_base = System.currentTimeMillis();
                        robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
                        robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
                        if (System.currentTimeMillis() > system_time_base + robot.GRAB_TIMER) {
                            load_state = autoLoadState.RAISE;
                            system_time_base = 0;
                        }
                        break;
                    case RAISE:
                        robot.scoopMotor.setTargetPosition(robot.DROPPING_POSITION);
                        robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftServo.setPosition(robot.LAUNCHER_LOAD_ANGLE);
                        if(robot.scoopMotor.getCurrentPosition() > robot.DROPPING_POSITION)
                            load_state = autoLoadState.RELEASE;
                        break;
                    case RELEASE:
                        if (system_time_base == 0)
                            system_time_base = System.currentTimeMillis();
                        robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
                        robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
                        if(System.currentTimeMillis() > system_time_base + robot.RELEASE_TIMER) {
                            system_time_base = 0;
                            load_state = autoLoadState.RESET;
                        }
                        break;
                    case RESET:
                        robot.scoopMotor.setTargetPosition(robot.SCOOPING_POSITION);
                        robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if(!robot.scoopMotor.isBusy()) {
                            load_state = autoLoadState.GRAB;
                            autoLoadToggle = false;
                        }
                        break;
                }
            }


            if(gamepad2.a && !gamepad2.start){
                robot.leftLauncherMotor.setPower(robot.shooterWheelSpeed);
                robot.rightLauncherMotor.setPower(robot.shooterWheelSpeed);
            }

            if(gamepad2.b && !gamepad2.start){
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

                    if (!gamepad1.left_stick_button) { //This is normal mode
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
//            telemetry.addData("FLD", robot.leftFrontDrive.getCurrentPosition());
//            telemetry.addData("FRD", robot.rightFrontDrive.getCurrentPosition());
//            telemetry.addData("RLD", robot.leftRearDrive.getCurrentPosition());
//            telemetry.addData("RRD", robot.rightFrontDrive.getCurrentPosition());
//            telemetry.addData("liftServoPosition", robot.liftServo.getPosition());
//            telemetry.addData("Auto Loader State", load_state);
//            telemetry.addData("Scoop State", scoop);
//            telemetry.addData("Auto Loader Toggle", autoLoadToggle);
            telemetry.addData("scoop motor", robot.scoopMotor.getCurrentPosition());
            telemetry.addData("Heading: ",robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Left Shooter Encoder", robot.leftLauncherMotor.getCurrentPosition());
            telemetry.addData("Right Shooter Encoder", robot.rightLauncherMotor.getCurrentPosition());
//            telemetry.addData("Left Distance One", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Left Distance Two", robot.leftDistanceSensorTwo.getDistance(DistanceUnit.INCH));
            telemetry.addData("Front Distance", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Rear Distance", robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Red", robot.colorSensor.red());
//            telemetry.addData("Blue", robot.colorSensor.blue());
//            telemetry.addData("Green", robot.colorSensor.green());
//            telemetry.addData("Alpha", robot.colorSensor.alpha());
            telemetry.update();
            
       }
    }
}