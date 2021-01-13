import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Two-Wobble Autonomous")

public class TwoWobbleAuto extends LinearOpMode{
    RobotHardware robot = new RobotHardware();
    Wobble wobble = new Wobble(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    Shooter shooter = new Shooter(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    WobbleTargetFinder wobbleFinder = new WobbleTargetFinder(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("voltage", robot.voltage);
        telemetry.update();
        waitForStart();

        WobbleTarget target = wobbleFinder.search();
        telemetry.addData("position", target);
        telemetry.update();

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);

        drive.backward(0.8, 60);
        gyroTurn.absolute(10);
        robot.liftServo.setPosition(robot.LAUNCHER_HIGH_ANGLE);
        robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
        robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);

        //Shoot the rings
        shooter.wheelsOn();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        if (target != WobbleTarget.A) {
            shooter.triggerFire();
            shooter.triggerReset();
        }
        else { //target == WobbleTarget.A
            gyroTurn.absolute(24);
            shooter.setLaunchAngle(robot.LAUNCHER_POWER_ANGLE);
            shooter.triggerFire();
            shooter.triggerReset();
        }
        shooter.wheelsOff();

        //Retrieve second wobble, then place in A
        if (target == WobbleTarget.A) {
            drive.backward(0.7,2);
            gyroTurn.absolute(2);
            wobble.moveArmUntilDone(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);

            //Second wobble
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            drive.forward(0.8, 48);
            gyroTurn.absolute(-85);
            drive.forward(0.5,24 - (int)robot.getDistanceToWall(robot.rearDistanceSensor, 13));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            gyroTurn.absolute(170);//prev 165
            drive.forward(0.7,46);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            drive.backward(0.5,6);
            strafe.left(0.5,20);
            drive.forward(0.5,8);
        }
        else if (target == WobbleTarget.B) {
            gyroTurn.absolute(36);//prev 40
            drive.backward(0.6,22);
            wobble.moveArmUntilDone(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            Thread.sleep(200);
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);

            drive.forward(0.6,18);
            gyroTurn.absolute(0);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            //Second wobble
            drive.forward(0.8, 44);
            double distance = robot.getDistanceToWall(robot.frontDistanceSensor, 13);
            if (distance > 13)
                drive.forward(0.3, (int)distance - 13);
            else
                drive.backward(0.3, 13 - (int)distance);
            gyroTurn.absolute(-90);
            drive.forward(0.5,27 - (int)robot.getDistanceToWall(robot.rearDistanceSensor, 24));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            gyroTurn.absolute(-173);//prev -175
            drive.forward(0.5,74);

            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            Thread.sleep(100);
            drive.backward(0.9,7);
        }
        else { //WobbleTarget.C
            gyroTurn.absolute(2);
            drive.backward(0.8, 36);
            wobble.moveArmUntilDone(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
            gyroTurn.absolute(0);

//            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
//            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            //Second wobble
            drive.forward(0.8, 85);
            Thread.sleep(500);
            double distance = robot.getDistanceToWall(robot.frontDistanceSensor, 13);
            if (distance > 13)
                drive.forward(0.3, (int)distance - 13);
            else
                drive.backward(0.3, 13 - (int)distance);
            gyroTurn.absolute(-85);
            drive.forward(0.6, 23 - (int)(int)robot.getDistanceToWall(robot.rearDistanceSensor, 24));
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
            Thread.sleep(500);
            robot.liftServo.setPosition(robot.RING_SERVO_BARELY_LIFTED);
            gyroTurn.absolute(-5);
            drive.backward(0.8, 87);
            gyroTurn.absolute(134);
            drive.forward(0.5,15);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            Thread.sleep(100);
            drive.backward(0.9,36);

//            drive.forward(0.5,15 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
//            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);

//            drive.forward(0.6,(int)robot.getDistanceToWall(robot.frontDistanceSensor,
//                    16) - 16);
//
//            gyroTurn.absolute(-90);
//            drive.forward(0.5,24 - (int)robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
//            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
//            gyroTurn.absolute(-5);
//            drive.backward(0.7,94);
//            gyroTurn.absolute(100);
//            drive.forward(0.6,6);
        }

    }

}
