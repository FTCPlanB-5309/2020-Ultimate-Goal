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
        if (target == WobbleTarget.C) {
            shooter.triggerFire();
            shooter.triggerReset();
            shooter.triggerFire();
            shooter.triggerReset();
            shooter.triggerFire();
            robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);
        }
        else if (target == WobbleTarget.B){ //target == WobbleTarget.A
            shooter.triggerFire();
            shooter.triggerReset();
            shooter.triggerFire();
            shooter.triggerReset();
            gyroTurn.absolute(24);
            shooter.setLaunchAngle(robot.LAUNCHER_POWER_ANGLE);
            shooter.triggerFire();
            robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);
        }
        else { //target == WobbleTarget.A
            shooter.triggerFire();
            shooter.triggerReset();
            gyroTurn.absolute(24);
            shooter.setLaunchAngle(robot.LAUNCHER_POWER_ANGLE);
            shooter.triggerFire();
            shooter.triggerReset();
            gyroTurn.absolute(30);
            shooter.triggerFire();
            robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);
        }
        shooter.wheelsOff();

        if (target == WobbleTarget.A) {
            drive.backward(0.7,2);
            gyroTurn.absolute(2);
            wobble.moveArmUntilDone(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);

            //Go back for second wobble goal
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            drive.forward(0.8, 48);
            driveToSecondWobble();
            gyroTurn.absolute(169);//prev 170
            drive.forward(0.7,50);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            drive.backward(0.5,6);
            strafe.left(0.5,20);
            drive.forward(0.5,12);
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

            //Go back for second wobble goal
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);
            drive.forward(0.8, 44);
            driveToSecondWobble();
            gyroTurn.absolute(-173);//prev -175
            drive.forward(0.6,72);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            Thread.sleep(100);
            drive.backward(0.9,5);
        }
        else { //WobbleTarget.C
            gyroTurn.absolute(2);
            drive.backward(0.8, 36);
            wobble.moveArmUntilDone(robot.WOBBLE_ARM_DOWN);
            wobble.release();
            wobble.moveArm(robot.WOBBLE_ARM_UP);
            Thread.sleep(500);
            gyroTurn.absolute(0);
            robot.scoopServo.setPosition(robot.SCOOPING_POSITION);

            //Go back for second wobble goal
            drive.forward(0.8, 85);
            driveToSecondWobble();
            Thread.sleep(500);
            robot.liftServo.setPosition(robot.RING_SERVO_BARELY_LIFTED);
            drive.backward(0.5,15);
            gyroTurn.absolute(-178);
            gyroTurn.absolute(180);
            drive.forward(0.8,95);
            robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_OPEN);
            robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_OPEN);
            Thread.sleep(100);
            drive.backward(0.9,28);
        }

    }

    private void driveToSecondWobble() throws InterruptedException{
        Thread.sleep(500);
        double distance = robot.getDistanceToWall(robot.frontDistanceSensor, 13);
        if (distance > 13)
            drive.forward(0.3, (int)distance - 13);
        else
            drive.backward(0.3, 13 - (int)distance);
        gyroTurn.absolute(-85);
        drive.forward(0.5,24 - (int)robot.getDistanceToWall(robot.rearDistanceSensor, robot.rearDistanceSensorTwo, 13));
        robot.leftGrabberServo.setPosition(robot.LEFT_GRABBER_CLOSED);
        robot.rightGrabberServo.setPosition(robot.RIGHT_GRABBER_CLOSED);
    }

}
