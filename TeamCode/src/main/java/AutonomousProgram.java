import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous")

public class AutonomousProgram extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Wobble wobble = new Wobble(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    Shooter shooter = new Shooter(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    WobbleTargetFinder wobbleFinder = new WobbleTargetFinder();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);
        drive.backward(0.5, 60);
        
        WobbleTarget target = wobbleFinder.search();

        gyroTurn.absolute(0);
        telemetry.addData("Wall Distance", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 20) {
            strafe.right(0.25, 1);
        }
        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 30) {
            strafe.left(0.25, 1);
        }
        gyroTurn.absolute(0);
        shooter.setLaunchAngle(robot.LAUNCHER_HIGH_ANGLE);
        shooter.wheelsOn();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.triggerFire();
        shooter.triggerReset();
        shooter.wheelsOff();

        //driveTo (either drive or strafe, or both)
//        drive.backward(0.25, 10);
//        wobble.moveArm(robot.WOBBLE_ARM_DOWN);
//        wobble.release();
//        wobble.moveArm(robot.WOBBLE_ARM_UP);
        wobble.place1stWobble(target);

        if (target == WobbleTarget.B)
            drive.forward(0.5, 6);
        else if (target == WobbleTarget.C) {
            gyroTurn.absolute(0);
            drive.forward(0.5, 18);
        }

        //driveTo (drive, turn, drive forward if needed)
        //wobble.moveArm(robot.WOBBLE_ARM_DOWN);
        //wobble.something
        //wobble.moveArm(robot.WOBBLE_ARM_UP);
        //gyroTurn.absolute(90);
    }
}
