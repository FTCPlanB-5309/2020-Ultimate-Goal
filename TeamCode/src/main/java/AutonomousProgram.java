import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.scoopServo.setPosition(robot.DRIVING_POSITION);
        wobble.moveArm(robot.WOBBLE_ARM_UP);
        drive.backward(0.5, 60);

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
        drive.backward(0.25, 10);
        wobble.moveArm(robot.WOBBLE_ARM_DOWN);
        wobble.release();
        wobble.moveArm(robot.WOBBLE_ARM_UP);


        //driveTo (drive, turn, drive forward if needed)
        //wobble.moveArm(robot.WOBBLE_ARM_DOWN);
        //wobble.something
        //wobble.moveArm(robot.WOBBLE_ARM_UP);
        //gyroTurn.absolute(90);
    }
}
