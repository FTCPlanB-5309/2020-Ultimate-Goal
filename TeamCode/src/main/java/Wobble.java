import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// Purpose: This is an object that can be used to manipulate the wobble attachment during the autonomous program

public class Wobble {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    Drive drive;
    GyroTurn gyroTurn;
    Strafe strafe;

    // Resturctured the Wobble constructor so that the needed objects are created here using the
    // object global variables declared above.  
    public Wobble (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        drive = new Drive(robot, telemetry, linearOpMode);
        gyroTurn = new GyroTurn(robot, telemetry, linearOpMode);
        strafe = new Strafe(robot, telemetry, linearOpMode);
    }

    public void moveArm (int clicks) {
        if (clicks > robot.wobbleMotor.getCurrentPosition())
            robot.wobbleMotor.setPower(1);
        else
            robot.wobbleMotor.setPower(-1);
        robot.wobbleMotor.setTargetPosition(clicks);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (Math.abs(robot.wobbleMotor.getCurrentPosition() - clicks) > 2) {
//            telemetry.addData("Arm Clicks", robot.wobbleMotor.getCurrentPosition());
//            telemetry.addData("Target Position", clicks);
//            telemetry.update();
//            Thread.yield();
//        }
    }

    public void moveArmUntilDone (int clicks) {
        if (clicks > robot.wobbleMotor.getCurrentPosition())
            robot.wobbleMotor.setPower(1);
        else
            robot.wobbleMotor.setPower(-1);
        robot.wobbleMotor.setTargetPosition(clicks);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(robot.wobbleMotor.getCurrentPosition() - clicks) > 2) {
            telemetry.addData("Arm Clicks", robot.wobbleMotor.getCurrentPosition());
            telemetry.addData("Target Position", clicks);
            telemetry.update();
            Thread.yield();
        }
    }

    public void release() throws  InterruptedException {
        robot.wobbleServo.setPosition(robot.WOBBLE_SERVO_OPEN);
        robot.wobbleServoTwoProng.setPosition(robot.WOBBLE_SERVO_TWO_PRONG_OPEN);
        Thread.sleep(500);
    }
}
