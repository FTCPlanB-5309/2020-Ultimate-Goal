
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Drive (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void backward(double speed, int distance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;

        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;


        int target = distance * robot.CLICKS_PER_INCH;
       robot.leftFrontDrive.setTargetPosition(target);
       robot.leftRearDrive.setTargetPosition(target);
       robot.rightFrontDrive.setTargetPosition(target);
       robot.rightRearDrive.setTargetPosition(target);

        robot.setupDriveTrain();

       robot.leftFrontDrive.setPower(speed);
       robot.leftRearDrive.setPower(speed);
       robot.rightFrontDrive.setPower(speed);
       robot.rightRearDrive.setPower(speed);

       telemetry.addData("linear opmode is working, target = ", target);

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                && robot.rightFrontDrive.isBusy() && linearOpMode.opModeIsActive()) {
        Thread.yield();
            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            }
            else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            }
            else {
                robot.leftFrontDrive.setPower(speed);
                robot.leftRearDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
            }
            currentDirection = robot.getHeading();
            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
       }

       robot.stop ();

    }

    public void forward(double speed, int distance) {
        if (!linearOpMode.opModeIsActive())
            return;

        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;

        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.setupDriveTrain();

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(-speed);

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
        while (robot.leftRearDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                && robot.rightFrontDrive.isBusy() && linearOpMode.opModeIsActive()) {
            Thread.yield();
            Thread.yield();
            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            }
            else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            }
            else {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftRearDrive.setPower(-speed);
                robot.rightFrontDrive.setPower(-speed);
                robot.rightRearDrive.setPower(-speed);
            }
            currentDirection = robot.getHeading();
            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }

        robot.stop ();

    }
}