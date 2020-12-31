import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    double currHeading;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    public GyroTurn(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void absolute(double target) {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.runUsingEncoder();

        updateHeading();
        double diff;
        int direction;
        diff = 2;
        if(target == 180)
            target = -180;
        while (Math.abs(diff) > 1 && linearOpMode.opModeIsActive()) {
            //right = -heading
            if(currHeading >= 0){
                if(target >= 0){
                    diff = target-currHeading;
                }
                else{
                    if((currHeading-target) >= ((180-currHeading)-(-180-target))){
                        diff = ((180-currHeading)-(-180-target));
                    }
                    else{
                        diff = -(currHeading - target);
                    }
                }
            }
            else{
                if(target >=0){
                    if((target-currHeading) >= ((180-target)- (-180-currHeading))){
                        diff = -((180-target)- (-180-currHeading));
                    }
                    else {
                        diff = (target-currHeading);
                    }
                }
                else{
                    diff = (target-currHeading);
                }
            }
            telemetry.addData("diff:", diff);
            telemetry.update();

            if (diff > 0) {
                if (Math.abs(diff) > 25) {
                    robot.leftFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.rightFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.HIGH_TURN_POWER);
                } else if (Math.abs(diff) > 5) {
                    robot.leftFrontDrive.setPower(robot.MEDIUM_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.MEDIUM_TURN_POWER);
                    robot.rightFrontDrive.setPower(-robot.MEDIUM_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.MEDIUM_TURN_POWER);
                } else {
                    robot.leftFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            if (diff < 0) {
                if (Math.abs(diff) > 25) {
                    robot.leftFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.rightFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.HIGH_TURN_POWER);
                } else if (Math.abs(diff) > 5) {
                    robot.leftFrontDrive.setPower(-robot.MEDIUM_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.MEDIUM_TURN_POWER);
                    robot.rightFrontDrive.setPower(robot.MEDIUM_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.MEDIUM_TURN_POWER);
                } else {
                    robot.leftFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.stop ();
    }


    public void updateHeading() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = robot.imu.getGravity();
        currHeading = angles.firstAngle;
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
    }
}