import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Shooter (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void setLaunchAngle(double launchAngle) throws InterruptedException {
        launchAngle = 0.733 + (13.2 - robot.voltage)*0.005;
        robot.liftServo.setPosition(launchAngle);
        Thread.sleep(500);
    }

    public void triggerFire() throws InterruptedException {
        robot.launchServo.setPosition(robot.LAUNCHER_FIRE_POSITION);
        Thread.sleep(750);
    }

    public void triggerReset() throws InterruptedException {
        robot.launchServo.setPosition(robot.LAUNCHER_RESET_POSITION);
        Thread.sleep(750);
    }

    public void wheelsOn() throws InterruptedException{
        robot.leftLauncherMotor.setPower(robot.SHOOTER_WHEEL_SPEED);
        robot.rightLauncherMotor.setPower(robot.SHOOTER_WHEEL_SPEED);
        Thread.sleep(600);
    }

    public  void wheelsOff() {
        robot.leftLauncherMotor.setPower(0);
        robot.rightLauncherMotor.setPower(0);
    }

}
