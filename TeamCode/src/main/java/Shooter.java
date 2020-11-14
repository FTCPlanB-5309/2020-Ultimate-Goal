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
        robot.leftLauncherMotor.setPower(1);
        robot.rightLauncherMotor.setPower(1);
        Thread.sleep(500);
    }

    public  void wheelsOff() {
        robot.leftLauncherMotor.setPower(0);
        robot.rightLauncherMotor.setPower(0);
    }

}
