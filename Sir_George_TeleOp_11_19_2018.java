package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class Sir_George_TeleOp_11_19_2018 extends LinearOpMode {
    private DcMotor wheel_left;
    private DcMotor wheel_right;
    private DcMotor elevator;
    private Servo left_scoop;
    private Servo right_scoop;
    
    private int drive_speed;
    
    @Override
    public void runOpMode() {
        wheel_left = hardwareMap.get(DcMotor.class, "left_wheel");
        wheel_right = hardwareMap.get(DcMotor.class, "right_wheel");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        left_scoop = hardwareMap.get(Servo.class, "left_scoop");
        right_scoop = hardwareMap.get(Servo.class, "right_scoop");
        
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        waitForStart();
        
        while (opModeIsActive()) {

            // Changing drive speed if right_bumper is pressed on 1st gamepad
            if (this.gamepad1.right_bumper) {
                drive_speed = 4;
            } else {
                if (this.gamepad1.left_bumper) {
                    drive_speed = 1;
                } else {
                    drive_speed = 2;
                }
            }

            wheel_left.setPower(this.gamepad1.left_stick_y / drive_speed);
            wheel_right.setPower(-this.gamepad1.right_stick_y / drive_speed);
            
            // Stops the elevator from moving if the motor:
            // - is greater than -4000 clicks on the encoder 
            // - is less than 0 clicks on the encoder
            // Elevator needs to be manually reset when program is initialized.
            double pos = elevator.getCurrentPosition();
            if (pos > -4000 && pos < 0
                    || pos >= 0 && this.gamepad2.left_stick_y < 0
                    || pos <= -4000 && this.gamepad2.left_stick_y > 0) {
                
                elevator.setPower(this.gamepad2.left_stick_y * 0.3);
                
                telemetry.addData("touch", elevator.getCurrentPosition());
                telemetry.update();
            } else {
                elevator.setPower(0);
            }
            
            // Triggers the scoops when the elevator is less than -3000 clicks
            // (on the encoder) and when the B button is pressed on the second game pad.
            if (pos < -3000 && this.gamepad2.b) {  
                left_scoop.setPosition(180);
                right_scoop.setPosition(0);
            } else {
                left_scoop.setPosition(0);
                right_scoop.setPosition(180);
            }
            
            telemetry.addData("right servo position", right_scoop.getPosition());
            telemetry.addData("left servo position", left_scoop.getPosition());
            telemetry.update();
        }
    }
}
