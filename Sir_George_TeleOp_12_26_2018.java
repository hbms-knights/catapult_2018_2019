package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class Sir_George_TeleOp_12_7_2018 extends LinearOpMode {
    private DcMotor wheel_left;
    private DcMotor wheel_right;
    private DcMotor elevator;
    private Servo left_scoop;
    private Servo right_scoop;
    private DcMotor collector;
    private TouchSensor touch;
    private DcMotor hook_motor;
    private double elevator_position;
    private double hook_position;
    private double right_servo_position = 1;
    private double left_servo_position = 0;
    
    @Override
    public void runOpMode() {
        // Map componets
        wheel_left  = hardwareMap.get(DcMotor.class, "left_wheel");
        wheel_right = hardwareMap.get(DcMotor.class, "right_wheel");
        elevator    = hardwareMap.get(DcMotor.class, "elevator");
        hook_motor = hardwareMap.get(DcMotor.class, "hook_motor");
        left_scoop  = hardwareMap.get(Servo.class, "left_scoop");
        right_scoop = hardwareMap.get(Servo.class, "right_scoop");
        collector   = hardwareMap.get(DcMotor.class, "collector");
        touch       = hardwareMap.get(TouchSensor.class, "touch");
        
        // Reset elevator position on init
        resetElevator();
        
        // Reset elevator motor encoders
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hook_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hook_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        
        while (opModeIsActive()) {
            // Some variables
            double drive_speed = 0.75;
            double arm_speed = 0;
            double collector_speed = 0;
            double hook_speed = 0;

            // Slower drive speed if right bumper is pressed
            if (this.gamepad1.right_bumper) {
                drive_speed = drive_speed * 0.25;
            }

            // Set power to wheel motors
            wheel_left.setPower(-this.gamepad1.left_stick_y * drive_speed);
            wheel_right.setPower(this.gamepad1.right_stick_y * drive_speed);
            
            // Get encoder position of the elevator
            elevator_position = Math.abs(elevator.getCurrentPosition());
            hook_position = hook_motor.getCurrentPosition();

            // Elevator is going up
            if ((this.gamepad2.left_stick_y < 0) && (elevator_position < 4000)) {
                arm_speed = 0.5;
            }
            
            // Elevator is going down
            if ((this.gamepad2.left_stick_y > 0) && (!touch.isPressed())) {
                if (elevator_position > 750) {
                    arm_speed = 0.5;
                } else {
                    arm_speed = 0.15;
                }
            }
 
            // Set power to elevator motor
            elevator.setPower(this.gamepad2.left_stick_y * arm_speed);

            //Hook motor going down
            if (hook_position >= 3500 && this.gamepad2.right_stick_y > 0 || hook_position <= 0 && this.gamepad2.right_stick_y < 0)
            {
                hook_speed = 0;
            }
            else
            {
                hook_speed = 0.75;
            }

            hook_motor.setPower(this.gamepad2.right_stick_y * hook_speed);

            // Raises the scoops in relation to the elevator encoder position
            // once it clears the collector. 0.00025 = 45 / 2000 which is the
            // 45 degrees divided by (max elevator height minus the height of
            // clearing the collector. I couldn't get the division to work inline
            // so we'll revisit that later.
            if (elevator_position > 2000) {
                left_servo_position = .00025 * (elevator_position - 2000);
                right_servo_position = 1 - (.00025 * (elevator_position - 2000));
            }
            
            // Set position to scoops
            if (this.gamepad2.b) {
                right_scoop.setPosition(0);
            } else {
                right_scoop.setPosition(right_servo_position);
            }
            
            if (this.gamepad2.x) {
                left_scoop.setPosition(1);
            } else {
                left_scoop.setPosition(left_servo_position);
            }
            
            // Collector spins forward
            if (this.gamepad2.left_bumper) {
                collector_speed = 1;
            }
            
            // Collector spins reverse
            if (this.gamepad2.right_bumper) {
                collector_speed = -1;
            }
            
            // Set power to collector motor
            collector.setPower(collector_speed);
            
            // Let's see some data, shall we?
            telemetry.update();
        }
    }
    
    private void resetElevator() {
        right_scoop.setPosition(1);
        left_scoop.setPosition(0);
        
        // run elevator down until it hits the touch sensor
        while (!touch.isPressed()) {
            elevator.setPower(0.3);
        }
        
        elevator.setPower(0);
        
        // run elevator up as long as the touch sensor is still pressed
        while (touch.isPressed()) {
            elevator.setPower(-0.1);
        }
        
        // once touch sensor is not pressed anymore, turn off power to the elevator motor
        elevator.setPower(0);
    }
}
