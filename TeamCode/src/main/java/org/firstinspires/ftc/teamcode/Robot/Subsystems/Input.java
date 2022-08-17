package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Input extends Subsystem {

	// the threshold at which a trigger is detected as a button press
	public double TRIGGER_DETECTION_THRESHOLD = 0.75;

	protected CommandScheduler scheduler;
	Gamepad gamepad;
	protected boolean cross = false;
	protected boolean square = false;
	protected boolean triangle = false;
	protected boolean circle = false;
	protected boolean left_stick_button = false;
	protected boolean right_stick_button = false;
	protected boolean dpad_up = false;
	protected boolean dpad_left = false;
	protected boolean dpad_down = false;
	protected boolean dpad_right = false;
	protected boolean share_button = false;
	protected boolean options = false;
	protected boolean ps_button = false;
	protected boolean left_bumper = false;
	protected boolean right_bumper = false;
	protected boolean left_trigger_press = false;
	protected boolean right_trigger_press = false;
	protected double left_trigger_value = 0;
	protected double right_trigger_value = 0;
	protected double left_stick_x = 0;
	protected double left_stick_y = 0;
	protected double right_stick_x = 0;
	protected double right_stick_y = 0;
	protected boolean touchpad = false;
	protected boolean touchpad_finger_1 = false;
	protected boolean touchpad_finger_2 = false;
	protected double touchpad_finger_1_x = 0;
	protected double touchpad_finger_1_y = 0;
	protected double touchpad_finger_2_x= 0;
	protected double touchpad_finger_2_y = 0;

	// previous value for each
	protected boolean cross_prev = false;
	protected boolean square_prev = false;
	protected boolean triangle_prev = false;
	protected boolean circle_prev = false;
	protected boolean left_stick_button_prev = false;
	protected boolean right_stick_button_prev = false;
	protected boolean dpad_up_prev = false;
	protected boolean dpad_left_prev = false;
	protected boolean dpad_down_prev = false;
	protected boolean dpad_right_prev = false;
	protected boolean share_button_prev = false;
	protected boolean options_prev = false;
	protected boolean ps_button_prev = false;
	protected boolean left_bumper_prev = false;
	protected boolean right_bumper_prev = false;
	protected boolean left_trigger_press_prev = false;
	protected boolean right_trigger_press_prev = false;
	protected double left_trigger_value_prev = 0;
	protected double right_trigger_value_prev = 0;
	protected double left_stick_x_prev = 0;
	protected double left_stick_y_prev = 0;
	protected double right_stick_x_prev = 0;
	protected double right_stick_y_prev = 0;
	protected boolean touchpad_prev = false;
	protected boolean touchpad_finger_1_prev = false;
	protected boolean touchpad_finger_2_prev = false;
	protected double touchpad_finger_1_x_prev = 0;
	protected double touchpad_finger_1_y_prev = 0;
	protected double touchpad_finger_2_x_prev = 0;
	protected double touchpad_finger_2_y_prev = 0;

	// Command handler for each
	protected Command cross_handler = null;
	protected Command square_handler = null;
	protected Command triangle_handler = null;
	protected Command circle_handler = null;
	protected Command left_stick_button_handler = null;
	protected Command right_stick_button_handler = null;
	protected Command dpad_up_handler = null;
	protected Command dpad_left_handler = null;
	protected Command dpad_down_handler = null;
	protected Command dpad_right_handler = null;
	protected Command share_button_handler = null;
	protected Command options_handler = null;
	protected Command ps_button_handler = null;
	protected Command left_bumper_handler = null;
	protected Command right_bumper_handler = null;
	protected Command left_trigger_press_handler = null;
	protected Command right_trigger_press_handler = null;	

	// Command handler setter for each
	public void whenCrossPressed(Command handler) { cross_handler = handler; }
	public void whenSquarePressed(Command handler) { square_handler = handler; }
	public void whenTrianglePressed(Command handler) { triangle_handler = handler; }
	public void whenCirclePressed(Command handler) { circle_handler = handler; }
	public void whenLeftStickButtonPressed(Command handler) { left_stick_button_handler = handler; }
	public void whenRightStickButtonPressed(Command handler) { right_stick_button_handler = handler; }
	public void whenDPadUpPressed(Command handler) { dpad_up_handler = handler; }
	public void whenDPadLeftPressed(Command handler) { dpad_left_handler = handler; }
	public void whenDPadDownPressed(Command handler) { dpad_down_handler = handler; }
	public void whenDPadRightPressed(Command handler) { dpad_right_handler = handler; }
	public void whenShareButtonPressed(Command handler) { share_button_handler = handler; }
	public void whenOptionsPressed(Command handler) { options_handler = handler; }
	public void whenPSButtonPressed(Command handler) { ps_button_handler = handler; }
	public void whenLeftBumperPressed(Command handler) { left_bumper_handler = handler; }
	public void whenRightBumperPressed(Command handler) { right_bumper_handler = handler; }
	public void whenLeftTriggerPressed(Command handler) { left_trigger_press_handler = handler; }
	public void whenRightTriggerPressed(Command handler) { right_trigger_press_handler = handler; }

	public Input(Gamepad gamepad1, CommandScheduler scheduler) { 
		this.gamepad = gamepad1; 
		this.scheduler = scheduler;
	}

	@Override
	public void initAuto(HardwareMap hwMap) { }

	@Override
	public void periodic() {
		updatePrevious();
		readGamepad();
		callHandlers();
	}

	@Override
	public void shutdown() {}


	protected void readGamepad() {
		cross = gamepad.cross;
		square = gamepad.square;
		triangle = gamepad.triangle;
		circle = gamepad.circle;
		left_stick_button = gamepad.left_stick_button;
		right_stick_button = gamepad.right_stick_button;
		dpad_up = gamepad.dpad_up;
		dpad_left = gamepad.dpad_left;
		dpad_down = gamepad.dpad_down;
		dpad_right = gamepad.dpad_right;
		share_button = gamepad.share;
		options = gamepad.options;
		ps_button = gamepad.ps;
		left_bumper = gamepad.left_bumper;
		right_bumper = gamepad.right_bumper;
		left_trigger_value = gamepad.left_trigger;
		right_trigger_value = gamepad.right_trigger;

		left_trigger_press = left_trigger_value > TRIGGER_DETECTION_THRESHOLD;
		right_trigger_press = right_trigger_value > TRIGGER_DETECTION_THRESHOLD;
		left_stick_x = gamepad.left_stick_x;
		left_stick_y = gamepad.left_stick_y;

		right_stick_x = gamepad.right_stick_x;
		right_stick_y = gamepad.right_stick_y;
		touchpad = gamepad.touchpad;
		touchpad_finger_1 = gamepad.touchpad_finger_1;
		touchpad_finger_2 = gamepad.touchpad_finger_2;
		touchpad_finger_1_x = gamepad.touchpad_finger_1_x;
		touchpad_finger_1_y = gamepad.touchpad_finger_1_y;
		touchpad_finger_2_x = gamepad.touchpad_finger_2_x;
		touchpad_finger_2_y = gamepad.touchpad_finger_2_y;

	}

	protected void updatePrevious() {
		cross_prev = cross;
		square_prev = square;
		triangle_prev = triangle;
		circle_prev = circle;
		left_stick_button_prev = left_stick_button;
		right_stick_button_prev = right_stick_button;
		dpad_up_prev = dpad_up;
		dpad_left_prev = dpad_left;
		dpad_down_prev = dpad_down;
		dpad_right_prev = dpad_right;
		share_button_prev = share_button;
		options_prev = options;
		ps_button_prev = ps_button;
		left_bumper_prev = left_bumper;
		right_bumper_prev = right_bumper;
		left_trigger_press_prev = left_trigger_press;
		right_trigger_press_prev = right_trigger_press;
		left_trigger_value_prev = left_trigger_value;
		right_trigger_value_prev = right_trigger_value;
		left_stick_x_prev = left_stick_x;
		left_stick_y_prev = left_stick_y;
		right_stick_x_prev = right_stick_x;
		right_stick_y_prev = right_stick_y;
		touchpad_prev = touchpad;
		touchpad_finger_1_prev = touchpad_finger_1;
		touchpad_finger_2_prev = touchpad_finger_2;
		touchpad_finger_1_x_prev = touchpad_finger_1_x;
		touchpad_finger_1_y_prev = touchpad_finger_1_y;
		touchpad_finger_2_x_prev = touchpad_finger_2_x;
		touchpad_finger_2_y_prev = touchpad_finger_2_y;
	}

	protected void callHandlers() {
		if (isCrossPressed() && cross_handler != null)
			scheduler.forceCommand(cross_handler);
		if (isSquarePressed() && square_handler != null)
			scheduler.forceCommand(square_handler);
		if (isTrianglePressed() && triangle_handler != null)
			scheduler.forceCommand(triangle_handler);
		if (isCirclePressed() && circle_handler != null)
			scheduler.forceCommand(circle_handler);
		if (isLeftStickButtonPressed() && left_stick_button_handler != null)
			scheduler.forceCommand(left_stick_button_handler);
		if (isRightStickButtonPressed() && right_stick_button_handler != null)
			scheduler.forceCommand(right_stick_button_handler);
		if (isDpadUpPressed() && dpad_up_handler != null)
			scheduler.forceCommand(dpad_up_handler);
		if (isDpadLeftPressed() && dpad_left_handler != null)
			scheduler.forceCommand(dpad_left_handler);
		if (isDpadDownPressed() && dpad_down_handler != null)
			scheduler.forceCommand(dpad_down_handler);
		if (isDpadRightPressed() && dpad_right_handler != null)
			scheduler.forceCommand(dpad_right_handler);
		if (isShareButtonPressed() && share_button_handler != null)
			scheduler.forceCommand(share_button_handler);
		if (isOptionsButtonPressed() && options_handler != null)
			scheduler.forceCommand(options_handler);
		if (isPs_button() && ps_button_handler != null)
			scheduler.forceCommand(ps_button_handler);
		if (isLeftBumperPressed() && left_bumper_handler != null)
			scheduler.forceCommand(left_bumper_handler);
		if (isRightBumperPressed() && right_bumper_handler != null)
			scheduler.forceCommand(right_bumper_handler);
		if (isLeftTriggerPressed() && left_trigger_press_handler != null)
			scheduler.forceCommand(left_trigger_press_handler);
		if (isRightTriggerPressed() && right_trigger_press_handler != null)
			scheduler.forceCommand(right_trigger_press_handler);
	}

	public double getTRIGGER_DETECTION_THRESHOLD() {
		return TRIGGER_DETECTION_THRESHOLD;
	}

	public Gamepad getGamepad() {
		return gamepad;
	}

	public boolean isCross() {
		return cross;
	}

	public boolean isSquare() {
		return square;
	}

	public boolean isTriangle() {
		return triangle;
	}

	public boolean isCircle() {
		return circle;
	}

	public boolean isLeft_stick_button() {
		return left_stick_button;
	}

	public boolean isRight_stick_button() {
		return right_stick_button;
	}

	public boolean isDpad_up() {
		return dpad_up;
	}

	public boolean isDpad_left() {
		return dpad_left;
	}

	public boolean isDpad_down() {
		return dpad_down;
	}

	public boolean isDpad_right() {
		return dpad_right;
	}

	public boolean isShare_button() {
		return share_button;
	}

	public boolean isOptions() {
		return options;
	}

	public boolean isPs_button() {
		return ps_button;
	}

	public boolean isLeft_bumper() {
		return left_bumper;
	}

	public boolean isRight_bumper() {
		return right_bumper;
	}

	public boolean isLeft_trigger_press() {
		return left_trigger_press;
	}

	public boolean isRight_trigger_press() {
		return right_trigger_press;
	}

	public double getLeft_trigger_value() {
		return left_trigger_value;
	}

	public double getRight_trigger_value() {
		return right_trigger_value;
	}

	public double getLeft_stick_x() {
		return left_stick_x;
	}

	public double getLeft_stick_y() {
		return left_stick_y;
	}

	public double getRight_stick_x() {
		return right_stick_x;
	}

	public double getRight_stick_y() {
		return right_stick_y;
	}

	public boolean isTouchpad() {
		return touchpad;
	}

	public boolean isTouchpad_finger_1() {
		return touchpad_finger_1;
	}

	public boolean isTouchpad_finger_2() {
		return touchpad_finger_2;
	}

	public double getTouchpad_finger_1_x() {
		return touchpad_finger_1_x;
	}

	public double getTouchpad_finger_1_y() {
		return touchpad_finger_1_y;
	}

	public double getTouchpad_finger_2_x() {
		return touchpad_finger_2_x;
	}

	public double getTouchpad_finger_2_y() {
		return touchpad_finger_2_y;
	}



	public double getForwardJoystick() {
		return -getLeft_stick_y();
	}

	public double getStrafeJoystick() {
		return getLeft_stick_x();
	}

	public double getTurnJoystick() {
		return getRight_stick_x();
	}
	public boolean isCrossPressed() {
		return cross && !cross_prev;
	}

	// returns true if the button is pressed and was not pressed in the previous loop
	public boolean isSquarePressed() {
		return square && !square_prev;
	}

	// returns true if the button is pressed and was not pressed in the previous loop
	public boolean isTrianglePressed() {
		return triangle && !triangle_prev;
	}

	public boolean isCirclePressed() {
		return circle && !circle_prev;
	}

	public boolean isLeftStickButtonPressed() {
		return left_stick_button && !left_stick_button_prev;
	}

	public boolean isRightStickButtonPressed() {
		return right_stick_button && !right_stick_button_prev;
	}

	public boolean isDpadUpPressed() {
		return dpad_up && !dpad_up_prev;
	}

	public boolean isDpadLeftPressed() {
		return dpad_left && !dpad_left_prev;
	}

	public boolean isDpadDownPressed() {
		return dpad_down && !dpad_down_prev;
	}

	public boolean isDpadRightPressed() {
		return dpad_right && !dpad_right_prev;
	}

	public boolean isShareButtonPressed() {
		return share_button && !share_button_prev;
	}

	public boolean isOptionsButtonPressed() {
		return options && !options_prev;
	}

	public boolean isPsButtonPressed() {
		return ps_button && !ps_button_prev;
	}

	public boolean isLeftBumperPressed() {
		return left_bumper && !left_bumper_prev;
	}

	public boolean isRightBumperPressed() {
		return right_bumper && !right_bumper_prev;
	}

	public boolean isLeftTriggerPressed() {
		return left_trigger_press && !left_trigger_press_prev;
	}

	public boolean isRightTriggerPressed() {
		return right_trigger_press && !right_trigger_press_prev;
	}

	public boolean isLeftTriggerValueChanged() {
		return left_trigger_value != left_trigger_value_prev;
	}

	public boolean isRightTriggerValueChanged() {
		return right_trigger_value != right_trigger_value_prev;
	}

	public boolean isLeftStickXChanged() {
		return left_stick_x != left_stick_x_prev;
	}

	public boolean isLeftStickYChanged() {
		return left_stick_y != left_stick_y_prev;
	}

	public boolean isRightStickXChanged() {
		return right_stick_x != right_stick_x_prev;
	}

	public boolean isRightStickYChanged() {
		return right_stick_y != right_stick_y_prev;
	}

	public boolean isTouchpadPressed() {
		return touchpad && !touchpad_prev;
	}

	public boolean isTouchpadFinger1Pressed() {
		return touchpad_finger_1 && !touchpad_finger_1_prev;
	}

	public boolean isTouchpadFinger2Pressed() {
		return touchpad_finger_2 && !touchpad_finger_2_prev;
	}

	public boolean isTouchpadFinger1XChanged() {
		return touchpad_finger_1_x != touchpad_finger_1_x_prev;
	}

	public boolean isTouchpadFinger1YChanged() {
		return touchpad_finger_1_y != touchpad_finger_1_y_prev;
	}


	public boolean isTouchpadFinger2XChanged() {
		return touchpad_finger_2_x != touchpad_finger_2_x_prev;
	}

	public boolean isTouchpadFinger2YChanged() {
		return touchpad_finger_2_y != touchpad_finger_2_y_prev;
	}

	public boolean isTouchpadFinger1XChanged(int threshold) {
		return Math.abs(touchpad_finger_1_x - touchpad_finger_1_x_prev) > threshold;
	}

	public boolean isTouchpadFinger1YChanged(int threshold) {
		return Math.abs(touchpad_finger_1_y - touchpad_finger_1_y_prev) > threshold;
	}

	public boolean isTouchpadFinger2XChanged(int threshold) {
		return Math.abs(touchpad_finger_2_x - touchpad_finger_2_x_prev) > threshold;
	}

	public boolean isTouchpadFinger2YChanged(int threshold) {
		return Math.abs(touchpad_finger_2_y - touchpad_finger_2_y_prev) > threshold;
	}


	public boolean isTouchpadReleased() {
		return !touchpad && touchpad_prev;
	}
	public boolean isTouchpadFinger1Released() {
		return !touchpad_finger_1 && touchpad_finger_1_prev;
	}
	public boolean isTouchpadFinger2Released() {
		return !touchpad_finger_2 && touchpad_finger_2_prev;
	}
	public boolean isTouchpadFinger1XReleased() {
		return touchpad_finger_1_x == touchpad_finger_1_x_prev;
	}
	public boolean isTouchpadFinger1YReleased() {
		return touchpad_finger_1_y == touchpad_finger_1_y_prev;
	}
	public boolean isTouchpadFinger2XReleased() {
		return touchpad_finger_2_x == touchpad_finger_2_x_prev;
	}
	public boolean isTouchpadFinger2YReleased() {
		return touchpad_finger_2_y == touchpad_finger_2_y_prev;
	}
	public boolean isTouchpadFinger1XReleased(int threshold) {
		return Math.abs(touchpad_finger_1_x - touchpad_finger_1_x_prev) < threshold;
	}
	public boolean isTouchpadFinger1YReleased(int threshold) {
		return Math.abs(touchpad_finger_1_y - touchpad_finger_1_y_prev) < threshold;
	}
	public boolean isTouchpadFinger2XReleased(int threshold) {
		return Math.abs(touchpad_finger_2_x - touchpad_finger_2_x_prev) < threshold;
	}
	public boolean isTouchpadFinger2YReleased(int threshold) {
		return Math.abs(touchpad_finger_2_y - touchpad_finger_2_y_prev) < threshold;
	}

	public boolean isCrossReleased() {
		return !cross && cross_prev;
	}
	public boolean isCircleReleased() {
		return !circle && circle_prev;
	}
	public boolean isSquareReleased() {
		return !square && square_prev;
	}
	public boolean isTriangleReleased() {
		return !triangle && triangle_prev;
	}

	public boolean isShareReleased() {
		return !share_button && share_button_prev;
	}
	public boolean isOptionsReleased() {
		return !options && options_prev;
	}
	public boolean isLeftBumperReleased() {
		return !left_bumper && left_bumper_prev;
	}
	public boolean isRightBumperReleased() {
		return !right_bumper && right_bumper_prev;
	}
	public boolean isLeftStickXReleased() {
		return left_stick_x == left_stick_x_prev;
	}
	public boolean isLeftStickYReleased() {
		return left_stick_y == left_stick_y_prev;
	}
	public boolean isRightStickXReleased() {
		return right_stick_x == right_stick_x_prev;
	}
	public boolean isRightStickYReleased() {
		return right_stick_y == right_stick_y_prev;
	}
	public boolean isLeftStickXReleased(int threshold) {
		return Math.abs(left_stick_x - left_stick_x_prev) < threshold;
	}
	public boolean isLeftStickYReleased(int threshold) {
		return Math.abs(left_stick_y - left_stick_y_prev) < threshold;
	}
	public boolean isRightStickXReleased(int threshold) {
		return Math.abs(right_stick_x - right_stick_x_prev) < threshold;
	}
	public boolean isRightStickYReleased(int threshold) {
		return Math.abs(right_stick_y - right_stick_y_prev) < threshold;
	}









}
