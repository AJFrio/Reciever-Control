import cv2
import mediapipe as mp
import math
import time

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None


# === Stepper Motor configuration =========================================
# Update these BCM pin numbers to match the Raspberry Pi pins wired to the L9110S driver board.
# For stepper motor control:
# - Coil A (first coil): controlled by a-1a and a-1b pins
# - Coil B (second coil): controlled by b-1a and b-2a pins
STEPPER_COIL_A_PIN1 = 17  # a-1a - Controls one direction of coil A
STEPPER_COIL_A_PIN2 = 18  # a-1b - Controls opposite direction of coil A
STEPPER_COIL_B_PIN1 = 22  # b-1a - Controls one direction of coil B
STEPPER_COIL_B_PIN2 = 23  # b-2a - Controls opposite direction of coil B

# When True, pointing LEFT triggers forward/clockwise rotation; when False, pointing RIGHT does.
LEFT_DIRECTION_IS_FORWARD = True

# Angle in degrees that counts as "neutral" (motor stops) around vertical.
MOTOR_NEUTRAL_ANGLE = 45.0

# Stepper motor speed as steps per second (1.0 to 200.0)
# This controls how fast the stepper motor rotates
STEPPER_SPEED = 4.0

# Stepper motor stepping mode: "full" for full steps, "half" for half steps
STEPPER_MODE = "full"


# === Video / processing configuration =====================================
# Optimized for Raspberry Pi performance (reduced resolution and processing)
TARGET_FRAME_WIDTH = 320  # Reduced from 640 for 4x less processing
TARGET_FRAME_HEIGHT = 240  # Reduced from 480 for 4x less processing
TARGET_FPS = 15  # Reduced from 30, but still smooth enough

# Run the face mesh every N frames to lighten CPU load. Set to 1 for every frame.
# Increased from 2 to 5 for better performance (face mesh is expensive)
# Set to 0 to completely disable face mesh for maximum performance
FACE_MESH_UPDATE_INTERVAL = 5

# Toggle visualization (window + overlays). Set to False for headless/low-power runs.
VISUALIZE = True


# === Landmark index caches =================================================
FINGER_TIPS = (
    mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP,
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP,
    mp.solutions.hands.HandLandmark.RING_FINGER_TIP,
    mp.solutions.hands.HandLandmark.PINKY_TIP,
)

FINGER_PIPS = (
    mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP,
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP,
    mp.solutions.hands.HandLandmark.RING_FINGER_PIP,
    mp.solutions.hands.HandLandmark.PINKY_PIP,
)

MIDDLE_FINGER_LANDMARKS = (
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP,
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP,
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_DIP,
    mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP,
)


class StepperMotorController:
    """Controls a stepper motor through an L9110S driver with proper sequencing."""

    # Step sequences for different stepping modes
    FULL_STEP_SEQUENCE = [
        # Coil A: PIN1, PIN2 | Coil B: PIN1, PIN2
        [1, 0, 0, 0],  # Step 1: Coil A forward only
        [1, 0, 1, 0],  # Step 2: Coil A forward, Coil B forward
        [0, 0, 1, 0],  # Step 3: Coil B forward only
        [0, 1, 1, 0],  # Step 4: Coil A reverse, Coil B forward
        [0, 1, 0, 0],  # Step 5: Coil A reverse only
        [0, 1, 0, 1],  # Step 6: Coil A reverse, Coil B reverse
        [0, 0, 0, 1],  # Step 7: Coil B reverse only
        [1, 0, 0, 1],  # Step 8: Coil A forward, Coil B reverse
    ]

    HALF_STEP_SEQUENCE = [
        [1, 0, 0, 0],  # Step 1: Coil A forward only
        [1, 0, 1, 0],  # Step 2: Coil A forward, Coil B forward
        [0, 0, 1, 0],  # Step 3: Coil B forward only
        [0, 1, 1, 0],  # Step 4: Coil A reverse, Coil B forward
        [0, 1, 0, 0],  # Step 5: Coil A reverse only
        [0, 1, 0, 1],  # Step 6: Coil A reverse, Coil B reverse
        [0, 0, 0, 1],  # Step 7: Coil B reverse only
        [1, 0, 0, 1],  # Step 8: Coil A forward, Coil B reverse
        # Half steps repeat the sequence for smoother motion
        [1, 0, 0, 0], [1, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0],
        [0, 1, 0, 0], [0, 1, 0, 1], [0, 0, 0, 1], [1, 0, 0, 1],
    ]

    def __init__(self, coil_a_pin1: int, coil_a_pin2: int, coil_b_pin1: int, coil_b_pin2: int,
                 *, gpio_mode: str = "BCM", speed: float = 100.0, mode: str = "full") -> None:
        self.coil_a_pin1 = coil_a_pin1
        self.coil_a_pin2 = coil_a_pin2
        self.coil_b_pin1 = coil_b_pin1
        self.coil_b_pin2 = coil_b_pin2

        self.pins = [coil_a_pin1, coil_a_pin2, coil_b_pin1, coil_b_pin2]

        self.current_state = "stopped"
        self.gpio_mode = gpio_mode
        self.speed = max(1.0, min(200.0, speed))  # Clamp speed between 1 and 200 steps/sec
        self.mode = mode.lower()
        self.available = GPIO is not None
        self._gpio_initialized = False

        # Stepper motor state
        self.current_step = 0
        self.direction = 1  # 1 for forward/clockwise, -1 for reverse/counterclockwise
        self.is_running = False

        if self.available:
            self._initialize_gpio()
        else:
            print(
                "Warning: RPi.GPIO is not available. Stepper motor control is disabled; "
                "update dependencies or run on a Raspberry Pi."
            )

    def _initialize_gpio(self) -> None:
        GPIO.setwarnings(False)

        if self.gpio_mode.upper() == "BCM":
            GPIO.setmode(GPIO.BCM)
        elif self.gpio_mode.upper() == "BOARD":
            GPIO.setmode(GPIO.BOARD)
        else:
            raise ValueError(f"Unsupported GPIO mode '{self.gpio_mode}'. Use 'BCM' or 'BOARD'.")

        # Set up all pins as outputs
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)

        self._gpio_initialized = True

    def _set_coil_states(self, coil_a1: bool, coil_a2: bool, coil_b1: bool, coil_b2: bool) -> None:
        """Set the state of all four coil control pins."""
        GPIO.output(self.coil_a_pin1, GPIO.HIGH if coil_a1 else GPIO.LOW)
        GPIO.output(self.coil_a_pin2, GPIO.HIGH if coil_a2 else GPIO.LOW)
        GPIO.output(self.coil_b_pin1, GPIO.HIGH if coil_b1 else GPIO.LOW)
        GPIO.output(self.coil_b_pin2, GPIO.HIGH if coil_b2 else GPIO.LOW)

    def _get_step_sequence(self):
        """Get the appropriate step sequence based on the configured mode."""
        if self.mode == "half":
            return self.HALF_STEP_SEQUENCE
        else:  # Default to full step
            return self.FULL_STEP_SEQUENCE

    def _step_motor(self) -> None:
        """Execute one step of the stepper motor."""
        if not self.available or not self.is_running:
            return

        sequence = self._get_step_sequence()
        step_pattern = sequence[self.current_step % len(sequence)]

        # Apply the step pattern: [coil_a1, coil_a2, coil_b1, coil_b2]
        self._set_coil_states(*step_pattern)

        # Move to next step
        self.current_step += self.direction

    def forward(self) -> None:
        """Start rotating forward (clockwise)."""
        self.direction = 1
        if not self.is_running:
            self.is_running = True
            self.current_state = "forward"

    def reverse(self) -> None:
        """Start rotating in reverse (counterclockwise)."""
        self.direction = -1
        if not self.is_running:
            self.is_running = True
            self.current_state = "reverse"

    def stop(self) -> None:
        """Stop the stepper motor."""
        self.is_running = False
        self.current_state = "stopped"
        if self.available:
            # Turn off all coils when stopped
            self._set_coil_states(0, 0, 0, 0)

    def set_speed(self, speed: float) -> None:
        """Set stepper motor speed in steps per second (1.0 to 200.0)."""
        self.speed = max(1.0, min(200.0, speed))

    def run_stepper_loop(self) -> None:
        """Main loop to run the stepper motor at the specified speed."""
        import threading
        import time

        def stepper_thread():
            step_delay = 1.0 / self.speed

            while self.is_running:
                self._step_motor()
                time.sleep(step_delay)

            # Ensure motor stops when thread ends
            if self.available:
                self._set_coil_states(0, 0, 0, 0)

        if self.is_running:
            thread = threading.Thread(target=stepper_thread, daemon=True)
            thread.start()

    def cleanup(self) -> None:
        """Cleanup GPIO resources."""
        self.stop()
        if self.available and self._gpio_initialized:
            GPIO.cleanup()


def is_hand_open(hand_landmarks, handedness) -> bool:
    """
    Checks if the hand is open based on the finger positions.
    A hand is considered open if at least 4 fingers are extended.
    """
    if not hand_landmarks:
        return False

    # Landmark indices for finger tips and PIP joints
    open_fingers = 0
    for tip_idx, pip_idx in zip(FINGER_TIPS, FINGER_PIPS):
        tip = hand_landmarks.landmark[tip_idx]
        pip = hand_landmarks.landmark[pip_idx]
        # If finger tip is above the PIP joint, it's considered open
        if tip.y < pip.y:
            open_fingers += 1

    # Check thumb separately
    thumb_tip = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_IP]

    is_thumb_open = False
    if handedness == "Right" and thumb_tip.x < thumb_ip.x:
        is_thumb_open = True
    elif handedness == "Left" and thumb_tip.x > thumb_ip.x:
        is_thumb_open = True

    if is_thumb_open:
        open_fingers += 1

    return open_fingers >= 4


def is_hand_at_head_height(hand_landmarks, face_landmarks) -> bool:
    """
    Checks if the hand is held up at head height.
    True if the wrist is vertically between the top of the head and the chin,
    with some vertical padding.
    """
    if not hand_landmarks or not face_landmarks:
        return False

    wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
    # Landmark 10 is the top of the forehead, 152 is the chin.
    top_of_head = face_landmarks.landmark[10]
    chin = face_landmarks.landmark[152]

    # Calculate head height to use for padding
    head_height = chin.y - top_of_head.y
    padding = head_height * 1.2  # 120% padding

    # Y decreases as you go up
    upper_bound = top_of_head.y - padding
    lower_bound = chin.y + padding

    return upper_bound <= wrist.y <= lower_bound


def get_distance_between_points(point1, point2) -> float:
    """Calculate Euclidean distance between two points."""
    return math.hypot(point1.x - point2.x, point1.y - point2.y)


def find_best_hand_for_person(hand_results, face_landmarks) -> tuple:
    """
    Find the best hand candidate that belongs to the detected person.
    Returns (hand_landmarks, handedness, is_open, is_at_head) for the best candidate,
    or (None, None, False, False) if none found.
    """
    if not hand_results.multi_hand_landmarks or not face_landmarks:
        return None, None, False, False
    
    # Get face center for reference
    face_center_x = sum(lm.x for lm in face_landmarks.landmark) / len(face_landmarks.landmark)
    face_center_y = sum(lm.y for lm in face_landmarks.landmark) / len(face_landmarks.landmark)
    face_center = type('Point', (), {'x': face_center_x, 'y': face_center_y})()
    
    best_hand = None
    best_handedness = None
    best_is_open = False
    best_is_at_head = False
    best_score = float('inf')
    
    for hand_index, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
        handedness = hand_results.multi_handedness[hand_index].classification[0].label
        
        # Get hand center (wrist position)
        wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
        
        # Calculate distance from hand to face center
        distance_to_face = get_distance_between_points(wrist, face_center)
        
        # Prioritize hands that are open and at head height
        is_open = is_hand_open(hand_landmarks, handedness)
        is_at_head = is_hand_at_head_height(hand_landmarks, face_landmarks)
        
        # Calculate score (lower is better)
        score = distance_to_face
        if is_open and is_at_head:
            score *= 0.1  # Heavily favor qualifying hands
        elif is_open:
            score *= 0.5  # Somewhat favor open hands
        elif is_at_head:
            score *= 0.7  # Somewhat favor hands at head height
            
        if score < best_score:
            best_score = score
            best_hand = hand_landmarks
            best_handedness = handedness
            best_is_open = is_open
            best_is_at_head = is_at_head

    return best_hand, best_handedness, best_is_open, best_is_at_head


def run_hand_joint_overlay(camera_index: int = 0, draw_labels: bool = False) -> None:
    """
    Open the webcam, detect hand landmarks using MediaPipe, and draw dots on all
    finger joints in real time. Focuses on only one person when multiple people
    are detected on screen.

    - Press 'q' to quit the viewer window.
    - Set draw_labels=True to render the landmark indices next to each joint.
    """

    video_capture = cv2.VideoCapture(camera_index)
    if not video_capture.isOpened():
        print(f"Error: Unable to open camera index {camera_index}.")
        return

    if TARGET_FRAME_WIDTH > 0:
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_FRAME_WIDTH)
    if TARGET_FRAME_HEIGHT > 0:
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_FRAME_HEIGHT)
    if TARGET_FPS > 0:
        video_capture.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        frame_interval = 1.0 / TARGET_FPS
    else:
        frame_interval = 0.0

    video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    draw_labels = draw_labels and VISUALIZE
    last_frame_timestamp = 0.0
    face_mesh_interval = max(1, FACE_MESH_UPDATE_INTERVAL)

    hands_solution = mp.solutions.hands
    face_mesh_solution = mp.solutions.face_mesh
    motor_controller = StepperMotorController(
        STEPPER_COIL_A_PIN1, STEPPER_COIL_A_PIN2,
        STEPPER_COIL_B_PIN1, STEPPER_COIL_B_PIN2,
        speed=STEPPER_SPEED, mode=STEPPER_MODE
    )

    with hands_solution.Hands(
        static_image_mode=False,
        model_complexity=0,  # Reduced from 1 to 0 (lightest model) for better performance
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands, face_mesh_solution.FaceMesh(
        static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5
    ) as face_mesh:
        # Note: Face mesh is optional for performance. Set FACE_MESH_UPDATE_INTERVAL to 0 to disable
        try:
            frame_counter = 0
            while True:
                frame_read_success, frame_bgr = video_capture.read()
                if not frame_read_success:
                    print("Warning: Failed to read frame from camera.")
                    break

                if frame_interval > 0.0:
                    now = time.perf_counter()
                    elapsed = now - last_frame_timestamp
                    if elapsed < frame_interval:
                        time.sleep(frame_interval - elapsed)
                    last_frame_timestamp = now

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                frame_rgb.flags.writeable = False
                hand_results = hands.process(frame_rgb)
                face_results = None
                if frame_counter % face_mesh_interval == 0:
                    face_results = face_mesh.process(frame_rgb)
                frame_rgb.flags.writeable = True

                image_height, image_width = frame_bgr.shape[:2]

                face_landmarks = None
                if face_results and face_results.multi_face_landmarks:
                    face_landmarks = face_results.multi_face_landmarks[0]

                # Find the best hand candidate for the detected person
                (
                    best_hand,
                    best_handedness,
                    best_is_open,
                    best_is_at_head,
                ) = find_best_hand_for_person(hand_results, face_landmarks)

                if face_landmarks and best_hand and best_handedness and best_is_open and best_is_at_head:

                    hand_landmarks = best_hand
                    handedness = best_handedness

                    if VISUALIZE:
                        # Draw only middle finger joints (landmarks 9-12)
                        for idx in MIDDLE_FINGER_LANDMARKS:
                            landmark = hand_landmarks.landmark[idx]
                            x_px = int(landmark.x * image_width)
                            y_px = int(landmark.y * image_height)

                            cv2.circle(
                                frame_bgr,
                                (x_px, y_px),
                                6,
                                (0, 255, 255),
                                -1,
                            )

                            if draw_labels:
                                cv2.putText(
                                    frame_bgr,
                                    str(idx),
                                    (x_px + 4, y_px - 4),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4,
                                    (0, 0, 0),
                                    1,
                                    cv2.LINE_AA,
                                )

                    # Middle finger landmarks: 9 (MCP), 10 (PIP), 11 (DIP), 12 (TIP)
                    middle_points = []
                    for idx in MIDDLE_FINGER_LANDMARKS:
                        lm = hand_landmarks.landmark[idx]
                        middle_points.append((lm.x, lm.y))

                    # Compute angle of the middle finger vs straight up (negative Y axis)
                    base_x, base_y = middle_points[0]
                    tip_x, tip_y = middle_points[3]
                    vec_x = tip_x - base_x
                    vec_y = tip_y - base_y
                    vec_len = math.hypot(vec_x, vec_y)
                    # Signed angle using atan2 for left/right detection.
                    # Up vector u = (0, -1): dot = u·v = -vy, cross_z = u_x*v_y - u_y*v_x = vx
                    dot_uv = -vec_y
                    cross_z = vec_x
                    signed_angle_deg = 0.0
                    if vec_len > 1e-6:
                        signed_angle_deg = math.degrees(
                            math.atan2(cross_z, dot_uv)
                        )
                    abs_angle_deg = abs(signed_angle_deg)

                    # Update motor state only when a meaningful change is detected
                    if abs_angle_deg < MOTOR_NEUTRAL_ANGLE:
                        motor_controller.stop()
                        current_drive_state = "stopped"
                    else:
                        if signed_angle_deg < 0:
                            target_state = "forward" if LEFT_DIRECTION_IS_FORWARD else "reverse"
                        else:
                            target_state = "reverse" if LEFT_DIRECTION_IS_FORWARD else "forward"

                        if target_state != current_drive_state:
                            current_drive_state = target_state
                            if current_drive_state == "forward":
                                motor_controller.forward()
                                motor_controller.run_stepper_loop()
                            elif current_drive_state == "reverse":
                                motor_controller.reverse()
                                motor_controller.run_stepper_loop()

                    if VISUALIZE:
                        if signed_angle_deg >= MOTOR_NEUTRAL_ANGLE:
                            box_color = (0, 255, 0)
                        elif signed_angle_deg <= -MOTOR_NEUTRAL_ANGLE:
                            box_color = (255, 0, 0)
                        else:
                            box_color = (0, 140, 255)

                        xs = [int(p[0] * image_width) for p in middle_points]
                        ys = [int(p[1] * image_height) for p in middle_points]
                        min_x, max_x = min(xs), max(xs)
                        min_y, max_y = min(ys), max(ys)
                        pad = 10
                        top_left = (max(min_x - pad, 0), max(min_y - pad, 0))
                        bottom_right = (
                            min(max_x + pad, image_width - 1),
                            min(max_y + pad, image_height - 1),
                        )
                        cv2.rectangle(frame_bgr, top_left, bottom_right, box_color, 2)

                        cv2.line(
                            frame_bgr,
                            (
                                int(base_x * image_width),
                                int(base_y * image_height),
                            ),
                            (
                                int(tip_x * image_width),
                                int(tip_y * image_height),
                            ),
                            (0, 140, 255),
                            2,
                        )

                        label = f"{abs_angle_deg:.1f}\u00B0 off up"
                        text_org = (top_left[0], max(top_left[1] - 8, 12))
                        cv2.putText(
                            frame_bgr,
                            label,
                            text_org,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 0, 0),
                            2,
                            cv2.LINE_AA,
                        )

                else:
                    # No suitable hand detected - stop the motor
                    motor_controller.stop()

                if VISUALIZE:
                    cv2.imshow("Hand joints - press 'q' to quit", frame_bgr)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break

                frame_counter += 1
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
            motor_controller.cleanup()


def main() -> None:
    run_hand_joint_overlay()


if __name__ == "__main__":
    main()
#updated