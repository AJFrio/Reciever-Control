import cv2
import mediapipe as mp
import math


def is_hand_open(hand_landmarks, handedness) -> bool:
    """
    Checks if the hand is open based on the finger positions.
    A hand is considered open if at least 4 fingers are extended.
    """
    if not hand_landmarks:
        return False

    # Landmark indices for finger tips and PIP joints
    finger_tips = [
        mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP,
        mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP,
        mp.solutions.hands.HandLandmark.RING_FINGER_TIP,
        mp.solutions.hands.HandLandmark.PINKY_TIP,
    ]
    finger_pips = [
        mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP,
        mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP,
        mp.solutions.hands.HandLandmark.RING_FINGER_PIP,
        mp.solutions.hands.HandLandmark.PINKY_PIP,
    ]

    open_fingers = 0
    for i in range(len(finger_tips)):
        tip = hand_landmarks.landmark[finger_tips[i]]
        pip = hand_landmarks.landmark[finger_pips[i]]
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


def run_hand_joint_overlay(camera_index: int = 1, draw_labels: bool = False) -> None:
    """
    Open the webcam, detect hand landmarks using MediaPipe, and draw dots on all
    finger joints in real time.

    - Press 'q' to quit the viewer window.
    - Set draw_labels=True to render the landmark indices next to each joint.
    """

    video_capture = cv2.VideoCapture(camera_index)
    if not video_capture.isOpened():
        print(f"Error: Unable to open camera index {camera_index}.")
        return

    hands_solution = mp.solutions.hands
    face_mesh_solution = mp.solutions.face_mesh

    with hands_solution.Hands(
        static_image_mode=False,
        model_complexity=1,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands, face_mesh_solution.FaceMesh(
        static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5
    ) as face_mesh:
        try:
            while True:
                frame_read_success, frame_bgr = video_capture.read()
                if not frame_read_success:
                    print("Warning: Failed to read frame from camera.")
                    break

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                frame_rgb.flags.writeable = False
                hand_results = hands.process(frame_rgb)
                face_results = face_mesh.process(frame_rgb)
                frame_rgb.flags.writeable = True

                image_height, image_width = frame_bgr.shape[:2]

                face_landmarks = None
                if face_results.multi_face_landmarks:
                    face_landmarks = face_results.multi_face_landmarks[0]

                if hand_results.multi_hand_landmarks:
                    for hand_index, hand_landmarks in enumerate(
                        hand_results.multi_hand_landmarks
                    ):
                        handedness = (
                            hand_results.multi_handedness[hand_index]
                            .classification[0]
                            .label
                        )
                        if is_hand_open(
                            hand_landmarks, handedness
                        ) and is_hand_at_head_height(hand_landmarks, face_landmarks):
                            for landmark_index, landmark in enumerate(
                                hand_landmarks.landmark
                            ):
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
                                        str(landmark_index),
                                        (x_px + 4, y_px - 4),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.4,
                                        (0, 0, 0),
                                        1,
                                        cv2.LINE_AA,
                                    )

                            # Middle finger landmarks: 9 (MCP), 10 (PIP), 11 (DIP), 12 (TIP)
                            middle_indices = [9, 10, 11, 12]
                            middle_points = []
                            for idx in middle_indices:
                                lm = hand_landmarks.landmark[idx]
                                middle_points.append(
                                    (
                                        int(lm.x * image_width),
                                        int(lm.y * image_height),
                                    )
                                )

                            # Compute angle of the middle finger vs straight up (negative Y axis)
                            base_x, base_y = middle_points[0]  # MCP
                            tip_x, tip_y = middle_points[3]  # TIP
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

                            # Decide box color:
                            # - Right deviation (positive) >= 45° -> green
                            # - Left deviation (negative) <= -45° -> blue
                            # - Otherwise -> orange
                            if signed_angle_deg >= 45.0:
                                box_color = (0, 255, 0)  # Green
                            elif signed_angle_deg <= -45.0:
                                box_color = (255, 0, 0)  # Blue
                            else:
                                box_color = (0, 140, 255)  # Orange

                            # Draw bounding box around middle finger
                            xs = [p[0] for p in middle_points]
                            ys = [p[1] for p in middle_points]
                            min_x, max_x = min(xs), max(xs)
                            min_y, max_y = min(ys), max(ys)
                            pad = 10
                            top_left = (max(min_x - pad, 0), max(min_y - pad, 0))
                            bottom_right = (
                                min(max_x + pad, image_width - 1),
                                min(max_y + pad, image_height - 1),
                            )
                            cv2.rectangle(
                                frame_bgr, top_left, bottom_right, box_color, 2
                            )

                            # Draw a line showing the middle finger direction
                            cv2.line(
                                frame_bgr, (base_x, base_y), (tip_x, tip_y), (0, 140, 255), 2
                            )

                            # Render the angle near the bounding box
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

                cv2.imshow("Hand joints - press 'q' to quit", frame_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()


def main() -> None:
    run_hand_joint_overlay()


if __name__ == "__main__":
    main()
