import cv2
import mediapipe as mp
import math


def run_hand_joint_overlay(camera_index: int = 0, draw_labels: bool = False) -> None:
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

    with hands_solution.Hands(
        static_image_mode=False,
        model_complexity=1,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:
        try:
            while True:
                frame_read_success, frame_bgr = video_capture.read()
                if not frame_read_success:
                    print("Warning: Failed to read frame from camera.")
                    break

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                frame_rgb.flags.writeable = False
                results = hands.process(frame_rgb)
                frame_rgb.flags.writeable = True

                image_height, image_width = frame_bgr.shape[:2]

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        for landmark_index, landmark in enumerate(hand_landmarks.landmark):
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
                                (int(lm.x * image_width), int(lm.y * image_height))
                            )

                        # Compute angle of the middle finger vs straight up (negative Y axis)
                        base_x, base_y = middle_points[0]  # MCP
                        tip_x, tip_y = middle_points[3]    # TIP
                        vec_x = tip_x - base_x
                        vec_y = tip_y - base_y
                        vec_len = math.hypot(vec_x, vec_y)
                        # Signed angle using atan2 for left/right detection.
                        # Up vector u = (0, -1): dot = u·v = -vy, cross_z = u_x*v_y - u_y*v_x = vx
                        dot_uv = -vec_y
                        cross_z = vec_x
                        signed_angle_deg = 0.0
                        if vec_len > 1e-6:
                            signed_angle_deg = math.degrees(math.atan2(cross_z, dot_uv))
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
                        cv2.rectangle(frame_bgr, top_left, bottom_right, box_color, 2)

                        # Draw a line showing the middle finger direction
                        cv2.line(frame_bgr, (base_x, base_y), (tip_x, tip_y), (0, 140, 255), 2)

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
