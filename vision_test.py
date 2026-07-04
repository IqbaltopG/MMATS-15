import cv2
import argparse
from ultralytics import YOLO
import supervision as sv
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Test YOLO AI with Gazebo")
    parser.add_argument('--camera', type=str, default='front', choices=['front', 'down'], help='Which camera to test (front or down)')
    args = parser.parse_args()

    print("[*] Loading YOLO model (Krti_model.pt)...")
    # Load your custom trained model
    model = YOLO("/home/ambatron/DRONE/Krti_model.pt")
    print("[+] Model loaded successfully!")

    # Set up Supervision annotators for beautiful UI
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    if args.camera == 'front':
        gst_pipeline = 'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'
    else:
        gst_pipeline = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1'

    print(f"[*] Connecting to {args.camera} camera via GStreamer...")
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[!] Failed to open video stream. Is Gazebo running?")
        return

    print("[+] Connected! Live AI window should pop up shortly.")
    print("Press 'q' in the video window to quit.")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # Run YOLO inference
        results = model(frame, verbose=False)[0]
        
        # Convert Ultralytics output to Supervision Detections
        detections = sv.Detections.from_ultralytics(results)

        # Annotate the frame with boxes
        annotated_frame = frame.copy()
        annotated_frame = box_annotator.annotate(scene=annotated_frame, detections=detections)
        
        # Add labels and confidence percentages
        labels = [
            f"{model.names[class_id]} {confidence:.2f}"
            for class_id, confidence
            in zip(detections.class_id, detections.confidence)
        ]
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels)

        # Draw a green crosshair in the exact center of the screen (the Drone's Nose)
        h, w = frame.shape[:2]
        cv2.line(annotated_frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 2)
        cv2.line(annotated_frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)

        # Draw a solid red dot on the exact target centroid (What the drone will aim for)
        centers = detections.get_anchors_coordinates(anchor=sv.Position.CENTER)
        for center in centers:
            x, y = int(center[0]), int(center[1])
            cv2.circle(annotated_frame, (x, y), 5, (0, 0, 255), -1)

        # Resize for i3wm display so it doesn't take up the whole monitor
        display_frame = cv2.resize(annotated_frame, (640, 480))
        # Show the live feed
        cv2.imshow(f"KRTI AI Test - {args.camera.upper()} CAMERA", display_frame)

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
