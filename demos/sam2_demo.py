import torch
import cv2
import numpy as np
from sam2.sam2_video_predictor import SAM2VideoPredictor
from sam2.build_sam import build_sam2_video_predictor

predictor = SAM2VideoPredictor.from_pretrained("facebook/sam2-hiera-large")

# Initialize webcam
cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

frame_idx = 0
state = None
object_ids = None
masks = None
frames_buffer = []
out = None
video_file = 'webcam_temp.mp4'

try:
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frames_buffer.append(frame)

            # Initialize state once we have collected some frames
            if state is None and len(frames_buffer) >= 30:  # ~1 second at 30fps
                # Write collected frames to MP4
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(video_file, fourcc, fps, (frame_width, frame_height))
                for buffered_frame in frames_buffer:
                    out.write(buffered_frame)
                out.release()
                out = None
                
                # Initialize SAM2 with the MP4 file
                state = predictor.init_state(video_file)
                
                # Reopen video writer to continue recording
                out = cv2.VideoWriter(video_file, fourcc, fps, (frame_width, frame_height))
                for buffered_frame in frames_buffer:
                    out.write(buffered_frame)
            
            # Continue writing new frames
            if out is not None:
                out.write(frame)

            # Display frame
            display_frame = frame.copy()
            
            # If we have masks, overlay them
            if masks is not None:
                for mask_idx, mask in enumerate(masks):
                    mask_uint8 = (mask[0] * 255).astype(np.uint8)
                    mask_colored = cv2.applyColorMap(mask_uint8, cv2.COLORMAP_JET)
                    display_frame = cv2.addWeighted(display_frame, 0.7, mask_colored, 0.3, 0)

            cv2.imshow("SAM2 Webcam Demo", display_frame)
            
            # Propagate masks to next frame
            if state is not None:
                for out_frame_idx, out_object_ids, out_masks in predictor.propagate_in_video(state):
                    object_ids = out_object_ids
                    masks = out_masks
                    break

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    cap.release()
    if out is not None:
        out.release()
    cv2.destroyAllWindows()