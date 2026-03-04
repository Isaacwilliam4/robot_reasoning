import cv2
import torch

from depth_anything_v2.dpt import DepthAnythingV2

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vitl' # or 'vits', 'vitb', 'vitg'

model = DepthAnythingV2(**model_configs[encoder])
model.load_state_dict(torch.load(f'./models/depth_anything_v2_{encoder}.pth', map_location='cpu'))
model = model.to(DEVICE).eval()

# Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Cannot open webcam")
    exit(1)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30

print(f"Webcam initialized: {frame_width}x{frame_height} @ {fps} FPS")
print(f"Using device: {DEVICE}")
print("Press 'q' to exit, 's' to save depth map")

import numpy as np

frame_count = 0

while True:
    ret, raw_img = cap.read()
    
    if not ret:
        print("Error: Failed to read frame from webcam")
        break
    
    frame_count += 1
    
    # Infer depth
    with torch.no_grad():
        depth = model.infer_image(raw_img)  # HxW raw depth map in numpy
    
    # Normalize depth for visualization (0-255)
    depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    # Apply colormap for better visualization
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
    
    # Resize depth map to match frame size
    depth_colored_resized = cv2.resize(depth_colored, (frame_width, frame_height))
    
    # Create side-by-side display
    combined = np.hstack([raw_img, depth_colored_resized])
    
    # Add text info
    cv2.putText(
        combined,
        f"Frame: {frame_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )
    cv2.putText(
        combined,
        "RGB (Left) | Depth (Right)",
        (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )
    
    # Display the result
    cv2.imshow("RGB & Depth Estimation", combined)
    
    # Handle keyboard input
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("Exiting...")
        break
    elif key == ord('s'):
        # Save depth map
        depth_filename = f"depth_map_{frame_count}.png"
        cv2.imwrite(depth_filename, depth_colored_resized)
        print(f"Saved depth map to {depth_filename}")

# Cleanup
cap.release()
cv2.destroyAllWindows()
print("Done!")