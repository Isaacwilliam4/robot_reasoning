"""
Scene Graph Demo
================
Takes a webcam feed or image file and produces a scene graph
in line with the Semantic Maps / Scene Graph lecture.

Architecture (matches lecture exactly):
  Layer 1 - Object detection  → candidate nodes with label beliefs p(ℓ_i)
  Layer 2 - Data association   → merge detections into stable nodes
  Layer 3 - Edge inference     → spatial relations (on, near, left-of, …)
  Layer 4 - Place layer        → room/scene context node at top

Requirements (install once):
  pip install ultralytics opencv-python matplotlib networkx numpy Pillow

Run modes:
  python scene_graph_demo.py                  # webcam live
  python scene_graph_demo.py --image path.jpg # single image
  python scene_graph_demo.py --image path.jpg --save out.png  # save result
"""

import argparse
import math
import sys
import time
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")          # change to "Qt5Agg" if TkAgg unavailable
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import networkx as nx

# ── Try to import YOLO (ultralytics).  Fall back to a mock detector so the
#    graph pipeline can be demonstrated even without a GPU / model download. ──
try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False
    print("[warn] ultralytics not found – using mock detector (install with: pip install ultralytics)")

# ─────────────────────────────────────────────────────────────────────────────
# COCO class → friendly name (subset used for spatial reasoning)
# ─────────────────────────────────────────────────────────────────────────────
FURNITURE   = {"chair", "couch", "sofa", "bed", "dining table", "table", "desk", "bench"}
CONTAINERS  = {"bottle", "cup", "bowl", "vase", "handbag", "backpack", "suitcase"}
ELECTRONICS = {"tv", "laptop", "cell phone", "remote", "keyboard", "mouse", "monitor"}
PEOPLE      = {"person"}
APPLIANCES  = {"microwave", "oven", "refrigerator", "toaster", "sink"}
FOOD        = {"banana", "apple", "sandwich", "orange", "pizza", "hot dog", "cake", "carrot", "donut"}

def object_category(label: str) -> str:
    label = label.lower()
    if label in PEOPLE:      return "agent"
    if label in FURNITURE:   return "furniture"
    if label in CONTAINERS:  return "container"
    if label in ELECTRONICS: return "electronics"
    if label in APPLIANCES:  return "appliance"
    if label in FOOD:        return "food"
    return "object"

# ─────────────────────────────────────────────────────────────────────────────
# DETECTION  (wrapper around YOLO or mock)
# ─────────────────────────────────────────────────────────────────────────────

class YOLODetector:
    def __init__(self, model_name="yolov8n.pt"):
        print(f"[info] Loading {model_name} ...")
        self.model = YOLO(model_name)

    def detect(self, frame):
        """Return list of dicts: {label, conf, bbox:[x1,y1,x2,y2]}"""
        results = self.model(frame, verbose=False)[0]
        detections = []
        for box in results.boxes:
            label = self.model.names[int(box.cls)]
            conf  = float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            detections.append(dict(label=label, conf=conf, bbox=[x1,y1,x2,y2]))
        return detections


class MockDetector:
    """Synthetic detections for demo/testing when YOLO is not installed."""
    SCENES = [
        [
            dict(label="person",       conf=0.93, bbox=[50,  30,  200, 400]),
            dict(label="dining table", conf=0.88, bbox=[220, 250, 580, 450]),
            dict(label="cup",          conf=0.81, bbox=[280, 190, 350, 255]),
            dict(label="laptop",       conf=0.76, bbox=[370, 170, 520, 255]),
            dict(label="chair",        conf=0.72, bbox=[540, 210, 620, 450]),
        ],
        [
            dict(label="couch",        conf=0.91, bbox=[80,  200, 500, 440]),
            dict(label="tv",           conf=0.87, bbox=[220, 50,  480, 190]),
            dict(label="remote",       conf=0.65, bbox=[280, 370, 340, 420]),
            dict(label="bottle",       conf=0.70, bbox=[500, 280, 560, 440]),
        ],
    ]
    def __init__(self): self._idx = 0
    def detect(self, frame):
        d = self.SCENES[self._idx % len(self.SCENES)]
        self._idx += 1
        return d

# ─────────────────────────────────────────────────────────────────────────────
# NODE   (lecture: v_i = (p̂_i, Σ_i, p(ℓ_i)) )
# ─────────────────────────────────────────────────────────────────────────────

class SceneNode:
    _counter = 0

    def __init__(self, label: str, conf: float, bbox: list):
        SceneNode._counter += 1
        self.id       = f"{label.replace(' ','_')}_{SceneNode._counter:02d}"
        self.label    = label
        self.category = object_category(label)
        self.bbox     = bbox          # [x1,y1,x2,y2] in pixel space
        self.conf     = conf          # p(ℓ) – label belief (simplified scalar)
        self.pose     = self._centroid(bbox)   # (cx, cy) – geometric estimate
        self.obs_count = 1

    @staticmethod
    def _centroid(bbox):
        return ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)

    def bayes_update(self, new_conf: float):
        """Lecture eq: p'(ℓ) ∝ p(z|ℓ) · p(ℓ)  – simplified multiplicative update."""
        self.conf = min(0.99, self.conf * new_conf / (self.conf * new_conf + (1-self.conf)*(1-new_conf) + 1e-9))
        self.obs_count += 1

    def bottom_center(self):
        return ((self.bbox[0]+self.bbox[2])//2, self.bbox[3])

    def top_center(self):
        return ((self.bbox[0]+self.bbox[2])//2, self.bbox[1])

    def area(self):
        return (self.bbox[2]-self.bbox[0]) * (self.bbox[3]-self.bbox[1])

    def __repr__(self):
        return f"<Node {self.id} p(ℓ)={self.conf:.2f}>"

# ─────────────────────────────────────────────────────────────────────────────
# EDGE INFERENCE   (lecture section 3)
# ─────────────────────────────────────────────────────────────────────────────

IOU_ASSOC_THRESH   = 0.25   # data association IoU threshold
NEAR_PX_THRESH     = 200    # pixels – near relation
ON_VERTICAL_THRESH = 0.15   # fraction of image height – "on" vertical gap
ON_OVERLAP_THRESH  = 0.4    # horizontal overlap for "on"

def _iou(a, b):
    ax1,ay1,ax2,ay2 = a
    bx1,by1,bx2,by2 = b
    ix1,iy1 = max(ax1,bx1), max(ay1,by1)
    ix2,iy2 = min(ax2,bx2), min(ay2,by2)
    iw,ih = max(0,ix2-ix1), max(0,iy2-iy1)
    inter = iw*ih
    union = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
    return inter/union if union else 0.0

def _dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def _horiz_overlap(a_bbox, b_bbox):
    """Fraction of a's width that horizontally overlaps b."""
    ax1,_,ax2,_ = a_bbox
    bx1,_,bx2,_ = b_bbox
    overlap = max(0, min(ax2,bx2) - max(ax1,bx1))
    return overlap / max(1, ax2-ax1)

def infer_edges(nodes: list, img_h: int) -> list:
    """
    Return list of (node_a, relation, node_b, confidence) tuples.
    Relations: on, near, left-of, right-of, above, contains, next-to
    Confidence = product of node label confidences × geometric certainty.
    """
    edges = []
    n = len(nodes)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            a, b = nodes[i], nodes[j]
            geo_conf = a.conf * b.conf  # both must be correctly labeled

            # ── ON: a rests on top of b ──────────────────────────────────────
            # a's bottom is close to (just above) b's top
            vert_gap = (a.bottom_center()[1] - b.top_center()[1]) / max(1, img_h)
            h_overlap = _horiz_overlap(a.bbox, b.bbox)
            if -ON_VERTICAL_THRESH < vert_gap < ON_VERTICAL_THRESH and h_overlap > ON_OVERLAP_THRESH:
                # a is smaller and on b (furniture/surface typically larger)
                if a.area() < b.area() * 0.7:
                    conf = geo_conf * h_overlap * max(0, 1 - abs(vert_gap)/ON_VERTICAL_THRESH)
                    edges.append((a, "on", b, round(conf, 2)))
                    continue

            # ── NEAR: centroids within threshold ────────────────────────────
            d = _dist(a.pose, b.pose)
            if d < NEAR_PX_THRESH:
                conf = geo_conf * max(0, 1 - d/NEAR_PX_THRESH)
                edges.append((a, "near", b, round(conf, 2)))

            # ── LEFT-OF / RIGHT-OF ──────────────────────────────────────────
            if a.pose[0] < b.pose[0] - 60:
                edges.append((a, "left-of", b, round(geo_conf * 0.9, 2)))
            elif a.pose[0] > b.pose[0] + 60:
                edges.append((a, "right-of", b, round(geo_conf * 0.9, 2)))

    return edges

# ─────────────────────────────────────────────────────────────────────────────
# DATA ASSOCIATION   (lecture section 4)
# ─────────────────────────────────────────────────────────────────────────────

def associate(existing: list, detections: list) -> list:
    """
    Greedy nearest-label + IoU association.
    Returns updated node list with Bayes-updated beliefs.
    """
    used = set()
    for node in existing:
        best_iou, best_idx = 0.0, -1
        for idx, det in enumerate(detections):
            if idx in used:
                continue
            if det["label"] != node.label:
                continue
            iou = _iou(node.bbox, det["bbox"])
            if iou > best_iou:
                best_iou, best_idx = iou, idx

        if best_iou > IOU_ASSOC_THRESH and best_idx >= 0:
            det = detections[best_idx]
            node.bayes_update(det["conf"])
            node.bbox  = det["bbox"]
            node.pose  = SceneNode._centroid(det["bbox"])
            used.add(best_idx)

    # Unmatched detections → new nodes
    for idx, det in enumerate(detections):
        if idx not in used:
            existing.append(SceneNode(det["label"], det["conf"], det["bbox"]))

    return existing

# ─────────────────────────────────────────────────────────────────────────────
# SCENE CONTEXT   (place-layer node, lecture section 8)
# ─────────────────────────────────────────────────────────────────────────────

SCENE_RULES = {
    "kitchen":    {"cup","bottle","microwave","oven","refrigerator","sink","bowl"},
    "office":     {"laptop","keyboard","mouse","monitor","chair","desk"},
    "living room":{"couch","sofa","tv","remote","coffee table"},
    "dining room":{"dining table","chair","cup","bowl","bottle"},
    "bedroom":    {"bed","pillow","lamp","wardrobe"},
}

def infer_scene(nodes: list) -> tuple:
    labels = {n.label.lower() for n in nodes}
    scores = {}
    for scene, keywords in SCENE_RULES.items():
        scores[scene] = len(labels & keywords) / max(1, len(keywords))
    best = max(scores, key=scores.get)
    return best, round(scores[best], 2)

# ─────────────────────────────────────────────────────────────────────────────
# VISUALIZATION
# ─────────────────────────────────────────────────────────────────────────────

CATEGORY_COLORS = {
    "agent":       "#E74C3C",
    "furniture":   "#3498DB",
    "container":   "#2ECC71",
    "electronics": "#9B59B6",
    "appliance":   "#E67E22",
    "food":        "#F39C12",
    "object":      "#95A5A6",
    "scene":       "#2C3E50",
}

EDGE_COLORS = {
    "on":       "#27AE60",
    "near":     "#BDC3C7",
    "left-of":  "#3498DB",
    "right-of": "#3498DB",
    "above":    "#9B59B6",
    "contains": "#E67E22",
    "in-scene": "#2C3E50",
}

def draw_scene_graph(frame, nodes, edges, scene_label, scene_conf, elapsed_ms, fig=None):
    h, w = frame.shape[:2]

    if fig is None:
        fig = plt.figure(figsize=(18, 9))
    else:
        fig.clear()
    fig.patch.set_facecolor("#1a1a2e")  

    # ── Left panel: annotated image ──────────────────────────────────────────
    ax_img = fig.add_axes([0.0, 0.0, 0.45, 1.0])
    ax_img.set_facecolor("#1a1a2e")
    ax_img.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    ax_img.axis("off")
    ax_img.set_title("Layer 1 — Object Detections  (p(ℓ_i) shown)",
                     color="white", fontsize=11, pad=8)

    for node in nodes:
        x1,y1,x2,y2 = node.bbox
        color = CATEGORY_COLORS.get(node.category, "#95A5A6")
        rect = mpatches.FancyBboxPatch((x1, y1), x2-x1, y2-y1,
            boxstyle="round,pad=2", linewidth=2,
            edgecolor=color, facecolor="none")
        ax_img.add_patch(rect)
        # Label badge
        ax_img.text(x1+4, y1-6,
            f"{node.label}  p={node.conf:.2f}  ×{node.obs_count}",
            color="white", fontsize=7.5, fontweight="bold",
            bbox=dict(boxstyle="round,pad=1.5", facecolor=color, alpha=0.85))

    # ── Right panel: scene graph ─────────────────────────────────────────────
    ax_g = fig.add_axes([0.46, 0.05, 0.53, 0.88])
    ax_g.set_facecolor("#1a1a2e")
    ax_g.set_title(
        f"Layer 2/3 — Scene Graph    scene: {scene_label}  p={scene_conf:.2f}    [{elapsed_ms:.0f} ms]",
        color="white", fontsize=11, pad=8)

    G = nx.DiGraph()
    scene_node_id = f"🏠 {scene_label}"
    G.add_node(scene_node_id, category="scene", conf=scene_conf)

    for node in nodes:
        G.add_node(node.id, category=node.category, conf=node.conf, label=node.label)
        G.add_edge(scene_node_id, node.id, relation="in-scene", conf=scene_conf)

    edge_labels = {}
    for a, rel, b, conf in edges:
        if conf > 0.15:  # only show confident edges
            key = (a.id, b.id)
            if key not in edge_labels or edge_labels[key][1] < conf:
                G.add_edge(a.id, b.id, relation=rel, conf=conf)
                edge_labels[key] = (rel, conf)

    # Layout: scene node at top, others in spring below
    fixed_pos = {scene_node_id: (0.5, 1.0)}
    try:
        pos = nx.spring_layout(G, seed=42, k=2.2, iterations=80,
                               pos=fixed_pos, fixed=[scene_node_id])
    except Exception:
        pos = nx.spring_layout(G, seed=42)

    # Draw edges
    for u, v, data in G.edges(data=True):
        rel  = data.get("relation", "")
        conf = data.get("conf", 0.5)
        color = EDGE_COLORS.get(rel, "#BDC3C7")
        alpha = max(0.2, min(0.9, conf))
        style = "dashed" if rel in ("near", "in-scene") else "solid"
        nx.draw_networkx_edges(G, pos, edgelist=[(u,v)],
            ax=ax_g, edge_color=[color], alpha=alpha,
            width=1.5 if rel == "in-scene" else 2.2,
            style=style, arrows=True,
            arrowsize=14, connectionstyle="arc3,rad=0.08",
            min_source_margin=18, min_target_margin=18)
        # Edge label
        mx = (pos[u][0]+pos[v][0])/2
        my = (pos[u][1]+pos[v][1])/2
        ax_g.text(mx, my, f"{rel}\n{conf:.2f}",
                  ha="center", va="center", fontsize=6.5, color=color, alpha=0.9)

    # Draw nodes
    for nid, data in G.nodes(data=True):
        cat   = data.get("category","object")
        color = CATEGORY_COLORS.get(cat, "#95A5A6")
        x, y  = pos[nid]
        circle = plt.Circle((x, y), 0.045, color=color, zorder=3)
        ax_g.add_patch(circle)
        short = nid if nid == scene_node_id else nid.split("_")[0].replace("_"," ")
        conf_str = f"\np={data.get('conf',0):.2f}"
        ax_g.text(x, y - 0.075, short + conf_str,
                  ha="center", va="top", fontsize=7, color="white",
                  fontweight="bold", zorder=4)

    ax_g.set_xlim(-0.1, 1.1)
    ax_g.set_ylim(-0.1, 1.2)
    ax_g.axis("off")

    # ── Legend ───────────────────────────────────────────────────────────────
    legend_patches = [mpatches.Patch(color=c, label=k)
                      for k, c in CATEGORY_COLORS.items()]
    ax_g.legend(handles=legend_patches, loc="lower right", fontsize=7,
                framealpha=0.3, labelcolor="white",
                facecolor="#1a1a2e", edgecolor="#444")

    return fig


def render_to_frame(fig):
    """Render matplotlib figure to numpy array for cv2 display."""
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    w, h = fig.canvas.get_width_height()
    return buf.reshape(h, w, 3)


def print_graph_summary(nodes, edges, scene_label, scene_conf):
    """Print lecture-style graph summary to terminal."""
    print("\n" + "="*60)
    print(f"  SCENE GRAPH  |  scene: {scene_label}  p={scene_conf:.2f}")
    print("="*60)
    print(f"\nNodes ({len(nodes)}):")
    for n in nodes:
        print(f"  [{n.category:12s}]  {n.id:30s}  p(ℓ)={n.conf:.2f}  obs={n.obs_count}")
    print(f"\nEdges ({len(edges)})  [conf > 0.15 shown]:")
    for a, rel, b, conf in sorted(edges, key=lambda x: -x[3]):
        if conf > 0.15:
            print(f"  ({a.id}, {rel:12s}, {b.id})  conf={conf:.2f}")
    print()

# ─────────────────────────────────────────────────────────────────────────────
# MAIN PIPELINE
# ─────────────────────────────────────────────────────────────────────────────

def process_frame(frame, nodes, detector):
    t0 = time.time()

    # Layer 1: detect
    detections = detector.detect(frame)

    # Layer 2: data association → update node beliefs
    nodes = associate(nodes, detections)

    # Layer 3: edge inference
    edges = infer_edges(nodes, frame.shape[0])

    # Place layer: scene classification
    scene_label, scene_conf = infer_scene(nodes)

    elapsed = (time.time() - t0) * 1000

    return nodes, edges, scene_label, scene_conf, elapsed


def run_image(path: str, save_path: str = None):
    frame = cv2.imread(path)
    if frame is None:
        sys.exit(f"[error] Cannot read image: {path}")

    detector = YOLODetector() if _YOLO_AVAILABLE else MockDetector()
    nodes = []
    nodes, edges, scene_label, scene_conf, elapsed = process_frame(frame, nodes, detector)

    print_graph_summary(nodes, edges, scene_label, scene_conf)

    fig = draw_scene_graph(frame, nodes, edges, scene_label, scene_conf, elapsed)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="#1a1a2e")
        print(f"[info] Saved to {save_path}")
    else:
        plt.show()
    plt.close(fig)


def run_webcam(camera_index: int = 0):
    detector = YOLODetector() if _YOLO_AVAILABLE else MockDetector()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        sys.exit(f"[error] Cannot open camera {camera_index}")

    print("[info] Webcam running. Press 'q' to quit, 's' to save snapshot.")

    nodes   = []
    fig     = None
    plt.ion()
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            # Process every 3rd frame to keep UI responsive
            if frame_count % 3 == 0:
                nodes, edges, scene_label, scene_conf, elapsed = process_frame(
                    frame, nodes, detector)

                # Reuse existing figure
                fig = draw_scene_graph(frame, nodes, edges, scene_label, scene_conf, elapsed, fig=fig)
                fig.canvas.draw_idle()
                plt.pause(0.05)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('s'):
                    fname = f"snapshot_{int(time.time())}.png"
                    fig.savefig(fname, dpi=150, bbox_inches="tight", facecolor="#1a1a2e")
                    print(f"[info] Snapshot saved: {fname}")
                    print_graph_summary(nodes, edges, scene_label, scene_conf)

    finally:
        cap.release()
        plt.ioff()
        plt.close("all")


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Image → Scene Graph Demo")
    parser.add_argument("--image",  type=str, default=None,
                        help="Path to an image file (omit for webcam)")
    parser.add_argument("--save",   type=str, default=None,
                        help="Save output to this path instead of displaying")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index for webcam mode (default 0)")
    args = parser.parse_args()

    if args.image:
        run_image(args.image, save_path=args.save)
    else:
        run_webcam(camera_index=args.camera)
