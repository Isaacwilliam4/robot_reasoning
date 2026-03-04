"""
Integrated Semantic Navigation Demo  v2
========================================
Monocular Visual Odometry  →  Semantic Map  →  Scene Graph  →  Qwen LLM  →  Navigation Goal

Display: pure OpenCV window — no matplotlib, no Tk, no Qt needed.

Install:
    pip install ultralytics opencv-python numpy torch transformers accelerate

Run:
    python integrated_demo.py                        # webcam + Qwen
    python integrated_demo.py --image path.jpg       # single image
    python integrated_demo.py --mock-llm             # no model download
    python integrated_demo.py --show-drift           # amplify drift 8× for teaching

Controls (click the OpenCV window first):
    q         quit
    s         save screenshot
    1-4       submit demo instruction
"""

import argparse, json, math, re, threading, queue, time, copy
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
try:
    import torch
except ImportError:
    torch = None

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────

QWEN_MODEL        = "Qwen/Qwen2.5-0.5B-Instruct"
DRIFT_SCALE       = 1.0
ASSOC_IOU_THRESH  = 0.25
CONF_THRESH       = 0.35
CLARIFY_THRESH    = 0.70
SIGMA0            = 0.10

# BGR colours (OpenCV uses BGR)
COLORS_BGR = {
    "agent":       (60,  60,  231),   # red
    "furniture":   (219, 152,  52),   # blue
    "container":   (113, 204,  46),   # green
    "electronics": (180,  89, 155),   # purple
    "appliance":   (34,  126, 230),   # orange
    "food":        (19,  156, 241),   # yellow
    "structure":   (179, 188,  28),   # teal
    "object":      (166, 166, 149),   # grey
}

ONTOLOGY = {
    "cup":     ["cup","mug","glass","tumbler"],
    "mug":     ["mug","cup"],
    "drink":   ["cup","mug","glass","bottle","can"],
    "table":   ["dining table","table","desk","coffee table"],
    "surface": ["dining table","table","desk","counter"],
    "laptop":  ["laptop","computer","notebook"],
    "kettle":  ["kettle","electric kettle"],
    "charger": ["charger","power adapter","cable"],
    "chair":   ["chair","stool","bench"],
}

DEMO_UTTERANCES = [
    "get me some water",
    "bring me the cup on the table",
    "find the charger",
    "bring me a mug",
]

def expand_labels(lbl):
    lbl = lbl.lower().strip()
    if lbl in ONTOLOGY: return ONTOLOGY[lbl]
    for k, v in ONTOLOGY.items():
        if lbl in k or k in lbl: return v
    return [lbl]

def obj_category(label):
    l = label.lower()
    if l in {"person"}:                                    return "agent"
    if l in {"chair","couch","sofa","bed","dining table",
              "table","desk","bench"}:                     return "furniture"
    if l in {"bottle","cup","bowl","vase","mug","glass"}:  return "container"
    if l in {"tv","laptop","cell phone","remote",
              "keyboard","monitor"}:                        return "electronics"
    if l in {"microwave","oven","refrigerator",
              "toaster","sink","kettle"}:                   return "appliance"
    if l in {"door","wall","window"}:                      return "structure"
    return "object"

def hex_to_bgr(h):
    h = h.lstrip("#")
    r,g,b = int(h[0:2],16), int(h[2:4],16), int(h[4:6],16)
    return (b,g,r)


# ─────────────────────────────────────────────────────────────────────────────
# MONOCULAR VISUAL ODOMETRY
# ─────────────────────────────────────────────────────────────────────────────

class MonocularVO:
    def __init__(self, focal=600.0, pp=(320.,240.), drift_scale=1.0):
        self.focal = focal; self.pp = pp; self.drift_scale = drift_scale
        self.pose  = np.zeros(3)          # x, y, heading
        self.cov   = np.eye(3)*1e-6
        self.trajectory = [self.pose.copy()]
        self.Q     = np.diag([0.002,0.002,0.001]) * drift_scale
        self.orb   = cv2.ORB_create(500)
        self.bf    = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev_kp = self.prev_des = None

    def process(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)
        delta = np.zeros(3)
        if self.prev_des is not None and des is not None and len(des) > 10:
            matches = sorted(self.bf.match(self.prev_des, des),
                             key=lambda x: x.distance)[:80]
            if len(matches) >= 8:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches])
                K = np.array([[self.focal,0,self.pp[0]],[0,self.focal,self.pp[1]],[0,0,1]],np.float64)
                E, mask = cv2.findEssentialMat(pts1,pts2,K,method=cv2.RANSAC,prob=0.999,threshold=1.0)
                if E is not None and E.shape==(3,3):
                    _, R, t, _ = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
                    t = t.flatten()          # (3,1) → (3,) on all OpenCV versions
                    dx  =  float(t[2])
                    dy  = -float(t[0])
                    dth =  math.atan2(R[1,0], R[0,0])
                    noise = np.random.randn(3)*np.sqrt(np.diag(self.Q))*self.drift_scale
                    delta = np.array([dx,dy,dth]) + noise
        th = self.pose[2]; c,s = math.cos(th), math.sin(th)
        self.pose[0] += c*delta[0]-s*delta[1]
        self.pose[1] += s*delta[0]+c*delta[1]
        self.pose[2] += delta[2]
        self.cov += self.Q
        self.trajectory.append(self.pose.copy())
        self.prev_kp = kp; self.prev_des = des
        return self.pose.copy(), self.cov.copy()

    @property
    def cov_trace(self): return float(np.trace(self.cov))
    @property
    def drift_m(self): return float(math.sqrt(self.cov[0,0]+self.cov[1,1]))


# ─────────────────────────────────────────────────────────────────────────────
# SCENE GRAPH DATA STRUCTURES
# ─────────────────────────────────────────────────────────────────────────────

_nc = 0

@dataclass
class SceneNode:
    id: str; label: str; category: str
    conf: float; world_pos: np.ndarray
    pose_cov: float; bbox: list
    obs_count: int = 1; is_ghost: bool = False; last_seen: int = 0

    def bayes_update(self, nc, np_, ncov, nb):
        p=self.conf; l=nc
        self.conf = min(0.99, p*l/(p*l+(1-p)*(1-l)+1e-9))
        w = nc/(self.conf+nc+1e-9)
        self.world_pos = (1-w)*self.world_pos + w*np_
        self.pose_cov  = min(self.pose_cov, ncov)
        self.bbox = nb; self.obs_count += 1

    @property
    def drift_penalty(self):
        return float(np.exp(-self.pose_cov/(2*SIGMA0**2)))

@dataclass
class Edge:
    src: str; relation: str; dst: str; conf: float

class SceneGraph:
    def __init__(self):
        self.nodes={}; self.edges=[]
    def add_node(self,n): self.nodes[n.id]=n
    def add_edge(self,e): self.edges.append(e)
    def edges_from(self,nid): return [e for e in self.edges if e.src==nid]
    def flag_ghosts(self):
        ids=list(self.nodes.keys())
        for i in range(len(ids)):
            for j in range(i+1,len(ids)):
                a,b=self.nodes[ids[i]],self.nodes[ids[j]]
                if a.label==b.label and np.linalg.norm(a.world_pos-b.world_pos)<0.5:
                    if a.conf<b.conf: a.is_ghost=True
                    else: b.is_ghost=True
    def serialize(self):
        lines=["NODES:"]
        for n in self.nodes.values():
            ef=", ".join(f"{e.relation}={e.dst}(conf={e.conf:.2f})"
                         for e in self.edges_from(n.id))
            g=" GHOST" if n.is_ghost else ""
            lines.append(f"  {n.id}: label={n.label}, p={n.conf:.2f}, "
                         f"pos=({n.world_pos[0]:.2f},{n.world_pos[1]:.2f}), "
                         f"cov={n.pose_cov:.3f}{g}, edges=[{ef}]")
        return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# DETECTORS
# ─────────────────────────────────────────────────────────────────────────────

class YOLODetector:
    def __init__(self):
        from ultralytics import YOLO
        print("[YOLO] Loading yolov8n.pt ...")
        self.model = YOLO("yolov8n.pt")
    def detect(self, frame):
        out=[]
        for box in self.model(frame,verbose=False)[0].boxes:
            c=float(box.conf)
            if c<CONF_THRESH: continue
            lbl=self.model.names[int(box.cls)]
            x1,y1,x2,y2=map(int,box.xyxy[0].tolist())
            out.append(dict(label=lbl,conf=c,bbox=[x1,y1,x2,y2]))
        return out

class MockDetector:
    SCENES=[
        [dict(label="dining table",conf=0.92,bbox=[100,200,500,400]),
         dict(label="mug",         conf=0.88,bbox=[160,150,230,205]),
         dict(label="laptop",      conf=0.85,bbox=[280,155,420,205])],
        [dict(label="dining table",conf=0.90,bbox=[110,205,505,405]),
         dict(label="mug",         conf=0.81,bbox=[165,152,232,207]),
         dict(label="chair",       conf=0.76,bbox=[530,220,620,400])],
        [dict(label="couch",       conf=0.89,bbox=[80,180,480,420]),
         dict(label="tv",          conf=0.84,bbox=[200,60,460,180])],
    ]
    def __init__(self): self._i=0
    def detect(self,frame):
        d=self.SCENES[self._i%len(self.SCENES)]; self._i+=1; return d


# ─────────────────────────────────────────────────────────────────────────────
# SEMANTIC MAP UPDATE
# ─────────────────────────────────────────────────────────────────────────────

def _iou(a,b):
    ax1,ay1,ax2,ay2=a; bx1,by1,bx2,by2=b
    iw=max(0,min(ax2,bx2)-max(ax1,bx1)); ih=max(0,min(ay2,by2)-max(ay1,by1))
    inter=iw*ih; union=(ax2-ax1)*(ay2-ay1)+(bx2-bx1)*(by2-by1)-inter
    return inter/union if union else 0.

def _bbox_world(bbox, pose, fshape, focal=600., cam_h=1.2):
    h,w=fshape[:2]
    cx=(bbox[0]+bbox[2])/2; by_=bbox[3]
    ang=math.atan2(by_-h/2, focal)
    dist=cam_h/(math.tan(ang)+1e-6) if ang>0 else 2.
    dist=max(0.3,min(dist,8.))
    lat=(cx-w/2)/focal*dist
    th=pose[2]
    return np.array([pose[0]+math.cos(th)*dist-math.sin(th)*lat,
                     pose[1]+math.sin(th)*dist+math.cos(th)*lat])

NODE_TTL = 45   # frames — node removed if not re-observed within this many frames
_current_frame = 0

def update_map(graph, detections, pose, cov_trace, fshape):
    global _nc, _current_frame
    _current_frame += 1
    # Prune stale nodes — not seen in last NODE_TTL frames
    stale = [nid for nid, n in graph.nodes.items()
             if (_current_frame - n.last_seen) > NODE_TTL and n.obs_count < 3]
    for nid in stale:
        graph.nodes.pop(nid)
        graph.edges = [e for e in graph.edges if e.src != nid and e.dst != nid]
    used=set()
    for node in graph.nodes.values():
        best_iou,best_idx=0.,-1
        for idx,det in enumerate(detections):
            if idx in used or det["label"]!=node.label: continue
            iou=_iou(node.bbox,det["bbox"])
            if iou>best_iou: best_iou,best_idx=iou,idx
        if best_iou>ASSOC_IOU_THRESH and best_idx>=0:
            det=detections[best_idx]
            node.bayes_update(det["conf"],_bbox_world(det["bbox"],pose,fshape),
                              cov_trace,det["bbox"])
            node.last_seen = _current_frame
            used.add(best_idx)
    for idx,det in enumerate(detections):
        if idx in used: continue
        _nc+=1
        nid=f"{det['label'].replace(' ','_')}_{_nc:02d}"
        graph.add_node(SceneNode(id=nid,label=det["label"],
            category=obj_category(det["label"]),conf=det["conf"],
            world_pos=_bbox_world(det["bbox"],pose,fshape),
            pose_cov=cov_trace,bbox=det["bbox"],last_seen=_current_frame))
    graph.edges.clear()
    nodes=list(graph.nodes.values())
    for a in nodes:
        for b in nodes:
            if a.id==b.id: continue
            dp=a.drift_penalty*b.drift_penalty
            if (a.bbox[3]>b.bbox[1] and a.bbox[3]<b.bbox[3] and
                    (a.bbox[2]-a.bbox[0])<(b.bbox[2]-b.bbox[0])*0.8):
                ho=max(0,min(a.bbox[2],b.bbox[2])-max(a.bbox[0],b.bbox[0]))
                if ho>0.3*(a.bbox[2]-a.bbox[0]):
                    graph.add_edge(Edge(a.id,"on",b.id,round(a.conf*b.conf*dp,2)))
            d=np.linalg.norm(a.world_pos-b.world_pos)
            if d<1.5:
                graph.add_edge(Edge(a.id,"near",b.id,
                               round(a.conf*b.conf*dp*max(0,1-d/1.5),2)))
    graph.flag_ghosts()


# ─────────────────────────────────────────────────────────────────────────────
# LLM PARSERS
# ─────────────────────────────────────────────────────────────────────────────

SYSTEM_PROMPT="""You are a semantic parser for a mobile robot with a live scene graph.
The user may ask a QUESTION about the scene, or give a NAVIGATION/TASK command.

Always output a single JSON object with exactly these fields:
{
  "intent": "QUESTION" | "NAVIGATE" | "EXPLORE",
  "answer": "string (for QUESTION intent, plain English answer from the graph, else null)",
  "action": "FETCH" | "GO_TO" | "SEQUENCE" | "EXPLORE" | null,
  "subgoals": [
    {
      "action": "FETCH" | "GO_TO" | "EXPLORE",
      "target_labels": ["label1", "label2"],
      "spatial_relation": "on" | "near" | null,
      "anchor_label": "label or null",
      "destination": "user" | null
    }
  ],
  "clarification_needed": false,
  "clarification_question": null,
  "reasoning": "one sentence"
}

Rules:
- For questions like "how many X", "where is X", "what do you see" → set intent=QUESTION, fill answer, set subgoals=[].
- For commands like "get X", "go to X", "bring me X" → set intent=NAVIGATE, fill action+subgoals, set answer=null.
- Only reference objects present in the scene graph. If object absent, set clarification_needed=true.
- Output ONLY valid JSON, no markdown, no extra text."""

def _repair(raw):
    raw=re.sub(r"```(?:json)?","",raw).strip()
    s,e=raw.find("{"),raw.rfind("}")
    if s==-1 or e==-1: return None
    c=raw[s:e+1]
    try: return json.loads(c)
    except: pass
    c=re.sub(r",\s*([}\]])",r"\1",c).replace("'",'"')
    try: return json.loads(c)
    except: return None

class QwenParser:
    def __init__(self):
        from transformers import AutoModelForCausalLM, AutoTokenizer
        print(f"[LLM] Loading {QWEN_MODEL} ...")
        self.tok=AutoTokenizer.from_pretrained(QWEN_MODEL)
        self.mdl=AutoModelForCausalLM.from_pretrained(
            QWEN_MODEL,
            torch_dtype=torch.float16,   # explicit fp16 — never fp32
            device_map="auto",
        )
        self.mdl.eval()
        print(f"[LLM] Ready on {next(self.mdl.parameters()).device}  dtype=float16")
    @staticmethod
    def _trim_graph(graph, max_nodes=12):
        """Return a compact graph string capped at max_nodes most-confident nodes."""
        nodes = sorted(graph.nodes.values(), key=lambda n: -n.conf)[:max_nodes]
        lines = ["NODES (top by confidence):"]
        for n in nodes:
            ef = ", ".join(
                f"{e.relation}={e.dst}(conf={e.conf:.2f})"
                for e in graph.edges_from(n.id)
            )
            g = " GHOST" if n.is_ghost else ""
            lines.append(
                f"  {n.id}: label={n.label}, p={n.conf:.2f}, "
                f"pos=({n.world_pos[0]:.1f},{n.world_pos[1]:.1f})"
                f"{g}, edges=[{ef}]"
            )
        return "\n".join(lines)

    def parse(self, utterance, graph):
        # Shrink graph text until the full prompt fits comfortably
        for max_n in (10, 6, 3, 1):
            graph_str = self._trim_graph(graph, max_nodes=max_n)
            msgs = [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",   "content":
                 f"Scene graph:\n{graph_str}\n\nUser: \"{utterance}\"\n\nJSON:"},
            ]
            # Tokenize via apply_chat_template — never truncate a chat template
            enc = self.tok.apply_chat_template(
                msgs,
                tokenize=True,
                add_generation_prompt=True,
                return_tensors="pt",
            )
            if enc.shape[1] <= 900:
                break  # fits — proceed

        input_ids = enc.to(self.mdl.device)
        n_in = input_ids.shape[1]

        try:
            with torch.no_grad():
                out = self.mdl.generate(
                    input_ids,
                    max_new_tokens=200,
                    do_sample=False,          # greedy — deterministic, no temp sampling
                    pad_token_id=self.tok.eos_token_id,
                    eos_token_id=self.tok.eos_token_id,
                    repetition_penalty=1.1,   # prevent degenerate loops
                )
            raw = self.tok.decode(out[0][n_in:], skip_special_tokens=True).strip()
            return raw, _repair(raw)
        except RuntimeError as e:
            raise RuntimeError(f"LLM generation failed: {e}") from e
        finally:
            torch.cuda.empty_cache()

class MockLLMParser:
    def parse(self,utterance,graph):
        u=utterance.lower()
        labels={n.label.lower() for n in graph.nodes.values()}
        if "water" in u or "coffee" in u:
            has_mug=any(l in labels for l in ["mug","cup"])
            has_bottle=any(l in labels for l in ["bottle","cup","glass","mug"])
            has_kettle=any(l in labels for l in ["kettle","electric kettle","sink","tap"])
            sg=[]
            if has_mug: sg.append({"action":"FETCH","target_labels":["mug","cup"],
                "spatial_relation":None,"anchor_label":None,"destination":None})
            sg.append({"action":"GO_TO","target_labels":["kettle"],
                "spatial_relation":None,"anchor_label":None,"destination":None})
            sg.append({"action":"FETCH","target_labels":["mug","cup"],
                "spatial_relation":None,"anchor_label":None,"destination":"user"})
            plan={"action":"SEQUENCE","subgoals":sg,
                  "clarification_needed":not has_kettle,
                  "clarification_question":"I don't see a kettle — should I explore?" if not has_kettle else None,
                  "reasoning":"Get water: find a cup, go to the tap/sink, deliver."}
        elif "cup" in u or "mug" in u:
            on_t="table" in u or "on" in u
            plan={"action":"FETCH","subgoals":[{"action":"FETCH",
                "target_labels":["mug","cup"],"spatial_relation":"on" if on_t else None,
                "anchor_label":"dining table" if on_t else None,"destination":"user"}],
                "clarification_needed":False,"clarification_question":None,
                "reasoning":"Fetch the requested cup/mug."}
        else:
            plan={"action":"EXPLORE","subgoals":[{"action":"EXPLORE",
                "target_labels":[u.split()[-1]],"spatial_relation":None,
                "anchor_label":None,"destination":None}],
                "clarification_needed":True,
                "clarification_question":f"I don't see '{utterance}' in the scene.",
                "reasoning":"Unknown target."}
        # QA questions — handled by _answer_locally in runner, but set intent
        if any(w in u for w in ["how many","where is","what do you","what can"]):
            plan = {"intent":"QUESTION","answer":None,"action":None,"subgoals":[],
                    "clarification_needed":False,"clarification_question":None,
                    "reasoning":"Answering question from scene graph."}
        else:
            plan["intent"] = "NAVIGATE"
        return json.dumps(plan,indent=2), plan


# ─────────────────────────────────────────────────────────────────────────────
# REFERENCE RESOLUTION
# ─────────────────────────────────────────────────────────────────────────────

def query_graph(graph,target_labels,spatial_relation,anchor_label):
    expanded=set()
    for l in target_labels: expanded.update(expand_labels(l))
    cands=[]
    for node in graph.nodes.values():
        if node.label.lower() not in expanded: continue
        ss=1.0
        if spatial_relation and anchor_label:
            ae=expand_labels(anchor_label)
            matched=[e for e in graph.edges_from(node.id)
                     if e.relation==spatial_relation
                     and graph.nodes.get(e.dst,SceneNode("","","",0,np.zeros(2),0,[])).label.lower() in ae]
            ss=max((e.conf for e in matched),default=0.05)
        cands.append((node,ss))
    return cands

def resolve_reference(cands):
    if not cands: return None,{},False
    unnorm={n.id:n.conf*s for n,s in cands}
    Z=sum(unnorm.values())
    if Z<1e-9: return None,{},False
    post={k:v/Z for k,v in unnorm.items()}
    bid=max(post,key=post.get)
    best=next(n for n,_ in cands if n.id==bid)
    return best,post,post[bid]<CLARIFY_THRESH


# ─────────────────────────────────────────────────────────────────────────────
# LLM TASK THREAD
# ─────────────────────────────────────────────────────────────────────────────

class LLMTaskRunner(threading.Thread):
    def __init__(self,parser):
        super().__init__(daemon=True)
        self.parser=parser; self.req_q=queue.Queue()
        self.log=deque(maxlen=40); self.goal_node=None; self.running=True; self.busy=False
    def submit(self,utt,graph): self.req_q.put((utt,graph))
    def stop(self): self.running=False
    def run(self):
        while self.running:
            # Drain queue — always take the LATEST request, drop stale ones
            utt, graph = None, None
            try:
                while True:
                    utt, graph = self.req_q.get_nowait()
            except queue.Empty:
                pass
            if utt is None:
                time.sleep(0.05)
                continue

            self.log.append(f">>> {utt}")
            self.busy = True
            try:
                raw, plan = self.parser.parse(utt, graph)
            except Exception as ex:
                self.log.append(f"[LLM error: {ex}]")
                self.busy = False
                continue
            finally:
                self.busy = False

            if plan is None:
                self.log.append("[JSON parse failed]")
                continue

            intent = plan.get("intent", "NAVIGATE")

            # ── QUESTION intent ───────────────────────────────────────────
            if intent == "QUESTION":
                answer = plan.get("answer") or self._answer_locally(utt, graph)
                self.log.append(f"Robot: {answer}")
                continue

            # ── Clarification needed ──────────────────────────────────────
            if plan.get("clarification_needed"):
                q = plan.get("clarification_question", "Could you clarify?")
                self.log.append(f"Robot: {q}")
                self.log.append("(object not found in graph)")
                continue

            self.log.append(f"Reasoning: {plan.get('reasoning','')}")

            # ── NAVIGATE / EXPLORE subgoals ───────────────────────────────
            for i, sg in enumerate(plan.get("subgoals", [])):
                # Check if a newer request arrived — abort this plan
                if not self.req_q.empty():
                    self.log.append("(new request — cancelling current plan)")
                    break
                tl = sg.get("target_labels", [])
                sr = sg.get("spatial_relation")
                al = sg.get("anchor_label")
                self.log.append(f"Subgoal {i+1}: {sg.get('action')} {tl}")
                cands = query_graph(graph, tl, sr, al)
                if not cands:
                    self.log.append(f"✗ '{tl}' not in graph → explore")
                    continue
                best, post, nc = resolve_reference(cands)
                bp = max(post.values()) if post else 0
                self.log.append(f"Resolved: {best.id if best else '—'} p={bp:.2f}")
                if nc:
                    self.log.append(f"⚠ Ambiguous (p<{CLARIFY_THRESH})")
                elif best:
                    if best.pose_cov > 0.05:
                        self.log.append(f"⚠ High cov={best.pose_cov:.3f} → active perc.")
                    if best.is_ghost:
                        self.log.append("⚠ Ghost node (drift duplicate)")
                    self.log.append(f"✓ GOAL: {best.id} pos={best.world_pos.round(2)}")
                    self.goal_node = best

    def _answer_locally(self, utt, graph):
        """Fast local fallback for simple counting/location questions."""
        u = utt.lower()
        nodes = list(graph.nodes.values())
        if not nodes:
            return "I haven't observed anything yet."
        # "how many X"
        import re as _re
        m = _re.search(r"how many (.+?)[\?\s]*$", u)
        if m:
            # extract just the object word, strip trailing question words
            target_raw = m.group(1).strip()
            target = re.sub(r"(are there|do you see|can you see).*","",target_raw).strip().rstrip("s")
            expanded = expand_labels(target)
            matches = [n for n in nodes if n.label.lower() in expanded]
            return f"I see {len(matches)} {target}(s): {', '.join(n.id for n in matches) or 'none'}."
        # "where is X"
        m = _re.search(r"where is (?:the |a )?(.+?)[\?\s]*$", u)
        if m:
            target = m.group(1).strip()
            expanded = expand_labels(target)
            matches = [n for n in nodes if n.label.lower() in expanded]
            if not matches:
                return f"I don't see any {target} in the scene."
            parts = []
            for n in matches:
                on_edges = [e for e in graph.edges_from(n.id) if e.relation == "on"]
                if on_edges:
                    anchor = graph.nodes.get(on_edges[0].dst)
                    loc = f"on the {anchor.label}" if anchor else "on a surface"
                else:
                    loc = f"at ({n.world_pos[0]:.1f}, {n.world_pos[1]:.1f})m"
                parts.append(f"{n.id} {loc}")
            return "  |  ".join(parts)
        # "what do you see" / "what is in the scene"
        if any(w in u for w in ["what do you see","what can you see","what is in","whats in"]):
            counts = {}
            for n in nodes:
                counts[n.label] = counts.get(n.label, 0) + 1
            summary = ", ".join(f"{v}x {k}" for k,v in sorted(counts.items()))
            return f"I can see: {summary}."
        return f"I see {len(nodes)} objects: {', '.join(n.label for n in nodes[:6])}."


# ─────────────────────────────────────────────────────────────────────────────
# PURE OPENCV RENDERING
# ─────────────────────────────────────────────────────────────────────────────

FONT      = cv2.FONT_HERSHEY_SIMPLEX
FONT_BOLD = cv2.FONT_HERSHEY_DUPLEX

def _put(img, text, xy, scale=0.45, color=(255,255,255), thick=1, bg=None):
    x,y=int(xy[0]),int(xy[1])
    if bg is not None:
        (tw,th),_ = cv2.getTextSize(text,FONT,scale,thick)
        cv2.rectangle(img,(x-2,y-th-3),(x+tw+2,y+3),bg,-1)
    cv2.putText(img,text,(x,y),FONT,scale,color,thick,cv2.LINE_AA)

def _panel_camera(frame, graph, cov_trace, pw, ph):
    """Left panel: annotated camera frame with YOLO boxes."""
    panel = cv2.resize(frame, (pw, ph))
    drift_warn = cov_trace > 0.05

    # Header bar
    bar_col = (30,30,180) if drift_warn else (40,40,40)
    cv2.rectangle(panel,(0,0),(pw,22),bar_col,-1)
    drift_txt = f"DRIFT HIGH drift={math.sqrt(cov_trace):.3f}m" if drift_warn else f"Drift OK  sigma={math.sqrt(cov_trace):.3f}m"
    _put(panel, "Camera + Detections  " + drift_txt, (6,16), 0.42,
         (80,80,255) if drift_warn else (200,200,200))

    sx = pw / frame.shape[1]
    sy = ph / frame.shape[0]

    for node in graph.nodes.values():
        x1,y1,x2,y2 = node.bbox
        x1,y1,x2,y2 = int(x1*sx),int(y1*sy),int(x2*sx),int(y2*sy)
        col = COLORS_BGR.get(node.category,(150,150,150))
        alpha = 0.35 if node.is_ghost else 1.0
        # blend for ghost
        if node.is_ghost:
            overlay=panel.copy()
            cv2.rectangle(overlay,(x1,y1),(x2,y2),col,2)
            cv2.addWeighted(overlay,0.35,panel,0.65,0,panel)
        else:
            cv2.rectangle(panel,(x1,y1),(x2,y2),col,2)
        ghost_tag="[G] " if node.is_ghost else ""
        lbl=f"{ghost_tag}{node.label} p={node.conf:.2f}"
        _put(panel,lbl,(x1+2,y1-4),0.38,col,1)

    # Legend bottom
    cats=list(COLORS_BGR.keys())
    for i,cat in enumerate(cats):
        cv2.rectangle(panel,(6+i*90,ph-18),(20+i*90,ph-6),COLORS_BGR[cat],-1)
        _put(panel,cat,(22+i*90,ph-6),0.30,(200,200,200),1)

    return panel

def _world_to_map(pos, origin, scale, mw, mh):
    """Convert world (x,y) to map pixel coords."""
    px = int(mw/2 + (pos[0]-origin[0])*scale)
    py = int(mh/2 - (pos[1]-origin[1])*scale)
    return px, py

def _panel_vomap(vo, graph, goal_node, pw, ph):
    """Centre panel: top-down VO map."""
    panel = np.full((ph,pw,3),18,np.uint8)
    cv2.rectangle(panel,(0,0),(pw,22),(40,40,40),-1)
    _put(panel,"Top-down VO Map  (dashed circle = 1-sigma uncertainty)",(6,16),0.40)

    # Determine view bounds from trajectory
    traj = np.array(vo.trajectory)
    if len(traj)>1:
        all_pts = np.vstack([traj[:,:2]] +
            [n.world_pos.reshape(1,2) for n in graph.nodes.values()])
        mn,mx = all_pts.min(0), all_pts.max(0)
        span  = max((mx-mn).max()*1.3, 4.0)
        origin= (mn+mx)/2
    else:
        span=4.0; origin=np.zeros(2)

    scale = (min(pw,ph)-40) / span

    # Grid
    for gv in np.arange(origin[0]-span,origin[0]+span,1.0):
        x,_=_world_to_map([gv,0],origin,scale,pw,ph)
        cv2.line(panel,(x,22),(x,ph),(30,30,30),1)
    for gv in np.arange(origin[1]-span,origin[1]+span,1.0):
        _,y=_world_to_map([0,gv],origin,scale,pw,ph)
        cv2.line(panel,(0,y),(pw,y),(30,30,30),1)

    # Trajectory
    if len(traj)>2:
        pts=[_world_to_map(p,origin,scale,pw,ph) for p in traj]
        for i in range(len(pts)-1):
            alpha=max(0.2,i/len(pts))
            col=(int(52*alpha),int(152*alpha),int(219*alpha))
            cv2.line(panel,pts[i],pts[i+1],col,1)

    # Robot position + uncertainty circle
    rx,ry=_world_to_map(vo.pose,origin,scale,pw,ph)
    sig_px=max(4,int(vo.drift_m*scale))
    cv2.circle(panel,(rx,ry),sig_px,(219,152,52),1,cv2.LINE_AA)  # uncertainty
    cv2.circle(panel,(rx,ry),5,(219,152,52),-1)
    cv2.drawMarker(panel,(rx,ry),(219,152,52),cv2.MARKER_TILTED_CROSS,12,2)

    # Scene nodes
    for node in graph.nodes.values():
        nx_,ny_=_world_to_map(node.world_pos,origin,scale,pw,ph)
        col=COLORS_BGR.get(node.category,(150,150,150))
        alpha_node=0.4 if node.is_ghost else 1.0
        ncov_px=max(3,int(math.sqrt(node.pose_cov+1e-6)*scale))
        cv2.circle(panel,(nx_,ny_),ncov_px,col,1,cv2.LINE_AA)
        marker=cv2.MARKER_DIAMOND if node.is_ghost else cv2.MARKER_SQUARE
        cv2.drawMarker(panel,(nx_,ny_),col,marker,10,2)
        short=node.label[:8]+("?" if node.is_ghost else "")
        _put(panel,short,(nx_+6,ny_+4),0.32,col,1)

    # Goal
    if goal_node is not None:
        gx,gy=_world_to_map(goal_node.world_pos,origin,scale,pw,ph)
        cv2.drawMarker(panel,(gx,gy),(0,215,255),cv2.MARKER_STAR,20,2)
        _put(panel,"GOAL",(gx+6,gy-6),0.42,(0,215,255),1)
        cv2.line(panel,(rx,ry),(gx,gy),(0,215,255),1,cv2.LINE_AA)

    _put(panel,f"pose({vo.pose[0]:.2f},{vo.pose[1]:.2f})  nodes:{len(graph.nodes)}",(6,ph-6),0.35,(150,150,150))
    return panel

def _panel_graph_log(graph, log_lines, goal_node, pw, ph):
    """Right panel: scene graph (adjacency list) + LLM log."""
    panel = np.full((ph,pw,3),18,np.uint8)
    cv2.rectangle(panel,(0,0),(pw,22),(40,40,40),-1)
    _put(panel,"Scene Graph + LLM Grounding Log",(6,16),0.40)

    # ── Graph section (top half) ──────────────────────────────────────────
    split=ph//2
    cv2.line(panel,(0,split),(pw,split),(50,50,50),1)

    y=32
    _put(panel,f"Nodes ({len(graph.nodes)}):",(6,y),0.38,(180,180,180))
    y+=16
    for node in graph.nodes.values():
        col=COLORS_BGR.get(node.category,(150,150,150))
        ghost_tag="[G] " if node.is_ghost else "    "
        txt=f"{ghost_tag}{node.id:<22s} p={node.conf:.2f}  cov={node.pose_cov:.3f}"
        _put(panel,txt,(6,y),0.33,col,1)
        y+=13
        if y>split-30: break

    y=max(y+4,split-26)
    _put(panel,f"Edges ({len(graph.edges)}):",(6,y),0.38,(180,180,180))
    y+=14
    edge_cols={"on":(113,204,46),"near":(80,80,80),"instance-of":(34,126,230)}
    for e in graph.edges[:8]:
        if e.conf<0.12: continue
        col=edge_cols.get(e.relation,(120,120,120))
        txt=f"  ({e.src[:14]}, {e.relation}, {e.dst[:14]})  {e.conf:.2f}"
        _put(panel,txt,(6,y),0.30,col,1)
        y+=12
        if y>split-4: break

    # ── LLM log section (bottom half) ────────────────────────────────────
    y=split+14
    _put(panel,"LLM Grounding Log:",(6,split+4),0.38,(180,180,180))
    for line in list(log_lines)[-(( ph-split-20)//13):]:
        # colour coding
        if line.startswith("✓"):    col=(113,204,46)
        elif line.startswith("✗"):  col=(60,60,200)
        elif line.startswith("⚠"):  col=(0,165,255)
        elif line.startswith(">>>"): col=(0,215,255)
        elif line.startswith("Robot:"): col=(180,180,255)
        else: col=(160,160,160)
        # strip unicode for cv2 (which can't render it)
        safe=line.replace("✓","OK").replace("✗","ERR").replace("⚠","WARN").replace(">>>",">>>")
        _put(panel,safe[:72],(6,y),0.30,col,1)
        y+=13

    # Controls hint
    _put(panel,"t or /: type command  |  1-4: demo  |  q: quit",(6,ph-6),0.30,(80,80,80),1)
    return panel

def build_display(frame, vo, graph, log_lines, goal_node):
    """Assemble three panels into one 1600×600 OpenCV image."""
    W,H = 2400,900
    pw  = W//3

    p1 = _panel_camera(frame,  graph, vo.cov_trace, pw, H)
    p2 = _panel_vomap(vo,      graph, goal_node,    pw, H)
    p3 = _panel_graph_log(graph,log_lines,goal_node,pw, H)

    canvas = np.hstack([p1,p2,p3])

    # Dividers
    cv2.line(canvas,(pw,0),(pw,H),(60,60,60),2)
    cv2.line(canvas,(pw*2,0),(pw*2,H),(60,60,60),2)

    # Global title bar
    cv2.rectangle(canvas,(0,0),(W,22),(20,20,20),-1)
    title = "Integrated Semantic Navigation  |  Monocular VO + YOLO + Scene Graph + Qwen LLM"
    _put(canvas,title,(W//2-350,15),0.44,(200,200,200),1)

    return canvas


# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def run(source, mock_llm=False, drift_scale=1.0):
    # Detector
    try:
        detector = YOLODetector() if not mock_llm else MockDetector()
    except Exception:
        print("[warn] YOLO unavailable — using mock detector"); detector=MockDetector()

    # Parser
    if mock_llm:
        print("[info] Using mock LLM"); parser=MockLLMParser()
    else:
        try: parser=QwenParser()
        except Exception as e:
            print(f"[warn] Qwen unavailable ({e}) — using mock"); parser=MockLLMParser()

    vo    = MonocularVO(drift_scale=drift_scale)
    graph = SceneGraph()
    runner= LLMTaskRunner(parser); runner.start()

    # Open source
    if isinstance(source,str):
        frame0=cv2.imread(source)
        if frame0 is None: raise FileNotFoundError(f"Cannot read: {source}")
        is_live=False
    else:
        cap=cv2.VideoCapture(source)
        if not cap.isOpened(): raise RuntimeError(f"Cannot open camera {source}")
        is_live=True

    WIN="Integrated Semantic Navigation Demo"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN,2400,900)

    frame_n=0
    auto_submitted=False
    canvas=np.zeros((600,1600,3),np.uint8)  # safe default before first render
    start_time=time.time()
    input_text=""          # live text input buffer
    input_active=False     # True while user is typing

    import os
    if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
        os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    os.environ.setdefault("CUDA_LAUNCH_BLOCKING", "0")  # set to 1 for debugging
    print("\n" + "="*60)
    print("  INTEGRATED SEMANTIC NAVIGATION DEMO")
    print("="*60)
    print("  Window: 'Integrated Semantic Navigation Demo'")
    print("  Keys (click window first):  q=quit  s=save  1-4=demo task")
    print("  Demo tasks:")
    for i,u in enumerate(DEMO_UTTERANCES,1):
        print(f"    {i}. {u}")
    print("="*60+"\n")

    try:
        while True:
            # ── Get frame ─────────────────────────────────────────────────
            if is_live:
                ret,frame=cap.read()
                if not ret: break
            else:
                frame=frame0.copy()
                jitter=np.random.randint(-3,4,frame.shape,dtype=np.int16)
                frame=np.clip(frame.astype(np.int16)+jitter,0,255).astype(np.uint8)

            frame_n+=1

            # ── VO ────────────────────────────────────────────────────────
            pose,cov=vo.process(frame)

            # ── Detect every 3rd frame ────────────────────────────────────
            if frame_n%3==0:
                dets=detector.detect(frame)
                update_map(graph,dets,pose,vo.cov_trace,frame.shape)

            # ── Auto-submit first task once scene has nodes (webcam + image) ─
            # Triggers when ≥3 nodes seen AND ≥5 seconds have elapsed.
            # Re-submits every 30s so grounding stays live during webcam use.
            elapsed = time.time() - start_time
            if (not auto_submitted
                    and len(graph.nodes) >= 3
                    and elapsed >= 5.0):
                snap = copy.deepcopy(graph)
                runner.submit(DEMO_UTTERANCES[0], snap)
                auto_submitted = True
                print(f"\n[auto] Scene ready ({len(graph.nodes)} nodes, {elapsed:.1f}s) "
                      f"— submitting: \"{DEMO_UTTERANCES[0]}\"")
                print("[auto] Press 1-4 in the window to submit another task.\n")

            # ── Periodic VRAM cache clear (every 60 frames ~2s) ──────────
            if frame_n % 60 == 0 and torch is not None:
                torch.cuda.empty_cache()

            # ── Periodic re-submission every 30s (keeps grounding live) ──
            if (auto_submitted
                    and len(graph.nodes) >= 2
                    and int(elapsed) % 30 == 0
                    and int(elapsed) > 0
                    and frame_n % 30 == 0):          # throttle to once per ~1s
                idx = (int(elapsed) // 30 - 1) % len(DEMO_UTTERANCES)
                utt = DEMO_UTTERANCES[idx]
                runner.submit(utt, copy.deepcopy(graph))
                print(f'[periodic] Re-submitting at t={elapsed:.0f}s: "{utt}"')

            # ── Render ────────────────────────────────────────────────────
            if frame_n%2==0:
                canvas=build_display(frame,vo,graph,runner.log,runner.goal_node)
                cv2.imshow(WIN,canvas)

            # ── Key handling ──────────────────────────────────────────────
            # Use longer wait when typing so keystrokes aren't dropped
            wait_ms = 30 if input_active else 1
            key = cv2.waitKey(wait_ms) & 0xFF

            if input_active:
                if key == 255:                       # no key — redraw box only
                    pass
                elif key == 13 or key == 10:         # Enter / Return → submit
                    if input_text.strip():
                        print(f"[user] {input_text}")
                        runner.submit(input_text.strip(), copy.deepcopy(graph))
                    input_text = ""; input_active = False
                elif key == 27:                      # Esc → cancel
                    input_text = ""; input_active = False
                elif key in (8, 127):                # Backspace / Delete
                    input_text = input_text[:-1]
                elif 32 <= key <= 126:               # printable ASCII
                    input_text += chr(key)
                # Always redraw with updated text while typing
                box_y = canvas.shape[0] - 44
                cv2.rectangle(canvas,(0,box_y),(canvas.shape[1],canvas.shape[0]),(15,15,15),-1)
                cv2.rectangle(canvas,(4,box_y+4),(canvas.shape[1]-4,canvas.shape[0]-4),(100,100,100),2)
                cursor = "_" if (int(time.time()*2) % 2 == 0) else " "  # blinking cursor
                prompt_txt = "Command: " + input_text + cursor
                _put(canvas, prompt_txt, (10, box_y+26), 0.65, (0,215,255), 2)
                _put(canvas, "Enter=send   Esc=cancel", (canvas.shape[1]-310, box_y+26), 0.45, (140,140,140), 1)
                cv2.imshow(WIN, canvas)
            else:
                if key == ord('q') or key == 27:
                    break
                elif key == ord('s'):
                    fn=f"snapshot_{int(time.time())}.png"
                    cv2.imwrite(fn,canvas)
                    print(f"[saved] {fn}")
                elif key == ord('t') or key == ord('/'):
                    input_active = True; input_text = ""
                elif key in (ord('1'),ord('2'),ord('3'),ord('4')):
                    idx=key-ord('1')
                    if idx<len(DEMO_UTTERANCES):
                        utt=DEMO_UTTERANCES[idx]
                        print(f"[task] {utt}")
                        runner.submit(utt, copy.deepcopy(graph))

            # ── Slow down for image mode so VO can accumulate ─────────────
            if not is_live:
                time.sleep(0.04)
                if frame_n>200: break

    finally:
        runner.stop()
        cv2.destroyAllWindows()
        if is_live: cap.release()

        print("\n"+"="*60)
        print("  FINAL SCENE GRAPH")
        print("="*60)
        for n in graph.nodes.values():
            g="[GHOST]" if n.is_ghost else "       "
            print(f"  {g}  {n.id:<25s} p={n.conf:.2f}  cov={n.pose_cov:.3f}")
        print("\n  LLM Log:")
        for line in runner.log:
            print(f"  {line}")
        print("="*60)


# ─────────────────────────────────────────────────────────────────────────────

if __name__=="__main__":
    ap=argparse.ArgumentParser(description="Integrated Semantic Navigation Demo v2")
    ap.add_argument("--image",      type=str, default=None)
    ap.add_argument("--camera",     type=int, default=0)
    ap.add_argument("--mock-llm",   action="store_true")
    ap.add_argument("--show-drift", action="store_true")
    args=ap.parse_args()

    source     = args.image if args.image else args.camera
    drift_scale= 8.0 if args.show_drift else DRIFT_SCALE
    run(source, mock_llm=args.mock_llm, drift_scale=drift_scale)