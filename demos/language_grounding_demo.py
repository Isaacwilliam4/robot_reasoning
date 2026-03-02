"""
Language Grounding Demo  —  Qwen2.5-0.5B-Instruct
===================================================
Demonstrates the full pipeline from the Language Grounding lecture:

  Utterance → Semantic Parsing (Qwen LLM) → Graph Query
           → Reference Resolution (Bayes posterior) → Goal + Plan

The scene graph is synthetic but realistic, matching the coffee-making
worked example and other examples from the lecture.

Install:
    pip install transformers torch accelerate

Run:
    python language_grounding_demo.py
    python language_grounding_demo.py --model Qwen/Qwen2.5-1.5B-Instruct  # better JSON
    python language_grounding_demo.py --utterance "bring me the cup on the table"
"""

import argparse
import json
import math
import re
import textwrap
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# DATA STRUCTURES  (lecture: nodes, edges, graph)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Node:
    id: str
    label: str          # detected class label
    category: str       # furniture / container / electronics / appliance / agent
    conf: float         # p(ℓ)  — label belief from Bayes-updated semantic map
    pose: tuple         # (x, y, z) metric position in metres
    obs_count: int = 1

    def __str__(self):
        return f"{self.id}  label={self.label}  p(ℓ)={self.conf:.2f}  pose={self.pose}  obs={self.obs_count}"


@dataclass
class Edge:
    src: str            # node id
    relation: str       # on / near / connects / in-room / instance-of
    dst: str            # node id
    conf: float         # edge confidence = f(p(ℓ_src), p(ℓ_dst), geometric)

    def __str__(self):
        return f"({self.src}, {self.relation}, {self.dst})  conf={self.conf:.2f}"


@dataclass
class SceneGraph:
    nodes: dict = field(default_factory=dict)   # id → Node
    edges: list = field(default_factory=list)   # list of Edge

    def add_node(self, node: Node):
        self.nodes[node.id] = node

    def add_edge(self, edge: Edge):
        self.edges.append(edge)

    def get_edges_from(self, node_id: str):
        return [e for e in self.edges if e.src == node_id]

    def get_edges_to(self, node_id: str):
        return [e for e in self.edges if e.dst == node_id]

    def serialize(self) -> str:
        """Compact text serialization for LLM context."""
        lines = ["NODES:"]
        for n in self.nodes.values():
            edge_summary = []
            for e in self.get_edges_from(n.id):
                edge_summary.append(f"{e.relation}={e.dst}(conf={e.conf:.2f})")
            edge_str = ", ".join(edge_summary) if edge_summary else "none"
            lines.append(
                f"  {n.id}: label={n.label}, category={n.category}, "
                f"p(label)={n.conf:.2f}, pose={n.pose}, edges=[{edge_str}]"
            )
        lines.append("EDGES:")
        for e in self.edges:
            lines.append(f"  ({e.src}, {e.relation}, {e.dst})  conf={e.conf:.2f}")
        return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# SYNTHETIC SCENE GRAPH  (kitchen + office scenario from the lecture)
# ─────────────────────────────────────────────────────────────────────────────

def build_demo_scene() -> SceneGraph:
    g = SceneGraph()

    # ── Nodes ─────────────────────────────────────────────────────────────────
    g.add_node(Node("kitchen",     "kitchen",      "room",        1.00, (0, 0, 0)))
    g.add_node(Node("hallway",     "hallway",      "room",        1.00, (5, 0, 0)))
    g.add_node(Node("table_01",    "dining table", "furniture",   0.92, (1.2, 0.8, 0.75)))
    g.add_node(Node("counter_01",  "counter",      "furniture",   0.85, (2.5, 0.2, 0.90)))
    g.add_node(Node("mug_01",      "mug",          "container",   0.91, (1.2, 0.8, 0.85)))
    g.add_node(Node("cup_02",      "cup",          "container",   0.63, (2.5, 0.2, 1.00), obs_count=1))
    g.add_node(Node("laptop_01",   "laptop",       "electronics", 0.88, (1.4, 0.8, 0.80)))
    g.add_node(Node("kettle_01",   "kettle",       "appliance",   0.79, (2.5, 0.4, 0.95)))
    g.add_node(Node("door_01",     "door",         "structure",   0.95, (3.5, 0.0, 1.00)))
    g.add_node(Node("person_01",   "person",       "agent",       0.99, (0.5, 1.5, 1.70)))

    # ── Edges ─────────────────────────────────────────────────────────────────
    # Spatial
    g.add_edge(Edge("mug_01",   "on",        "table_01",   0.82))
    g.add_edge(Edge("cup_02",   "on",        "counter_01", 0.55))
    g.add_edge(Edge("laptop_01","on",        "table_01",   0.87))
    g.add_edge(Edge("kettle_01","on",        "counter_01", 0.74))
    g.add_edge(Edge("mug_01",   "near",      "laptop_01",  0.71))

    # Containment
    g.add_edge(Edge("table_01",  "in-room",  "kitchen",    0.95))
    g.add_edge(Edge("counter_01","in-room",  "kitchen",    0.92))
    g.add_edge(Edge("mug_01",    "in-room",  "kitchen",    0.91))
    g.add_edge(Edge("cup_02",    "in-room",  "kitchen",    0.63))
    g.add_edge(Edge("laptop_01", "in-room",  "kitchen",    0.88))
    g.add_edge(Edge("kettle_01", "in-room",  "kitchen",    0.79))

    # Connectivity
    g.add_edge(Edge("door_01",   "connects", "kitchen",    0.95))
    g.add_edge(Edge("door_01",   "connects", "hallway",    0.95))

    # Instance-of (ontology)
    g.add_edge(Edge("mug_01",   "instance-of", "cup",     1.00))
    g.add_edge(Edge("cup_02",   "instance-of", "cup",     1.00))

    return g


# ─────────────────────────────────────────────────────────────────────────────
# ONTOLOGY  —  synonym / hypernym expansion
# ─────────────────────────────────────────────────────────────────────────────

ONTOLOGY = {
    "cup":      ["cup", "mug", "glass", "tumbler", "beaker"],
    "mug":      ["mug", "cup"],
    "drink":    ["cup", "mug", "glass", "bottle", "can"],
    "table":    ["dining table", "table", "desk", "coffee table", "workbench"],
    "surface":  ["dining table", "table", "desk", "counter", "shelf"],
    "counter":  ["counter", "countertop", "worktop"],
    "laptop":   ["laptop", "computer", "notebook"],
    "kettle":   ["kettle", "electric kettle"],
    "charger":  ["charger", "power adapter", "cable"],
    "food":     ["apple", "banana", "sandwich", "pizza", "cake"],
}

def expand_labels(label: str) -> list:
    label = label.lower().strip()
    if label in ONTOLOGY:
        return ONTOLOGY[label]
    # partial match
    for key, synonyms in ONTOLOGY.items():
        if label in key or key in label:
            return synonyms
    return [label]


# ─────────────────────────────────────────────────────────────────────────────
# QWEN LLM WRAPPER
# ─────────────────────────────────────────────────────────────────────────────

SYSTEM_PROMPT = """You are a semantic parser for a mobile robot. 
Given the robot's current scene graph and a user instruction, output a JSON task plan.

The JSON must have exactly these fields:
{
  "action": "FETCH" | "GO_TO" | "DELIVER" | "EXPLORE" | "SEQUENCE",
  "subgoals": [
    {
      "action": "FETCH" | "GO_TO" | "DELIVER" | "EXPLORE",
      "target_labels": ["label1", "label2"],
      "spatial_relation": "on" | "near" | "in-room" | null,
      "anchor_label": "label or null",
      "destination": "user" | "node_id" | null
    }
  ],
  "clarification_needed": true | false,
  "clarification_question": "string or null",
  "reasoning": "one sentence explanation"
}

Rules:
- Use "target_labels" as a list of synonyms (e.g. ["cup","mug","glass"]).
- If the instruction is a single action, subgoals has one entry.
- If the instruction requires a sequence (e.g. "make coffee"), list all subgoals in order.
- If the scene graph has no suitable node, set clarification_needed to true.
- Output ONLY the JSON object, no other text, no markdown fences."""


def build_user_prompt(utterance: str, graph: SceneGraph) -> str:
    return f"""Scene graph:
{graph.serialize()}

User instruction: "{utterance}"

Output the JSON task plan:"""


class QwenParser:
    def __init__(self, model_name: str = "Qwen/Qwen2.5-0.5B-Instruct"):
        print(f"\n[LLM] Loading {model_name}  (first run downloads ~1 GB) ...")
        from transformers import AutoModelForCausalLM, AutoTokenizer
        import torch

        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype="auto",
            device_map="auto",
        )
        self.model_name = model_name
        print(f"[LLM] Loaded on {next(self.model.parameters()).device}")

    def parse(self, utterance: str, graph: SceneGraph) -> dict:
        messages = [
            {"role": "system",  "content": SYSTEM_PROMPT},
            {"role": "user",    "content": build_user_prompt(utterance, graph)},
        ]
        text = self.tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = self.tokenizer([text], return_tensors="pt").to(self.model.device)

        with __import__("torch").no_grad():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=512,
                temperature=0.1,
                do_sample=True,
                pad_token_id=self.tokenizer.eos_token_id,
            )

        generated = outputs[0][inputs.input_ids.shape[1]:]
        raw = self.tokenizer.decode(generated, skip_special_tokens=True).strip()
        return raw, _repair_json(raw)


def _repair_json(raw: str) -> Optional[dict]:
    """
    Robustly extract and parse a JSON object from LLM output.
    Handles markdown fences, leading/trailing text, and common small errors.
    """
    # Strip markdown fences
    raw = re.sub(r"```(?:json)?", "", raw).strip()

    # Find first { ... } block
    start = raw.find("{")
    end   = raw.rfind("}")
    if start == -1 or end == -1:
        return None
    candidate = raw[start:end+1]

    # Attempt direct parse
    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        pass

    # Fix common issues: trailing commas, single quotes
    candidate = re.sub(r",\s*([}\]])", r"\1", candidate)   # trailing commas
    candidate = candidate.replace("'", '"')                  # single → double quotes
    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        pass

    return None


# ─────────────────────────────────────────────────────────────────────────────
# GRAPH QUERY  (lecture section 2)
# ─────────────────────────────────────────────────────────────────────────────

def query_graph(graph: SceneGraph, target_labels: list,
                spatial_relation: Optional[str],
                anchor_label: Optional[str]) -> list:
    """
    Return list of (node, utterance_likelihood) for candidate nodes.
    utterance_likelihood = edge_conf if spatial constraint matched, else small prior.
    """
    # Expand target labels via ontology
    expanded = set()
    for lbl in target_labels:
        expanded.update(expand_labels(lbl))

    candidates = []
    for node in graph.nodes.values():
        if node.label.lower() not in expanded:
            continue

        # Check spatial constraint
        spatial_score = 1.0  # no constraint → neutral
        if spatial_relation and anchor_label:
            anchor_expanded = expand_labels(anchor_label)
            matched_edges = [
                e for e in graph.get_edges_from(node.id)
                if e.relation == spatial_relation
                and graph.nodes.get(e.dst, Node("","","",0,(0,0,0))).label.lower()
                   in anchor_expanded
            ]
            if matched_edges:
                spatial_score = max(e.conf for e in matched_edges)
            else:
                spatial_score = 0.05   # spatial constraint stated but not found

        candidates.append((node, spatial_score))

    return candidates


# ─────────────────────────────────────────────────────────────────────────────
# REFERENCE RESOLUTION  (lecture section 3 — Bayes posterior)
# ─────────────────────────────────────────────────────────────────────────────

CLARIFICATION_THRESHOLD = 0.70   # τ from lecture

def resolve_reference(candidates: list) -> tuple:
    """
    Compute P(v* = v_i | u, G) ∝ P(u | v_i, G) · P(v_i | G)
    where P(v_i | G) = node.conf   and   P(u | v_i, G) = spatial_score.

    Returns (best_node, posterior_dict, needs_clarification).
    """
    if not candidates:
        return None, {}, False

    # Unnormalized
    unnorm = {}
    for node, spatial_score in candidates:
        unnorm[node.id] = node.conf * spatial_score

    Z = sum(unnorm.values())
    if Z < 1e-9:
        return None, {}, False

    posterior = {nid: v/Z for nid, v in unnorm.items()}
    best_id   = max(posterior, key=posterior.get)
    best_node = next(n for n, _ in candidates if n.id == best_id)

    needs_clarification = posterior[best_id] < CLARIFICATION_THRESHOLD

    return best_node, posterior, needs_clarification


# ─────────────────────────────────────────────────────────────────────────────
# GOAL EXTRACTION  (lecture section 5, stage 4)
# ─────────────────────────────────────────────────────────────────────────────

def extract_goal(node: Node) -> dict:
    """Translate resolved node to a geometric goal for the metric planner."""
    return {
        "node_id":    node.id,
        "label":      node.label,
        "pose":       node.pose,
        "conf":       node.conf,
        "action":     "NAVIGATE_TO",
        "note":       "Hand to metric planner (A* / RRT*)",
    }


# ─────────────────────────────────────────────────────────────────────────────
# PRETTY PRINTING
# ─────────────────────────────────────────────────────────────────────────────

W = 72  # column width

def hr(char="─"): print(char * W)

def banner(title):
    hr("═")
    print(f"  {title}")
    hr("═")

def section(title):
    print()
    hr()
    print(f"  {title}")
    hr()

def indent(text, n=4):
    pad = " " * n
    for line in text.strip().splitlines():
        print(pad + line)


def print_scene_graph(g: SceneGraph):
    section("SCENE GRAPH  (lecture: G = (V, E))")
    print("  Nodes:")
    for n in g.nodes.values():
        print(f"    [{n.category:12s}]  {n.id:15s}  p(ℓ)={n.conf:.2f}  pose={n.pose}")
    print("\n  Edges:")
    for e in g.edges:
        print(f"    ({e.src:15s}, {e.relation:12s}, {e.dst:15s})  conf={e.conf:.2f}")


def print_llm_output(raw: str, parsed: Optional[dict]):
    section("STAGE 1 — Semantic Parsing  (Qwen2.5-0.5B-Instruct)")
    print("  Raw LLM output:")
    indent(raw)
    print()
    if parsed:
        print("  Parsed task plan:")
        indent(json.dumps(parsed, indent=4))
    else:
        print("  [!] JSON parsing failed — falling back to clarification request")


def print_query_results(subgoal: dict, candidates: list):
    section(f"STAGE 2 — Graph Query")
    tl = subgoal.get("target_labels", [])
    sr = subgoal.get("spatial_relation")
    al = subgoal.get("anchor_label")
    print(f"  target_labels : {tl}")
    print(f"  expanded      : {sorted(set(l for t in tl for l in expand_labels(t)))}")
    print(f"  spatial       : {sr} → {al}")
    print()
    if candidates:
        print("  Candidate nodes:")
        for node, spatial_score in candidates:
            print(f"    {node.id:15s}  p(ℓ)={node.conf:.2f}  spatial_score={spatial_score:.2f}"
                  f"  unnorm={node.conf*spatial_score:.3f}")
    else:
        print("  [!] No candidates found in scene graph")


def print_resolution(best: Optional[Node], posterior: dict, needs_clarification: bool):
    section("STAGE 3 — Reference Resolution  P(v* | u, G)")
    print(f"  τ (clarification threshold) = {CLARIFICATION_THRESHOLD}")
    print()
    for nid, p in sorted(posterior.items(), key=lambda x: -x[1]):
        bar = "█" * int(p * 30)
        marker = "  ← selected" if (best and nid == best.id) else ""
        print(f"    {nid:15s}  {p:.3f}  {bar}{marker}")
    print()
    if needs_clarification:
        print(f"  [!] max posterior < τ — clarification needed")
    elif best:
        print(f"  ✓  Resolved to: {best}")


def print_goal(goal: Optional[dict]):
    section("STAGE 4/5 — Goal Extraction → Metric Planner")
    if goal:
        print(f"  Node  : {goal['node_id']}  ({goal['label']})")
        print(f"  Pose  : {goal['pose']}  metres")
        print(f"  Conf  : {goal['conf']:.2f}")
        print(f"  → {goal['note']}")
    else:
        print("  [!] No goal — robot reports object not found or requests clarification")


def print_failure_analysis(subgoal: dict, candidates: list,
                           best: Optional[Node], posterior: dict,
                           needs_clarification: bool):
    section("FAILURE MODE ANALYSIS  (lecture section 8)")

    if not candidates:
        print("  ✗ EMPTY QUERY RESULT")
        print("    → Object not in scene graph.  Robot should explore or report.")
        print("    → Check: ontology mismatch? Object not yet observed?")
        return

    if needs_clarification:
        print("  ⚠ AMBIGUOUS REFERENCE")
        print(f"    → Posterior peak {max(posterior.values()):.2f} < τ={CLARIFICATION_THRESHOLD}")
        print("    → Robot should ask: 'Which one did you mean?'")
        return

    if best:
        print("  ✓ Clean resolution — no failure modes triggered")
        if best.obs_count == 1:
            print("  ⚠ POSE UNCERTAINTY: node observed only once.")
            print("    → Consider active perception pass before execution.")


# ─────────────────────────────────────────────────────────────────────────────
# FALLBACK MOCK PARSER  (used when Qwen is not installed)
# ─────────────────────────────────────────────────────────────────────────────

MOCK_PLANS = {
    "bring me the cup on the table": {
        "action": "FETCH",
        "subgoals": [{
            "action": "FETCH",
            "target_labels": ["cup", "mug"],
            "spatial_relation": "on",
            "anchor_label": "table",
            "destination": "user"
        }],
        "clarification_needed": False,
        "clarification_question": None,
        "reasoning": "User wants the cup that is on the table."
    },
    "make me some coffee": {
        "action": "SEQUENCE",
        "subgoals": [
            {"action": "FETCH",  "target_labels": ["mug","cup"],
             "spatial_relation": None, "anchor_label": None, "destination": None},
            {"action": "GO_TO",  "target_labels": ["kettle"],
             "spatial_relation": None, "anchor_label": None, "destination": None},
            {"action": "DELIVER","target_labels": [],
             "spatial_relation": None, "anchor_label": None, "destination": "user"},
        ],
        "clarification_needed": False,
        "clarification_question": None,
        "reasoning": "Making coffee requires fetching a mug, using the kettle, and delivering."
    },
    "get me something to drink": {
        "action": "FETCH",
        "subgoals": [{
            "action": "FETCH",
            "target_labels": ["cup", "mug", "glass", "bottle"],
            "spatial_relation": None,
            "anchor_label": None,
            "destination": "user"
        }],
        "clarification_needed": False,
        "clarification_question": None,
        "reasoning": "User wants a drinkable container."
    },
    "find the charger": {
        "action": "EXPLORE",
        "subgoals": [{
            "action": "EXPLORE",
            "target_labels": ["charger", "power adapter"],
            "spatial_relation": None,
            "anchor_label": None,
            "destination": None
        }],
        "clarification_needed": True,
        "clarification_question": "I don't see a charger yet. Should I search the other rooms?",
        "reasoning": "No charger node found in scene graph."
    },
}

class MockParser:
    def parse(self, utterance: str, graph: SceneGraph):
        key = utterance.lower().strip().rstrip(".")
        plan = MOCK_PLANS.get(key, {
            "action": "FETCH",
            "subgoals": [{
                "action": "FETCH",
                "target_labels": [utterance.split()[-1]],
                "spatial_relation": None,
                "anchor_label": None,
                "destination": "user"
            }],
            "clarification_needed": False,
            "clarification_question": None,
            "reasoning": "Generic fetch task."
        })
        raw = json.dumps(plan, indent=2)
        return raw, plan


# ─────────────────────────────────────────────────────────────────────────────
# MAIN PIPELINE
# ─────────────────────────────────────────────────────────────────────────────

def run_pipeline(utterance: str, graph: SceneGraph, parser):
    banner(f'LANGUAGE GROUNDING PIPELINE  —  "{utterance}"')
    print_scene_graph(graph)

    # ── Stage 1: Semantic parsing ─────────────────────────────────────────────
    raw, plan = parser.parse(utterance, graph)
    print_llm_output(raw, plan)

    if plan is None:
        print("\n[!] Could not parse LLM output. Requesting clarification.")
        return

    if plan.get("clarification_needed"):
        print(f"\n  Robot: \"{plan.get('clarification_question', 'Could you clarify?')}\"")
        return

    subgoals = plan.get("subgoals", [])
    if not subgoals:
        print("\n[!] No subgoals in plan.")
        return

    print(f"\n  Reasoning: {plan.get('reasoning','')}")
    print(f"  Subgoals to execute: {len(subgoals)}")

    # ── Process each subgoal ──────────────────────────────────────────────────
    for i, subgoal in enumerate(subgoals):
        print(f"\n{'━'*W}")
        print(f"  Subgoal {i+1}/{len(subgoals)}:  action={subgoal.get('action')}")
        print(f"{'━'*W}")

        target_labels   = subgoal.get("target_labels", [])
        spatial_relation = subgoal.get("spatial_relation")
        anchor_label    = subgoal.get("anchor_label")

        if subgoal.get("action") == "DELIVER":
            print("  → DELIVER to user — motion primitive, no graph query needed.")
            continue

        if not target_labels:
            print("  → No target labels — skipping.")
            continue

        # Stage 2: Graph query
        candidates = query_graph(graph, target_labels, spatial_relation, anchor_label)
        print_query_results(subgoal, candidates)

        # Stage 3: Reference resolution
        best, posterior, needs_clarification = resolve_reference(candidates)
        print_resolution(best, posterior, needs_clarification)

        # Stage 4/5: Goal + plan
        goal = extract_goal(best) if (best and not needs_clarification) else None
        print_goal(goal)

        # Failure analysis
        print_failure_analysis(subgoal, candidates, best, posterior, needs_clarification)

    hr("═")
    print("  Pipeline complete.")
    hr("═")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# INTERACTIVE LOOP
# ─────────────────────────────────────────────────────────────────────────────

DEMO_UTTERANCES = [
    "bring me the cup on the table",
    "make me some coffee",
    "get me something to drink",
    "find the charger",
]

def interactive_loop(parser, graph: SceneGraph):
    print("\nEnter a natural language instruction (or 'quit' to exit).")
    print("Demo utterances:")
    for i, u in enumerate(DEMO_UTTERANCES, 1):
        print(f"  {i}. {u}")
    print()

    while True:
        try:
            raw = input("Instruction> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw or raw.lower() in ("quit","exit","q"):
            break

        # Allow number shortcuts
        if raw.isdigit() and 1 <= int(raw) <= len(DEMO_UTTERANCES):
            raw = DEMO_UTTERANCES[int(raw) - 1]
            print(f"  → Using: \"{raw}\"")

        run_pipeline(raw, graph, parser)


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Language Grounding Demo — Qwen2.5")
    ap.add_argument("--model",     default="Qwen/Qwen2.5-0.5B-Instruct",
                    help="HuggingFace model ID (default: Qwen/Qwen2.5-0.5B-Instruct)")
    ap.add_argument("--utterance", default=None,
                    help="Single utterance to process (omit for interactive mode)")
    ap.add_argument("--mock",      action="store_true",
                    help="Use mock parser (no model download, for testing graph pipeline)")
    args = ap.parse_args()

    # Build scene graph
    graph = build_demo_scene()

    # Choose parser
    if args.mock:
        print("[info] Using mock parser (--mock flag set)")
        parser = MockParser()
    else:
        try:
            parser = QwenParser(model_name=args.model)
        except ImportError:
            print("[warn] transformers/torch not found — falling back to mock parser")
            print("       Install with: pip install transformers torch accelerate")
            parser = MockParser()

    # Run
    if args.utterance:
        run_pipeline(args.utterance, graph, parser)
    else:
        interactive_loop(parser, graph)
