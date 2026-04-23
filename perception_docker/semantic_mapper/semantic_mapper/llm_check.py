#!/usr/bin/env python3
"""
llm_check.py — end-to-end sanity check for the LLM backend, no ROS.

Run this INSIDE the perception container BEFORE launching the pipeline
to verify that:
  1. The configured LLM backend (Ollama or OpenAI-compat) is reachable.
  2. The classifier prompt produces a sensible room label.
  3. The oracle prompt produces a parseable probability vector that
     normalizes to ~1.0.

Usage:
    python3 -m semantic_mapper.llm_check
    python3 -m semantic_mapper.llm_check --target "car keys"
    LLM_MODEL=llama3.2:3b python3 -m semantic_mapper.llm_check

Exits with code 0 on full success, 1 on any check failure. Intended
for use in CI / smoke-test scripts too.
"""

import argparse
import sys
import time

from semantic_mapper.llm_client import LLMClient
from semantic_mapper.room_classifier_node import (
    SYSTEM_PROMPT_TEMPLATE as CLS_SYS,
    USER_PROMPT_TEMPLATE as CLS_USER,
    DEFAULT_LABEL_SET,
    _format_object_list,
)
from semantic_mapper.llm_oracle_node import (
    SYSTEM_PROMPT as ORA_SYS,
    USER_PROMPT_TEMPLATE as ORA_USER,
    _format_rooms_block,
)


def _ok(s):   print(f"  \033[32m✓\033[0m {s}")
def _fail(s): print(f"  \033[31m✗\033[0m {s}")
def _hdr(s):  print(f"\n── {s} ──")


def check_ping(llm):
    _hdr("1. Ping LLM server")
    print(f"  backend={llm.cfg.backend}  url={llm.cfg.base_url}  "
          f"model={llm.cfg.model}")
    if llm.ping():
        _ok("server answered")
        return True
    _fail(f"server at {llm.cfg.base_url} did NOT answer")
    print("    fixes to try:")
    print("      - is Ollama running?         ollama serve &")
    print("      - is the model pulled?       ollama pull qwen2.5:3b-instruct")
    print("      - does --net=host work?      curl http://localhost:11434/api/tags")
    return False


def check_classifier(llm):
    _hdr("2. Room classifier prompt")
    objs = ["refrigerator", "sink", "refrigerator", "dining_table", "oven"]
    sys_p = CLS_SYS.format(label_set=", ".join(DEFAULT_LABEL_SET))
    user_p = CLS_USER.format(obj_list=_format_object_list(objs))
    print(f"  objects: {objs}")

    t0 = time.time()
    try:
        reply = llm.chat_json(sys_p, user_p)
    except Exception as e:
        _fail(f"call failed: {e}")
        return False
    dt = time.time() - t0
    print(f"  round-trip: {dt:.2f}s")
    print(f"  reply: {reply}")

    label = str(reply.get("label", "")).lower()
    if label in DEFAULT_LABEL_SET:
        _ok(f"label '{label}' is in the configured label set")
    else:
        _fail(f"label '{label}' is NOT in the label set (will be coerced to 'unknown')")
        return False
    # Soft check: a kitchen-like prompt ideally returns 'kitchen'.
    # Not strictly required; small models sometimes say dining_room.
    if label == "kitchen":
        _ok("label is 'kitchen' as expected for these objects")
    else:
        print(f"  note: expected 'kitchen', got '{label}'. Usually OK, "
              "but if this is consistently wrong consider a larger model.")
    return True


def check_oracle(llm, target):
    _hdr("3. LLM oracle prompt")
    # A small synthetic scene: one kitchen-like room lightly searched,
    # one bedroom-like room heavily searched. For target "car keys" we'd
    # expect the bedroom to win (common commonsense), but we mainly
    # verify the REPLY SHAPE here, not the specific numbers.
    rooms = [
        {"id": 0, "label": "kitchen",
         "time_in_room_s": 5.0, "frontier_clusters": 3,
         "objects": [{"class": "refrigerator"}, {"class": "sink"}]},
        {"id": 1, "label": "bedroom",
         "time_in_room_s": 60.0, "frontier_clusters": 1,
         "objects": [{"class": "bed"}, {"class": "nightstand"}]},
        {"id": 2, "label": "hallway",
         "time_in_room_s": 0.0, "frontier_clusters": 2,
         "objects": []},
    ]
    user_p = ORA_USER.format(
        target=target,
        rooms_block=_format_rooms_block(rooms),
        n_rooms=len(rooms),
    )
    print(f"  target: {target!r}")

    t0 = time.time()
    try:
        reply = llm.chat_json(ORA_SYS, user_p)
    except Exception as e:
        _fail(f"call failed: {e}")
        return False
    dt = time.time() - t0
    print(f"  round-trip: {dt:.2f}s")
    print(f"  reply: {reply}")

    entries = reply.get("rooms") if isinstance(reply, dict) else None
    if not isinstance(entries, list) or not entries:
        _fail("reply did not contain a non-empty 'rooms' list")
        return False
    _ok(f"reply contains {len(entries)} room entries")

    # Quick shape check + normalization.
    per_id = {}
    for e in entries:
        try:
            rid = int(e["id"])
            p   = float(e["probability"])
        except (KeyError, TypeError, ValueError):
            _fail(f"malformed entry: {e}")
            return False
        per_id[rid] = p

    # Every input room present?
    missing = [r["id"] for r in rooms if r["id"] not in per_id]
    if missing:
        print(f"  note: LLM omitted rooms {missing} (they will get 0 after merging)")
    else:
        _ok("every input room got a probability")

    total = sum(per_id.values())
    if total <= 0:
        _fail("all probabilities are 0; oracle would fall back to uniform")
        return False
    normed = {k: v / total for k, v in per_id.items()}
    top = sorted(normed.items(), key=lambda kv: -kv[1])
    top_str = ", ".join(f"R{k}={v:.2f}" for k, v in top)
    print(f"  normalized: {top_str}   (sum = 1.00)")
    _ok("probabilities normalize cleanly")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="car keys",
                    help="target object for the oracle test")
    args = ap.parse_args()

    print(f"\n=== LLM backend sanity check ===")
    llm = LLMClient.from_env()
    passed = []
    passed.append(check_ping(llm))
    if not passed[-1]:
        print("\nStopping — server unreachable means the next checks will "
              "all fail the same way.")
        sys.exit(1)
    passed.append(check_classifier(llm))
    passed.append(check_oracle(llm, args.target))

    print()
    if all(passed):
        print("\033[32mAll checks passed.\033[0m Launching the pipeline should work.")
        sys.exit(0)
    print("\033[31mOne or more checks failed.\033[0m See messages above.")
    sys.exit(1)


if __name__ == "__main__":
    main()