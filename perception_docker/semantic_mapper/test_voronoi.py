"""
Synthetic 2-room test: two rectangular rooms joined by a doorway.

Strategy: load the source, strip the ROS-y imports and the SemanticMapper
class via AST surgery, exec what remains. Then test VoronoiGraph alone.
"""
import ast

src = open("/home/claude/semantic_mapper_node.py").read()
tree = ast.parse(src)

KEEP_STDLIB = {"json", "math", "time", "collections"}
KEEP_THIRD  = {"numpy", "scipy", "skimage"}

new_body = []
for node in tree.body:
    if isinstance(node, ast.Import):
        keep = [n for n in node.names if n.name.split(".")[0] in KEEP_STDLIB | KEEP_THIRD]
        if keep:
            node.names = keep
            new_body.append(node)
    elif isinstance(node, ast.ImportFrom):
        mod = (node.module or "").split(".")[0]
        if mod in KEEP_STDLIB | KEEP_THIRD:
            new_body.append(node)
    elif isinstance(node, ast.ClassDef) and node.name in ("BEVGrid", "VoronoiGraph"):
        new_body.append(node)

tree.body = new_body
ast.fix_missing_locations(tree)
ns = {}
exec(compile(tree, "smn_isolated", "exec"), ns)
VoronoiGraph = ns["VoronoiGraph"]

import numpy as np
from scipy.ndimage import distance_transform_edt

# ── Test 1: two rooms joined by a corridor ──
H, W = 60, 80
free = np.zeros((H, W), dtype=bool)
free[5:35, 5:35]  = True
free[5:35, 45:75] = True
free[15:19, 35:45] = True
print(f"[setup] free cells: {free.sum()}")

vg = VoronoiGraph()
vg.build(free, sparsify_dist_cells=4)
n_edges = sum(len(v) for v in vg.adj.values()) // 2
print(f"[build] nodes: {len(vg.nodes)}, edges: {n_edges}")

comp_of, n = vg.components()
print(f"[before cut] components: {n}")
assert n == 1, f"FAIL: expected 1 before cut, got {n}"

door_cell = (40, 17)
vg.cut_at_doors([door_cell], sigma_cells=4.0, threshold=0.6)
comp_of, n = vg.components()
print(f"[after  cut] components: {n}")
assert n == 2, f"FAIL: expected 2 after cut, got {n}"

# Cell assignment
node_mask = (vg.node_of >= 0)
comp_field = np.zeros_like(vg.node_of, dtype=np.int32)
for nid, cid in comp_of.items():
    cx, cy = vg.nodes[nid]
    comp_field[cy, cx] = cid + 1
_, idx = distance_transform_edt(~node_mask, return_indices=True)
ny, nx = idx
comp_per_cell = comp_field[ny, nx] - 1
cell_to_comp = np.where(free, comp_per_cell, -1)
left  = int(cell_to_comp[20, 20])
right = int(cell_to_comp[20, 60])
print(f"[assign] left={left}, right={right}")
assert left >= 0 and right >= 0 and left != right, \
    f"FAIL: rooms share or unassigned ({left} vs {right})"

# ── Test 2: no doors → still 1 component ──
vg2 = VoronoiGraph()
vg2.build(free, sparsify_dist_cells=4)
_, n2 = vg2.components()
assert n2 == 1, f"FAIL test2: expected 1, got {n2}"
print(f"[test2 no-door] components: {n2}  OK")

# ── Test 3: door far from any edge → no cut ──
vg4 = VoronoiGraph()
vg4.build(free, sparsify_dist_cells=4)
vg4.cut_at_doors([(0, 0)], sigma_cells=2.0, threshold=0.6)
_, n4 = vg4.components()
assert n4 == 1, f"FAIL test4: far door should not cut; got {n4}"
print(f"[test3 far door] components: {n4}  OK")

print("\n*** ALL VORONOI TESTS PASS ***")