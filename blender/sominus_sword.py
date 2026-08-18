"""
Parametric sword generator for Unreal Engine props.  Blender 3.x - 5.x.

Builds a single game-ready static mesh (blade + crossguard + grip + pommel),
marks sharp edges, generates UVs, builds UCX_ convex collision, and exports
FBX + GLB ready to drag into Unreal.

RUN FROM THE BLENDER GUI
    Scripting tab -> Open -> this file -> Run Script (Alt+P)

RUN HEADLESS (no Blender window)
    blender --background --python sominus_sword.py -- --out ./export

Everything you would want to change is in the PARAMS block below.
All lengths are in CENTIMETRES; the script converts to Blender metres so the
FBX lands in Unreal at the correct 1 uu = 1 cm scale with default settings.
"""

import sys
import math
import os

import bpy
import bmesh


# ----------------------------------------------------------------------------
# PARAMS - tune these against the reference image
# ----------------------------------------------------------------------------

ASSET_NAME = "SM_SominusSword"

BLADE_LENGTH      = 82.0   # guard to tip
BLADE_WIDTH       = 4.6    # full width at the base
BLADE_THICKNESS   = 0.90   # full thickness at the base

GUARD_SPAN        = 21.0   # tip to tip across the quillons
GUARD_DEPTH       = 2.6    # front to back
GUARD_HEIGHT      = 2.2    # along the blade axis
GUARD_SWEEP       = 0.14   # 0 = straight bar, higher = quillons curve blade-ward

GRIP_LENGTH       = 17.0
GRIP_WIDTH        = 3.1    # across the flats (matches the blade's wide axis)
GRIP_DEPTH        = 2.3

POMMEL_HEIGHT     = 5.0
POMMEL_RADIUS     = 3.2

# Fuller: the shallow groove running down the flat of the blade.
USE_FULLER        = True
FULLER_DEPTH      = 0.46   # fraction of half-thickness; lower = deeper groove
FULLER_START      = 0.05   # fraction along the blade
FULLER_END        = 0.62
FULLER_RAMP       = 0.06   # fade in/out length

# Where the object origin sits. Unreal rotates the prop around this point when
# it is socketed to a hand, so it belongs where the hand grips.
PIVOT_MODE        = "GRIP"     # GRIP | GUARD | POMMEL

# Which way the blade points in Blender's local space.
#   Z -> blade up  (safest, imports predictably, rotate in the socket)
#   X -> blade along +X, matching Unreal's socket-forward convention
BLADE_AXIS        = "Z"

SHARP_ANGLE_DEG   = 36.0   # edges sharper than this stop being smooth-shaded
UV_TEXEL_SCALE    = 12.0   # only used by the fallback UV projector

EXPORT_FBX        = True
EXPORT_GLB        = True
SAVE_BLEND        = True


# ----------------------------------------------------------------------------
# Cross-section shape constants
# ----------------------------------------------------------------------------

CM = 0.01          # centimetres -> Blender metres

SHOULDER = 0.74    # where the flat of the blade breaks into the edge bevel
FULLER_X = 0.36    # half-width of the fuller groove

# (fraction along blade, width multiplier, thickness multiplier)
BLADE_PROFILE = [
    (0.000, 1.000, 1.000),   # ricasso, straight off the guard
    (0.045, 1.000, 0.985),
    (0.200, 0.962, 0.930),
    (0.400, 0.918, 0.855),
    (0.600, 0.868, 0.770),
    (0.760, 0.806, 0.678),
    (0.860, 0.724, 0.578),
    (0.930, 0.552, 0.462),
    (0.970, 0.330, 0.330),
    (0.985, 0.120, 0.180),   # tip vertex is added past this
]

# (height fraction, radius multiplier) - grip barrels slightly in the middle
GRIP_PROFILE = [
    (0.00, 1.000),
    (0.10, 1.055),
    (0.35, 1.030),
    (0.60, 0.985),
    (0.85, 0.945),
    (1.00, 0.930),
]

# (x fraction across the span, depth mult, height mult, blade-ward offset mult)
GUARD_STATIONS = [
    (-1.00, 0.42, 0.44, 1.00),
    (-0.80, 0.68, 0.76, 0.62),
    (-0.42, 0.94, 0.97, 0.18),
    ( 0.00, 1.00, 1.00, 0.00),
    ( 0.42, 0.94, 0.97, 0.18),
    ( 0.80, 0.68, 0.76, 0.62),
    ( 1.00, 0.42, 0.44, 1.00),
]

# (height fraction, radius multiplier) - revolved to make the pommel
POMMEL_PROFILE = [
    (0.00, 0.22),
    (0.16, 0.70),
    (0.40, 1.000),
    (0.68, 0.930),
    (0.88, 0.640),
    (1.00, 0.360),
]

MAT_BLADE, MAT_FITTING, MAT_GRIP = 0, 1, 2


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------

def lerp(a, b, k):
    return a + (b - a) * k


def smoothstep(k):
    k = min(max(k, 0.0), 1.0)
    return k * k * (3.0 - 2.0 * k)


def sample_profile(profile, s):
    """Linearly sample a list of (position, value) pairs."""
    if s <= profile[0][0]:
        return profile[0][1]
    if s >= profile[-1][0]:
        return profile[-1][1]
    for (s0, v0), (s1, v1) in zip(profile, profile[1:]):
        if s0 <= s <= s1:
            span = s1 - s0
            return v0 if span <= 0.0 else lerp(v0, v1, (s - s0) / span)
    return profile[-1][1]


class MeshBuilder:
    """Accumulates verts and faces, remembering a material slot per face."""

    def __init__(self):
        self.verts = []
        self.faces = []

    def vert(self, co):
        self.verts.append(co)
        return len(self.verts) - 1

    def face(self, indices, material):
        self.faces.append((tuple(indices), material))

    def ring(self, coords):
        return [self.vert(co) for co in coords]

    def bridge(self, ring_a, ring_b, material):
        n = len(ring_a)
        for k in range(n):
            k2 = (k + 1) % n
            self.face([ring_a[k], ring_a[k2], ring_b[k2], ring_b[k]], material)

    def fan(self, ring, apex_co, material):
        apex = self.vert(apex_co)
        n = len(ring)
        for k in range(n):
            self.face([ring[k], ring[(k + 1) % n], apex], material)

    def loft(self, rings_coords, material, cap_start=True, cap_end=True, tip=None):
        rings = [self.ring(r) for r in rings_coords]
        for a, b in zip(rings, rings[1:]):
            self.bridge(a, b, material)
        if cap_start:
            self.face(list(reversed(rings[0])), material)
        if tip is not None:
            self.fan(rings[-1], tip, material)
        elif cap_end:
            self.face(rings[-1], material)
        return rings

    def to_mesh(self, name):
        me = bpy.data.meshes.new(name)
        me.from_pydata(self.verts, [], [f for f, _ in self.faces])
        me.update()
        for poly, (_, material) in zip(me.polygons, self.faces):
            poly.material_index = material
        me.validate(verbose=False)
        return me


# ----------------------------------------------------------------------------
# Cross sections
# ----------------------------------------------------------------------------

def blade_section(half_w, half_t, fuller, z):
    """Ten-vertex lenticular section. fuller=1.0 means no groove."""
    return [
        (-half_w,            0.0,             z),
        (-half_w * SHOULDER,  half_t,         z),
        (-half_w * FULLER_X,  half_t * fuller, z),
        ( half_w * FULLER_X,  half_t * fuller, z),
        ( half_w * SHOULDER,  half_t,         z),
        ( half_w,            0.0,             z),
        ( half_w * SHOULDER, -half_t,         z),
        ( half_w * FULLER_X, -half_t * fuller, z),
        (-half_w * FULLER_X, -half_t * fuller, z),
        (-half_w * SHOULDER, -half_t,         z),
    ]


def fuller_at(s):
    if not USE_FULLER:
        return 1.0
    a, b, ramp = FULLER_START, FULLER_END, max(FULLER_RAMP, 1e-4)
    if s <= a - ramp or s >= b + ramp:
        return 1.0
    if s < a:
        return lerp(1.0, FULLER_DEPTH, smoothstep((s - (a - ramp)) / ramp))
    if s > b:
        return lerp(1.0, FULLER_DEPTH, smoothstep(((b + ramp) - s) / ramp))
    return FULLER_DEPTH


def ellipse_ring(rx, ry, z, n=12):
    return [
        (rx * math.cos(2.0 * math.pi * i / n),
         ry * math.sin(2.0 * math.pi * i / n),
         z)
        for i in range(n)
    ]


def ellipse_ring_yz(x, ry, rz, z_centre, n=10):
    return [
        (x,
         ry * math.cos(2.0 * math.pi * i / n),
         z_centre + rz * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


# ----------------------------------------------------------------------------
# Part builders
# ----------------------------------------------------------------------------

def build_blade(mb, z0, length):
    rings = []
    for s, ws, ts in BLADE_PROFILE:
        half_w = BLADE_WIDTH * CM * ws * 0.5
        half_t = BLADE_THICKNESS * CM * ts * 0.5
        rings.append(blade_section(half_w, half_t, fuller_at(s), z0 + s * length))
    mb.loft(rings, MAT_BLADE, cap_start=True, tip=(0.0, 0.0, z0 + length))


def build_guard(mb, z0, height):
    half_span = GUARD_SPAN * CM * 0.5
    half_depth = GUARD_DEPTH * CM * 0.5
    half_height = height * 0.5
    centre = z0 + half_height
    sweep = GUARD_SWEEP * height

    rings = [
        ellipse_ring_yz(half_span * xf,
                        half_depth * yf,
                        half_height * zf,
                        centre + sweep * off)
        for xf, yf, zf, off in GUARD_STATIONS
    ]
    mb.loft(rings, MAT_FITTING, cap_start=True, cap_end=True)


def build_grip(mb, z0, length):
    rings = []
    for hf, rf in GRIP_PROFILE:
        rings.append(ellipse_ring(GRIP_WIDTH * CM * 0.5 * rf,
                                  GRIP_DEPTH * CM * 0.5 * rf,
                                  z0 + hf * length))
    mb.loft(rings, MAT_GRIP, cap_start=True, cap_end=True)


def build_pommel(mb, z0, height):
    radius = POMMEL_RADIUS * CM * 0.5
    rings = []
    for hf, rf in POMMEL_PROFILE:
        rings.append(ellipse_ring(radius * rf,
                                  radius * rf * 0.80,
                                  z0 + hf * height))
    mb.loft(rings, MAT_FITTING, cap_start=False, cap_end=True,
            tip=None)
    # close the underside with a fan down to a centre point
    bottom = ellipse_ring(radius * POMMEL_PROFILE[0][1],
                          radius * POMMEL_PROFILE[0][1] * 0.80, z0)
    ring = mb.ring(bottom)
    mb.fan(ring, (0.0, 0.0, z0 - radius * 0.18), MAT_FITTING)


# ----------------------------------------------------------------------------
# Materials, shading, UVs
# ----------------------------------------------------------------------------

def make_material(name, base_color, metallic, roughness):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = base_color
        for key, value in (("Metallic", metallic), ("Roughness", roughness)):
            if key in bsdf.inputs:
                bsdf.inputs[key].default_value = value
    return mat


def mark_sharp_edges(me, angle_deg):
    threshold = math.radians(angle_deg)
    for poly in me.polygons:
        poly.use_smooth = True
    bm = bmesh.new()
    bm.from_mesh(me)
    for edge in bm.edges:
        if len(edge.link_faces) == 2:
            edge.smooth = edge.calc_face_angle(0.0) <= threshold
    bm.to_mesh(me)
    bm.free()
    me.update()


def fallback_box_uv(me, scale):
    """Planar projection per face along its dominant axis. Islands overlap but
    every loop gets a valid UV, which is what matters for import."""
    if not me.uv_layers:
        me.uv_layers.new(name="UVMap")
    uv_data = me.uv_layers.active.data
    for poly in me.polygons:
        n = poly.normal
        axis = max(range(3), key=lambda i: abs(n[i]))
        for loop_index in poly.loop_indices:
            co = me.vertices[me.loops[loop_index].vertex_index].co
            if axis == 0:
                u, v = co.y, co.z
            elif axis == 1:
                u, v = co.x, co.z
            else:
                u, v = co.x, co.y
            uv_data[loop_index].uv = (u * scale, v * scale)


def unwrap(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    try:
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        try:
            bpy.ops.uv.smart_project(angle_limit=math.radians(66.0),
                                     island_margin=0.02)
        except TypeError:
            bpy.ops.uv.smart_project(angle_limit=66.0, island_margin=0.02)
        bpy.ops.object.mode_set(mode="OBJECT")
        print("  UVs: smart project")
        return
    except Exception as exc:
        try:
            bpy.ops.object.mode_set(mode="OBJECT")
        except Exception:
            pass
        print("  UVs: smart project unavailable (%s), using planar fallback" % exc)
    fallback_box_uv(obj.data, UV_TEXEL_SCALE)


# ----------------------------------------------------------------------------
# Collision
# ----------------------------------------------------------------------------

def make_box(name, centre, half_extents, collection):
    mb = MeshBuilder()
    cx, cy, cz = centre
    hx, hy, hz = half_extents
    corners = [
        (cx - hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz), (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz), (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz + hz),
    ]
    for co in corners:
        mb.vert(co)
    for quad in [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
                 (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]:
        mb.face(quad, 0)
    me = mb.to_mesh(name)
    obj = bpy.data.objects.new(name, me)
    collection.objects.link(obj)
    obj.display_type = "WIRE"
    return obj


def build_collision(base_name, spans, collection):
    """spans: list of (z_low, z_high, half_x, half_y) in local space."""
    boxes = []
    for i, (z0, z1, hx, hy) in enumerate(spans):
        name = "UCX_%s_%02d" % (base_name, i)
        boxes.append(make_box(name,
                              (0.0, 0.0, (z0 + z1) * 0.5),
                              (hx, hy, (z1 - z0) * 0.5),
                              collection))
    return boxes


# ----------------------------------------------------------------------------
# Assembly
# ----------------------------------------------------------------------------

def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def build():
    clear_scene()

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "CENTIMETERS"

    pommel_h = POMMEL_HEIGHT * CM
    grip_h = GRIP_LENGTH * CM
    guard_h = GUARD_HEIGHT * CM
    blade_l = BLADE_LENGTH * CM

    z_pommel = 0.0
    z_grip = z_pommel + pommel_h
    z_guard = z_grip + grip_h
    z_blade = z_guard + guard_h
    z_tip = z_blade + blade_l

    mb = MeshBuilder()
    build_pommel(mb, z_pommel, pommel_h)
    build_grip(mb, z_grip, grip_h)
    build_guard(mb, z_guard, guard_h)
    build_blade(mb, z_blade, blade_l)

    me = mb.to_mesh(ASSET_NAME)
    obj = bpy.data.objects.new(ASSET_NAME, me)
    scene.collection.objects.link(obj)

    for mat in (
        make_material("MI_Sword_Blade",   (0.62, 0.64, 0.68, 1.0), 1.0, 0.18),
        make_material("MI_Sword_Fittings",(0.34, 0.30, 0.24, 1.0), 1.0, 0.42),
        make_material("MI_Sword_Grip",    (0.13, 0.09, 0.07, 1.0), 0.0, 0.78),
    ):
        me.materials.append(mat)

    # outward-facing normals
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-6)
    bm.to_mesh(me)
    bm.free()
    me.update()

    mark_sharp_edges(me, SHARP_ANGLE_DEG)
    unwrap(obj)

    # collision, still in pre-pivot space
    half_blade_x = BLADE_WIDTH * CM * 0.55
    half_blade_y = BLADE_THICKNESS * CM * 0.60
    collision = build_collision(ASSET_NAME, [
        (z_blade, z_tip, half_blade_x, half_blade_y),
        (z_guard, z_blade, GUARD_SPAN * CM * 0.5, GUARD_DEPTH * CM * 0.55),
        (z_pommel, z_guard, POMMEL_RADIUS * CM * 0.55, POMMEL_RADIUS * CM * 0.45),
    ], scene.collection)

    # shift everything so the chosen pivot lands on the origin
    pivot_z = {
        "GRIP": z_grip + grip_h * 0.5,
        "GUARD": z_guard + guard_h * 0.5,
        "POMMEL": z_pommel,
    }.get(PIVOT_MODE.upper(), z_grip + grip_h * 0.5)

    for target in [obj] + collision:
        for vert in target.data.vertices:
            vert.co.z -= pivot_z
        target.data.update()

    if BLADE_AXIS.upper() == "X":
        rot = math.radians(90.0)
        for target in [obj] + collision:
            for vert in target.data.vertices:
                x, y, z = vert.co
                vert.co = (z, y, -x)   # +Z becomes +X
            target.data.update()
        _ = rot

    for target in collision:
        target.parent = obj

    return obj, collision, (z_tip - z_pommel)


# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------

def export(obj, collision, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    for c in collision:
        c.select_set(True)
    bpy.context.view_layer.objects.active = obj

    written = []

    if EXPORT_FBX:
        path = os.path.join(out_dir, ASSET_NAME + ".fbx")
        try:
            bpy.ops.export_scene.fbx(
                filepath=path,
                use_selection=True,
                object_types={"MESH"},
                apply_unit_scale=True,
                apply_scale_options="FBX_SCALE_NONE",
                global_scale=1.0,
                bake_space_transform=False,
                mesh_smooth_type="EDGE",
                use_mesh_modifiers=True,
                use_triangles=False,
                add_leaf_bones=False,
                axis_forward="-Z",
                axis_up="Y",
                path_mode="COPY",
            )
            written.append(path)
        except Exception as exc:
            print("  FBX export failed: %s" % exc)

    if EXPORT_GLB:
        path = os.path.join(out_dir, ASSET_NAME + ".glb")
        try:
            bpy.ops.export_scene.gltf(
                filepath=path,
                export_format="GLB",
                use_selection=True,
            )
            written.append(path)
        except Exception as exc:
            print("  GLB export failed: %s" % exc)

    if SAVE_BLEND:
        path = os.path.join(out_dir, ASSET_NAME + ".blend")
        try:
            bpy.ops.wm.save_as_mainfile(filepath=path)
            written.append(path)
        except Exception as exc:
            print("  .blend save failed: %s" % exc)

    return written


def parse_out_dir():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
        if "--out" in argv:
            i = argv.index("--out")
            if i + 1 < len(argv):
                return os.path.abspath(argv[i + 1])
    if bpy.data.filepath:
        return os.path.join(os.path.dirname(bpy.data.filepath), "export")
    return os.path.abspath("./export")


def main():
    print("Blender %s" % bpy.app.version_string)
    obj, collision, total_len = build()

    me = obj.data
    tris = sum(len(p.vertices) - 2 for p in me.polygons)
    dims = [d / CM for d in obj.dimensions]

    out_dir = parse_out_dir()
    written = export(obj, collision, out_dir)

    print("-" * 60)
    print("  asset        %s" % ASSET_NAME)
    print("  verts/tris   %d / %d" % (len(me.vertices), tris))
    print("  material ids %d" % len(me.materials))
    print("  overall      %.1f cm  (blade %.1f cm)" % (total_len / CM, BLADE_LENGTH))
    print("  bounds       %.1f x %.1f x %.1f cm" % (dims[0], dims[1], dims[2]))
    print("  pivot        %s" % PIVOT_MODE)
    print("  blade axis   +%s" % BLADE_AXIS.upper())
    print("  collision    %d UCX hulls" % len(collision))
    for path in written:
        print("  wrote        %s" % path)
    print("-" * 60)


if __name__ == "__main__":
    main()
