"""
Weather station enclosure -- bayonet-mount stack
=================================================

Stack:    foot  ->  tier_esp32  ->  tier_*  ->  ...  ->  cap

Foot:     just a wide thick plate with a bayonet neck on top. That is all.
          The ESP32 does NOT live in the foot -- it lives in a tier above.

Tier:     cylindrical donut. Inner column (cable shaft) + outer wall +
          tent-shaped floor (two slopes meeting at a ridge along the X axis,
          like an inverted roof) with two drain holes at the y-axis low points.
          Bayonet skirt below, bayonet neck above. The neck is connected to
          the body via an inward-tapered shoulder (so it isn't floating).
          Outer wall comes in three flavours:
            solid    closed cylinder
            louvred  Stevenson-screen: tilted annular slats held by 4 vertical
                     posts, with air gaps on BOTH inner and outer faces, so air
                     actually flows through the wall. Slats tilt outward-and-
                     down so vertical rain runs off the outer edge.
            esp32    solid wall + USB-C cutout, for the ESP32-bearing tier

Cap:      tier-style body (no floor, lets cables drop through) +
          tent roof with rain and BH1750 sensor pockets.

Run:      python3 generate_enclosure.py
Outputs:  foot.stl, tier_solid.stl, tier_louvred.stl, tier_esp32.stl,
          cap.stl, assembly_preview.step
"""

import cadquery as cq
import math
import os

OUT = os.path.dirname(os.path.abspath(__file__))


# =============================================================================
# Parameters (mm)
# =============================================================================
OD              = 120     # outer diameter of every body section
COL_ID          = 20      # cable column hole diameter
WALL            = 3       # default wall thickness
TIER_INNER_H    = 60      # body height per tier
NECK_H          = 16      # bayonet neck/skirt overlap height

# Foot (the stand)
FOOT_OD         = 200     # plate diameter (broad for stability)
FOOT_T          = 8       # plate thickness

# Bayonet (EXTERNAL design: lugs project OUTWARD from the lower piece's neck;
# the upper piece's skirt has L-slots cut all the way THROUGH the skirt wall,
# so both features are visible from outside, and the lug pokes through the
# slot when locked -- you can literally see it from outside.)
LUG_COUNT       = 2       # 2 lugs at 0 and 180 degrees
LUG_W_DEG       = 30      # lug arc width
LUG_H           = 5       # lug height (z dim)
LUG_PROJECT     = 5       # radial protrusion of lug from neck outer face
TWIST_LOCK_DEG  = 50      # rotation needed to lock
SKIRT_CLEAR     = 1.0     # radial clearance: skirt_inner - neck_outer
SLOT_CLEAR      = 0.6     # vertical clearance for the lug in the slot

# Floor (TENT-shape): two slopes meeting at a ridge along the X axis (just
# like the cap's roof but inverted INSIDE the tier), so any water inside flows
# down the slopes to TWO drain holes (one on each side, at the low points).
# Pitch matches the roof (15 deg) for visual consistency.
FLOOR_PITCH_DEG = 15
FLOOR_T_RIDGE   = 16      # floor thickness at the ridge (high point, along x axis)
FLOOR_T_EDGE    = 1.5     # floor thickness at the y-axis (low points, where drains are)
DRAIN_D         = 6       # diameter of each drain hole through the outer wall

# Body-to-neck shoulder: the neck (r=51-54) sits inside the body footprint
# (r=57-60), so without a transition the neck would just float above the body
# unattached. The top of every body section therefore tapers INWARD from
# r=BODY_INNER_R (at z = height - SHOULDER_H) to r=NECK_INNER_R (at z = height)
# so the neck has something to mate with.
SHOULDER_H      = 5

# Cable slots (through inner column wall) - run along x axis (perpendicular to
# the y-axis drains and along the floor's ridge)
CABLE_SLOT_W    = 12
CABLE_SLOT_H    = 10
CABLE_SLOT_COUNT = 2

# ESP32 tier (USB cutout -- intentionally generous so it clears any USB cable)
USB_W           = 22
USB_H           = 14
USB_Z_OFFSET    = 22      # USB centre this far above the inner floor surface

# Louvred wall (REAL Stevenson-screen): 4 vertical posts hold a stack of tilted
# annular slats. Slats DON'T touch each other -- there are horizontal gaps on
# BOTH the inner and outer faces, so air can actually pass through. The slats
# tilt outward-and-down so rain runs off the outer edge and falls clear.
SLAT_THICKNESS  = 2.5     # vertical (z) thickness of each slat at the inner face
SLAT_TILT       = 4       # outer edge of slat sits this far below the inner edge
POST_COUNT      = 4       # vertical posts that hold the slats in place
POST_W_DEG      = 10      # tangential width of each post (in degrees)
N_SLATS_TARGET  = 9       # target number of slats (pitch is auto-computed)
LOUVRE_MARGIN_BOT = 6     # height of the bottom plinth (at the inner face)
LOUVRE_MARGIN_TOP = 6     # height of the top shoulder (at the inner face)

# Cap
CAP_BODY_H      = 18
ROOF_PITCH_DEG  = 15
ROOF_OVERHANG   = 5

# Sensor pockets in cap roof
RAIN_X          = 52
RAIN_Y          = 38
RAIN_DEPTH      = 2.5
RAIN_CABLE_D    = 5
BH_X            = 22
BH_Y            = 14
BH_DEPTH        = 2.5
BH_CABLE_D      = 4

# =============================================================================
# Derived
# =============================================================================
COL_OUTER_R     = COL_ID / 2 + WALL          # 13: outer radius of inner column wall
BODY_INNER_R    = OD / 2 - WALL              # 57: inner face of body wall

# Bayonet sizing: the neck is intentionally narrower than the body so the skirt
# (which is at body OD) has a thick wall around the slot cuts. The lug projects
# from the neck outer face out to (or just shy of) the skirt outer face, so it
# pokes visibly THROUGH the slot when the bayonet is locked.
NECK_R          = OD / 2 - 6                 # 54: outer radius of neck
NECK_INNER_R    = NECK_R - WALL              # 51: inner radius of neck (3mm wall)
SKIRT_R_OUT     = OD / 2                     # 60: outer radius of skirt (= body OD)
SKIRT_R_IN      = NECK_R + SKIRT_CLEAR       # 55: inner radius of skirt (1mm air gap)
LUG_OUT_R       = NECK_R + LUG_PROJECT       # 59: tip of lug (just shy of skirt OD)


# =============================================================================
# Helpers
# =============================================================================

def annular_sector(r_in, r_out, theta_a, theta_b, h, z=0, n_seg=None):
    """Solid annular sector (donut wedge) extruded along Z. Angles in degrees."""
    if n_seg is None:
        n_seg = max(8, int(abs(theta_b - theta_a) / 2))
    pts = []
    for i in range(n_seg + 1):
        t = math.radians(theta_a + (theta_b - theta_a) * i / n_seg)
        pts.append((r_out * math.cos(t), r_out * math.sin(t)))
    for i in range(n_seg + 1):
        t = math.radians(theta_b - (theta_b - theta_a) * i / n_seg)
        pts.append((r_in * math.cos(t), r_in * math.sin(t)))
    return cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0, 0, z))


def hollow_cylinder(r_out, r_in, h, z=0):
    return (cq.Workplane("XY")
            .circle(r_out).circle(r_in)
            .extrude(h)
            .translate((0, 0, z)))


# =============================================================================
# Bayonet
# =============================================================================
# Mechanism (now actually working AND visible):
#   - The LOWER piece's NECK has 2 outward-projecting LUGS near the top. The
#     lugs stick out past the neck so they're visible bumps on its outer face.
#   - The UPPER piece's SKIRT has 2 L-shaped SLOTS cut all the way through the
#     skirt wall. The slots open at the bottom edge of the skirt and turn
#     sideways at the top.
#   - Drop the upper piece down: the lugs (on the lower's neck, sticking out)
#     enter the slots (in the upper's skirt) from below, and slide up the
#     vertical channel as the upper descends.
#   - When the upper's body bottom rests on the lower's neck top, the lugs are
#     at the top of the vertical channel. Twist TWIST_LOCK_DEG (e.g. 50 deg)
#     and the lugs slide sideways into the horizontal lock channel.
#   - Below the lock channel (in the same theta range) the skirt is solid, so
#     the lugs cannot slide back down -- the upper piece is mechanically
#     trapped on the lower piece. Twist back and lift to release.

def make_neck(z_base):
    """Lower bayonet half (male). Hollow ring with 2 outward-projecting lugs
    near the top."""
    neck = hollow_cylinder(NECK_R, NECK_INNER_R, NECK_H, z_base)
    for i in range(LUG_COUNT):
        theta = 360 * i / LUG_COUNT
        lug_z_bot = z_base + NECK_H - LUG_H - SLOT_CLEAR / 2
        lug = annular_sector(
            r_in=NECK_R - 0.1,                  # slight overlap into neck wall
            r_out=LUG_OUT_R,
            theta_a=theta - LUG_W_DEG / 2,
            theta_b=theta + LUG_W_DEG / 2,
            h=LUG_H,
            z=lug_z_bot,
        )
        neck = neck.union(lug)
    return neck


def make_skirt(z_top):
    """Upper bayonet half (female). Hollow ring hanging below the body, with 2
    L-slots cut through the wall."""
    skirt = hollow_cylinder(SKIRT_R_OUT, SKIRT_R_IN, NECK_H, z_top - NECK_H)
    for i in range(LUG_COUNT):
        theta = 360 * i / LUG_COUNT
        skirt = cut_skirt_l_slot(skirt, theta, z_top)
    return skirt


def cut_skirt_l_slot(skirt, theta_slot, z_top):
    """Cut an L-shaped slot through the skirt wall at angle theta_slot.

    Geometry (in the skirt's local z frame, with z_top = top of skirt = upper
    body's bottom, skirt occupies z = z_top - NECK_H to z_top):
      Vertical channel: full skirt height, opens at the bottom edge of the
                        skirt so the lug can enter from below as the upper
                        piece descends.
      Lock channel:     at the TOP of the vertical channel, extends sideways
                        by TWIST_LOCK_DEG. After the upper is fully seated,
                        a twist slides the lug into here.
    """
    z_bot = z_top - NECK_H
    vert_arc_w = LUG_W_DEG + 4
    lock_h = LUG_H + SLOT_CLEAR

    # Vertical channel (open at bottom, runs full skirt height)
    vert_cut = annular_sector(
        r_in=SKIRT_R_IN - 0.6,
        r_out=SKIRT_R_OUT + 0.6,
        theta_a=theta_slot - vert_arc_w / 2,
        theta_b=theta_slot + vert_arc_w / 2,
        h=NECK_H + 0.6,
        z=z_bot - 0.3,
    )
    # Lock channel (top of vertical, extends sideways)
    lock_cut = annular_sector(
        r_in=SKIRT_R_IN - 0.6,
        r_out=SKIRT_R_OUT + 0.6,
        theta_a=theta_slot + vert_arc_w / 2 - 1,
        theta_b=theta_slot + vert_arc_w / 2 + TWIST_LOCK_DEG,
        h=lock_h + 0.3,
        z=z_top - lock_h,
    )
    return skirt - vert_cut - lock_cut


# =============================================================================
# Outer-wall styles
# =============================================================================
# Every body section's top must TAPER INWARD from r=BODY_INNER_R (57) at
# z=height-SHOULDER_H to r=NECK_INNER_R (51) at z=height, so the neck has a
# wide flange of body material to mate with (otherwise the r=51-54 neck would
# just float above the r=57-60 body wall, unconnected to anything).

def make_outer_wall_solid(height, z_base):
    """Solid wall, hollow tube with the inward-tapered top shoulder."""
    z_taper = z_base + height - SHOULDER_H
    pts = [
        (BODY_INNER_R, z_base),                # inner-bot
        (OD / 2,       z_base),                # outer-bot
        (OD / 2,       z_base + height),       # outer-top
        (NECK_INNER_R, z_base + height),       # inner-top (= neck inner radius)
        (BODY_INNER_R, z_taper),               # taper start at inner face
    ]
    return (cq.Workplane("XZ").polyline(pts).close().revolve(360))


def make_outer_wall_louvred(height, z_base):
    """Real Stevenson-screen wall: tilted annular slats held by N vertical
    posts. Air gaps on BOTH the inner AND outer faces so air actually flows
    through the wall. Slats tilt outward-and-down so vertical rain runs off
    the outer edge and falls clear.

    Top and bottom are NOT free-floating rings -- they're integrated structural
    pieces:
      - bottom plinth : solid ring with sloped TOP that matches the bottom slat
                        (inner-high, outer-low), so the lowest slat seamlessly
                        sits on it without an air gap. Connects to the body
                        floor.
      - top shoulder  : solid wedge with sloped BOTTOM matching the top slat,
                        and the TOP face tapering INWARD from r=BODY_INNER_R
                        to r=NECK_INNER_R so the bayonet neck has something to
                        mate with. NOT a floating ring -- attached to the slat
                        stack along its entire bottom AND to the neck above.
      - posts         : 4 vertical bars at theta=0,90,180,270 connecting plinth
                        to shoulder through every slat.
      - slats         : tilted annular rings of variable count, evenly spaced
                        between plinth top and shoulder bottom.
    """
    plinth_h_inner = LOUVRE_MARGIN_BOT
    shoulder_h_inner = LOUVRE_MARGIN_TOP

    # ----- bottom plinth: sloped top matching slat tilt -----
    plinth_pts = [
        (BODY_INNER_R, z_base),                       # inner-bot
        (OD / 2,       z_base),                       # outer-bot
        (OD / 2,       z_base + plinth_h_inner - SLAT_TILT),  # outer-top (lower)
        (BODY_INNER_R, z_base + plinth_h_inner),      # inner-top (higher)
    ]
    plinth = (cq.Workplane("XZ").polyline(plinth_pts).close().revolve(360))

    # ----- top shoulder: sloped bottom matching slat tilt + tapered top -----
    z_shoulder_inner_bot = z_base + height - shoulder_h_inner
    z_shoulder_outer_bot = z_shoulder_inner_bot - SLAT_TILT
    z_top = z_base + height
    shoulder_pts = [
        (BODY_INNER_R, z_shoulder_inner_bot),    # inner-bot of shoulder
        (OD / 2,       z_shoulder_outer_bot),    # outer-bot of shoulder (matches slat tilt)
        (OD / 2,       z_top),                   # outer-top
        (NECK_INNER_R, z_top),                   # inner-top (= neck inner)
    ]
    shoulder = (cq.Workplane("XZ").polyline(shoulder_pts).close().revolve(360))

    result = plinth.union(shoulder)

    # ----- 4 vertical posts spanning plinth-top to shoulder-bottom -----
    post_zone_bot = z_base + plinth_h_inner - SLAT_TILT  # outer-top of plinth
    post_zone_top = z_top - shoulder_h_inner              # inner-bot of shoulder
    # actually the posts need to span enough z to support the slats; use a
    # generous range that overlaps with both plinth and shoulder for clean union
    post_z_bot = z_base + 0.5
    post_z_top = z_top - 0.5
    for i in range(POST_COUNT):
        theta = 360 * i / POST_COUNT
        post = annular_sector(
            r_in=BODY_INNER_R - 0.1,
            r_out=OD / 2 + 0.1,
            theta_a=theta - POST_W_DEG / 2,
            theta_b=theta + POST_W_DEG / 2,
            h=post_z_top - post_z_bot,
            z=post_z_bot,
        )
        result = result.union(post)

    # ----- slats: evenly distributed between plinth-top and shoulder-bottom -----
    # First slat sits with its INNER-bot at plinth_inner_top.
    # Last slat sits with its INNER-top at shoulder_inner_bot.
    # So slat[0].z_in_bot = z_base + plinth_h_inner, slat[n-1].z_in_top = z_shoulder_inner_bot.
    slat_zone_bot = z_base + plinth_h_inner
    slat_zone_top = z_shoulder_inner_bot
    n_slats = N_SLATS_TARGET
    if n_slats > 1:
        actual_pitch = (slat_zone_top - slat_zone_bot - SLAT_THICKNESS) / (n_slats - 1)
    else:
        actual_pitch = 0
    for i in range(n_slats):
        z_in_bot = slat_zone_bot + i * actual_pitch
        z_in_top = z_in_bot + SLAT_THICKNESS
        z_out_bot = z_in_bot - SLAT_TILT
        z_out_top = z_in_top - SLAT_TILT
        slat = (cq.Workplane("XZ")
                .polyline([
                    (BODY_INNER_R, z_in_bot),
                    (BODY_INNER_R, z_in_top),
                    (OD / 2,       z_out_top),
                    (OD / 2,       z_out_bot),
                ]).close()
                .revolve(360))
        result = result.union(slat)

    return result


def make_outer_wall_esp32(height, z_base):
    """Solid wall + USB-C cutout. For the tier that actually houses the ESP32."""
    wall = make_outer_wall_solid(height, z_base)
    usb_z = z_base + FLOOR_T_RIDGE + USB_Z_OFFSET - USB_H / 2
    usb = (cq.Workplane("XY")
           .box(OD, USB_W, USB_H, centered=(False, True, False))
           .translate((0, 0, usb_z)))
    return wall - usb


WALL_BUILDERS = {
    "solid":   make_outer_wall_solid,
    "louvred": make_outer_wall_louvred,
    "esp32":   make_outer_wall_esp32,
}


# =============================================================================
# Tier components: inner column wall, sloped floor, drain, cable slots
# =============================================================================

def make_inner_column_wall(height, z_base):
    return hollow_cylinder(COL_OUTER_R, COL_ID / 2, height, z_base)


def make_floor(z_base):
    """Tent-shaped floor: two slopes meeting at a ridge along the X axis, like
    an inverted roof. Water flows down both slopes to the two drain holes at
    y=+/-BODY_INNER_R (the low points).

    Geometry:
      - Bottom face: flat annular disk at z = z_base
      - Top face: tent (two planes meeting at ridge along x-axis)
      - Ridge height: z_base + FLOOR_T_RIDGE  (high, on x-axis at y=0)
      - Edge height: z_base + FLOOR_T_EDGE   (low, where y = +/-BODY_INNER_R)
      - Slope: ~15 deg (matches the cap roof for visual consistency)

    Construction: build the tent as a triangular-prism solid extruded along X,
    then INTERSECT with the annular cylindrical region (between the inner
    column and the outer body wall).
    """
    # Tent profile in YZ plane (the prism cross-section): triangle with apex
    # at the top centre (the ridge) and base at z=0.
    margin = 2  # extra reach to make sure the prism covers the full disk
    tent_profile = [
        (-(BODY_INNER_R + margin), 0),
        (-(BODY_INNER_R + margin), FLOOR_T_EDGE),
        (0,                         FLOOR_T_RIDGE),
        ( (BODY_INNER_R + margin),  FLOOR_T_EDGE),
        ( (BODY_INNER_R + margin),  0),
    ]
    tent = (cq.Workplane("YZ")
            .polyline(tent_profile).close()
            .extrude(BODY_INNER_R + margin, both=True))

    annular = (cq.Workplane("XY")
               .circle(BODY_INNER_R)
               .circle(COL_OUTER_R)
               .extrude(FLOOR_T_RIDGE + 1))

    return tent.intersect(annular).translate((0, 0, z_base))


def make_drain_cutter(z_base):
    """TWO drain holes at the two low points of the tent floor, one on the +y
    side and one on the -y side (where the slopes meet the outer wall).
    Bottom of each drain sits at floor level so pooled water always finds them."""
    drain_z = z_base + DRAIN_D / 2 + 0.3
    # Cylindrical drain along Y axis, centred at y=0 (will be cut on both
    # +y and -y sides since it's a long cylinder through the entire diameter).
    drain = (cq.Workplane("XZ")
             .center(0, drain_z)
             .circle(DRAIN_D / 2)
             .extrude(OD + 10, both=True))
    return drain


def make_cable_slot_cutter(z_base, body_h, theta):
    """Rectangular cutter through the inner column wall."""
    z_center = z_base + body_h / 2
    cutter = (cq.Workplane("XY")
              .box(20, CABLE_SLOT_W, CABLE_SLOT_H, centered=(False, True, False))
              .translate((COL_ID / 2 - 1, 0, z_center - CABLE_SLOT_H / 2)))
    return cutter.rotate((0, 0, 0), (0, 0, 1), theta)


# =============================================================================
# Main pieces
# =============================================================================

def make_foot():
    """Just a wide stable plate with a bayonet neck on top.
    The ESP32 lives in a tier above; the foot itself is sealed solid."""
    plate = (cq.Workplane("XY")
             .circle(FOOT_OD / 2)
             .extrude(FOOT_T))
    neck = make_neck(FOOT_T)
    return plate.union(neck)


def make_tier(wall_style="louvred"):
    """Stackable middle module. Body bottom at z=0, neck at z=TIER_INNER_H..+NECK_H.
    Skirt hangs below at z=-NECK_H..0. Floor at z=0..FLOOR_T (sloping)."""
    if wall_style not in WALL_BUILDERS:
        raise ValueError(f"unknown wall_style {wall_style!r}")

    body_z = 0
    body_h = TIER_INNER_H

    outer_wall = WALL_BUILDERS[wall_style](body_h, body_z)
    inner_col = make_inner_column_wall(body_h, body_z)
    floor = make_floor(body_z)

    body = outer_wall.union(inner_col).union(floor)
    body = body - make_drain_cutter(body_z)

    for i in range(CABLE_SLOT_COUNT):
        # 90 deg offset from bayonet positions (which are at 0 and 180)
        theta = 360 * i / CABLE_SLOT_COUNT + 90
        body = body - make_cable_slot_cutter(body_z, body_h, theta)

    skirt = make_skirt(0)
    neck = make_neck(body_h)

    return body.union(skirt).union(neck)


def make_cap():
    """Top piece. Tier-style body (no floor, so cables drop through) + tent roof
    with sensor pockets."""
    body_z = 0
    body_h = CAP_BODY_H

    outer = hollow_cylinder(OD / 2, BODY_INNER_R, body_h, body_z)
    inner_col = make_inner_column_wall(body_h, body_z)

    body = outer.union(inner_col)
    for i in range(CABLE_SLOT_COUNT):
        theta = 360 * i / CABLE_SLOT_COUNT + 90
        body = body - make_cable_slot_cutter(body_z, body_h, theta)

    skirt = make_skirt(body_z)
    roof = make_tent_roof(body_z + body_h)

    return body.union(skirt).union(roof)


def make_tent_roof(z_base):
    """Triangular-prism tent, trimmed to a circular plan view.
    Ridge along X axis. Two slopes pitched ROOF_PITCH_DEG. Sensor pockets are
    recessed into each slope."""
    half = OD / 2 + ROOF_OVERHANG
    rise = half * math.tan(math.radians(ROOF_PITCH_DEG))

    # Triangular prism
    pts = [(-half, 0), (half, 0), (0, rise)]
    tent = (cq.Workplane("YZ")
            .polyline(pts).close()
            .extrude(2 * half)
            .translate((-half, 0, z_base)))

    # Trim to a cylinder so the plan view is round, not square
    trim = (cq.Workplane("XY")
            .circle(half)
            .extrude(rise + 1)
            .translate((0, 0, z_base)))
    tent = tent.intersect(trim)

    # Rain sensor pocket on -Y slope
    rain_pocket = (cq.Workplane("XY")
                   .box(RAIN_X, RAIN_Y, RAIN_DEPTH + 2, centered=(True, True, False))
                   .translate((0, 0, -RAIN_DEPTH - 1)))
    rain_pocket = rain_pocket.rotate((0, 0, 0), (1, 0, 0), ROOF_PITCH_DEG)
    rain_pocket = rain_pocket.translate((0, -half / 2, z_base + rise / 2))
    tent = tent - rain_pocket

    rain_cable = (cq.Workplane("XY")
                  .center(0, -half / 2)
                  .circle(RAIN_CABLE_D / 2)
                  .extrude(rise + 5)
                  .translate((0, 0, z_base - 1)))
    tent = tent - rain_cable

    # BH1750 pocket on +Y slope
    bh_pocket = (cq.Workplane("XY")
                 .box(BH_X, BH_Y, BH_DEPTH + 2, centered=(True, True, False))
                 .translate((0, 0, -BH_DEPTH - 1)))
    bh_pocket = bh_pocket.rotate((0, 0, 0), (1, 0, 0), -ROOF_PITCH_DEG)
    bh_pocket = bh_pocket.translate((0, half / 2, z_base + rise / 2))
    tent = tent - bh_pocket

    bh_cable = (cq.Workplane("XY")
                .center(0, half / 2)
                .circle(BH_CABLE_D / 2)
                .extrude(rise + 5)
                .translate((0, 0, z_base - 1)))
    tent = tent - bh_cable

    return tent


# =============================================================================
# Generate + export
# =============================================================================

def export(shape, name):
    path = os.path.join(OUT, f"{name}.stl")
    cq.exporters.export(shape, path)
    print(f"  -> {name}.stl")
    return shape


if __name__ == "__main__":
    print("Generating bayonet stack...")

    print("  building foot...")
    foot = export(make_foot(), "foot")

    print("  building tier_solid...")
    t_solid = export(make_tier("solid"), "tier_solid")

    print("  building tier_louvred...")
    t_louvred = export(make_tier("louvred"), "tier_louvred")

    print("  building tier_esp32...")
    t_esp32 = export(make_tier("esp32"), "tier_esp32")

    print("  building cap...")
    cap = export(make_cap(), "cap")

    # Assembly preview: foot + esp32 tier + louvred tier + cap
    print("  building assembly preview...")
    asm = cq.Assembly()
    pitch = TIER_INNER_H + NECK_H
    z = 0
    asm.add(foot,     loc=cq.Location(cq.Vector(0, 0, z)),                      name="foot")
    z += FOOT_T + NECK_H
    asm.add(t_esp32,  loc=cq.Location(cq.Vector(0, 0, z)),                      name="tier_esp32")
    z += pitch
    asm.add(t_louvred,loc=cq.Location(cq.Vector(0, 0, z)),                      name="tier_louvred")
    z += pitch
    asm.add(cap,      loc=cq.Location(cq.Vector(0, 0, z)),                      name="cap")

    asm.save(os.path.join(OUT, "assembly_preview.step"))
    print("  -> assembly_preview.step")

    rise = (OD / 2 + ROOF_OVERHANG) * math.tan(math.radians(ROOF_PITCH_DEG))
    total_h = z + CAP_BODY_H + rise
    print(f"\nDone. Stack (foot + 2 tiers + cap): ~{total_h:.0f} mm tall, "
          f"{OD} mm OD, {FOOT_OD} mm foot.")
