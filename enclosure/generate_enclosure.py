"""
Weather station enclosure -- bayonet-mount stack
=================================================

Stack:    foot  ->  tier_esp32  ->  tier_*  ->  ...  ->  cap

Foot:     just a wide thick plate with a bayonet neck on top. That is all.
          The ESP32 does NOT live in the foot -- it lives in a tier above.

Tier:     cylindrical donut. Inner column (cable shaft) + outer wall +
          tent-shaped floor (two slopes meeting at a ridge along the X axis,
          like an inverted roof) with two drain holes at the y-axis low points.
          Bayonet skirt below (sleeves over the lower piece's neck, with
          BLIND L-shaped pockets carved into its inner face -- outer face is
          smooth, flush with body OD), bayonet neck above (steps inward by
          NECK_INSET, with 4 outward-projecting cammed lugs at 90deg spacing).
          The neck is connected to the body via an inward-tapered shoulder
          (so it isn't floating).
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
# Mid-size weather station: ~165mm tall stack, 90mm body OD, 165mm-wide foot.
# (1.5x of the original 60mm prototype.) Walls and the bayonet are sized so
# the slot pocket leaves ~46% of wall thickness as solid outer material -- a
# big improvement over the original 33% that was cracking.

OD              = 90      # outer diameter of every body section
COL_ID          = 18      # cable column hole diameter (cables route through here)
WALL            = 3       # default wall thickness
TIER_INNER_H    = 48      # body height per tier
NECK_H          = 9       # bayonet neck/skirt overlap height

# Foot (the stand)
FOOT_OD         = 165     # plate diameter (broad for stability)
FOOT_T          = 8       # plate thickness

# Bayonet -- replicates the geometry of the user's reference bayonetbox.3mf:
# - 4 OUTWARD-projecting lugs on the LOWER piece's neck (90deg spacing)
# - Lugs have chamfered top+bottom outer edges -> cam ramps that wedge into
#   the lock arc, pre-loading the joint as you twist
# - The UPPER piece's skirt OUTER FACE IS SMOOTH AND FLUSH with body OD
#   (no visible slots from outside, just like the reference)
# - L-shaped slots are BLIND POCKETS cut only into the skirt's INNER face.
#   They do NOT cut through to the outside.
LUG_COUNT       = 4       # 4 lugs at 0/90/180/270 (matches reference)
LUG_W_DEG       = 17      # lug arc width, in DEGREES (does not scale with size)
LUG_H           = 4       # lug height (z dim, INCLUDING ramps)
LUG_PROJECT     = 1.2     # radial protrusion of lug from neck outer face.
LUG_CHAMFER     = 0.75    # height of cam ramps at lug top and bottom; the lug
                          # has a "flat peak" in the middle and tapers to the
                          # neck face at top and bottom over LUG_CHAMFER mm.
LUG_Z_CENTER_FRAC = 0.5   # lug centered in the neck height (fraction 0..1)
TWIST_LOCK_DEG  = 25      # rotation needed to fully engage lock (degrees)
SKIRT_CLEAR     = 0.2     # radial clearance: skirt_inner - neck_outer (TIGHT,
                          # FDM-tolerance value, does NOT scale with size)
SLOT_DEPTH      = 1.5     # how deep the BLIND L-pocket cuts into skirt wall.
                          # Skirt wall is 2.8mm; this leaves ~1.3mm of outer
                          # material (46% of wall) -- 2x the original 0.6mm
                          # strip that was cracking, close to the reference's
                          # 51% ratio. LUG_PROJECT must be < SLOT_DEPTH.
SLOT_LOCK_W_DEG = 30      # how far the lock arc extends beyond the vert channel
SLOT_LOCK_H_CLEAR = 0.3   # vertical clearance for the lug in the lock arc
NECK_INSET      = 3       # neck OD = body OD - 2*NECK_INSET (3mm step at this
                          # 1.5x scale)

# Floor (TENT-shape): two slopes meeting at a ridge along the X axis (just
# like the cap's roof but inverted INSIDE the tier), so any water inside flows
# down the slopes to TWO drain holes (one on each side, at the low points).
# Pitch matches the roof (15 deg) for visual consistency.
FLOOR_PITCH_DEG = 15
FLOOR_T_RIDGE   = 14      # floor thickness at the ridge (high point, along x axis)
FLOOR_T_EDGE    = 1.8     # floor thickness at the y-axis (low points, where drains are)
DRAIN_D         = 6       # diameter of each drain hole through the outer wall

# Body-to-neck shoulder: tapers inward from r=BODY_INNER_R at z=height-SHOULDER_H
# to r=NECK_INNER_R at z=height, so the neck has body material to mate with
# (otherwise the neck would float above the body wall, unconnected).
SHOULDER_H      = 5

# Cable slots (through inner column wall) - run along x axis (perpendicular to
# the y-axis drains and along the floor's ridge)
CABLE_SLOT_W    = 11
CABLE_SLOT_H    = 9
CABLE_SLOT_COUNT = 2

# ESP32 tier (USB cutout sized for a USB-C plug + cable strain relief).
# USB_W/USB_H are physical-connector dimensions and DO NOT scale with the body.
USB_W           = 14
USB_H           = 8
USB_Z_OFFSET    = 20      # USB centre this far above the inner floor surface

# Louvred wall (REAL Stevenson-screen): 4 vertical posts hold a stack of tilted
# annular slats. Slats DON'T touch each other -- there are horizontal gaps on
# BOTH the inner and outer faces, so air can actually pass through. The slats
# tilt outward-and-down so rain runs off the outer edge and falls clear.
SLAT_THICKNESS  = 2.4     # vertical (z) thickness of each slat at the inner face
SLAT_TILT       = 4       # outer edge of slat sits this far below the inner edge
POST_COUNT      = 4       # vertical posts that hold the slats in place
POST_W_DEG      = 12      # tangential width of each post (in degrees, no scale)
N_SLATS_TARGET  = 6       # target number of slats (pitch is auto-computed)
LOUVRE_MARGIN_BOT = 6     # height of the bottom plinth (at the inner face)
LOUVRE_MARGIN_TOP = 6     # height of the top shoulder (at the inner face)

# Cap. Body now contains the same sloped-floor + drain mechanism as the tiers
# (so any water that gets in via the sensor pockets drains out). CAP_BODY_H
# must fit FLOOR_T_RIDGE + the cable slot above it + a small margin.
# ROOF_OVERHANG is intentionally large (15mm) so the roof slope is wide enough
# for the 55x41mm rain sensor PCB to fit (90mm OD body alone would be too
# narrow for a sensor that big).
CAP_BODY_H      = 26
ROOF_PITCH_DEG  = 15
ROOF_OVERHANG   = 15

# ----- Sensor pockets in the cap roof -----
# These match physical sensor PCB dimensions and do NOT scale with the body.
#
# Rain sensor (MH-RD style): rectangular PCB with a small 2-pin connector
# hanging off one long edge. The connector body extends past the PCB edge
# (RAIN_PIN_OUT) and pins protrude downward (RAIN_PIN_DOWN) below the PCB.
# Recess geometry:
#   - main pocket (where the PCB lays flat): RAIN_X+2*MARGIN x RAIN_Y+2*MARGIN,
#     depth = RAIN_T + MARGIN
#   - pin sub-pocket (a through-hole adjacent to one long edge of the main
#     pocket): RAIN_PIN_W x (RAIN_PIN_OUT+MARGIN), goes ALL the way through the
#     roof so cables can come up from inside the cap to the connector pins.
RAIN_X          = 55      # PCB long dim (along ridge / X axis)
RAIN_Y          = 41      # PCB short dim (down the slope / Y axis)
RAIN_T          = 1       # PCB thickness
RAIN_MARGIN     = 1       # clearance per side around the PCB
RAIN_PIN_OUT    = 5       # connector body extends past PCB edge by this much
RAIN_PIN_DOWN   = 5       # pins protrude this far below the PCB
RAIN_PIN_W      = 30      # width of the pin sub-pocket along the PCB edge

# BMP280 / BME280: small 16x12 mm breakout PCB. Cables solder up from below to
# pads on the PCB underside, so we need a through-hole under the sensor.
# - sensor recess: BH_X+2*MARGIN x BH_Y+2*MARGIN, depth = BH_T + MARGIN
# - cable hole (through roof, centred under recess): 1mm smaller than the
#   recess in each dim (so the PCB rests on a 0.5mm-wide lip all around).
BH_X            = 16      # PCB long dim
BH_Y            = 12      # PCB short dim
BH_T            = 1       # PCB thickness
BH_MARGIN       = 1       # clearance per side around the PCB
BH_HOLE_INSET   = 0.5     # cable hole edge is this far inside the recess edge
                          # (per side; total inset = 1mm in each dim)

# =============================================================================
# Derived
# =============================================================================
COL_OUTER_R     = COL_ID / 2 + WALL          # outer radius of inner column wall
BODY_INNER_R    = OD / 2 - WALL              # inner face of body wall

# Bayonet sizing -- matches the bayonetbox.3mf reference architecture:
# Body OD steps inward by NECK_INSET to form the neck (smaller r). The next
# tier's skirt sleeves over the neck, with skirt OD == body OD (flush, smooth
# outer surface). Lugs project outward from the neck and engage L-pockets that
# are cut into the skirt's inner face but DO NOT cut through the outer face.
NECK_R          = OD / 2 - NECK_INSET        # outer radius of neck (=42 at OD=90)
NECK_INNER_R    = NECK_R - WALL              # inner radius of neck (=39)
SKIRT_R_OUT     = OD / 2                     # outer radius of skirt (=45, flush w/body)
SKIRT_R_IN      = NECK_R + SKIRT_CLEAR       # inner radius of skirt (=42.2)
                                             # -> skirt wall = 2.8mm
LUG_OUT_R       = NECK_R + LUG_PROJECT       # tip of lug (=43.2)
SLOT_FLOOR_R    = SKIRT_R_IN + SLOT_DEPTH    # bottom of L-pocket inside wall (=43.7)
                                             # -> ~1.3mm of skirt wall remains outside
                                             #    the slot (46% of wall thickness;
                                             #    2x the original 0.6mm strip that
                                             #    was cracking).


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
# Bayonet -- replicates the bayonetbox.3mf reference geometry
# =============================================================================
# Mechanism (verified by analysing bayonetbox.3mf):
#   - LOWER piece has a NECK that steps inward from the body OD by NECK_INSET.
#     4 LUGS project OUTWARD from the neck at theta=0/90/180/270. Lugs have
#     chamfered top+bottom outer edges (cam ramps).
#   - UPPER piece has a SKIRT that hangs below its body. Skirt OD == body OD
#     (FLUSH, smooth outer surface -- no visible slots from outside). Skirt
#     INNER sleeves over the neck with a tight 0.2mm radial clearance.
#   - L-shaped POCKETS are carved into the skirt's INNER face only -- they do
#     NOT cut through to the outer face. The outer face stays smooth, exactly
#     like the reference.
#   - Drop upper onto lower: skirt sleeves over neck, lugs enter the open
#     vertical channel of each L-pocket from BELOW (channel opens at skirt
#     bottom edge). When the skirt bottom rests on the body shelf at the top
#     of the lower piece's body, the lugs are at the top of the vertical
#     channel.
#   - Twist the upper piece TWIST_LOCK_DEG and the lugs slide sideways into
#     the horizontal lock arc at the TOP of the L. The skirt material above
#     the lock arc forms a "ceiling" that the lug's chamfered top-edge cam
#     ramps wedge against, pre-loading the joint tight. Skirt material BELOW
#     the lock arc prevents the lug dropping back down.
#   - Twist back and lift to release.

def make_chamfered_lug(theta_center, z_bot):
    """Outward-projecting cammed lug at theta_center, occupying z=z_bot..z_bot+LUG_H.
    Built as 3 stacked annular sectors approximating a hexagonal r-z profile:
        top:   thin ramp tapering from full projection back to neck face
        mid:   full LUG_PROJECT, flat peak (where the lug grips the lock arc)
        bot:   thin ramp tapering from neck face up to full projection
    The bottom ramp guides the lug smoothly into the slot as the upper piece
    descends; the top ramp wedges against the lock arc ceiling as you twist,
    pre-loading the joint.
    """
    half_w = LUG_W_DEG / 2
    r_taper = NECK_R + LUG_PROJECT * 0.5    # half-projection at the ramp tips
    r_full = NECK_R + LUG_PROJECT
    bot = annular_sector(
        r_in=NECK_R - 0.1, r_out=r_taper,
        theta_a=theta_center - half_w, theta_b=theta_center + half_w,
        h=LUG_CHAMFER, z=z_bot,
    )
    mid = annular_sector(
        r_in=NECK_R - 0.1, r_out=r_full,
        theta_a=theta_center - half_w, theta_b=theta_center + half_w,
        h=LUG_H - 2 * LUG_CHAMFER, z=z_bot + LUG_CHAMFER,
    )
    top = annular_sector(
        r_in=NECK_R - 0.1, r_out=r_taper,
        theta_a=theta_center - half_w, theta_b=theta_center + half_w,
        h=LUG_CHAMFER, z=z_bot + LUG_H - LUG_CHAMFER,
    )
    return bot.union(mid).union(top)


def make_neck(z_base):
    """Lower bayonet half (male). Hollow ring with 4 outward-projecting cammed
    lugs at theta=0/90/180/270, centred in the neck height."""
    neck = hollow_cylinder(NECK_R, NECK_INNER_R, NECK_H, z_base)
    # Centre the lug Z within the neck so the upper skirt has solid material
    # both above the lug (forms the lock-arc ceiling) and below the lug (forms
    # the lock-arc floor that traps the lug from sliding back down).
    lug_z_bot = z_base + (NECK_H - LUG_H) * LUG_Z_CENTER_FRAC
    for i in range(LUG_COUNT):
        theta = 360 * i / LUG_COUNT
        lug = make_chamfered_lug(theta, lug_z_bot)
        neck = neck.union(lug)
    return neck


def make_skirt(z_top):
    """Upper bayonet half (female). Hollow ring hanging below the body. Outer
    face is smooth and flush with body OD (= reference's "perfect seam").
    L-shaped POCKETS are carved into the inner face only, at theta=0/90/180/270.
    """
    skirt = hollow_cylinder(SKIRT_R_OUT, SKIRT_R_IN, NECK_H, z_top - NECK_H)
    for i in range(LUG_COUNT):
        theta = 360 * i / LUG_COUNT
        skirt = cut_skirt_l_pocket(skirt, theta, z_top)
    return skirt


def cut_skirt_l_pocket(skirt, theta_slot, z_top):
    """Cut a BLIND L-shaped pocket into the inner face of the skirt at theta_slot.

    The pocket ONLY removes material from the inner face up to SLOT_DEPTH deep.
    It does NOT cut through to the outer face -- the outer face stays smooth
    (this is what makes the bayonetbox look "perfect" from outside).

    Vertical channel: opens at bottom edge of skirt, runs full skirt height,
                      LUG_W_DEG + 4 deg wide. The lug enters here from below
                      as the upper piece descends.
    Lock arc:         at the same Z range as the LUG (when seated), extending
                      sideways (in +theta direction) by SLOT_LOCK_W_DEG. After
                      the upper is seated, a twist of TWIST_LOCK_DEG slides
                      the lug into here. Skirt material BELOW the lock arc
                      traps the lug from sliding back down. Skirt material
                      ABOVE the lock arc forms the "ceiling" the lug's top cam
                      ramp wedges against, pre-loading the joint.
    """
    z_bot = z_top - NECK_H
    vert_arc_w = LUG_W_DEG + 4                          # 2 deg clearance each side
    # Lock arc Z range: centred on where the lug sits when the upper is seated.
    lug_z_in_skirt = (NECK_H - LUG_H) * LUG_Z_CENTER_FRAC  # lug bottom rel. to skirt bot
    lock_h = LUG_H + SLOT_LOCK_H_CLEAR
    lock_z_bot = z_bot + lug_z_in_skirt - SLOT_LOCK_H_CLEAR / 2

    # Vertical channel (BLIND pocket: cuts radially only, NOT through outer face).
    # Opens at skirt bottom (extends 0.3mm below for clean bottom-edge break).
    vert_cut = annular_sector(
        r_in=SKIRT_R_IN - 0.05,             # slight overlap into wall
        r_out=SLOT_FLOOR_R,                 # blind: stops before reaching outer face
        theta_a=theta_slot - vert_arc_w / 2,
        theta_b=theta_slot + vert_arc_w / 2,
        h=NECK_H + 0.6,
        z=z_bot - 0.3,
    )
    # Lock arc (BLIND pocket at the lug's Z, extends sideways from end of vert channel)
    lock_cut = annular_sector(
        r_in=SKIRT_R_IN - 0.05,
        r_out=SLOT_FLOOR_R,
        theta_a=theta_slot + vert_arc_w / 2 - 1,    # 1 deg overlap with vert channel
        theta_b=theta_slot + vert_arc_w / 2 + SLOT_LOCK_W_DEG,
        h=lock_h,
        z=lock_z_bot,
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
    """Solid wall + USB-C cutout + rain hood awning above the cutout.

    Rain hood is a triangular wedge that sits just above the USB hole and
    projects outward in +X. Its top surface slopes DOWN going outward, so any
    water running down the wall above is diverted forward and drops off the
    front edge clear of the USB hole. The hood is wider and longer than the
    USB hole so rain can't sneak in around the sides. The underside is at
    ~45 deg overhang so it prints without supports.
    """
    wall = make_outer_wall_solid(height, z_base)
    usb_z_bot = z_base + FLOOR_T_RIDGE + USB_Z_OFFSET - USB_H / 2
    usb_z_top = usb_z_bot + USB_H
    usb = (cq.Workplane("XY")
           .box(OD, USB_W, USB_H, centered=(False, True, False))
           .translate((0, 0, usb_z_bot)))

    hood_proj      = 10             # how far the hood projects past the wall
    hood_back_h    = 12             # back-edge height above USB top
    hood_tip_h     = 8              # tip height above USB top (45 deg under-side)
    hood_width     = USB_W + 14     # wider than USB hole on each side
    hood_lip       = 2              # how far the front lip drops below USB top
    x_back         = OD / 2 - 1     # 1mm overlap into wall for solid union
    x_front        = OD / 2 + hood_proj

    # Build the hood as an explicit triangular prism in 3D space.
    # The triangle lives in the XZ plane (y=0), and we extrude in +Y by hood_width.
    # Using cq.Wire/Face/Solid directly avoids the workplane orientation
    # ambiguity that bit us before.
    tri_pts = [
        cq.Vector(x_back,  0, usb_z_top - hood_lip),     # back: starts 1.5mm BELOW USB top
        cq.Vector(x_front, 0, usb_z_top + hood_tip_h),   # front tip
        cq.Vector(x_back,  0, usb_z_top + hood_back_h),  # back-top
        cq.Vector(x_back,  0, usb_z_top - hood_lip),     # close
    ]
    wire = cq.Wire.makePolygon(tri_pts)
    face = cq.Face.makeFromWires(wire)
    hood_solid = cq.Solid.extrudeLinear(face, cq.Vector(0, hood_width, 0))
    hood = (cq.Workplane("XY")
            .add(hood_solid)
            .translate((0, -hood_width / 2, 0)))

    return (wall - usb).union(hood)


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
    """TWO small circular drain HOLES through the outer wall, one at +y and
    one at -y (where the tent floor's slopes reach the wall, so pooled water
    flows down the slopes and exits through these holes).

    Each is a SHORT cylinder cutting through ONLY the outer wall thickness --
    NOT a long tunnel through the whole floor (which would just hold water
    horizontally). The hole's bottom sits just above the floor edge so pooled
    water always finds it.

    Outer wall spans y = +/-(OD/2-WALL) to +/-(OD/2). Each drain cylinder is
    placed so its axis crosses the wall, with 1mm of overlap on each side for
    a clean Boolean cut.
    """
    drain_z = z_base + FLOOR_T_EDGE + DRAIN_D / 2 + 0.2
    r = DRAIN_D / 2
    cyl_len = WALL + 2     # length along Y: wall thickness + 1mm overlap each side
    # +Y drain: starts 1mm INSIDE the inner face, extends across the wall to 1mm OUTSIDE
    cyl_p = cq.Solid.makeCylinder(
        r, cyl_len,
        pnt=cq.Vector(0, BODY_INNER_R - 1, drain_z),    # y = inner_r - 1
        dir=cq.Vector(0, 1, 0),                         # extends +y by cyl_len
    )
    # -Y drain: starts 1mm OUTSIDE the outer face, extends INWARD across the wall
    cyl_n = cq.Solid.makeCylinder(
        r, cyl_len,
        pnt=cq.Vector(0, -OD / 2 - 1, drain_z),         # y = -outer_r - 1
        dir=cq.Vector(0, 1, 0),                         # extends +y by cyl_len
    )
    return cq.Workplane("XY").add(cyl_p).add(cyl_n)


def make_cable_slot_cutter(z_base, body_h, theta, z_center=None):
    """Rectangular cutter through the inner column wall.
    If z_center is given, the slot is centred on that absolute Z; otherwise
    it sits at the body's mid-height. The cap uses an explicit z_center to
    place the slot above its sloped floor."""
    if z_center is None:
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
    """Top piece. SAME architecture as a tier (sloped tent floor + drain holes
    out the sides) so any rain that gets in via the sensor pockets drains out
    instead of pooling. Then a tent ROOF on top with sensor pockets."""
    body_z = 0
    body_h = CAP_BODY_H

    outer = hollow_cylinder(OD / 2, BODY_INNER_R, body_h, body_z)
    inner_col = make_inner_column_wall(body_h, body_z)
    floor = make_floor(body_z)

    body = outer.union(inner_col).union(floor)
    body = body - make_drain_cutter(body_z)

    # Cable slots ABOVE the floor ridge, so cables exit the column into the
    # cap's interior cavity (between the floor and the roof) where they reach
    # the sensors mounted in the roof.
    slot_z_center = body_z + FLOOR_T_RIDGE + CABLE_SLOT_H / 2 + 1
    for i in range(CABLE_SLOT_COUNT):
        theta = 360 * i / CABLE_SLOT_COUNT + 90
        body = body - make_cable_slot_cutter(body_z, body_h, theta,
                                              z_center=slot_z_center)

    skirt = make_skirt(body_z)
    roof = make_tent_roof(body_z + body_h)

    return body.union(skirt).union(roof)


def _slope_pocket(local_pts_2d, depth, slope_sign, z_base, half, rise):
    """Build a pocket cutter on one of the cap roof slopes.

    local_pts_2d: list of (x, y) points (in the slope's local frame, before
                  rotation) defining a polygon to extrude.
    depth:        how far the cutter extends BELOW the roof surface (mm); for a
                  through-hole, pass a value larger than the roof rise so the
                  cutter goes all the way through.
    slope_sign:   +1 for the +Y slope (BMP280 side), -1 for the -Y slope (rain
                  sensor side).
    Returns the cutter solid in world coordinates, ready to subtract from the
    tent.
    """
    extra = 2  # extend slightly above the roof surface so the boolean is clean
    poly = (cq.Workplane("XY")
            .polyline(local_pts_2d).close()
            .extrude(depth + extra)
            .translate((0, 0, -depth)))
    # Rotate the cutter to match the slope's pitch. Rotating around X by
    # +ROOF_PITCH_DEG tilts the +Z direction toward -Y (so the cutter sits
    # perpendicular to the +Y slope). For the -Y slope we negate.
    pitch = ROOF_PITCH_DEG * (-slope_sign)
    poly = poly.rotate((0, 0, 0), (1, 0, 0), pitch)
    # Slide the cutter to the centre of the slope (mid-way down it).
    poly = poly.translate((0, slope_sign * half / 2, z_base + rise / 2))
    return poly


def make_tent_roof(z_base):
    """Triangular-prism tent, trimmed to a circular plan view.
    Ridge along the X axis. Two slopes pitched ROOF_PITCH_DEG.
    -Y slope (front) holds the rain sensor; +Y slope (back) holds the BMP280.
    """
    half = OD / 2 + ROOF_OVERHANG
    rise = half * math.tan(math.radians(ROOF_PITCH_DEG))

    pts = [(-half, 0), (half, 0), (0, rise)]
    tent = (cq.Workplane("YZ")
            .polyline(pts).close()
            .extrude(2 * half)
            .translate((-half, 0, z_base)))

    trim = (cq.Workplane("XY")
            .circle(half)
            .extrude(rise + 1)
            .translate((0, 0, z_base)))
    tent = tent.intersect(trim)

    # ---------------- Rain sensor on -Y slope ----------------
    # Main pocket: PCB-sized recess (1mm board + 1mm air-gap clearance above).
    rain_recess_w = RAIN_X + 2 * RAIN_MARGIN          # 57mm wide along the ridge
    rain_recess_h = RAIN_Y + 2 * RAIN_MARGIN          # 43mm tall along the slope
    rain_recess_d = RAIN_T + RAIN_MARGIN              # 2mm deep
    rain_main_pts = [
        (-rain_recess_w / 2, -rain_recess_h / 2),
        ( rain_recess_w / 2, -rain_recess_h / 2),
        ( rain_recess_w / 2,  rain_recess_h / 2),
        (-rain_recess_w / 2,  rain_recess_h / 2),
    ]
    rain_main = _slope_pocket(rain_main_pts, rain_recess_d, -1, z_base, half, rise)
    tent = tent - rain_main

    # Pin sub-pocket: a through-hole adjacent to the upper edge of the main
    # pocket (toward the ridge). Cables come up from inside the cap through
    # this hole to connect to the rain-sensor pins.
    pin_y_lo = rain_recess_h / 2                          # upper edge of main pocket
    pin_y_hi = pin_y_lo + (RAIN_PIN_OUT + RAIN_MARGIN)    # 6mm beyond it
    pin_pts = [
        (-RAIN_PIN_W / 2, pin_y_lo),
        ( RAIN_PIN_W / 2, pin_y_lo),
        ( RAIN_PIN_W / 2, pin_y_hi),
        (-RAIN_PIN_W / 2, pin_y_hi),
    ]
    # depth: through the roof. rise+CAP_BODY_H is more than enough.
    rain_pin = _slope_pocket(pin_pts, rise + CAP_BODY_H + 5,
                              -1, z_base, half, rise)
    tent = tent - rain_pin

    # ---------------- BMP280 on +Y slope ----------------
    bh_recess_w = BH_X + 2 * BH_MARGIN                   # 18mm
    bh_recess_h = BH_Y + 2 * BH_MARGIN                   # 14mm
    bh_recess_d = BH_T + BH_MARGIN                       # 2mm deep
    bh_main_pts = [
        (-bh_recess_w / 2, -bh_recess_h / 2),
        ( bh_recess_w / 2, -bh_recess_h / 2),
        ( bh_recess_w / 2,  bh_recess_h / 2),
        (-bh_recess_w / 2,  bh_recess_h / 2),
    ]
    bh_main = _slope_pocket(bh_main_pts, bh_recess_d, +1, z_base, half, rise)
    tent = tent - bh_main

    # Cable hole: BH_HOLE_INSET inside each edge of the recess, going right
    # through the roof so cables can come up from below to the BMP280 pads.
    bh_hole_w = bh_recess_w - 2 * BH_HOLE_INSET          # 17mm
    bh_hole_h = bh_recess_h - 2 * BH_HOLE_INSET          # 13mm
    bh_hole_pts = [
        (-bh_hole_w / 2, -bh_hole_h / 2),
        ( bh_hole_w / 2, -bh_hole_h / 2),
        ( bh_hole_w / 2,  bh_hole_h / 2),
        (-bh_hole_w / 2,  bh_hole_h / 2),
    ]
    bh_hole = _slope_pocket(bh_hole_pts, rise + CAP_BODY_H + 5,
                             +1, z_base, half, rise)
    tent = tent - bh_hole

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
    print(f"\nDone. Stack (foot + 2 tiers + cap): "
          f"~{total_h:.0f}mm tall x {OD}mm OD x {FOOT_OD}mm foot")
