// ==========================================
// ESP32 38-Pin Type-C (ESP-WROOM-32) + SN65HVD230 CAN Enclosure
// Plain Open Interior (No Rails / No Ribs / No LED Holes)
// Single Beveled Stadium USB-C Port + Side & Top Air Vents
// Internal: 88.8 x 27.8 x 14.0mm
// External: 92.0 x 31.0 x 16.8mm
// ==========================================

/* [Outer Dimensions & Walls] */
outer_l  = 92.0;      // Total outer length (X) - fits ~54mm 38-pin ESP32 + ~28mm CAN + bend
outer_w  = 31.0;      // Total outer width (Y) - fits 25.4mm wide 38-pin board + clearance
inner_h  = 14.0;      // Internal height clearance (Z)
wall     = 1.6;       // Wall thickness
floor_t  = 1.4;       // Bottom floor thickness
lid_t    = 1.4;       // Top lid thickness
corner_r = 2.2;       // Outer corner fillet radius

/* [Printer Tolerance] */
tol      = 0.25;      // Clearance gap for snap-fit lip

/* [Professional USB-C Port & Bezel Dimensions] */
usbc_w      = 9.2;    // Inner metal tongue cutout width
usbc_h      = 3.4;    // Inner metal tongue cutout height
usbc_z      = 5.0;    // Raised 5.0mm above inner floor (for 1mm carrier PCB + standoffs)

bezel_w     = 12.4;   // Outer cable overmold recess width
bezel_h     = 6.4;    // Outer cable overmold recess height
bezel_depth = 0.6;    // Depth of outer stepped bezel into front wall

can_hole_d  = 4.5;    // Diameter of CAN-H / CAN-L wire exit hole

/* [Side Air Vents] */
vent_count   = 12;    // Number of vertical vent slots per side
vent_w       = 1.4;   // Width of each slot
vent_h       = 6.2;   // Height of each slot
vent_spacing = 5.4;   // Center-to-center distance between slots

/* [Top Lid Air Vents (Full Span, No LED Holes)] */
top_vent_count   = 12;    // Number of top lid slots
top_vent_w       = 1.4;   // Width of each top slot
top_vent_len     = 15.0;  // Length of each top slot along Y
top_vent_spacing = 5.4;   // Center-to-center spacing along X

$fn = 48;

// Derived dimensions
outer_h = inner_h + floor_t + lid_t; // 16.8mm total outer height
inner_l = outer_l - 2 * wall;        // 88.8mm internal length
inner_w = outer_w - 2 * wall;        // 27.8mm internal width
base_h  = outer_h - lid_t;           // 15.4mm base box height

// ==========================================
// Render Both Parts Side-by-Side
// ==========================================
base_box();

translate([0, outer_w + 6, 0])
    top_lid();

// ==========================================
// Modules
// ==========================================

module rounded_rect(l, w, h, r) {
    hull() {
        for (x = [r, l - r]) {
            for (y = [r, w - r]) {
                translate([x, y, 0])
                    cylinder(h = h, r = r);
            }
        }
    }
}

module stadium_x(w, h, depth) {
    r = h / 2;
    hull() {
        translate([0, -(w / 2 - r), 0])
            rotate([0, 90, 0])
                cylinder(h = depth, r = r);
        translate([0, (w / 2 - r), 0])
            rotate([0, 90, 0])
                cylinder(h = depth, r = r);
    }
}

module side_vent_slot(w, h, depth) {
    r = w / 2;
    hull() {
        translate([0, 0, r])
            rotate([-90, 0, 0])
                cylinder(h = depth, r = r);
        translate([0, 0, h - r])
            rotate([-90, 0, 0])
                cylinder(h = depth, r = r);
    }
}

module top_vent_slot(w, len, depth) {
    r = w / 2;
    hull() {
        translate([0, r, 0])
            cylinder(h = depth, r = r);
        translate([0, len - r, 0])
            cylinder(h = depth, r = r);
    }
}

module base_box() {
    usbc_center_z = floor_t + usbc_z + usbc_h / 2;

    difference() {
        // Outer shell
        rounded_rect(outer_l, outer_w, base_h, corner_r);

        // Plain open inner cavity (88.8 x 27.8 x 14.0mm)
        translate([wall, wall, floor_t])
            rounded_rect(inner_l, inner_w, base_h, max(0.5, corner_r - wall));

        // Step recess on top rim to accept lid lip
        translate([wall - 0.6, wall - 0.6, base_h - 1.2])
            rounded_rect(inner_l + 1.2, inner_w + 1.2, 1.5, max(0.5, corner_r - wall + 0.6));

        // 1. Inner USB-C Stadium Cutout (Centered on Left Wall, raised 5.0mm above inner floor)
        translate([-0.5, outer_w / 2, usbc_center_z])
            stadium_x(usbc_w, usbc_h, wall + 1.0);

        // 2. Outer Stepped USB-C Cable Overmold Bezel
        translate([-0.1, outer_w / 2, usbc_center_z])
            stadium_x(bezel_w, bezel_h, bezel_depth + 0.1);

        // CAN Wire Exit Hole (Right Wall, centered vertically in cavity)
        translate([outer_l - wall - 0.5, outer_w / 2, floor_t + inner_h / 2])
            rotate([0, 90, 0])
                cylinder(h = wall + 1.0, d = can_hole_d);

        // Side Air Vents (Front & Back Walls)
        total_vent_span = (vent_count - 1) * vent_spacing;
        start_x = (outer_l - total_vent_span) / 2;
        vent_z = floor_t + ((base_h - 1.2 - floor_t) - vent_h) / 2;

        for (i = [0 : vent_count - 1]) {
            vx = start_x + i * vent_spacing;
            translate([vx, -0.5, vent_z])
                side_vent_slot(vent_w, vent_h, wall + 1.0);
            translate([vx, outer_w - wall - 0.5, vent_z])
                side_vent_slot(vent_w, vent_h, wall + 1.0);
        }
    }
}

module top_lid() {
    lip_h = 1.1;
    lip_w = 0.55;

    difference() {
        union() {
            // Outer top plate
            rounded_rect(outer_l, outer_w, lid_t, corner_r);

            // Friction-fit inner lip
            translate([wall - lip_w + tol, wall - lip_w + tol, lid_t])
                difference() {
                    rounded_rect(
                        inner_l + 2 * (lip_w - tol),
                        inner_w + 2 * (lip_w - tol),
                        lip_h,
                        max(0.5, corner_r - wall + lip_w - tol)
                    );
                    translate([1.0, 1.0, -0.1])
                        rounded_rect(
                            inner_l + 2 * (lip_w - tol) - 2.0,
                            inner_w + 2 * (lip_w - tol) - 2.0,
                            lip_h + 0.2,
                            0.5
                        );
                }
        }

        // Centered Top Lid Ventilation Slots (12 slots across length)
        top_total_span = (top_vent_count - 1) * top_vent_spacing;
        top_start_x = (outer_l - top_total_span) / 2;
        top_vent_y = (outer_w - top_vent_len) / 2;

        for (i = [0 : top_vent_count - 1]) {
            tx = top_start_x + i * top_vent_spacing;
            translate([tx, top_vent_y, -0.5])
                top_vent_slot(top_vent_w, top_vent_len, lid_t + 1.0);
        }
    }
}
