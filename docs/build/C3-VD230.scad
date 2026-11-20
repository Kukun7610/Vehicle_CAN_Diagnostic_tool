// ==========================================
// ESP32-C3 + SN65HVD230 CAN Enclosure
// Plain Open Interior (No Rails / No Ribs)
// Internal Height: 12.0mm | Outer: 58 x 24 x 14.8mm
// ==========================================

/* [Outer Dimensions & Walls] */
outer_l  = 58.0;      // Total outer length (X)
outer_w  = 24.0;      // Total outer width (Y)
inner_h  = 12.0;      // Exact internal height clearance (Z)
wall     = 1.6;       // Wall thickness
floor_t  = 1.4;       // Bottom floor thickness
lid_t    = 1.4;       // Top lid thickness
corner_r = 2.0;       // Outer corner fillet radius

/* [Printer Tolerance] */
tol      = 0.25;      // Clearance gap for snap-fit lip

/* [Professional USB-C Port & Bezel Dimensions] */
usbc_w      = 9.2;    // Inner metal tongue cutout width
usbc_h      = 3.4;    // Inner metal tongue cutout height
usbc_z      = 5.0;    // Raised 5.0mm above inner floor

bezel_w     = 12.4;   // Outer cable overmold recess width
bezel_h     = 6.4;    // Outer cable overmold recess height
bezel_depth = 0.6;    // Depth of outer stepped bezel into front wall

can_hole_d  = 4.5;    // Diameter of CAN-H / CAN-L wire exit hole

/* [Side Air Vents] */
vent_count   = 8;     // Number of vertical vent slots per side
vent_w       = 1.4;   // Width of each slot
vent_h       = 4.8;   // Height of each slot
vent_spacing = 4.2;   // Center-to-center distance between slots

/* [Top Lid Vents & Diagonal Status LED Holes] */
top_vent_count   = 6;     // Number of top lid slots
top_vent_w       = 1.4;   // Width of each top slot
top_vent_len     = 11.0;  // Length of each top slot along Y
top_vent_spacing = 4.2;   // Center-to-center spacing along X
top_vent_start_x = 27.5;  // X start position (keeps clear of LED holes)

led_hole_d = 2.2;         // Diameter of each LED viewing hole
led_pw_x   = wall + 4.5;  // Red PW LED X position
led_pw_y   = 16.5;        // Red PW LED Y position
led_blue_x = wall + 11.5; // Blue GPIO-8 LED X position
led_blue_y = 7.5;         // Blue GPIO-8 LED Y position

$fn = 48;

// Derived dimensions
outer_h = inner_h + floor_t + lid_t; // 14.8mm total outer height
inner_l = outer_l - 2 * wall;        // 54.8mm internal length
inner_w = outer_w - 2 * wall;        // 20.8mm internal width
base_h  = outer_h - lid_t;           // 13.4mm base box height

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

        // Plain open inner cavity (54.8 x 20.8 x 12.0mm)
        translate([wall, wall, floor_t])
            rounded_rect(inner_l, inner_w, base_h, max(0.5, corner_r - wall));

        // Step recess on top rim to accept lid lip
        translate([wall - 0.6, wall - 0.6, base_h - 1.2])
            rounded_rect(inner_l + 1.2, inner_w + 1.2, 1.5, max(0.5, corner_r - wall + 0.6));

        // 1. Inner USB-C Stadium Cutout (raised 5.0mm above inner floor)
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

        // Top Lid Ventilation Slots
        top_vent_y = (outer_w - top_vent_len) / 2;
        for (i = [0 : top_vent_count - 1]) {
            tx = top_vent_start_x + i * top_vent_spacing;
            translate([tx, top_vent_y, -0.5])
                top_vent_slot(top_vent_w, top_vent_len, lid_t + 1.0);
        }

        // Red PW LED Hole (mirrored Y for lid flip)
        translate([led_pw_x, outer_w - led_pw_y, -0.5])
            cylinder(h = lid_t + 1.0, d = led_hole_d);

        // Blue GPIO-8 LED Hole (mirrored Y for lid flip)
        translate([led_blue_x, outer_w - led_blue_y, -0.5])
            cylinder(h = lid_t + 1.0, d = led_hole_d);
    }
}
