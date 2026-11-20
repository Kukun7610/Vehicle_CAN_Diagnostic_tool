// ==========================================
// YD-ESP32-S3 N16R8 + SN65HVD230 CAN Enclosure
// Plain Open Interior (No Rails / No Ribs)
// Dual Beveled Stadium USB-C Ports + Vents
// Internal: 100.8 x 28.8 x 14.0mm
// External: 104.0 x 32.0 x 16.8mm
// ==========================================

/* [Outer Dimensions & Walls] */
outer_l  = 104.0;     // Total outer length (X) - fits 69mm S3 + 28mm CAN + wire bend
outer_w  = 32.0;      // Total outer width (Y) - fits 25.4mm wide S3 board + clearance
inner_h  = 14.0;      // Internal height clearance (Z)
wall     = 1.6;       // Wall thickness
floor_t  = 1.4;       // Bottom floor thickness
lid_t    = 1.4;       // Top lid thickness
corner_r = 2.2;       // Outer corner fillet radius

/* [Printer Tolerance] */
tol      = 0.25;      // Clearance gap for snap-fit lip

/* [Dual Stadium USB-C Ports & Bezels] */
usbc_w        = 9.2;  // Inner metal tongue cutout width
usbc_h        = 3.4;  // Inner metal tongue cutout height
usbc_z        = 5.0;  // Raised 5.0mm above inner floor (for 1mm carrier PCB + standoffs)
usbc_pitch    = 12.7; // Center-to-center spacing between YD-ESP32-S3 USB & UART ports

bezel_w       = 11.6; // Outer cable overmold recess width (sized to leave clean center bridge)
bezel_h       = 6.4;  // Outer cable overmold recess height
bezel_depth   = 0.6;  // Depth of outer stepped bezel into front wall

can_hole_d    = 4.5;  // Diameter of CAN-H / CAN-L wire exit hole

/* [Side Air Vents] */
vent_count    = 14;   // Number of vertical vent slots per side
vent_w        = 1.4;  // Width of each slot
vent_h        = 6.2;  // Height of each slot
vent_spacing  = 5.4;  // Center-to-center distance between slots

/* [Top Lid Vents & Dual Status LED Holes] */
top_vent_count   = 10;    // Number of top lid slots (over rear S3 / CAN zone)
top_vent_w       = 1.4;   // Width of each top slot
top_vent_len     = 16.0;  // Length of each top slot along Y
top_vent_spacing = 5.4;   // Center-to-center spacing along X
top_vent_start_x = 42.0;  // X start position (keeps clear of front LEDs & RF antenna)

led_hole_d   = 2.4;       // Diameter of each LED viewing hole
led_pwr_x    = wall + 12.0; // Red PWR LED X distance from front USB-C edge
led_pwr_y    = 11.0;        // Red PWR LED Y position
led_rgb_x    = wall + 14.5; // Addressable RGB LED (GPIO 48) X distance
led_rgb_y    = 21.0;        // Addressable RGB LED Y position

$fn = 48;

// Derived dimensions
outer_h = inner_h + floor_t + lid_t; // 16.8mm total outer height
inner_l = outer_l - 2 * wall;        // 100.8mm internal length
inner_w = outer_w - 2 * wall;        // 28.8mm internal width
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
    port1_y = (outer_w / 2) - (usbc_pitch / 2); // Native USB port
    port2_y = (outer_w / 2) + (usbc_pitch / 2); // UART serial port

    difference() {
        // Outer shell
        rounded_rect(outer_l, outer_w, base_h, corner_r);

        // Plain open inner cavity (100.8 x 28.8 x 14.0mm)
        translate([wall, wall, floor_t])
            rounded_rect(inner_l, inner_w, base_h, max(0.5, corner_r - wall));

        // Step recess on top rim to accept lid lip
        translate([wall - 0.6, wall - 0.6, base_h - 1.2])
            rounded_rect(inner_l + 1.2, inner_w + 1.2, 1.5, max(0.5, corner_r - wall + 0.6));

        // Dual USB-C Stadium Cutouts & Outer Bezels (Left Wall, X = 0)
        for (py = [port1_y, port2_y]) {
            translate([-0.5, py, usbc_center_z])
                stadium_x(usbc_w, usbc_h, wall + 1.0);
            translate([-0.1, py, usbc_center_z])
                stadium_x(bezel_w, bezel_h, bezel_depth + 0.1);
        }

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

        // Red PWR LED Hole (mirrored Y for lid flip)
        translate([led_pwr_x, outer_w - led_pwr_y, -0.5])
            cylinder(h = lid_t + 1.0, d = led_hole_d);

        // RGB WS2812 LED Hole (mirrored Y for lid flip)
        translate([led_rgb_x, outer_w - led_rgb_y, -0.5])
            cylinder(h = lid_t + 1.0, d = led_hole_d);
    }
}
