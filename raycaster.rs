#![feature(core_intrinsics)]
#![no_std]

use core::{
    f32::consts::{FRAC_PI_2, PI, TAU},
    intrinsics,
    panic::PanicInfo,
};

const FIVE_PI_SQUARED: f32 = 5.0 * (PI * PI);

const SCREEN_SIZE: u32 = 160;
const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;
const GAMEPAD1: *const u8 = 0x16 as *const u8;

const BUTTON_LEFT: u8 = 16;
const BUTTON_RIGHT: u8 = 32;
const BUTTON_UP: u8 = 64;
const BUTTON_DOWN: u8 = 128;

const STEP_SIZE: f32 = 0.05;
const FOV: f32 = PI / 2.7;
const HALF_FOV: f32 = FOV / 2.0;

static mut GAME: Game<16> = Game {
    player_x: 1.5,
    player_y: 1.5,
    player_angle: TAU,
    map: [
        0b1111111111111111,
        0b1000011100000001,
        0b1111011101110111,
        0b1000000101000001,
        0b1101101101111101,
        0b1000100100100001,
        0b1101110101110101,
        0b1000000000000001,
        0b1101101001110111,
        0b1100110000110001,
        0b1001111001111001,
        0b1100111001001011,
        0b1001001000001011,
        0b1100000010101001,
        0b1000011100000001,
        0b1111111111111111,
    ],
};

#[no_mangle]
unsafe fn update() {
    GAME.update(
        *GAMEPAD1 & BUTTON_UP != 0,
        *GAMEPAD1 & BUTTON_DOWN != 0,
        *GAMEPAD1 & BUTTON_LEFT != 0,
        *GAMEPAD1 & BUTTON_RIGHT != 0,
    );

    for (x, ray) in GAME.get_view().iter().enumerate() {
        let wall_height = (20.0 / (ray.distance * cosf(ray.angle_diff))) * 5.0;

        if ray.vertical {
            *DRAW_COLORS = 0x2;
        } else {
            *DRAW_COLORS = 0x3;
        }

        vline(
            SCREEN_SIZE as i32 - x as i32 - 1,
            ((SCREEN_SIZE as f32 - wall_height) / 2.0) as i32,
            wall_height as u32,
        );
    }
}

#[derive(Copy, Clone)]
struct Ray {
    pub distance: f32,
    pub angle_diff: f32,
    pub vertical: bool,
}

struct Game<const H: usize> {
    player_x: f32,
    player_y: f32,
    player_angle: f32,
    map: [u16; H],
}

impl<const H: usize> Game<H> {
    /// update the player's position
    pub fn update(&mut self, up: bool, down: bool, left: bool, right: bool) {
        let previous_position = (self.player_x, self.player_y);
        let mut moved = false;

        if up {
            self.player_x += cosf(self.player_angle) * STEP_SIZE;
            self.player_y -= sinf(self.player_angle) * STEP_SIZE;
            moved = true;
        }

        if down {
            self.player_x -= cosf(self.player_angle) * STEP_SIZE;
            self.player_y += sinf(self.player_angle) * STEP_SIZE;
            moved = true;
        }

        if moved && coord_contains_wall(&self.map, self.player_x, self.player_y) {
            (self.player_x, self.player_y) = previous_position;
        }

        if left {
            self.player_angle += STEP_SIZE;
        }

        if right {
            self.player_angle -= STEP_SIZE;
        }

        if self.player_angle > TAU {
            self.player_angle -= TAU;
        } else if self.player_angle <= 0.0 {
            self.player_angle += TAU;
        }
    }

    /// get all rays from the player's current position
    pub fn get_view(&self) -> [Ray; 160] {
        let mut rays = [Ray {
            distance: 0.0,
            angle_diff: 0.0,
            vertical: false,
        }; 160];

        let angle_step = FOV / 160.0;
        let starting_angle = self.player_angle - HALF_FOV;

        for (num, ray) in rays.iter_mut().enumerate() {
            let angle = starting_angle + num as f32 * angle_step;

            let h = self.horizontal_intersection(angle);
            let v = self.vertical_intersection(angle);

            if h.distance < v.distance {
                *ray = h;
            } else {
                *ray = v;
            }
        }

        rays
    }

    /// Returns the closest horizontal wall intersection's distance
    fn horizontal_intersection(&self, angle: f32) -> Ray {
        let up = absf(floorf(angle / PI) % 2.0) == 0.0;

        let first_y = if up {
            floorf(self.player_y) - self.player_y
        } else {
            ceilf(self.player_y) - self.player_y
        };
        let first_x = first_y / -tanf(angle);

        let dy = if up { -1.0 } else { 1.0 };
        let dx = dy / -tanf(angle);

        let mut next_x = first_x;
        let mut next_y = first_y;

        for _ in 0..(H * 16) {
            let cell_x = (next_x + self.player_x) as i32;
            let cell_y = if up {
                (next_y + self.player_y) as i32 - 1
            } else {
                (next_y + self.player_y) as i32
            };

            if coord_contains_wall(&self.map, cell_x as f32, cell_y as f32) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        Ray {
            distance: distance(
                self.player_x,
                self.player_y,
                self.player_x + next_x,
                self.player_y + next_y,
            ),
            angle_diff: angle - self.player_angle,
            vertical: false,
        }
    }

    /// Returns the closest horizontal wall intersection's distance
    fn vertical_intersection(&self, angle: f32) -> Ray {
        let right = absf(floorf((angle - FRAC_PI_2) / PI) % 2.0) != 0.0;

        let first_x = if right {
            ceilf(self.player_x) - self.player_x
        } else {
            floorf(self.player_x) - self.player_x
        };
        let first_y = -tanf(angle) * first_x;

        let dx = if right { 1.0 } else { -1.0 };
        let dy = dx * -tanf(angle);

        let mut next_x = first_x;
        let mut next_y = first_y;

        for _ in 0..(H * 16) {
            let cell_x = if right {
                (next_x + self.player_x) as i32
            } else {
                (next_x + self.player_x) as i32 - 1
            };
            let cell_y = (next_y + self.player_y) as i32;

            if coord_contains_wall(&self.map, cell_x as f32, cell_y as f32) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        Ray {
            distance: distance(
                self.player_x,
                self.player_y,
                self.player_x + next_x,
                self.player_y + next_y,
            ),
            angle_diff: angle - self.player_angle,
            vertical: true,
        }
    }
}

fn coord_contains_wall<const H: usize>(map: &[u16; H], x: f32, y: f32) -> bool {
    if let Some(line) = map.get(y as usize) {
        (line & (0b1 << x as usize)) != 0
    } else {
        true
    }
}

fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    let dy = y2 - y1;
    let dx = x2 - x1;

    unsafe { intrinsics::sqrtf32((dy * dy) + (dx * dx)) }
}

/// extremely elegant Bhaskara I sin approximation
fn sinf(mut x: f32) -> f32 {
    let y = x / TAU;
    let z = y - unsafe { intrinsics::floorf32(y) };
    x = z * TAU;

    fn sinf_imp(x: f32) -> f32 {
        (16.0 * x * (PI - x)) / (FIVE_PI_SQUARED - (4.0 * x * (PI - x)))
    }

    if x > PI {
        -sinf_imp(x - PI)
    } else {
        sinf_imp(x)
    }
}

fn cosf(x: f32) -> f32 {
    sinf(x + FRAC_PI_2)
}

fn tanf(x: f32) -> f32 {
    sinf(x) / cosf(x)
}

fn absf(x: f32) -> f32 {
    unsafe { intrinsics::fabsf32(x) }
}

fn floorf(x: f32) -> f32 {
    unsafe { intrinsics::floorf32(x) }
}

fn ceilf(x: f32) -> f32 {
    unsafe { intrinsics::ceilf32(x) }
}

extern "C" {
    fn vline(x: i32, y: i32, len: u32);
}

#[panic_handler]
fn phandler(_: &PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}
