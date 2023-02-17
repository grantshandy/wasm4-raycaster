#![feature(core_intrinsics)]
#![allow(non_upper_case_globals)]
#![no_std]

use core::{
    f32::consts::{FRAC_PI_2, PI, TAU},
    intrinsics,
};

extern "C" {
    fn vline(x: i32, y: i32, len: u32);
}

const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;
const GAMEPAD1: *const u8 = 0x16 as *const u8;

const BUTTON_LEFT: u8 = 16;
const BUTTON_RIGHT: u8 = 32;
const BUTTON_UP: u8 = 64;
const BUTTON_DOWN: u8 = 128;

const FIVE_PI_SQUARED: f32 = 5.0 * (PI * PI);
const STEP_SIZE: f32 = 0.045;
const FOV: f32 = PI / 2.7;
const HALF_FOV: f32 = FOV * 0.5;
const ANGLE_STEP: f32 = FOV / 160.0;
const MAP_HEIGHT: usize = 8;

const cosf: fn(f32) -> f32 = |x: f32| sinf(x + FRAC_PI_2);
const tanf: fn(f32) -> f32 = |x: f32| sinf(x) / cosf(x);
const absf: fn(f32) -> f32 = |x: f32| unsafe { intrinsics::fabsf32(x) };
const floorf: fn(f32) -> f32 = |x: f32| unsafe { intrinsics::floorf32(x) };
const ceilf: fn(f32) -> f32 = |x: f32| unsafe { intrinsics::ceilf32(x) };

static mut GAME: Game = Game {
    player_x: 1.5,
    player_y: 1.5,
    player_angle: FIVE_PI_SQUARED,
    map: [
        0b1111111111111111,
        0b1000001010000101,
        0b1011100000110101,
        0b1000111010010001,
        0b1010001011110111,
        0b1011101001100001,
        0b1000100000001101,
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
        let wall_height =
            (100.0 / (ray.distance * cosf(ray.angle_diff))).to_int_unchecked::<i32>();

        if ray.vertical {
            *DRAW_COLORS = 0x2;
        } else {
            *DRAW_COLORS = 0x3;
        }

        vline(159 - x as i32, 80 - (wall_height / 2), wall_height as u32);
    }
}

struct Game {
    player_x: f32,
    player_y: f32,
    player_angle: f32,
    map: [u16; MAP_HEIGHT],
}

impl Game {
    pub fn update(&mut self, up: bool, down: bool, left: bool, right: bool) {
        let previous_position = (self.player_x, self.player_y);
        let mut x_diff = cosf(self.player_angle) * STEP_SIZE;
        let mut y_diff = -sinf(self.player_angle) * STEP_SIZE;
        let mut angle_diff = STEP_SIZE;

        if down {
            x_diff *= -1.0;
            y_diff *= -1.0;
        }

        if up || down {
            self.player_x += x_diff;
            self.player_y += y_diff;
        }

        if right {
            angle_diff *= -1.0;
        }

        if left || right {
            self.player_angle += angle_diff;
        }

        if coord_contains_wall(&self.map, self.player_x, self.player_y) {
            (self.player_x, self.player_y) = previous_position;
        }
    }

    pub fn get_view(&self) -> [Ray; 160] {
        let mut rays = [Ray {
            distance: 0.0,
            angle_diff: 0.0,
            vertical: false,
        }; 160];

        let starting_angle = self.player_angle - HALF_FOV;

        for (num, ray) in rays.iter_mut().enumerate() {
            let angle = starting_angle + num as f32 * ANGLE_STEP;

            let horizontal = self.horizontal_intersection(angle);
            let vertical = self.vertical_intersection(angle);

            if horizontal.distance < vertical.distance {
                *ray = horizontal;
            } else {
                *ray = vertical;
            }
        }

        rays
    }

    fn horizontal_intersection(&self, angle: f32) -> Ray {
        let up = absf(floorf(angle / PI) % 2.0) == 0.0;

        let first_y = snap_to_grid(self.player_y, !up);
        let first_x = first_y / -tanf(angle);

        let dy = if up { -1.0 } else { 1.0 };
        let dx = dy / -tanf(angle);

        let mut next_x = first_x;
        let mut next_y = first_y;

        for _ in 0..256 {
            let cell_x = (next_x + self.player_x) as i32;
            let mut cell_y = unsafe { (next_y + self.player_y).to_int_unchecked::<i32>() };

            if up {
                cell_y -= 1;
            }

            if coord_contains_wall(&self.map, cell_x as f32, cell_y as f32) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        Ray::new(next_x, next_y, self.player_angle, angle, false)
    }

    fn vertical_intersection(&self, angle: f32) -> Ray {
        let right = absf(floorf((angle - FRAC_PI_2) / PI) % 2.0) != 0.0;

        let first_x = snap_to_grid(self.player_x, right);
        let first_y = -tanf(angle) * first_x;

        let dx = if right { 1.0 } else { -1.0 };
        let dy = dx * -tanf(angle);

        let mut next_x = first_x;
        let mut next_y = first_y;

        for _ in 0..256 {
            let mut cell_x = unsafe { (next_x + self.player_x).to_int_unchecked::<i32>() };
            let cell_y = unsafe { (next_y + self.player_y).to_int_unchecked::<i32>() };

            if !right {
                cell_x -= 1;
            }

            if coord_contains_wall(&self.map, cell_x as f32, cell_y as f32) {
                break;
            }

            next_x += dx;
            next_y += dy;
        }

        Ray::new(next_x, next_y, self.player_angle, angle, true)
    }
}

#[derive(Copy, Clone)]
struct Ray {
    pub distance: f32,
    pub angle_diff: f32,
    pub vertical: bool,
}

impl Ray {
    pub fn new(next_x: f32, next_y: f32, player_angle: f32, angle: f32, vertical: bool) -> Self {
        Self {
            distance: unsafe { intrinsics::sqrtf32((next_x * next_x) + (next_y * next_y)) },
            angle_diff: angle - player_angle,
            vertical,
        }
    }
}

fn coord_contains_wall(map: &[u16; MAP_HEIGHT], x: f32, y: f32) -> bool {
    if let Some(line) = map.get(y as usize) {
        (line & (0b1 << x as usize)) != 0
    } else {
        true
    }
}

/// "elegant" Bhaskara I sin approximation
fn sinf(mut x: f32) -> f32 {
    let y = x / TAU;
    let z = y - floorf(y);
    x = z * TAU;

    let sin_impl = |x: f32| (16.0 * x * (PI - x)) / (FIVE_PI_SQUARED - (4.0 * x * (PI - x)));

    if x > PI {
        -sin_impl(x - PI)
    } else {
        sin_impl(x)
    }
}

fn snap_to_grid(x: f32, ceil: bool) -> f32 {
    if ceil {
        ceilf(x) - x
    } else {
        floorf(x) - x
    }
}

#[panic_handler]
fn phandler(_: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}
