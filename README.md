# Magic Mouse Inertia

Palm-aware, kinetic scrolling for the Apple Magic Mouse on Linux.

`magic-mouse-inertia` reads the mouse's raw multitouch events and emits
validated scrolling through a scroll-only virtual input device. Pointer
movement and physical clicks continue through the kernel unchanged.

The result is responsive vertical and horizontal scrolling with a short,
natural deceleration tail—without treating every resting finger or palm touch
as a scroll gesture.

## Highlights

- Smooth high-resolution scrolling and configurable kinetic decay
- Horizontal and vertical axis locking
- Palm and stationary-contact rejection
- Immediate cancellation on a new touch, click, physical scroll, or disconnect
- Release velocity calculated from raw touch positions, including short flicks
- Automatic recovery after dropped input events or stale multitouch slots
- No device grab: normal pointer movement and buttons remain kernel-managed
- Small C implementation built directly on `libevdev` and `uinput`

## How it works

The Linux `hid_magicmouse` driver exposes pointer, button, wheel, and raw
multitouch events. This daemon observes that physical event device without
grabbing it. After validating the gesture, it sends `REL_WHEEL_HI_RES` and
`REL_HWHEEL_HI_RES` events through a virtual device named `Magic Mouse
Inertia`.

Kernel touch-to-wheel emulation must be disabled, otherwise both the kernel
and this daemon will generate scrolling. The daemon also needs read access to
the physical event node and write access to `/dev/uinput`.

If libevdev reports `SYN_DROPPED`, the daemon drains the synchronization
stream, reconstructs every multitouch slot from current kernel state, and
discards pending generated motion. It also reconciles tracked contacts after
each event batch. This prevents a missed release from leaving phantom fingers
that block scrolling until restart.

## Requirements

- Linux with the `hid_magicmouse` and `uinput` kernel modules
- A Magic Mouse exposing raw multitouch events through evdev
- A C11 compiler
- `pkg-config`
- `libevdev` development headers

On Arch Linux and derivatives:

```sh
sudo pacman -S --needed base-devel libevdev
```

On Debian and Ubuntu:

```sh
sudo apt install build-essential pkg-config libevdev-dev
```

## Build

```sh
git clone https://github.com/domhurr/magic-mouse-inertia.git
cd magic-mouse-inertia
make
make check
```

The resulting `magic-mouse-inertia` binary is self-contained apart from its
runtime dependency on `libevdev`.

## Configure Linux input

Load `uinput` now and arrange for it to load at boot:

```sh
sudo modprobe uinput
```

Disable the kernel's basic wheel emulation so scrolling is produced only once:

```text
options hid_magicmouse emulate_3button=0 emulate_scroll_wheel=0
```

Place that line in a file under `/etc/modprobe.d/`, then reload the driver or
reboot. Changing driver parameters can temporarily disconnect the mouse, so
keep another input method available.

For regular use, run the daemon as a dedicated locked-down account. Grant that
account access only to the selected Magic Mouse event node and `/dev/uinput`
with udev rules. Avoid adding your desktop account to the broad `input` group:
it grants access to keystrokes from every input device.

Exact event node numbers are not stable across reconnects. Find the device by
name instead:

```sh
grep -H . /sys/class/input/event*/device/name | grep -i 'magic mouse'
```

## Run

Pass the evdev device name shown by the previous command:

```sh
./magic-mouse-inertia --device-name "Apple Inc. Magic Mouse" --verbose
```

The setup this project originated from uses the following tuned profile:

```sh
./magic-mouse-inertia \
  --device-name "Apple Inc. Magic Mouse" \
  --idle-ms 42 \
  --decay-ms 340 \
  --gain 0.90 \
  --min-velocity 240 \
  --max-velocity 11000 \
  --max-duration-ms 1300 \
  --touch-scale 0.62 \
  --min-flick-distance 90 \
  --verbose
```

Use `./magic-mouse-inertia --help` for all options and built-in defaults.

## Tuning

- `--touch-scale` controls direct finger-to-scroll distance.
- `--gain` controls the velocity transferred into the inertia tail.
- `--decay-ms` controls how quickly inertia fades.
- `--idle-ms` controls how long a quiet touch waits before release detection.
- `--min-flick-distance` rejects tiny movements and coordinate noise.
- `--min-velocity`, `--max-velocity`, and `--max-duration-ms` bound momentum.

Change one value at a time. Start with `--touch-scale` for the base gesture,
then adjust `--gain` and `--decay-ms` for the release feel.

## Troubleshooting

### Pointer works but scrolling does not

Confirm the daemon can read the physical event node and open `/dev/uinput`.
Run with `--verbose` and look for `raw scroll armed`, `inertia start`, or a
permission error.

### Scrolling happens twice or feels too fast

Confirm `hid_magicmouse` reports `emulate_scroll_wheel` as disabled. Kernel
wheel emulation and this daemon must not run simultaneously.

### Scrolling stops after a Bluetooth interruption

The daemon reconnects when the named device reappears and repairs stale touch
state after dropped events. Verbose logs report repairs as `touch state
repaired`.

### The mouse is not found

Device names vary by model, firmware, and Bluetooth configuration. Supply the
exact value from `/sys/class/input/event*/device/name` with `--device-name`.

## Tested environment

Developed and tested with an Apple Magic Mouse 2 on CachyOS Linux and Hyprland.
The daemon operates at the evdev/uinput layer and does not depend on a specific
Wayland compositor.

## Security

Raw input access and `uinput` injection are privileged capabilities. Use a
dedicated service account, narrow udev matching to the intended mouse, and
sandbox the service if you install it system-wide. This repository intentionally
contains the daemon source rather than a machine-specific privileged installer.

## License

[MIT](LICENSE)
