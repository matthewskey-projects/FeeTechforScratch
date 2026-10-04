# FeeTechforScratch

A [TurboWarp](https://turbowarp.org) extension that lets Scratch projects drive an
[SO-100 robot arm](https://github.com/TheRobotStudio/SO-ARM100) and read feedback from its
Feetech STS3215 servos. It talks to the servo bus through a Feetech URT-1 (or any USB-to-TTL
bus servo adapter) using the browser's Web Serial API, so nothing needs to be installed.

## Requirements

- Chrome, Edge or Opera on a desktop or laptop (Web Serial is not available in Firefox, Safari or on tablets).
- TurboWarp: the website at turbowarp.org or the desktop app.
- The arm wired and powered: six STS3215 servos with IDs 1 to 6 at 1,000,000 baud, the URT-1 in TTL mode,
  and an external power supply that matches your servos. Check the arm moves in Feetech FD or LeRobot first.
- Close any other program using the COM port (Feetech FD, LeRobot) before connecting.

## Loading the extension

1. Open TurboWarp in Chrome or Edge.
2. Click **Add Extension** (bottom left), then **Custom Extension**.
3. Choose the **Files** tab and pick `so100.js`, or the **Text** tab and paste its contents.
4. Tick **Run extension without sandbox** and click **Load**. The extension needs this to reach the serial port.

The **SO-100 Arm** category appears in the block palette.

## Using it

Click the **connect to robot** block itself. The browser shows a list of serial ports; choose the URT-1.
The extension then finds which of motors 1 to 6 answer and starts reading them about 30 times a second.

| Block | What it does |
| --- | --- |
| connect to robot / disconnect robot | Open or close the serial port (disconnect turns the motors off) |
| robot connected?, motor [joint] found? | Connection checks |
| move [joint] to [angle] degrees | Set a joint's target angle; the script continues straight away |
| move [joint] to [angle] degrees and wait | Same, but waits until the joint arrives (up to 5 seconds) |
| turn [joint] by [angle] degrees | Move relative to where the joint is now |
| move arm to pan … gripper … | Set all six joints at once so they start together |
| set gripper to [percent] % open | 0 % is the gripper's minimum position limit, 100 % its maximum |
| set [joint] speed to [percent] % | Joint speed; 100 % is 3000 steps per second |
| turn motor [joint] on / off | Torque on or off. With torque off you can pose the arm by hand |
| emergency stop | Turns torque off on every motor at once. The arm will drop under gravity |
| [joint] angle | Current angle in degrees |
| [joint] load % / speed / temperature / voltage / current / raw position | Live motor feedback |
| [joint] moving? | True while the joint is moving |
| when [joint] load > [percent] % | Runs a script when a joint pushes against something |
| remember pose as [name] / go to pose [name] | Record the arm's current position and return to it later |
| set [joint] zero to current position | Calibrate: the current position becomes 0 degrees |

Angles are measured from each joint's zero. By default zero is the servo's middle position (raw 2048), which
is the middle of the range after LeRobot calibration. Use **set [joint] zero to current position** to choose your
own zero; it is remembered in this browser.

Safety behaviour:

- Every target is clamped to the min and max position limits stored in each servo (LeRobot calibration writes these).
- Moves are sent at most about 30 times a second, so a `forever` loop cannot flood the bus.
- The red stop button cancels queued moves and holds every joint where it is.
- Turning torque on first sets the target to the current position, so the arm does not jump.

## How it works

`so100.js` is a single file. The top half implements the Feetech STS protocol (packet building, checksums,
PING, READ, WRITE, SYNC READ and SYNC WRITE) and a half-duplex serial bus that sends one request at a time.
The bottom half is the Scratch extension. Move blocks only record a target; a background loop sends all pending
targets in one SYNC WRITE and then reads the state of every motor with one SYNC READ. Reporter blocks return the
latest values from that loop, so they never wait on the serial port.

Register addresses come from the STS/SMS control table in
[LeRobot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py).

## Development

```sh
npm test
```

runs the protocol tests (packet bytes, parsing, echo handling, timeouts) and a check that every block maps to a
method, using Node 18 or newer. They do not need the arm.
