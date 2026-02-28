# Autonomous Pool-Playing Robot

A VEX IQ powered robot that can independently locate a pool ball anywhere on a standard 44" × 88" table, navigate to it, calculate the optimal pocket to target, and sink the ball — all without any user input beyond specifying the robot's starting corner.

> 🏫 Built for the University of Waterloo MTE 121 course in collaboration with Seunghee Choi, Jun Kim, and Matthew Chan.

---

## Project Context

The robot operates under tight hardware constraints: a single distance sensor, an onboard gyroscope, and four motors total — three for movement and one for the striker. This meant that tasks which could otherwise be solved with richer sensing (e.g. cameras, multiple range finders) had to be solved entirely through geometry, careful filtering, and deliberate algorithm design.

Understanding the drive configuration is key to understanding the software. The robot uses a **Kiwi-drive** — three omni-wheels spaced 120° apart — which enables the orbital movement needed to position behind the ball at the correct shot angle. However, Kiwi-drive motion cannot be decomposed into simple x/y velocity components without transformation. A `kiwiToReal()` conversion function translates front-wheel motor rotations into real-world distance, and all position tracking is built on top of this.

---

## Goals

1. Reliably detect a pool ball using only a single linear distance sensor.
2. Maintain accurate positional self-tracking as the robot moves across the table.
3. Calculate the optimal angle to pocket the ball into the nearest pocket.
4. Strike the ball cleanly and repeatably in a straight line.

---

## Software Architecture

The program is structured as a sequential state machine with three top-level phases:

**1. Startup & Configuration**
- `configureAllSensors()` — calibrates the inertial sensor, zeroes all motor encoders, and resets the timer.
- `configureCartesianPlane()` — reads the controller input to identify which corner the robot starts in, then dead-reckons to the center of the table. The table center becomes the coordinate origin for all subsequent calculations.

**2. Ball Pocketing Loop**
The core loop runs until no ball is detected, executing the following sequence each iteration:

```
findBall → findBallCentre → goToBall → findOptimalAngle → orbitBall → strikeBall → resetCoordinates
```

**3. Shutdown**
If `findBall` completes a full 360° scan with no detection, the program concludes with a celebratory spin and terminates.

---

## Key Algorithms

### Ball Detection — `wallEstimate` + `findBall`

Because the robot has no direct way to "see" the ball, detection works by contrast: if the distance sensor reads *less* than the expected distance to the wall, something must be in the way.

`wallEstimate()` calculates the theoretical distance to the wall given the robot's current position and gyroscope heading. Using the heading, it determines which quadrant the robot is facing, then applies trigonometry against the known table dimensions to return the expected wall distance. During the scan rotation, `findBall()` continuously compares the live sensor reading against this estimate.

To prevent false positives from sensor noise, detection requires **40 consecutive readings** that are consistently below the wall estimate before the ball is declared found. Solitary outliers are rejected by also requiring that successive readings stay within 70 mm of each other — if a single anomalous spike appears, the count resets.

Once the ball is confirmed, its absolute (x, y) coordinates on the table are computed using the robot's heading and the measured distance:

```
ballX = pos[x] + sin(heading) × distance
ballY = pos[y] + cos(heading) × distance
```

### Ball Centering — `findBallCentre`

`findBall` points the robot roughly at the ball, but not necessarily at its center. `findBallCentre` refines this by first advancing to within 300 mm, then slowly rotating to scan for the edges of the ball — identified as the angles where the distance sensor reading abruptly increases and decreases. The two edge angles are averaged to find the true center, and the robot re-orients to face it precisely before approaching.

### Shot Angle Calculation — `findOptimalAngle`

With the ball's (x, y) position known and the coordinates of all six pockets hardcoded, this function calculates the straight-line angle from the ball to the nearest pocket. This angle becomes the target heading the robot must achieve before striking.

### Orbiting — `orbitBall`

To reposition behind the ball without disturbing it, the robot moves in an arc around the ball using Kiwi-drive's omnidirectional capability. The orbital motor speeds were empirically tuned to trace a consistent circular path. The robot continues orbiting until the gyroscope heading matches the target angle from `findOptimalAngle`.

### Position Recalibration — `resetCoordinates`

Odometry error accumulates over time due to wheel slip, especially during the fast strike motion. After each attempt, `resetCoordinates` corrects the robot's position by rotating 360° and measuring distances to the two nearest perpendicular walls. The true position is back-calculated from these two wall distances, overwriting the drifted estimate before the next shot.

### Striking — `strikeBall`

The strike is a hardcoded motion sequence: the robot drives forward slightly to overcome the ball's static friction (the striker motor alone was insufficient to initiate motion from rest), then fires the striker mechanism. After the strike, the robot reverses through the same motion to return to its pre-strike position, minimizing the odometry disruption from the rapid, uncontrolled forward movement.

---

## Results

| Metric | Success Rate |
|---|---|
| Overall ball pocketing | ~70% |
| Ball detection & navigation | ~90% |
| Straight-line striker accuracy | 100% |

The 30% overall failure rate was primarily attributable to position tracking drift over multiple attempts — which `resetCoordinates` partially mitigates — and the distance sensor's reduced accuracy beyond ~1 m, making far-side-of-table shots less reliable.

---

## Authors

**Austin Li**, Seunghee Choi, Jun Kim, Matthew Chan<br>
University of Waterloo — Department of Mechanical and Mechatronics Engineering<br>
[GitHub](https://github.com) · [LinkedIn](https://www.linkedin.com) · a564li@uwaterloo.ca
