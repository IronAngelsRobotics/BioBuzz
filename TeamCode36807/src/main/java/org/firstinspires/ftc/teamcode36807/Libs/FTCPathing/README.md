# FTC Mecanum Safe Pathing (SDK 12.x Java example)

Place the `src/org/...` Java files into matching TeamCode package folders, or change package declarations to match your project.

## Classes
- Pose2D: immutable position snapshot (inches, degrees)
- BezierPath: cubic Bézier geometry
- WaypointMode: STOP / PASS_THROUGH
- PathTarget / PathSequence: requested destinations
- RestrictedZone: rectangular no-go areas with clearance and segment intersection
- AStarPathfinder: 3-inch grid, 8-direction A*, line-of-sight simplification
- PathPlanner: inflated zones, safe segment generation and verification
- PathSegment / PlannedRoute: preplanned curve sequence
- MecanumPathFollower: nonblocking pose-snapshot follower with lookahead, heading, slew, and segment transitions
- PathingTeleOpLinear: FTC LinearOpMode example

## Setup / safety
1. Set actual motor hardware names and directions, Pinpoint offsets, pod directions, pose origin, heading convention, and coordinate signs before enabling drive.
2. Example zones and destinations are fictional. Enter measured field locations and verify all zones and clearance. The robot center is constrained within the field using its half-diagonal plus a margin.
3. Test with wheels raised first, then in an empty field at low power. Confirm forward, strafe, and turn signs, and that the robot moves toward (not away from) a nearby target.
4. The planner only avoids **static** zones. It does not detect other robots or moving obstacles. Stop/cancel on unsafe conditions.
5. Curve validation uses 150 sampled line segments. This is a practical approximation, not formal continuous-curve collision certification. Reduce step length or use adaptive subdivision for safety-critical narrow gaps.
6. PASS_THROUGH uses a near-end progress and proximity threshold. Very sharp turns, overshoot, or odometry slip can cause a segment to stall; cancel manually if needed. Tune at low speed.
7. Current heading convention in the follower assumes 0° = +field Y and positive CCW. Travel directions use atan2(Y,X), so 0° = +field X. Adjust transform if your Pinpoint axes differ.
8. STOP waypoints stop before advancing to subsequent segments; PASS_THROUGH preserves motor command continuity.
9. A* uses 3-inch grid and conservative obstacle inflation. If it returns no route, it does not drive. Extremely tight passages may be rejected.

## Design note
The planner can fall back from a curved segment to a straight cubic Bézier if the curve would intersect a restricted zone. Consequently, this implementation prioritizes obstacle avoidance over guaranteed smooth tangent continuity. It is a starting point for tuning, not a competition-validated navigation system.
