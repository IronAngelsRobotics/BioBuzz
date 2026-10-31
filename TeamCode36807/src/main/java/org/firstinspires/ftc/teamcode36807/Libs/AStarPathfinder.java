package org.firstinspires.ftc.teamcode36807.Libs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Eight-neighbor A* pathfinding with obstacle-aware costs.
 *
 * RestrictedZone objects supplied by PathPlanner are ALREADY expanded to
 * account for the robot footprint and PathPlanner's extraMargin. Do not
 * expand them a second time here.
 *
 * This class does not alter the robot controller or its tuning.
 */
public final class AStarPathfinder {
    private final double fieldSize, grid;
    private final List<RestrictedZone> zones;
    private final double minCenter, maxCenter;
    private final int count;

    // Tuning parameters, in inches. These add a SOFT preference for clearance,
    // not an additional hard obstacle boundary.
    private static final double PROXIMITY_RANGE = 8.0;
    private static final double PROXIMITY_WEIGHT = 3.0;
    private static final int PROXIMITY_DIRECTIONS = 16;
    private static final double COST_SAMPLE_STEP = 1.0;

    public AStarPathfinder(double fieldSize, double grid, double clearance,
                           List<RestrictedZone> zones) {
        this.fieldSize = fieldSize;
        this.grid = grid;
        this.zones = zones;
        minCenter = clearance;
        maxCenter = fieldSize - clearance;
        count = (int) Math.floor((maxCenter - minCenter) / grid) + 1;
        if (count < 2) throw new IllegalArgumentException("Grid too large for field");
    }

    private int index(int x, int y) { return y * count + x; }

    private BezierPath.Point point(int x, int y) {
        return new BezierPath.Point(minCenter + x * grid, minCenter + y * grid);
    }

    private boolean free(double x, double y) {
        if (x < minCenter || x > maxCenter || y < minCenter || y > maxCenter) return false;
        for (RestrictedZone z : zones) if (z.contains(x, y)) return false;
        return true;
    }

    public boolean clearLine(BezierPath.Point a, BezierPath.Point b) {
        if (!free(a.x, a.y) || !free(b.x, b.y)) return false;
        for (RestrictedZone z : zones) {
            if (z.intersects(a.x, a.y, b.x, b.y)) return false;
        }
        return true;
    }

    /**
     * Estimate proximity to an expanded restricted zone by casting short rays.
     * Uses only the existing RestrictedZone.contains/intersects API.
     *
     * Returns 0 beyond PROXIMITY_RANGE and increases toward an obstacle.
     * Each returned value is a cost per inch of travel.
     */
    private double proximityPenalty(double x, double y) {
        if (zones.isEmpty()) return 0.0;

        double nearest = PROXIMITY_RANGE;
        for (RestrictedZone z : zones) {
            if (z.contains(x, y)) return PROXIMITY_WEIGHT;
            for (int i = 0; i < PROXIMITY_DIRECTIONS; i++) {
                double angle = 2.0 * Math.PI * i / PROXIMITY_DIRECTIONS;
                double dx = Math.cos(angle);
                double dy = Math.sin(angle);

                // Test increasing distances; first hit approximates proximity.
                for (double r = 1.0; r <= PROXIMITY_RANGE; r += 1.0) {
                    if (r >= nearest) break;
                    double ex = x + dx * r;
                    double ey = y + dy * r;
                    if (z.intersects(x, y, ex, ey)) {
                        nearest = r;
                        break;
                    }
                }
            }
        }

        double fraction = (PROXIMITY_RANGE - nearest) / PROXIMITY_RANGE;
        return PROXIMITY_WEIGHT * fraction * fraction;
    }

    /**
     * Weighted cost of traversing a clear line. Sampling also ensures that
     * shortcut evaluation uses the same proximity preference as A*.
     */
    private double travelCost(BezierPath.Point a, BezierPath.Point b) {
        double length = Math.hypot(b.x - a.x, b.y - a.y);
        if (length < 1e-9) return 0.0;
        int steps = Math.max(1, (int) Math.ceil(length / COST_SAMPLE_STEP));
        double step = length / steps;
        double cost = 0.0;
        for (int i = 0; i < steps; i++) {
            double t = (i + 0.5) / steps;
            double x = a.x + t * (b.x - a.x);
            double y = a.y + t * (b.y - a.y);
            cost += step * (1.0 + proximityPenalty(x, y));
        }
        return cost;
    }

    private int closestVisibleNode(BezierPath.Point p) {
        int best = -1;
        double bestD = Double.POSITIVE_INFINITY;
        for (int y = 0; y < count; y++) {
            for (int x = 0; x < count; x++) {
                BezierPath.Point q = point(x, y);
                if (!clearLine(p, q)) continue;
                double d = Math.hypot(q.x - p.x, q.y - p.y);
                if (d < bestD) { bestD = d; best = index(x, y); }
            }
        }
        return best;
    }

    private static final class Node {
        final int id;
        final double f;
        Node(int id, double f) { this.id = id; this.f = f; }
    }

    public List<BezierPath.Point> find(BezierPath.Point start, BezierPath.Point goal) {
        if (!free(start.x, start.y) || !free(goal.x, goal.y)) return null;
        if (clearLine(start, goal)) return Arrays.asList(start, goal);

        int s = closestVisibleNode(start);
        int g = closestVisibleNode(goal);
        if (s < 0 || g < 0) return null;

        int n = count * count;
        double[] dist = new double[n];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        int[] prev = new int[n];
        Arrays.fill(prev, -1);
        boolean[] closed = new boolean[n];
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(a -> a.f));
        dist[s] = 0.0;
        open.add(new Node(s, 0.0));

        int[] moves = {-1, 0, 1};
        BezierPath.Point goalGrid = point(g % count, g / count);

        while (!open.isEmpty()) {
            Node node = open.poll();
            int id = node.id;
            if (closed[id]) continue;
            closed[id] = true;
            if (id == g) break;

            int x = id % count;
            int y = id / count;
            BezierPath.Point a = point(x, y);
            for (int dy : moves) {
                for (int dx : moves) {
                    if (dx == 0 && dy == 0) continue;
                    int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= count || yy >= count) continue;
                    int ni = index(xx, yy);
                    if (closed[ni]) continue;
                    BezierPath.Point b = point(xx, yy);
                    if (!clearLine(a, b)) continue;

                    // The Euclidean heuristic remains admissible because
                    // proximity penalties are nonnegative.
                    double alt = dist[id] + travelCost(a, b);
                    if (alt < dist[ni]) {
                        dist[ni] = alt;
                        prev[ni] = id;
                        double heuristic = Math.hypot(b.x - goalGrid.x, b.y - goalGrid.y);
                        open.add(new Node(ni, alt + heuristic));
                    }
                }
            }
        }

        if (!closed[g]) return null;
        List<BezierPath.Point> path = new ArrayList<>();
        for (int id = g; id >= 0; id = prev[id]) {
            path.add(point(id % count, id / count));
            if (id == s) break;
        }
        Collections.reverse(path);
        path.add(0, start);
        path.add(goal);

        // Clearance-aware simplification: only remove waypoints if the direct
        // segment is collision-free AND has no greater weighted cost than the
        // subpath it replaces. This prevents shortcutting away the benefit of
        // proximity-aware A* routing.
        List<BezierPath.Point> simple = new ArrayList<>();
        int i = 0;
        simple.add(path.get(0));
        while (i < path.size() - 1) {
            int chosen = i + 1;
            double originalCost = 0.0;
            for (int j = i + 1; j < path.size(); j++) {
                originalCost += travelCost(path.get(j - 1), path.get(j));
                if (!clearLine(path.get(i), path.get(j))) continue;
                double directCost = travelCost(path.get(i), path.get(j));
                if (directCost <= originalCost + 1e-6) chosen = j;
            }
            simple.add(path.get(chosen));
            i = chosen;
        }
        return simple;
    }
}
