#!/usr/bin/env python3
"""Generate topology events from Walker Delta constellation orbit simulation.
Outputs JSON with ISL/feeder link changes at each timestep.
Simplified: satellites move on circular orbits, ISL established if distance < threshold.
"""
import json, math, sys

# Walker Delta: 12 satellites, 3 planes × 4 sats/plane, 550km, 53° incl
NUM_SATS = 12
PLANES = 3
SATS_PER_PLANE = 4
ALTITUDE_KM = 550
EARTH_RADIUS_KM = 6371
INCLINATION_DEG = 53
ORBIT_PERIOD_S = 3600  # compressed for more events in 100s window
PHASE_FACTOR = 1

# ISL: tighter threshold = more topology changes
ISL_MAX_DISTANCE_KM = 3500
INTRA_PLANE_ISL = True
INTER_PLANE_ISL = True

# Ground stations (3 for more feeder link events)
GS_POSITIONS = [
    ("GS-E", 116.4, 39.9),    # Beijing
    ("GS-W", -77.0, 38.9),    # Washington DC
    ("GS-S", 18.4, -33.9),    # Cape Town
]
GS_MIN_ELEVATION_DEG = 10

# Simulation: 100s for NS-3 compatibility
SIM_DURATION_S = 100
TIME_STEP_S = 5  # finer granularity

def satellite_positions(t_seconds):
    """Compute satellite positions (lat, lon, alt_km) at time t."""
    positions = []
    for plane in range(PLANES):
        raan = plane * 180.0 / PLANES  # RAAN spread
        for sat in range(SATS_PER_PLANE):
            # Mean anomaly
            mean_anomaly = (360.0 * t_seconds / ORBIT_PERIOD_S
                          + sat * 360.0 / SATS_PER_PLANE
                          + plane * PHASE_FACTOR * 360.0 / (PLANES * SATS_PER_PLANE)) % 360
            # Simplified: circular orbit, inclination affects lat
            ma_rad = math.radians(mean_anomaly)
            raan_rad = math.radians(raan)
            incl_rad = math.radians(INCLINATION_DEG)

            # Latitude from spherical geometry
            lat = math.degrees(math.asin(math.sin(incl_rad) * math.sin(ma_rad)))
            # Longitude
            lon_correction = math.degrees(math.atan2(math.cos(incl_rad) * math.sin(ma_rad), math.cos(ma_rad)))
            lon = (raan + lon_correction + 360.0 * t_seconds / ORBIT_PERIOD_S * math.cos(incl_rad)) % 360
            if lon > 180: lon -= 360

            positions.append((lat, lon, ALTITUDE_KM))
    return positions

def distance_km(p1, p2):
    """Approximate distance between two satellites (spherical)."""
    lat1, lon1, alt1 = p1
    lat2, lon2, alt2 = p2
    # Convert to radians
    lat1r, lon1r = math.radians(lat1), math.radians(lon1)
    lat2r, lon2r = math.radians(lat2), math.radians(lon2)
    # Central angle
    dlat = lat2r - lat1r
    dlon = lon2r - lon1r
    a = math.sin(dlat/2)**2 + math.cos(lat1r)*math.cos(lat2r)*math.sin(dlon/2)**2
    central_angle = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    # Chord distance at orbital altitude
    r = EARTH_RADIUS_KM + alt1
    return 2 * r * math.sin(central_angle / 2)

def elevation_angle(sat_pos, gs_lat, gs_lon):
    """Compute elevation angle from ground station to satellite."""
    s_lat, s_lon, s_alt = sat_pos
    s_lat_r = math.radians(s_lat)
    s_lon_r = math.radians(s_lon)
    g_lat_r = math.radians(gs_lat)
    g_lon_r = math.radians(gs_lon)

    # Central angle
    dlon = s_lon_r - g_lon_r
    dlat = s_lat_r - g_lat_r
    a = math.sin(dlat/2)**2 + math.cos(g_lat_r)*math.cos(s_lat_r)*math.sin(dlon/2)**2
    central = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    r = EARTH_RADIUS_KM + s_alt
    # Elevation from horizon
    elev = math.degrees(math.atan2(
        math.cos(central) - EARTH_RADIUS_KM / r,
        math.sin(central)
    ))
    return elev

def generate_events():
    """Generate topology event timeline."""
    # Setup: satellite indices 1-6, GS indices 7-8
    # Plane A: sats 1,2,3  Plane B: sats 4,5,6
    # GS-E=7, GS-W=8

    prev_topology = None
    events = []

    for t in range(0, SIM_DURATION_S + TIME_STEP_S, TIME_STEP_S):
        positions = satellite_positions(t)

        # Build ISL edges
        edges = {}
        # Intra-plane ISL (always connected for simplicity)
        for plane in range(PLANES):
            base = plane * SATS_PER_PLANE
            for i in range(SATS_PER_PLANE):
                a = base + i
                b = base + (i + 1) % SATS_PER_PLANE
                dist = distance_km(positions[a], positions[b])
                edges[(a, b)] = {'bw': 200, 'type': 'intra-plane', 'dist_km': dist}

        # Inter-plane ISL (distance limited)
        for a in range(SATS_PER_PLANE):      # Plane A sats
            for b_prime in range(SATS_PER_PLANE):  # Plane B sats
                b = SATS_PER_PLANE + b_prime
                dist = distance_km(positions[a], positions[b])
                if dist < ISL_MAX_DISTANCE_KM:
                    edges[(a, b)] = {'bw': 180, 'type': 'inter-plane', 'dist_km': dist}

        # Feeder links (GS ↔ nearest visible satellite)
        gs_edges = {}
        for gs_idx, (gs_name, gs_lon, gs_lat) in enumerate(GS_POSITIONS):
            gs_node = 7 + gs_idx  # node indices 7,8
            best_sat = None
            best_elev = 0
            for sat_idx in range(NUM_SATS):
                elev = elevation_angle(positions[sat_idx], gs_lat, gs_lon)
                if elev > GS_MIN_ELEVATION_DEG and (best_sat is None or elev > best_elev):
                    best_sat = sat_idx
                    best_elev = elev
            if best_sat is not None:
                gs_edges[(gs_node, best_sat)] = {'bw': 350, 'type': 'feeder', 'elev_deg': round(best_elev, 1)}

        # Build topology string for comparison
        current = []
        for (a, b), info in sorted(edges.items()):
            current.append(f"{a+1},{b+1},{info['bw']}")
        for (gs, sat), info in sorted(gs_edges.items()):
            current.append(f"{gs},{sat+1},{info['bw']}")
        current_key = ";".join(current)

        # Check if topology changed
        if prev_topology != current_key:
            event_edges = []
            for (a, b), info in edges.items():
                event_edges.append({"a": a+1, "b": b+1, "bw": info['bw']})  # 1-indexed
            for (gs, sat), info in gs_edges.items():
                event_edges.append({"a": gs, "b": sat+1, "bw": info['bw']})

            # Simple event description
            if prev_topology is None:
                desc = "INITIAL"
            else:
                # Detect what changed
                prev_set = set(prev_topology.split(";"))
                curr_set = set(current)
                added = curr_set - prev_set
                removed = prev_set - curr_set
                if removed and not added:
                    desc = f"LOST: {' '.join(sorted(removed))}"
                elif added and not removed:
                    desc = f"NEW: {' '.join(sorted(added))}"
                else:
                    desc = "TOPOLOGY CHANGE"

            events.append({"t": t, "desc": desc, "edges": event_edges})
            prev_topology = current_key

    return events

if __name__ == "__main__":
    events = generate_events()
    # Output JSON
    print(json.dumps({"events": events, "num_sats": NUM_SATS, "gs_names": ["GS-E", "GS-W"],
                      "sim_duration": SIM_DURATION_S, "time_step": TIME_STEP_S}, indent=2))
    print(f"\nGenerated {len(events)} events over {SIM_DURATION_S}s", file=sys.stderr)