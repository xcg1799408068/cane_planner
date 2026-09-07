#!/usr/bin/env python
"""Measure the Ours intervention trigger signals from a pilot rosbag.

Answers one question: is T_autonomy (and J_safe*) able to separate
"human is drifting out of the corridor" from "normal walking"? If yes, it
prints the separable band that T_guide / J_guide should sit in. If no, it
says so instead of inventing a threshold.

Usage:
    source /opt/ros/noetic/setup.bash
    python analyze_intervention_bag.py ours_pilot.bag [more.bag ...]

Record with:
    rosbag record -O ours_pilot.bag \
      /mpc/intervention/metrics /mpc/intervention/state \
      /mpc/debug_metrics /mpc/walking_corridor_debug \
      /sim_odom /astar/path
"""
from __future__ import print_function

import math
import re
import sys
from collections import OrderedDict

import rosbag

# publishIntervention() field order; see planner_manager.cpp.
M = OrderedDict([
    ("total_count", 0), ("safe_count", 1), ("eta", 2), ("horizon", 3),
    ("T_auto", 4), ("J_star", 5), ("selected", 6), ("robust_selected", 7),
    ("selected_heading", 8), ("dangerous", 9), ("pending_stop", 10),
    ("stop_advice", 11), ("danger_vote", 12), ("recovery_vote", 13),
    ("best_margin", 14), ("median_margin", 15),
    ("best_lateral_margin", 16), ("median_lateral_margin", 17),
    ("margin_argmin_fraction", 18),
    ("applied_correction", 19), ("walk_time", 20),
    ("T_exit_free", 21), ("T_required", 22), ("M_time", 23),
    ("T_eval_free", 24), ("coverage_ratio", 25),
    ("exit_observed", 26), ("horizon_safe", 27),
    ("required_horizon_covered", 28), ("native_best", 29),
    ("effective_selected", 30), ("u_assist_cmd", 31),
    ("u_assist_applied", 32), ("u_human_api", 33),
    ("u_total_api", 34), ("policy_stop_requested", 35),
    ("intervention_stop_applied", 36), ("final_arbiter_stop", 37),
    ("stop_enforced", 38), ("schema_version", 39),
    # Schema-v3 append-only fields. Index 39 intentionally remains 2.0.
    ("current_lfpc_heading", 40), ("target_valid", 41),
    ("delta_heading_required", 42), ("heading_at_reaction_end", 43),
    ("T_turn_required", 44), ("turn_steps", 45),
    ("turn_recoverable", 46), ("turn_status", 47),
    ("T_required_dynamic", 48), ("M_time_dynamic", 49),
    ("dynamic_evidence_valid", 50), ("right_censored_dynamic", 51),
    ("dynamic_evidence_status", 52), ("dynamic_adequate_coverage", 53),
    ("old_trigger_raw", 54), ("old_guidance_effective", 55),
    ("new_enter_raw", 56), ("new_release_raw", 57),
    ("new_trigger_latched", 58), ("new_guidance_effective", 59),
    ("use_dynamic_trigger", 60), ("pure_exit_time_insufficient", 61),
    ("cfg_t_sup", 62), ("cfg_reaction_delay", 63),
    ("cfg_compliance", 64), ("cfg_cap", 65), ("cfg_rate", 66),
    ("cfg_heading_tolerance", 67), ("cfg_buffer", 68),
    ("cfg_max_steps", 69), ("cfg_M_enter", 70),
    ("cfg_M_release", 71), ("schema_version_v3", 72),
])
DBG_VALID_RATIO, DBG_STATIC_REJECT, DBG_CORRIDOR_REJECT = 1, 6, 11
EPISODE_GAP = 1.5      # s without metrics => new episode
PRE_WINDOW = 2.0       # s looked back from the first safe_count==0 frame
DRIFT_ON_ROUTE = 0.25  # m; below this the human is still on the global route


def pct(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    k = (len(ordered) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - k) + ordered[hi] * (k - lo)


def point_to_polyline(p, path):
    if len(path) < 2:
        return float("nan")
    best = float("inf")
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        if den < 1e-12:
            continue
        u = ((p[0] - ax) * dx + (p[1] - ay) * dy) / den
        u = max(0.0, min(1.0, u))
        best = min(best, math.hypot(p[0] - (ax + u * dx), p[1] - (ay + u * dy)))
    return best


def read(paths):
    frames, states, routes, odom, corridor, dbg = [], [], [], [], [], []
    for path in paths:
        with rosbag.Bag(path) as bag:
            for topic, msg, t in bag.read_messages():
                ts = t.to_sec()
                if topic.endswith("/intervention/metrics") and len(msg.data) >= 14:
                    frames.append((ts, list(msg.data)))
                elif topic.endswith("/intervention/state"):
                    states.append((ts, msg.data))
                elif topic == "/astar/path":
                    routes.append((ts, [(q.pose.position.x, q.pose.position.y)
                                        for q in msg.poses]))
                elif topic == "/sim_odom":
                    orientation = msg.pose.pose.orientation
                    yaw = math.atan2(
                        2.0 * (orientation.w * orientation.z +
                               orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y * orientation.y +
                                     orientation.z * orientation.z))
                    odom.append((ts, (msg.pose.pose.position.x,
                                      msg.pose.pose.position.y, yaw)))
                elif topic.endswith("walking_corridor_debug"):
                    corridor.append((ts, msg.data))
                elif topic == "/mpc/debug_metrics" and len(msg.data) >= 19:
                    dbg.append((ts, list(msg.data)))
    for seq in (frames, states, routes, odom, corridor, dbg):
        seq.sort(key=lambda r: r[0])
    return frames, states, routes, odom, corridor, dbg


def latest_before(seq, ts):
    found = None
    for t, v in seq:
        if t <= ts:
            found = v
        else:
            break
    return found


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def old_trigger_raw(d):
    return (d[M["safe_count"]] > 0 and
            d[M["T_auto"]] < 4.0 and d[M["J_star"]] > 0.06)


def proxy_turn_time(current_heading, target_heading, human_api=0.0,
                    reaction=0.35, t_sup=0.35, compliance=0.6,
                    cap=0.174532925199, rate=0.1, tolerance=0.02,
                    max_steps=30):
    """Offline proxy only; sparse odom yaw is not exact runtime LFPC heading."""
    heading = current_heading
    for _ in range(int(math.ceil(max(0.0, reaction) / t_sup - 1e-12))):
        heading = wrap_angle(heading + human_api)
    previous = 0.0
    if abs(wrap_angle(target_heading - heading)) <= tolerance:
        return 0.0, True
    for step in range(1, max_steps + 1):
        cmd = max(-cap, min(cap, compliance * wrap_angle(target_heading - heading)))
        previous += max(-rate, min(rate, cmd - previous))
        heading = wrap_angle(heading + human_api + previous)
        if abs(wrap_angle(target_heading - heading)) <= tolerance:
            return step * t_sup, True
    return float("nan"), False


def schema_v3_report(frames):
    v3 = [(ts, d) for ts, d in frames
          if len(d) > M["schema_version_v3"] and d[M["schema_version_v3"]] >= 3.0]
    if not v3:
        return
    disagreements = sum(1 for _, d in v3
                        if bool(d[M["old_guidance_effective"]] > 0.5) !=
                        bool(d[M["new_guidance_effective"]] > 0.5))
    unknown = sum(1 for _, d in v3 if d[M["dynamic_evidence_status"]] < 0.5)
    censored = sum(1 for _, d in v3 if d[M["right_censored_dynamic"]] > 0.5)
    unrecoverable = sum(1 for _, d in v3 if d[M["turn_recoverable"]] < 0.5)
    enter = sum(1 for _, d in v3 if d[M["new_enter_raw"]] > 0.5)
    release = sum(1 for _, d in v3 if d[M["new_release_raw"]] > 0.5)
    print("\nschema-v3 dynamic shadow")
    print("  frames=%d disagreements=%d enter_raw=%d release_raw=%d" %
          (len(v3), disagreements, enter, release))
    print("  UNKNOWN=%d right_censored=%d unrecoverable=%d" %
          (unknown, censored, unrecoverable))

    latch_events = []
    previous = False
    active_duration = 0.0
    for i, (ts, d) in enumerate(v3):
        active = d[M["new_trigger_latched"]] > 0.5
        if active != previous:
            latch_events.append((d[M["walk_time"]], active))
        if active and i + 1 < len(v3):
            active_duration += max(0.0,
                                   v3[i + 1][1][M["walk_time"]] -
                                   d[M["walk_time"]])
        previous = active
    entries = [t for t, active in latch_events if active]
    releases = [t for t, active in latch_events if not active]
    print("  latch entries=%d releases=%d active_walk_duration=%.3fs" %
          (len(entries), len(releases), active_duration))

    first_new = entries[0] if entries else None
    first_stop = next((d[M["walk_time"]] for _, d in v3
                       if d[M["stop_advice"]] > 0.5), None)
    first_zero = next((d[M["walk_time"]] for _, d in v3
                       if d[M["safe_count"]] == 0), None)
    first_cmd = next((d[M["walk_time"]] for _, d in v3
                      if abs(d[M["u_assist_applied"]]) > 1e-6), None)
    if first_new is not None:
        print("  first new latch walk_time=%.3f; lead to STOP=%s, safe_count=0=%s, command=%s" %
              (first_new,
               "%.3f" % (first_stop - first_new) if first_stop is not None else "n/a",
               "%.3f" % (first_zero - first_new) if first_zero is not None else "n/a",
               "%.3f" % (first_cmd - first_new) if first_cmd is not None else "n/a"))


def main(paths):
    frames, states, routes, odom, corridor, dbg = read(paths)
    if not frames:
        print("no /mpc/intervention/metrics in bag(s) -- was intervention_enable:=true?")
        return 1

    episodes, current = [], [frames[0]]
    for prev, cur in zip(frames, frames[1:]):
        if cur[0] - prev[0] > EPISODE_GAP:
            episodes.append(current)
            current = [cur]
        else:
            current.append(cur)
    episodes.append(current)

    healthy_T, healthy_J, pre_T, pre_J = [], [], [], []
    healthy_Mg, pre_Mg = [], []
    healthy_Lt, pre_Lt, argmin = [], [], []
    print("=" * 78)
    for ep_i, ep in enumerate(episodes):
        t0 = ep[0][0]
        collapse = next((ts for ts, d in ep if d[M["safe_count"]] == 0), None)
        stopped = any(d[M["stop_advice"]] > 0.5 for _, d in ep)
        drifts = []
        for ts, _ in ep:
            pos, route = latest_before(odom, ts), latest_before(routes, ts)
            if pos and route:
                drifts.append(point_to_polyline(pos, route))
        src = latest_before(corridor, ep[-1][0]) or ""
        m = re.search(r"source=(\S+)", src)
        print("episode %d  frames=%3d  dur=%5.1fs  stop_advice=%s  drift_max=%s  corridor_source=%s"
              % (ep_i + 1, len(ep), ep[-1][0] - t0, "yes" if stopped else "no",
                 "%.2fm" % max(drifts) if drifts else "n/a",
                 m.group(1) if m else "?"))
        if collapse is not None:
            d = latest_before(dbg, collapse + 0.1)
            print("           collapse at t+%.1fs: valid_ratio=%s static_reject=%s corridor_reject=%s"
                  % (collapse - t0,
                     "%.2f" % d[DBG_VALID_RATIO] if d else "?",
                     "%d" % int(d[DBG_STATIC_REJECT]) if d else "?",
                     "%d" % int(d[DBG_CORRIDOR_REJECT]) if d else "?"))

        for idx, (ts, d) in enumerate(ep):
            T, J = d[M["T_auto"]], d[M["J_star"]]
            drift = drifts[idx] if idx < len(drifts) else float("nan")
            mg = d[M["best_margin"]] if len(d) > M["best_margin"] else -9.0
            lt = d[M["best_lateral_margin"]] if len(d) > M["best_lateral_margin"] else -9.0
            if len(d) > M["margin_argmin_fraction"] and d[M["margin_argmin_fraction"]] >= 0.0:
                argmin.append(d[M["margin_argmin_fraction"]])
            if collapse is not None and 0 < collapse - ts <= PRE_WINDOW:
                pre_T.append(T)
                if J >= 0:
                    pre_J.append(J)
                if mg > -8.0:
                    pre_Mg.append(mg)
                if lt > -8.0:
                    pre_Lt.append(lt)
            elif d[M["safe_count"]] > 0 and (drift != drift or drift <= DRIFT_ON_ROUTE):
                healthy_T.append(T)
                if J >= 0:
                    healthy_J.append(J)
                if mg > -8.0:
                    healthy_Mg.append(mg)
                if lt > -8.0:
                    healthy_Lt.append(lt)

    # Response-model health. A guidance layer that keeps asking for the same
    # saturated turn every step is a limit cycle, not guidance; before field 19
    # existed that had to be reconstructed from odometry.
    applied = [d[M["applied_correction"]] for _, d in frames
               if len(d) > M["applied_correction"]]
    acting = [a for a in applied if abs(a) > 1e-6]
    if applied:
        cap = max(abs(a) for a in applied)
        saturated = sum(1 for a in acting if abs(a) >= cap - 1e-6)
        same_sign, run, best_run = 0, 0, 0
        prev = 0.0
        for a in applied:
            if abs(a) > 1e-6 and a * prev > 0.0:
                run += 1
                best_run = max(best_run, run)
            else:
                run = 0
            if abs(a) > 1e-6:
                prev = a
        print("\napplied heading correction (field 19)")
        print("  frames commanding a turn : %d / %d" % (len(acting), len(applied)))
        if acting:
            print("  |correction| p50=%.3f p90=%.3f max=%.3f rad"
                  % (pct([abs(a) for a in acting], 50), pct([abs(a) for a in acting], 90), cap))
            print("  frames at the cap       : %d" % saturated)
        print("  longest same-direction run: %d frames  (a full 2*pi orbit needs"
              " ~%d frames at the cap)" % (best_run, int(math.ceil(2 * math.pi / cap)) if cap > 1e-6 else 0))
        if best_run > 0 and cap > 1e-6 and best_run >= 2 * math.pi / cap:
            print("  -> LIMIT CYCLE: the response held one direction long enough to"
                  " close a full circle")
    else:
        print("\napplied heading correction (field 19): not present -- bag predates the field")

    def band(name, healthy, pre, cfg):
        print("\n%s" % name)
        print("  on-route, safe frames : n=%-4d p10=%.3f p50=%.3f p90=%.3f"
              % (len(healthy), pct(healthy, 10), pct(healthy, 50), pct(healthy, 90)))
        print("  %.1fs before collapse : n=%-4d p10=%.3f p50=%.3f p90=%.3f"
              % (PRE_WINDOW, len(pre), pct(pre, 10), pct(pre, 50), pct(pre, 90)))
        print("  configured now        : %.3f" % cfg)
        hi, lo = pct(pre, 90), pct(healthy, 10)
        if not healthy or not pre:
            print("  -> not enough frames to judge")
        elif hi < lo:
            print("  -> SEPARABLE: put the threshold in (%.3f, %.3f), e.g. %.3f"
                  % (hi, lo, 0.5 * (hi + lo)))
        else:
            print("  -> NOT SEPARABLE: the pre-collapse band overlaps normal walking;")
            print("     this signal alone cannot gate GUIDANCE, a trend/margin term is needed")

    band("T_autonomy (T_guide fires when T_auto < threshold)", healthy_T, pre_T, 4.0)
    band("J_safe*    (J_guide fires when J* > threshold)", healthy_J, pre_J, 0.06)
    if healthy_Mg or pre_Mg:
        band("best_margin (a margin gate would fire when margin < threshold)",
             healthy_Mg, pre_Mg, float("nan"))
    else:
        print("\nbest_margin: not in this bag -- re-record after the margin publish")
    if healthy_Lt or pre_Lt:
        band("best_lateral_margin (a width gate would fire when it drops)",
             healthy_Lt, pre_Lt, float("nan"))
    else:
        print("best_lateral_margin: not in this bag")
    if argmin:
        print("\n2-D margin argmin, as a fraction of the evaluation horizon")
        print("  n=%-4d p10=%.2f p50=%.2f p90=%.2f" % (len(argmin), pct(argmin, 10),
                                                       pct(argmin, 50), pct(argmin, 90)))
        print("  near 0 or 1 => the 2-D minimum is a cell entry/exit face, not a")
        print("  lateral pinch, which is why that signal carries no width information")
    print("\nnote: GUIDANCE also needs safe_count > 0; frames with safe_count == 0")
    print("      fall into CAUTION first, so the threshold must fire before collapse.")

    old_raw_count = sum(1 for _, d in frames if old_trigger_raw(d))
    print("\nexact Phase A old raw trigger replay")
    print("  T_auto<4.0 && J_star>0.06 && safe_count>0: %d / %d frames" %
          (old_raw_count, len(frames)))
    schema_v3_report(frames)

    # Historical schema-v2 bags cannot reconstruct runtime Phase B evidence.
    # Give only a clearly labelled sensitivity proxy from the recorded target and
    # nearest sparse odometry segment bearing; never use it as an activation gate.
    if all(len(d) <= M["schema_version_v3"] for _, d in frames):
        print("\nOFFLINE PROXY / NOT RUNTIME-EQUIVALENT dynamic analysis")
        print("  Historical bag lacks exact LFPC heading, estimator config, and dynamic")
        print("  coverage evidence. Recorded selected heading plus sparse odometry yaw is")
        print("  insufficient to approve activation; a schema-v3 shadow bag is required.")
        proxy_ok = 0
        proxy_times = []
        for ts, d in frames:
            if len(d) <= M["selected_heading"]:
                continue
            pos_now = latest_before(odom, ts)
            pos_prev = latest_before(odom, ts - 0.2)
            if not pos_now or not pos_prev:
                continue
            dx, dy = pos_now[0] - pos_prev[0], pos_now[1] - pos_prev[1]
            # Historical pilot21 logs sparse odometry, but they do contain the
            # measured quaternion yaw. Use that yaw directly; displacement bearing
            # is only a fallback for older bags whose orientation is unavailable.
            current_heading = pos_now[2] if len(pos_now) > 2 and math.isfinite(pos_now[2]) \
                else (math.atan2(dy, dx) if math.hypot(dx, dy) >= 1e-4 else float("nan"))
            if not math.isfinite(current_heading):
                continue
            turn, ok = proxy_turn_time(current_heading, d[M["selected_heading"]])
            if ok:
                proxy_ok += 1
                proxy_times.append(turn)
        print("  proxy recoverable frames=%d; T_turn p50=%s p90=%s s" %
              (proxy_ok,
               "%.3f" % pct(proxy_times, 50) if proxy_times else "n/a",
               "%.3f" % pct(proxy_times, 90) if proxy_times else "n/a"))
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1:]))
