#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# M4.1 + M4.2 LTSSM extended states / LFPS full wiring: scenario sequencer.
#
# Design contract: notes/M4.1-M4.2-LTSSM扩态与LFPS全接线-设计钉死-2026-10-03.md (section 6).
# One simulation runs the scenario matrix serially. Each scenario is a list of phases; each
# phase has a time window (max(scaled spec time, unscaled floor) cycles), optional actions
# (LTSSM input pulses/levels + fault switches) and assertions (required/banned LTSSM states
# and outputs sampled inside the window). Per-scenario PASS/FAIL is printed, then the global
# verdict:
#   M41M42: ALL PASS / M41M42: FAIL
#
# All scenario/proxy delays are divided by timer_scale like the LTSSM timers, except the
# unscaled floors that cover the RxEQ TSEQ training time (65536 ordered sets, not scaled).

from migen import *
from migen.genlib.misc import WaitTimer
# Display/Finish come from the migen namespace (fhdl.structure): this migen version has no
# migen.genlib.sim module.

from litex.gen import *


# Scenario matrix -----------------------------------------------------------------------------------

# phase keys:
#   t     : spec seconds (divided by timer_scale) or None
#   floor : unscaled cycle floor (covers non-scaled TSEQ training time)
#   set   : {"side": {name: value}} level actions ("sys" for fault switches)
#   pulse : [(side, name, width)] 1-cycle pulse actions at phase entry
#   req   : ["h.State" | "d.out:tx_lau" | "h.Polling.*"] must be seen inside window
#   ban   : same, must NOT be seen inside window

SCENARIOS = [
    dict(sid="S0", name="baseline_regression", scales=(1, 100), phases=[
        dict(t=12e-3, floor=1_400_000,
             req=["h.Polling.Entry", "d.Polling.Entry", "h.Polling.LFPS", "d.Polling.LFPS",
                  "h.Polling.RxEQ", "d.Polling.RxEQ", "h.Polling.Active", "d.Polling.Active",
                  "h.Polling.Configuration", "d.Polling.Configuration",
                  "h.Polling.Idle", "d.Polling.Idle", "h.U0", "d.U0"],
             ban=["h.Recovery.Active", "d.Recovery.Active", "h.eSS.Inactive", "d.eSS.Inactive",
                  "h.Compliance", "d.Compliance", "h.Loopback", "d.Loopback",
                  "h.HotReset.Active", "d.HotReset.Active", "h.U1", "d.U1", "h.U2", "d.U2", "h.U3", "d.U3",
                  "h.Polling.ExitToRxDetect", "d.Polling.ExitToRxDetect", "h.WarmReset", "d.WarmReset"]),
    ]),
    dict(sid="S1", name="u0_u1_roundtrip", scales=(1, 100), phases=[
        dict(t=50e-6, floor=120_000, pulse=[("h", "req_u1", 2)],
             req=["h.U1", "d.U1", "d.out:tx_lau"],
             ban=["h.Recovery.Active", "d.Recovery.Active", "h.eSS.Inactive", "d.eSS.Inactive",
                  "h.U2", "d.U2", "h.U3", "d.U3"]),
        dict(t=30e-6, floor=60_000,
             req=["h.U1", "d.U1"],
             ban=["h.Polling.ExitToRxDetect", "d.Polling.ExitToRxDetect",
                  "h.Recovery.Active", "d.Recovery.Active", "h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(t=200e-6, floor=400_000, pulse=[("h", "exit_u1_req", 2)],
             req=["h.Recovery.Active", "d.Recovery.Active", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive", "h.Polling.ExitToRxDetect", "d.Polling.ExitToRxDetect"]),
    ]),
    dict(sid="S2", name="u0_u2_roundtrip", scales=(1, 100), phases=[
        dict(t=50e-6, floor=120_000, pulse=[("h", "req_u2", 2)],
             req=["h.U2", "d.U2"],
             ban=["h.U1", "d.U1", "h.Recovery.Active", "d.Recovery.Active", "h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(t=20e-6, floor=40_000, req=["h.U2", "d.U2"]),
        dict(t=600e-6, floor=400_000, pulse=[("h", "exit_u2_req", 2)],
             req=["h.Recovery.Active", "d.Recovery.Active", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    # scale=100 only (PORT_U2_TIMEOUT sandbox value 1ms -> 1330 cycles simulated).
    # Both sides degrade at nearly the same time (~1343/1351): the dev PM_ENTRY delay (8us)
    # scales to ~10 cycles, negligible vs the 1330-cycle U2-inactivity timeout. Phase 1 proves
    # no early degrade (ban U2 below timeout), phase 2 proves the direct U1->U2 transition
    # (no Recovery detour), phase 3 exits back to U0.
    dict(sid="S3", name="u1_to_u2_autodegrade", scales=(100,), phases=[
        dict(floor=1_000, set={"h": {"u2_inactivity_enable": 1}, "d": {"u2_inactivity_enable": 1}},
             pulse=[("h", "req_u1", 2)],
             req=["h.U1", "d.U1"],
             ban=["h.U2", "d.U2", "h.Recovery.Active", "d.Recovery.Active",
                  "h.eSS.Inactive", "d.eSS.Inactive"]), # negative: no early degrade
        dict(floor=2_000, set={"h": {"u2_inactivity_enable": 1}, "d": {"u2_inactivity_enable": 1}},
             req=["h.U2", "d.U2"],
             ban=["h.Recovery.Active", "d.Recovery.Active",
                  "h.eSS.Inactive", "d.eSS.Inactive"]), # direct U1->U2, no Recovery detour
        dict(floor=400_000, set={"h": {"u2_inactivity_enable": 0}, "d": {"u2_inactivity_enable": 0}},
             pulse=[("h", "exit_u2_req", 2)],
             req=["h.U0", "d.U0"], ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    dict(sid="S4", name="u3_roundtrip", scales=(1, 100), phases=[
        dict(t=50e-6, floor=120_000, pulse=[("h", "req_u3", 2)],
             req=["h.U3", "d.U3", "d.out:tx_lau"],       # negative: UFP must LAU (7.2.4.2.4)
             ban=["d.out:tx_lxu", "h.Recovery.Active", "d.Recovery.Active",
                  "h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(t=20e-6, floor=40_000, req=["h.U3", "d.U3"]),
        dict(t=3e-3, floor=600_000, pulse=[("h", "wakeup_u3_req", 2)],
             req=["h.Recovery.Active", "d.Recovery.Active", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    # scale=1 only (PM_LC_TIMER 4us at spec value).
    dict(sid="S5", name="pm_lc_timeout", scales=(1,), phases=[
        dict(floor=2_000, set={"sys": {"f_pm_h2d": 1, "f_pm_d2h": 1}}),
        dict(t=100e-6, floor=200_000, set={"sys": {"f_pm_h2d": 1, "f_pm_d2h": 1}},
             pulse=[("h", "req_u1", 2)],
             req=["h.Recovery.Active", "d.Recovery.Active", "h.U0", "d.U0"],
             ban=["h.U1", "d.U1", "h.eSS.Inactive", "d.eSS.Inactive"]), # negative: must not enter U1
        dict(floor=4_000, set={"sys": {"f_pm_h2d": 0, "f_pm_d2h": 0}}, req=["h.U0", "d.U0"]),
    ]),
    # scale=100 only (360ms tPollingLFPSTimeout).
    dict(sid="S6", name="compliance_roundtrip", scales=(100,), phases=[
        dict(floor=2_000, set={"sys": {"f_devpoll": 1}, "h": {"compliance_enable": 1}}),
        dict(t=2e-3, floor=400_000, set={"sys": {"f_devpoll": 1}, "h": {"compliance_enable": 1}},
             pulse=[("h", "warm_reset_req", 2)],
             req=["h.WarmReset", "d.WarmReset"]),
        dict(t=10e-3, floor=1_400_000, set={"sys": {"f_devpoll": 1}, "h": {"compliance_enable": 1}},
             req=["h.Compliance", "d.Compliance"],
             ban=["h.U0", "d.U0", "h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(t=6e-3, floor=1_500_000, set={"sys": {"f_devpoll": 0}}, pulse=[("h", "warm_reset_req", 2)],
             req=["h.U0", "d.U0"], ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(floor=2_000, set={"h": {"compliance_enable": 0}}, req=["h.U0", "d.U0"]),
    ]),
    # scale=1 only (tPollingIdleTimeout 2ms at spec value).
    dict(sid="S7", name="polling_idle_timeout", scales=(1,), phases=[
        dict(floor=2_000, set={"sys": {"f_idle8_h": 1, "f_idle8_d": 1}},
             pulse=[("h", "inject_polling", 2), ("d", "inject_polling", 2)]),
        dict(floor=1_300_000, set={"sys": {"f_idle8_h": 1, "f_idle8_d": 1}},
             req=["h.Polling.Idle", "d.Polling.Idle"],
             ban=["h.U0", "d.U0"]),
        dict(floor=800_000, set={"sys": {"f_idle8_h": 1, "f_idle8_d": 1}},
             req=["h.Polling.ExitToRxDetect", "d.Polling.ExitToRxDetect"], # 2ms timeout evidence
             ban=["h.U0", "d.U0"]),
        dict(floor=1_600_000, set={"sys": {"f_idle8_h": 0, "f_idle8_d": 0}},
             req=["h.U0", "d.U0"], ban=["h.Compliance", "d.Compliance"]),
    ]),
    # scale=100 only (Warm Reset burst 100ms).
    dict(sid="S8", name="warm_reset_roundtrip", scales=(100,), phases=[
        dict(t=3e-3, floor=1_500_000, pulse=[("h", "warm_reset_req", 2)],
             req=["h.WarmReset", "d.WarmReset", "h.U0", "d.U0"],
             ban=["h.Compliance", "d.Compliance"]),
    ]),
    # scale=100 only (tNoLFPSResponseTimeout U3 10ms + tU3WakeupRetryDelay 100ms).
    dict(sid="S9", name="u3_wakeup_no_response", scales=(100,), phases=[
        dict(t=50e-6, floor=120_000, pulse=[("h", "req_u3", 2)],
             req=["h.U3", "d.U3", "d.out:tx_lau"]),
        dict(t=200e-6, floor=40_000, set={"sys": {"f_wakeup_d": 1}}, pulse=[("h", "wakeup_u3_req", 2)],
             ban=["h.Recovery.Active", "h.Polling.ExitToRxDetect", "h.U0", "h.eSS.Inactive"]), # negative
        dict(t=5e-3, floor=600_000, set={"sys": {"f_wakeup_d": 0}},
             req=["h.Recovery.Active", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    # scale=1 only (Loopback exit burst 100us / tLoopbackExitTimeout 2ms at spec values).
    dict(sid="S10", name="loopback_roundtrip", scales=(1,), phases=[
        dict(t=1e-3, floor=600_000, set={"h": {"directed_loopback": 1, "loopback_capable": 1}},
             req=["h.Recovery.Active", "h.Loopback", "d.Loopback"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(t=20e-6, floor=40_000, set={"h": {"directed_loopback": 1, "loopback_capable": 1}},
             req=["h.Loopback", "d.Loopback"]),
        dict(t=2e-3, floor=1_400_000, set={"h": {"directed_loopback": 0}},
             req=["h.Loopback.Exit", "d.Loopback.Exit", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    dict(sid="S10b", name="loopback_exit_no_response", scales=(1,), phases=[
        dict(t=1e-3, floor=600_000, set={"h": {"directed_loopback": 1, "loopback_capable": 1}},
             req=["h.Loopback", "d.Loopback"]),
        dict(t=20e-3, floor=3_400_000, set={"h": {"directed_loopback": 0}, "sys": {"f_lbexit_d": 1}},
             req=["h.eSS.Inactive", "h.U0", "d.U0"]), # 2ms guard -> eSS.Inactive -> retrain
        dict(floor=2_000, set={"sys": {"f_lbexit_d": 0}}, req=["h.U0", "d.U0"]),
    ]),
    # scale=1 only (tHotResetActiveTimeout 12ms at spec value).
    dict(sid="S11", name="hot_reset_roundtrip", scales=(1,), phases=[
        dict(t=2e-3, floor=600_000, pulse=[("h", "hot_reset_req", 2)],
             req=["h.HotReset.Active", "d.HotReset.Active", "h.HotReset.Exit", "d.HotReset.Exit",
                  "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    dict(sid="S11b", name="hot_reset_timeout", scales=(1,), phases=[
        dict(t=30e-3, floor=4_800_000, set={"sys": {"f_ts2rst_d": 1}}, pulse=[("h", "hot_reset_req", 2)],
             req=["h.eSS.Inactive", "h.U0", "d.U0"],
             ban=["d.HotReset.Active"]), # negative: dev must not react
        dict(floor=2_000, set={"sys": {"f_ts2rst_d": 0}}, req=["h.U0", "d.U0"]),
    ]),
    # scale=1 only (Recovery timeouts 12/6/2ms at spec values).
    dict(sid="S12", name="recovery_triggered", scales=(1,), phases=[
        dict(t=2e-3, floor=600_000, pulse=[("h", "inject_ts1", 16)],
             req=["h.Recovery.Active", "h.Recovery.Idle", "d.Recovery.Idle", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
    ]),
    dict(sid="S12b", name="recovery_idle_timeout", scales=(1,), phases=[
        dict(floor=20_000, set={"sys": {"f_idle8_h": 1, "f_idle8_d": 1}}, pulse=[("h", "inject_ts1", 16)],
             req=["h.Recovery.Idle", "d.Recovery.Idle"]),
        dict(t=4e-3, floor=700_000, set={"sys": {"f_idle8_h": 1, "f_idle8_d": 1}},
             req=["h.eSS.Inactive", "d.eSS.Inactive"],
             ban=["h.U0", "d.U0"]),
        dict(t=20e-3, floor=3_400_000, set={"sys": {"f_idle8_h": 0, "f_idle8_d": 0}},
             req=["h.U0", "d.U0"]),
    ]),
    # scale=100 only (tU1PingTimeout 300ms / Ping repeat 200ms).
    dict(sid="S13", name="u1_ping", scales=(100,), phases=[
        dict(t=50e-6, floor=120_000, pulse=[("h", "req_u1", 2)], req=["h.U1", "d.U1"]),
        dict(t=50e-6, floor=40_000, req=["h.U1", "d.U1"],
             ban=["h.Polling.ExitToRxDetect"]), # Ping keeps DS tU1PingTimeout reset
    ]),
    dict(sid="S13b", name="u1_ping_lost", scales=(100,), phases=[
        dict(t=2e-3, floor=200_000, set={"sys": {"f_ping": 1}},
             ban=["h.Polling.ExitToRxDetect", "h.Polling.Entry", "h.U0"]), # negative: no early exit
        dict(t=8e-3, floor=1_600_000, set={"sys": {"f_ping": 1}},
             req=["h.Polling.ExitToRxDetect", "h.Polling.Entry", "h.U0", "d.U0"],
             ban=["h.eSS.Inactive", "d.eSS.Inactive"]),
        dict(floor=2_000, set={"sys": {"f_ping": 0}}, req=["h.U0", "d.U0"]),
    ]),
]

# States considered "idle handshake capable" by the rx_idle8 proxy (8 received + 16 sent model).
IDLE_CAPABLE = ["Polling.Idle", "U0", "HotReset.Exit", "Recovery.Idle"]

# Level outputs driven per side, default 0.
SIDE_LEVELS = ["directed_loopback", "loopback_capable", "compliance_enable", "u2_inactivity_enable"]
# Pulse outputs driven per side, default 0.
SIDE_PULSES = ["req_u1", "req_u2", "req_u3", "warm_reset_req", "hot_reset_req",
               "exit_u1_req", "exit_u2_req", "wakeup_u3_req", "directed_recovery",
               "inject_ts1", "inject_polling"]
# Fault switches (sys), default 0.
SYS_FAULTS  = ["f_pm_h2d", "f_pm_d2h", "f_idle8_h", "f_idle8_d",
               "f_lbexit_d", "f_wakeup_d", "f_ping", "f_devpoll", "f_ts2rst_d"]

# Sim-level gates derived from fault switches (applied in sim.py wiring).
FAULT_GATES = {
    "dev_tx_polling_en":  "f_devpoll",
    "dev_tx_ping_en":     "f_ping",
    "dev_rx_exit_u2_en":  "f_lbexit_d",
    "dev_rx_wakeup_u3_en": "f_wakeup_d",
}

class LTSSMScenarios(LiteXModule):
    """LTSSM scenario sequencer + proxies/fault switches (design section 6)."""
    def __init__(self, host, dev, sys_clk_freq, timer_scale=1):
        self.host, self.dev = host, dev

        scaled = lambda t: max(1, int(t*sys_clk_freq)//timer_scale)

        # Flatten phases, compute skip, window sizes.
        run        = lambda sc: timer_scale in sc["scales"]
        self.skip  = [sc["sid"] for sc in SCENARIOS if not run(sc)]
        phases     = []
        phase_sc   = [] # scenario index per phase
        for sc in SCENARIOS:
            for p in sc["phases"]:
                p = dict(p)
                if not run(sc):
                    p["t"], p["floor"], p["req"], p["ban"], p["set"], p["pulse"] = None, 2, [], [], {}, []
                window = 0
                if p.get("t") is not None:
                    window = scaled(p["t"])
                p["window"] = max(window, p.get("floor", 0))
                phases.append(p)
                phase_sc.append(sc)
        n_phases = len(phases)

        # Phase engine --------------------------------------------------------------------------------
        phase      = self.phase = Signal(bits_for(n_phases))
        pcnt       = self.pcnt = Signal(32)
        pmax       = Array([p["window"] for p in phases])
        finished   = self.finished = Signal()
        advance    = (pcnt + 1) >= pmax[phase]
        phase_bit  = self.phase_bit = Signal(n_phases, reset=1)

        self.sync += [
            pcnt.eq(pcnt + 1),
            If(advance,
                pcnt.eq(0),
                phase_bit.eq(phase_bit << 1),
                If(phase != (n_phases - 1),
                    phase.eq(phase + 1)
                ).Else(
                    finished.eq(1)
                )
            )
        ]

        # Outputs -----------------------------------------------------------------------------------
        # Per-side level/pulse outputs and fault switches, driven from per-phase value tables.
        level_values = {}
        pulse_phases = {}
        for side in ["h", "d"]:
            for name in SIDE_LEVELS:
                level_values[f"{side}.{name}"] = [0]*n_phases
            for name in SIDE_PULSES:
                pulse_phases[f"{side}.{name}"] = []
        for name in SYS_FAULTS:
            level_values[f"sys.{name}"] = [0]*n_phases
        for k, p in enumerate(phases):
            for side, sets in p.get("set", {}).items():
                for name, value in sets.items():
                    level_values[f"{side}.{name}"][k] = value
            for side, name, width in p.get("pulse", []):
                pulse_phases[f"{side}.{name}"].append((k, width))

        # Level outputs: comb from Array tables.
        for key, values in level_values.items():
            sig = Signal(name=key.replace(".", "_"))
            setattr(self, key.replace(".", "_"), sig)
            self.comb += sig.eq(Array(values)[self.phase])

        # Pulse outputs: 1 cycle (or width) at phase entry.
        for key, entries in pulse_phases.items():
            sig = Signal(name=key.replace(".", "_"))
            setattr(self, key.replace(".", "_"), sig)
            cond = 0
            for k, width in entries:
                cond = cond | ((self.phase == k) & (pcnt < width))
            self.comb += sig.eq(cond)

        # Proxies -------------------------------------------------------------------------------------
        def delayed(source, delay):
            # Pulse delay line (PM proxy delay model, scaled).
            pend = Signal(8)
            self.sync += [
                If(source,
                    pend.eq(delay)
                ).Elif(pend != 0,
                    pend.eq(pend - 1)
                )
            ]
            return pend == 1

        pm_delay = max(1, 8//timer_scale)
        # Host <- Dev (gated by f_pm_d2h).
        self.host_lau    = Signal(); self.host_lxu = Signal()
        self.comb += [
            self.host_lau.eq(delayed(dev.tx_lau, pm_delay) & ~self.sys_f_pm_d2h),
            self.host_lxu.eq(delayed(dev.tx_lxu, pm_delay) & ~self.sys_f_pm_d2h),
        ]
        for u in ["u1", "u2", "u3"]:
            sig = Signal(name=f"host_lgo_{u}")
            setattr(self, f"host_lgo_{u}", sig)
            self.comb += sig.eq(delayed(getattr(dev, f"tx_lgo_{u}"), pm_delay) & ~self.sys_f_pm_d2h)
        # Dev <- Host (gated by f_pm_h2d).
        self.dev_lau    = Signal(); self.dev_lxu = Signal()
        self.comb += [
            self.dev_lau.eq(delayed(host.tx_lau, pm_delay) & ~self.sys_f_pm_h2d),
            self.dev_lxu.eq(delayed(host.tx_lxu, pm_delay) & ~self.sys_f_pm_h2d),
        ]
        for u in ["u1", "u2", "u3"]:
            sig = Signal(name=f"dev_lgo_{u}")
            setattr(self, f"dev_lgo_{u}", sig)
            self.comb += sig.eq(delayed(getattr(host, f"tx_lgo_{u}"), pm_delay) & ~self.sys_f_pm_h2d)

        # rx_idle8 proxy: both sides idle-handshake-capable for N cycles (8 received + 16 sent model).
        idle8_thr = max(4, 48//timer_scale)
        enc_h, enc_d = host.fsm.encoding, dev.fsm.encoding
        def capable(fsm, enc):
            cond = 0
            for name in IDLE_CAPABLE:
                cond = cond | (fsm.state == enc[name])
            return cond
        idle8_cnt = Signal(16)
        self.sync += [
            If(capable(host.fsm, enc_h) & capable(dev.fsm, enc_d),
                idle8_cnt.eq(idle8_cnt + 1)
            ).Else(
                idle8_cnt.eq(0)
            )
        ]
        self.host_rx_idle8 = Signal()
        self.dev_rx_idle8  = Signal()
        self.comb += [
            self.host_rx_idle8.eq((idle8_cnt >= idle8_thr) & ~self.sys_f_idle8_h),
            self.dev_rx_idle8.eq((idle8_cnt >= idle8_thr) & ~self.sys_f_idle8_d),
        ]

        # TS2 bit-plane proxies (virtual interconnect, design section 4).
        self.host_rx_ts2_reset    = Signal(); self.dev_rx_ts2_reset    = Signal()
        self.host_rx_ts2_loopback = Signal(); self.dev_rx_ts2_loopback = Signal()
        self.comb += [
            self.host_rx_ts2_reset.eq(dev.tx_ts2_reset),
            self.dev_rx_ts2_reset.eq(host.tx_ts2_reset & ~self.sys_f_ts2rst_d),
            self.host_rx_ts2_loopback.eq(dev.tx_ts2_loopback),
            self.dev_rx_ts2_loopback.eq(host.tx_ts2_loopback),
        ]

        # Link activity: trigger 4 (tU0RecoveryTimeout) unexercised.
        self.host_link_activity = Signal(reset=1)
        self.dev_link_activity  = Signal(reset=1)

        # Assertion machinery ---------------------------------------------------------------------------
        # Tracks: state levels (all states, both sides) + output pulses ("out:xxx").
        tracks   = {}
        for side, ltssm in [("h", host), ("d", dev)]:
            enc = ltssm.fsm.encoding
            for name in enc:
                tracks[f"{side}.{name}"] = (ltssm.fsm.state == enc[name])
        for key in ["d.tx_lau", "d.tx_lxu", "h.tx_lau", "h.tx_lxu"]:
            side, name = key.split(".")
            ltssm = host if side == "h" else dev
            tracks[key.replace(name, f"out:{name}")] = getattr(ltssm, name)

        seen = {}
        for key, value in tracks.items():
            sig = Signal(n_phases, name=f"seen_{key.replace('.', '_').replace(':', '_')}")
            seen[key] = sig
            self.sync += [
                If(value,
                    sig.eq(sig | phase_bit)
                )
            ]

        # Per-phase check expressions (sticky: valid once the phase has run).
        ok_exprs = []
        for k, p in enumerate(phases):
            cond = 1
            for name in p.get("req", []):
                cond = cond & seen[name][k]
            for name in p.get("ban", []):
                cond = cond & ~seen[name][k]
            ok_exprs.append(cond)
        ok_current = Array(ok_exprs)[phase]

        # Scenario results & report -----------------------------------------------------------------------
        def phase_range(sc):
            indices = [k for k, s in enumerate(phase_sc) if s is sc]
            return indices[0], indices[-1] + 1

        # First failing phase registers. self.fail_phase is the run-wide first bad phase (global
        # 1-based flat index, kept for the summary line). fail_phase_<sid> is per scenario: the
        # scenario-local 1-based phase number of its OWN first failing phase (0 = none), so every
        # FAIL line reports a fresh value instead of a stale one from an earlier scenario.
        self.fail_phase = Signal(bits_for(n_phases + 1), reset=0)
        sc_first   = {}
        sc_nphases = {}
        sc_fail_phase = {}
        for sc in SCENARIOS:
            first, last = phase_range(sc)
            sc_first[sc["sid"]]   = first
            sc_nphases[sc["sid"]] = last - first
            sc_fail_phase[sc["sid"]] = Signal(bits_for(last - first + 1), reset=0,
                                              name=f"fail_phase_{sc['sid']}")
        self.sync += [
            If(advance & ~ok_current & (self.fail_phase == 0),
                self.fail_phase.eq(phase + 1)
            )
        ]
        for k, p in enumerate(phases):
            sc   = phase_sc[k]
            reg  = sc_fail_phase[sc["sid"]]
            lidx = k - sc_first[sc["sid"]] + 1 # scenario-local 1-based phase number
            self.sync += [
                If((phase == k) & advance & ~ok_current & (reg == 0),
                    reg.eq(lidx)
                )
            ]

        all_pass = 1
        start_displays = []
        result_displays = []
        for sc in SCENARIOS:
            first, last = phase_range(sc)
            sc_pass = 1
            for k in range(first, last):
                sc_pass = sc_pass & ok_exprs[k]
            all_pass = all_pass & sc_pass
            sid, name = sc["sid"], sc["name"]
            if run(sc):
                start_displays.append(
                    If((phase == first) & (pcnt == 0),
                        Display(f"[{sid}] {name}: start")))
                result_displays.append(
                    If(finished & sc_pass,
                        Display(f"[{sid}] {name}: PASS")))
                result_displays.append(
                    If(finished & ~sc_pass,
                        Display(f"[{sid}] {name}: FAIL (phase %d/%d of scenario, timer_scale=%d)",
                                sc_fail_phase[sid], sc_nphases[sid], timer_scale)))
            else:
                start_displays.append(
                    If((phase == first) & (pcnt == 0),
                        Display(f"[{sid}] {name}: SKIP (timer_scale=%d, scales={list(sc['scales'])})",
                                timer_scale)))

        skipped = " ".join(sc["sid"] for sc in SCENARIOS if not run(sc))
        self.sync += start_displays
        self.sync += result_displays
        self.sync += [
            If(finished,
                If(all_pass,
                    Display(f"M41M42: ALL PASS (timer_scale=%d, skipped: {skipped})", timer_scale)
                ).Else(
                    Display("M41M42: FAIL (timer_scale=%d, first bad phase %d)", timer_scale, self.fail_phase)
                ),
                Finish()
            )
        ]

        # sim.py wiring aliases: h_/d_ prefixed outputs also exposed as host_/dev_; fault switches
        # also exposed as active-high enable gates for the sim-level PIPE/LTSSM face gating.
        for pre, full in [("h_", "host_"), ("d_", "dev_")]:
            for name in SIDE_LEVELS + SIDE_PULSES:
                setattr(self, full + name, getattr(self, pre + name))
            for name in ["rx_idle8", "lgo_u1", "lgo_u2", "lgo_u3", "lau", "lxu",
                         "rx_ts2_reset", "rx_ts2_loopback", "link_activity"]:
                if hasattr(self, pre + name): # proxies are already host_/dev_ named
                    setattr(self, full + name, getattr(self, pre + name))
        for gate, fault in FAULT_GATES.items():
            setattr(self, gate, ~getattr(self, "sys_" + fault))
