#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe.serdes import *
from usb3_pipe        import USB3PIPE
from usb3_core.core   import USB3Core
from usb3_ltssm       import USB3LTSSM

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    default_clk_name = "sys_clk"

    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Simulation Serializer/Deserializer Model ---------------------------------------------------------

class USB3SerDesModel(LiteXModule):
    def __init__(self, phy_dw=20, rx_word_shift=0):
        assert phy_dw in [20, 40]
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.tx     = stream.Endpoint([("data", phy_dw)])
        self.rx     = stream.Endpoint([("data", phy_dw)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()       # i
        self.tx_idle     = Signal()       # i
        self.tx_pattern  = Signal(phy_dw) # i

        self.rx_polarity = Signal() # i
        self.rx_idle     = Signal() # o
        self.rx_align    = Signal() # i

        # # #

        nwords = phy_dw//10

        # Control.
        self.comb += self.ready.eq(self.enable) # Ready when enabled

        # Datapath.
        tx_datapath = TXDatapath(phy_dw=nwords*8)
        rx_datapath = RXDatapath(phy_dw=nwords*8)
        self.submodules += tx_datapath, rx_datapath
        self.comb += [
            self.sink.connect(tx_datapath.sink),
            rx_datapath.word_aligner.enable.eq(self.rx_align),
            rx_datapath.source.connect(self.source)
        ]

        # 8b10b Encoders/Decoders.
        encoder  = Encoder(nwords, True)
        decoders = [Decoder(True) for _ in range(nwords)]
        self.submodules += encoder, decoders
        self.comb += tx_datapath.source.ready.eq(1)
        self.comb += rx_datapath.sink.valid.eq(1)
        for i in range(nwords):
            self.comb += [
                encoder.k[i].eq(tx_datapath.source.ctrl[i]),
                encoder.d[i].eq(tx_datapath.source.data[8*i:8*(i+1)]),
                rx_datapath.sink.ctrl[i].eq(decoders[i].k),
                rx_datapath.sink.data[8*i:8*(i+1)].eq(decoders[i].d),
            ]

        # Pattern/Polarity/Shift emulation.
        tx_data    = Signal(phy_dw)
        rx_data    = Signal(phy_dw)
        rx_data_sr = Signal(2*phy_dw)
        self.comb += [
            If(self.tx_pattern != 0,
                tx_data.eq(self.tx_pattern)
            ).Else(
                tx_data.eq(Cat(*[encoder.output[i] for i in range(nwords)])),
            ),
            If(self.tx_polarity,
                self.tx.data.eq(~tx_data)
            ).Else(
                self.tx.data.eq(tx_data)
            )
        ]
        self.comb += [
            If(self.rx_polarity,
                rx_data.eq(~self.rx.data)
            ).Else(
                rx_data.eq(self.rx.data)
            )
        ]
        self.sync += rx_data_sr.eq(Cat(rx_data, rx_data_sr))
        for i in range(nwords):
            self.comb += decoders[i].input.eq(rx_data_sr[10*(rx_word_shift+i):10*(rx_word_shift+i+1)])

    def connect(self, serdes):
        self.comb += [
            self.tx.connect(serdes.rx),
            serdes.tx.connect(self.rx),
            self.rx_idle.eq(serdes.tx_idle),
            serdes.rx_idle.eq(self.tx_idle),
        ]

# USB3PIPESim --------------------------------------------------------------------------------------

class USB3PIPESim(SoCMini):
    def __init__(self, phy_dw=20, timer_scale=1, with_scenarios=False):
        sys_clk_freq = int(133e6)

        # Platform.
        platform = Platform()
        self.comb += platform.trace.eq(1)

        # SoC Mini.
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # USB3 Host.
        host_usb3_serdes = USB3SerDesModel(phy_dw=phy_dw)
        host_usb3_pipe   = USB3PIPE(
            serdes       = host_usb3_serdes,
            sys_clk_freq = sys_clk_freq,
            timer_scale  = timer_scale)
        self.submodules += host_usb3_serdes, host_usb3_pipe

        # USB3 Host LTSSM.
        host_usb3_ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale)
        self.submodules += host_usb3_ltssm
        self.comb += host_usb3_ltssm.ds_port.eq(1) # Host = Downstream Port.
        self.comb += self.wire_ltssm_pipe(host_usb3_ltssm, host_usb3_pipe)
        host_usb3_ltssm.finalize()

        host_usb3_core = USB3Core(platform, daisho_core="daisho_mod")
        self.host_usb3_core = host_usb3_core
        self.comb += [
            host_usb3_serdes.tx_polarity.eq(1), # Inverse TX polarity to test RX auto-polarity
            host_usb3_pipe.source.connect(host_usb3_core.sink),
            host_usb3_core.source.connect(host_usb3_pipe.sink),
            host_usb3_core.reset.eq(~host_usb3_ltssm.u0),
        ]

        # USB3 Device.
        dev_usb3_serdes = USB3SerDesModel(phy_dw=phy_dw)
        dev_usb3_pipe   = USB3PIPE(
            serdes       = dev_usb3_serdes,
            sys_clk_freq = sys_clk_freq,
            timer_scale  = timer_scale)
        self.submodules += dev_usb3_serdes, dev_usb3_pipe
        dev_usb3_pipe.finalize()

        # USB3 Device LTSSM.
        dev_usb3_ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale)
        self.submodules += dev_usb3_ltssm
        self.comb += dev_usb3_ltssm.ds_port.eq(0) # Device = Upstream Port.
        self.comb += self.wire_ltssm_pipe(dev_usb3_ltssm, dev_usb3_pipe)
        dev_usb3_ltssm.finalize()

        dev_usb3_core = USB3Core(platform, daisho_core="daisho_mod")
        self.dev_usb3_core = dev_usb3_core
        self.comb += [
            dev_usb3_serdes.tx_polarity.eq(1), # Inverse TX polarity to test RX auto-polarity
            dev_usb3_pipe.source.connect(dev_usb3_core.sink),
            dev_usb3_core.source.connect(dev_usb3_pipe.sink),
            dev_usb3_core.reset.eq(~dev_usb3_ltssm.u0),
        ]

        # Connect Host <--> Device.
        host_usb3_serdes.connect(dev_usb3_serdes)

        # LTSSM lower-layer inputs: scenario sequencer proxies or M4.0 defaults.
        if with_scenarios:
            from sim_ltssm_scenario import LTSSMScenarios
            self.scenarios = scenarios = LTSSMScenarios(
                host         = host_usb3_ltssm,
                dev          = dev_usb3_ltssm,
                sys_clk_freq = sys_clk_freq,
                timer_scale  = timer_scale)
            self.comb += self.wire_scenarios(scenarios, host_usb3_ltssm, dev_usb3_ltssm,
                                             host_usb3_pipe, dev_usb3_pipe)
        else:
            # M4.0 baseline behaviour: idle handshake pre-completed, no PM/scenario activity.
            for ltssm in [host_usb3_ltssm, dev_usb3_ltssm]:
                self.comb += [
                    ltssm.rx_idle8.eq(1),
                    ltssm.link_activity.eq(1),
                ]

        # Simulation Timer.
        timer = Signal(32)
        self.sync += timer.eq(timer + 1)

        # Simulation Status.
        for pipe, fsm in [
            ["host",  host_usb3_ltssm.fsm],
            ["dev ",  dev_usb3_ltssm.fsm]]:
            for state, value in fsm.encoding.items():
                self.sync += [
                    If(fsm.next_state != fsm.state,
                        If(fsm.next_state == value,
                            Display("[%08d] {} entering {} state".format(pipe.upper(), state), timer)
                        )
                    )
                ]

        # Simulation End.
        # M4.0 baseline: Finish on dual U0 (2^16 cycles). Scenario mode: sequencer owns the end.
        # (Scenario mode: rx_idle8/link_activity are driven exclusively by the sequencer proxies
        # in wire_scenarios() -- do NOT add the M4.0 default drives here: multiple comb drivers
        # compile to last-assignment-wins blocking assigns and would override the fault switches.)
        if not with_scenarios:
            end_timer = WaitTimer(2**16)
            self.submodules += end_timer
            self.comb += end_timer.wait.eq(host_usb3_ltssm.u0 & dev_usb3_ltssm.u0)
            self.sync += If(end_timer.done, Finish())

    @staticmethod
    def wire_ltssm_pipe(ltssm, pipe):
        """LTSSM <-> PIPE wiring (LFPS/TS face, design section 3)."""
        return [
            # LTSSM -> PIPE
            pipe.lfps_tx_polling.eq(ltssm.lfps_tx_polling),
            pipe.lfps_tx_idle.eq(ltssm.lfps_tx_idle),
            pipe.lfps_tx_ping.eq(ltssm.lfps_tx_ping),
            pipe.lfps_tx_exit_u1.eq(ltssm.lfps_tx_exit_u1),
            pipe.lfps_tx_exit_u2.eq(ltssm.lfps_tx_exit_u2),
            pipe.lfps_tx_wakeup_u3.eq(ltssm.lfps_tx_wakeup_u3),
            pipe.lfps_tx_reset.eq(ltssm.lfps_tx_reset),
            pipe.ts_rx_enable.eq(ltssm.ts_rx_enable),
            pipe.ts_tx_enable.eq(ltssm.ts_tx_enable),
            pipe.ts_tx_tseq.eq(ltssm.ts_tx_tseq),
            pipe.ts_tx_ts1.eq(ltssm.ts_tx_ts1),
            pipe.ts_tx_ts2.eq(ltssm.ts_tx_ts2),

            pipe.serdes_rx_align.eq(ltssm.serdes_rx_align),
            pipe.serdes_rx_polarity.eq(ltssm.serdes_rx_polarity),

            pipe.rx_ready.eq(ltssm.rx_ready),
            pipe.tx_ready.eq(ltssm.tx_ready),

            # PIPE -> LTSSM
            ltssm.lfps_rx_polling.eq(pipe.lfps_rx_polling),
            ltssm.lfps_tx_count.eq(pipe.lfps_tx_count),
            ltssm.lfps_rx_ping.eq(pipe.lfps_rx_ping),
            ltssm.lfps_rx_exit_u1.eq(pipe.lfps_rx_exit_u1),
            ltssm.lfps_rx_exit_u2.eq(pipe.lfps_rx_exit_u2),
            ltssm.lfps_rx_wakeup_u3.eq(pipe.lfps_rx_wakeup_u3),
            ltssm.lfps_rx_reset.eq(pipe.lfps_rx_reset),

            ltssm.ts_rx_ts1.eq(pipe.ts_rx_ts1),
            ltssm.ts_rx_ts1_inv.eq(pipe.ts_rx_ts1_inv),
            ltssm.ts_rx_ts2.eq(pipe.ts_rx_ts2),
            ltssm.ts_tx_done.eq(pipe.ts_tx_done),
        ]

    @staticmethod
    def wire_scenarios(sc, host, dev, host_pipe, dev_pipe):
        """Scenario sequencer -> LTSSM inputs (proxies, fault switches, injections)."""
        wiring = []
        # Host side.
        wiring += [
            host.req_u1.eq(sc.host_req_u1),
            host.req_u2.eq(sc.host_req_u2),
            host.req_u3.eq(sc.host_req_u3),
            host.lgo_u1.eq(sc.host_lgo_u1),
            host.lgo_u2.eq(sc.host_lgo_u2),
            host.lgo_u3.eq(sc.host_lgo_u3),
            host.lau.eq(sc.host_lau),
            host.lxu.eq(sc.host_lxu),
            host.rx_idle8.eq(sc.host_rx_idle8),
            host.rx_ts2_reset.eq(sc.host_rx_ts2_reset),
            host.rx_ts2_loopback.eq(sc.host_rx_ts2_loopback),
            host.directed_recovery.eq(sc.host_directed_recovery),
            host.directed_loopback.eq(sc.host_directed_loopback),
            host.loopback_capable.eq(sc.host_loopback_capable),
            host.compliance_enable.eq(sc.host_compliance_enable),
            host.hot_reset_req.eq(sc.host_hot_reset_req),
            host.warm_reset_req.eq(sc.host_warm_reset_req),
            host.exit_u1_req.eq(sc.host_exit_u1_req),
            host.exit_u2_req.eq(sc.host_exit_u2_req),
            host.wakeup_u3_req.eq(sc.host_wakeup_u3_req),
            host.u2_inactivity_enable.eq(sc.host_u2_inactivity_enable),
            host.link_activity.eq(sc.host_link_activity),
            host.lfps_rx_polling.eq(host_pipe.lfps_rx_polling | sc.host_inject_polling),
            host.ts_rx_ts1.eq(host_pipe.ts_rx_ts1 | sc.host_inject_ts1),
        ]
        # Device side.
        wiring += [
            dev.req_u1.eq(sc.dev_req_u1),
            dev.req_u2.eq(sc.dev_req_u2),
            dev.req_u3.eq(sc.dev_req_u3),
            dev.lgo_u1.eq(sc.dev_lgo_u1),
            dev.lgo_u2.eq(sc.dev_lgo_u2),
            dev.lgo_u3.eq(sc.dev_lgo_u3),
            dev.lau.eq(sc.dev_lau),
            dev.lxu.eq(sc.dev_lxu),
            dev.rx_idle8.eq(sc.dev_rx_idle8),
            dev.rx_ts2_reset.eq(sc.dev_rx_ts2_reset),
            dev.rx_ts2_loopback.eq(sc.dev_rx_ts2_loopback),
            dev.directed_recovery.eq(sc.dev_directed_recovery),
            dev.directed_loopback.eq(sc.dev_directed_loopback),
            dev.loopback_capable.eq(sc.dev_loopback_capable),
            dev.compliance_enable.eq(sc.dev_compliance_enable),
            dev.hot_reset_req.eq(sc.dev_hot_reset_req),
            dev.warm_reset_req.eq(sc.dev_warm_reset_req),
            dev.exit_u1_req.eq(sc.dev_exit_u1_req),
            dev.exit_u2_req.eq(sc.dev_exit_u2_req),
            dev.wakeup_u3_req.eq(sc.dev_wakeup_u3_req),
            dev.u2_inactivity_enable.eq(sc.dev_u2_inactivity_enable),
            dev.link_activity.eq(sc.dev_link_activity),
            dev.lfps_rx_polling.eq(dev_pipe.lfps_rx_polling | sc.dev_inject_polling),
        ]
        # Fault switches (sim-level gates on PIPE/LTSSM faces).
        wiring += [
            # S6: dev silent (no Polling.LFPS transmission).
            dev_pipe.lfps_tx_polling.eq(dev.lfps_tx_polling & sc.dev_tx_polling_en),
            # S13: UFP Ping disabled.
            dev_pipe.lfps_tx_ping.eq(dev.lfps_tx_ping & sc.dev_tx_ping_en),
            # S10b: Loopback exit LFPS not seen by dev.
            dev.lfps_rx_exit_u2.eq(dev_pipe.lfps_rx_exit_u2 & sc.dev_rx_exit_u2_en),
            # S9: U3 Wakeup LFPS not seen by dev.
            dev.lfps_rx_wakeup_u3.eq(dev_pipe.lfps_rx_wakeup_u3 & sc.dev_rx_wakeup_u3_en),
        ]
        return wiring

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PIPE Simulation")
    parser.add_argument("--trace",        action="store_true", help="Enable VCD tracing.")
    parser.add_argument("--trace-start",  default=0,           help="Cycle to start VCD tracing.")
    parser.add_argument("--trace-end",    default=-1,          help="Cycle to end VCD tracing.")
    parser.add_argument("--timer-scale",  default=1,           help="LTSSM/LFPS timers scale factor (1 = spec exact).")
    parser.add_argument("--scenario",     action="store_true", help="Run the M4.1/M4.2 LTSSM scenario matrix.")
    parser.add_argument("--only",         default=None,        help="Comma-separated scenario ids to keep (e.g. --only S1,S3). Requires --scenario.")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
    os.system("cp usb3_core/daisho/usb3/*.init build/sim/gateware/")

    if args.only is not None:
        # Debug/probe support: restrict the scenario matrix before the sequencer is built.
        import sim_ltssm_scenario as _scen
        keep = set(args.only.split(","))
        _scen.SCENARIOS = [sc for sc in _scen.SCENARIOS if sc["sid"] in keep]
        args.scenario = True

    soc = USB3PIPESim(timer_scale=int(args.timer_scale), with_scenarios=args.scenario)
    builder = Builder(soc)
    builder.build(sim_config=sim_config,
        opt_level   = "O0",
        interactive = False,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))

if __name__ == "__main__":
    main()
