#!/usr/bin/env python
# -*- coding: utf-8 -*-

# MIT License
#
# Copyright (c) 2018 BayLibre
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import re
from time import time

from regicecommon import open_resource
from regicepmu.pmu import PMU, PMUCounter
from regicepmu.perf import CPULoad, VendorEvent
from svd.extension import SVDExtension

class ARMCounter(PMUCounter):
    """
        Provide support of ARM PMU counters
    """
    def __init__(self, pmu, register):
        super(ARMCounter, self).__init__(pmu, register, support_event=True)
        ids = re.findall(r'\d+', self.register.name)
        if ids:
            self.cnt_id = int(ids[0])

    def _enable(self):
        self.pmu.peripheral.PMCNTENSET |= (1 << self.cnt_id)
        return True

    def _disable(self):
        self.pmu.peripheral.PMCNTENCLR |= (1 << self.cnt_id)

    def _enabled(self):
        return self.pmu.peripheral.PMCNTENSET & (1 << self.cnt_id)

    def _set_event(self, event_id):
        pmevtyper = getattr(self.pmu.peripheral, "PMXEVTYPER{}".format(self.cnt_id))
        pmevtyper.write(event_id)
        return True

class PMCCNTRCounter(ARMCounter):
    """
        Provide support for PMCCNTR counter which differ a little from the
        other counters.
    """
    def __init__(self, pmu):
        super(PMCCNTRCounter, self).__init__(pmu, pmu.peripheral.PMCCNTR)
        self.support_event = False
        self.cnt_id = 31

class ARMPMU(PMU):
    """
        Manage the ARM PMU
    """
    def __init__(self, device, peripheral, name):
        super(ARMPMU, self).__init__(device, name)
        self.peripheral = peripheral
        PMCCNTRCounter(self)
        for index in range(0, int(peripheral.PMCFGR.N)):
            register = getattr(peripheral, 'PMXEVCNTR{}'.format(index))
            ARMCounter(self, register)
        self.events = {
            0x00: ['SW_INCR', 'Instruction architecturally executed,'
                              ' condition code check pass, software increment'],
            0x01: ['L1I_CACHE_REFILL', 'Level 1 instruction cache refill'],
            0x02: ['L1I_TLB_REFILL', 'Level 1 instruction TLB refill'],
            0x03: ['L1D_CACHE_REFILL', 'Level 1 data cache refill'],
            0x04: ['L1D_CACHE', 'Level 1 data cache access'],
            0x05: ['L1D_TLB_REFILL', 'Level 1 data TLB refill'],
            0x06: ['LD_RETIRED', 'Instruction architecturally executed,'
                                 ' condition code check pass, load'],
            0x07: ['ST_RETIRED', 'Instruction architecturally executed,'
                                 ' condition code check pass, store'],
            0x08: ['INST_RETIRED', 'Instruction architecturally executed'],
            0x09: ['EXC_TAKEN', 'Exception taken'],
            0x0A: ['EXC_RETURN', 'Instruction architecturally executed,'
                                 ' condition code check pass, exception return'],
            0x0B: ['CID_WRITE_RETIRED', 'Instruction architecturally executed,'
                                        ' condition code check pass, write to CONTEXTIDR'],
            0x0C: ['PC_WRITE_RETIRED', 'Instruction architecturally executed,'
                                       ' condition code check pass, software change of the PC'],
            0x0D: ['BR_IMMED_RETIRED', 'Instruction architecturally executed, immediate branch'],
            0x0E: ['BR_RETURN_RETIRED', 'Instruction architecturally executed,'
                                        ' condition code check pass, procedure return'],
            0x0F: ['UNALIGNED_LDST_RETIRED', 'Instruction architecturally executed,'
                                             ' condition code check pass, unaligned load or store'],
            0x10: ['BR_MIS_PRED', 'Mispredicted or not predicted branch speculatively executed'],
            0x11: ['CPU_CYCLES', 'Cycle'],
            0x12: ['BR_PRED', 'Predictable branch speculatively executed'],
            0x13: ['MEM_ACCESS', 'Data memory access'],
            0x14: ['L1I_CACHE', 'Level 1 instruction cache access'],
            0x15: ['L1D_CACHE_WB', 'Level 1 data cache write-back'],
            0x16: ['L2D_CACHE', 'Level 2 data cache access'],
            0x17: ['L2D_CACHE_REFILL', 'Level 2 data cache refill'],
            0x18: ['L2D_CACHE_WB', 'Level 2 data cache write-back'],
            0x19: ['BUS_ACCESS', 'Bus access'],
            0x1A: ['MEMORY_ERROR', 'Local memory error'],
            0x1B: ['INST_SPEC', 'Instruction speculatively executed'],
            0x1C: ['TTBR_WRITE_RETIRED', 'Instruction architecturally executed,'
                                         ' condition code check pass, write to TTBR'],
            0x1D: ['BUS_CYCLES', 'Bus cycle'],
        }

    def _enable(self):
        self.peripheral.PMCR.E |= 1

    def _disable(self):
        self.peripheral.PMCR.E &= 0

    def _enabled(self):
        return self.peripheral.PMCR.E == 1

    def pause(self):
        pass

    def resume(self):
        pass

    def reset(self):
        pass

class ARMCPULoad(CPULoad):
    """
        Compute the CPU load using ARM PMU
    """
    def __init__(self, pmu, cpu_id, freq_min, freq_max):
        super(ARMCPULoad, self).__init__(pmu, cpu_id)
        self.freq_min = freq_min
        self.freq_max = freq_max
        self.time = 0
        self.pmccntr = pmu.get_counter('PMCCNTR')

    def _enable(self):
        self.time = time()
        self.pmccntr.enable()
        self.pmu.enable()

    def get_value(self):
        cpu_cycles = self.pmccntr.read_diff()
        now = time()
        tdiff = now - self.time
        self.time = now
        return (cpu_cycles / (tdiff * self.freq_max)) * 100

class ARMPerfEvent(VendorEvent):
    def __init__(self, pmu, cpu_id, event_id):
        name =  '{}-{}'.format(pmu.get_events()[event_id][0], cpu_id)
        super(ARMPerfEvent, self).__init__(pmu, name, event_id)
        self.event_id = event_id
        self.cntr = None
        self.time = 0

    def _enable(self):
        self.time = time()
        self.cntr = self.pmu.enable_event(self.event_id)

    def _disable(self):
        self.cntr = self.pmu.disable_event(self.cntr)

    def get_value(self):
        cntr = self.cntr.read_diff()
        now = time()
        tdiff = now - self.time
        self.time = now
        return cntr / (tdiff)

    def get_desc(self):
        return self.pmu.get_events()[self.event_id][1]

def device_add_pmu(device, svd_name, address, dim=1, dim_increment=0):
    properties = {
        'PMU%s': {
            'baseAddress': str(address),
            'dim': str(dim),
            'dimIncrement': str(dim_increment),
        }
    }
    file = open_resource(__name__, svd_name)
    svdext = SVDExtension(file.read())
    svdext.configure_peripherals(properties)
    svdext.parse()
    svdext.append_to(device.svd)
    device.update_peripherals()

def add_generic_events(pmu, cpu_id):
    for event_id in pmu.events:
        ARMPerfEvent(pmu, cpu_id, event_id)
