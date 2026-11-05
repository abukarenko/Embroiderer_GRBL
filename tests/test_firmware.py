"""Execute the actual Thumb firmware routines with simulated GPIO and time.

Requires: python -m pip install unicorn pyelftools
Run after build.ps1: python tests/test_firmware.py [build/auto/Vyshivatel.elf]
This checks firmware logic, not the board's electrical characteristics.
"""
import pathlib
import struct
import sys
import unittest
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1

ELF = pathlib.Path(sys.argv.pop(1)) if len(sys.argv) > 1 else pathlib.Path(__file__).parents[1] / 'build/manual/Vyshivatel.elf'


class Board:
    def __init__(self):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for address, size in [(0x08000000, 0x40000), (0x20000000, 0x10000),
                              (0x40000000, 0x30000), (0xE000E000, 0x2000)]:
            self.uc.mem_map(address, size)
        with ELF.open('rb') as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            self.sizes = {s.name: s['st_size'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            for segment in elf.iter_segments():
                if segment['p_type'] == 'PT_LOAD':
                    self.uc.mem_write(segment['p_vaddr'], segment.data())
        self.tick = 0
        self.events = []
        self.polls = 0
        self.pwm_history = []
        self.planned = []
        self.mock_planner = False
        self.uc.hook_add(UC_HOOK_CODE, self.hook)

    def write(self, name, value):
        self.uc.mem_write(self.symbols[name], bytes([value]))

    def read(self, name):
        return self.uc.mem_read(self.symbols[name], self.sizes[name])

    def put32(self, address, value):
        self.uc.mem_write(address, struct.pack('<I', value))

    def get32(self, address):
        return struct.unpack('<I', self.uc.mem_read(address, 4))[0]

    def needle(self, down):
        self.put32(0x40010C08, 0x8000 if down else 0)

    def ret(self, value=0):
        self.uc.reg_write(UC_ARM_REG_R0, value)
        self.uc.reg_write(UC_ARM_REG_PC, self.uc.reg_read(UC_ARM_REG_LR))

    def hook(self, uc, address, size, data):
        def at(name):
            return address == (self.symbols.get(name, 0) & ~1)
        if at('HAL_GetTick'):
            self.ret(self.tick)
        elif any(at(n) for n in ['BoardTest_Task', 'grbl_serial_flush', 'grbl_platform_update_display']):
            self.ret()
        elif at('protocol_buffer_synchronize'):
            self.ret()
        elif at('plan_buffer_line') and self.mock_planner:
            self.planned.append((self.polls, bytes(uc.mem_read(uc.reg_read(UC_ARM_REG_R1), 12))))
            self.ret(1)
        elif at('protocol_execute_realtime'):
            self.polls += 1
            if self.events:
                self.events.pop(0)(self)
            elif self.polls > 1000:
                raise AssertionError('Firmware wait did not terminate')
            self.tick += 10
            self.pwm_history.append(self.get32(0x40012C34))
            # Substitute realtime service with the actual platform task.
            uc.reg_write(UC_ARM_REG_PC, self.symbols['grbl_platform_task'] | 1)

    def call(self, name, value=0, second=0):
        self.uc.reg_write(UC_ARM_REG_SP, 0x2000F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x0803F001)
        self.uc.reg_write(UC_ARM_REG_R0, value)
        self.uc.reg_write(UC_ARM_REG_R1, second)
        self.uc.emu_start(self.symbols[name] | 1, 0x0803F000, count=2000000)
        self.assert_finished()
        return self.uc.reg_read(UC_ARM_REG_R0)

    def assert_finished(self):
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x0803F000, 'Instruction limit reached'


def level(down):
    return lambda board: board.needle(down)


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        self.b = Board()

    def test_neutral_on_init_and_stop(self):
        b = self.b
        b.call('grbl_platform_spindle_init')
        self.assertEqual(b.get32(0x40012C34), 500)
        b.call('grbl_platform_spindle_set_pwm', 700)
        self.assertEqual(b.get32(0x40012C34), 700)
        b.call('spindle_stop')
        self.assertEqual(b.get32(0x40012C34), 500)

    def test_needle_polarity_uses_probe_inversion(self):
        b = self.b
        b.needle(1)
        b.write('probe_invert_mask', 0)
        self.assertEqual(b.call('embroidery_needle_down'), 1)
        b.write('probe_invert_mask', 1)
        self.assertEqual(b.call('embroidery_needle_down'), 0)
        b.needle(0)
        self.assertEqual(b.call('embroidery_needle_down'), 1)

    def test_one_fresh_cycle_per_stitch(self):
        b = self.b
        b.events = [level(0), level(0), level(1), level(1), level(0)]
        self.assertEqual(b.call('embroidery_wait_stitch'), 1)
        self.assertEqual(b.polls, 5)
        b.events = [level(0), level(0), level(0), lambda b: b.uc.mem_write(b.symbols['sys'] + 1, b'\x01')]
        self.assertEqual(b.call('embroidery_wait_stitch'), 0)
        self.assertEqual(b.polls, 9)

    def test_operator_pause_does_not_consume_edges(self):
        b = self.b
        b.events = [lambda b: b.write('sys', 16), level(1), level(0),
                    lambda b: b.write('sys', 0), level(0), level(1), level(0)]
        self.assertEqual(b.call('embroidery_wait_stitch'), 1)
        self.assertEqual(b.polls, 7)

    def test_needle_blocks_isr_without_changing_position(self):
        b = self.b
        b.needle(1)
        b.put32(0x40000010, 1)  # TIM2 UIF
        before = b.read('sys_position')
        stepper = b.read('st')
        b.call('TIM2_IRQHandler')
        self.assertEqual(b.read('sys_position'), before)
        self.assertEqual(b.read('st'), stepper)
        self.assertEqual(b.get32(0x40010C10), 0)  # No X step write
        self.assertEqual(b.get32(0x40011010), 0)  # No Y step write

    def test_trim_is_400_for_one_second_then_neutral(self):
        b = self.b
        b.call('embroidery_trim')
        self.assertEqual(b.tick, 1000)
        self.assertTrue(all(pwm == 400 for pwm in b.pwm_history))
        self.assertEqual(b.get32(0x40012C34), 500)

    def test_reset_during_trim_restores_neutral(self):
        b = self.b
        b.events = [lambda b: b.uc.mem_write(b.symbols['sys'] + 1, b'\x01')]
        b.call('embroidery_trim')
        self.assertEqual(b.get32(0x40012C34), 500)
        self.assertLess(b.tick, 1000)

    def test_feedhold_during_trim_restores_neutral(self):
        b = self.b
        b.events = [lambda b: b.write('sys', 16)]
        b.call('embroidery_trim')
        self.assertEqual(b.get32(0x40012C34), 500)
        self.assertLess(b.tick, 1000)

    def test_check_mode_does_not_trim(self):
        b = self.b
        b.write('sys', 2)
        b.call('embroidery_trim')
        self.assertEqual(b.polls, 0)

    def test_reject_unsupported_machine_commands(self):
        b = self.b
        for command in [b'G2X1I1', b'G1Z1F100', b'M4S650', b'G38.2X1F100', b'M6X1']:
            b.uc.mem_write(0x2000E000, command + b'\0')
            self.assertEqual(b.call('gc_execute_line', 0x2000E000), 20, command)

    def test_s_value_and_override_are_bounded(self):
        b = self.b
        b.uc.mem_write(b.symbols['sys'] + 9, b'\x64')
        for value, expected in [(0.0, 0), (1.0, 580), (650.0, 650), (1000.0, 860)]:
            bits = struct.unpack('<I', struct.pack('<f', value))[0]
            self.assertEqual(b.call('spindle_compute_pwm_value', bits), expected)
        b.uc.mem_write(b.symbols['sys'] + 9, b'\xc8')
        self.assertEqual(b.call('spindle_compute_pwm_value', struct.unpack('<I', struct.pack('<f', 650.0))[0]), 860)

    def test_auto_speed_boundaries_and_monotonicity(self):
        b = self.b
        if 'embroidery_speed_for_distance_squared' not in b.symbols:
            self.skipTest('Automatic speed is not linked in manual variant')
        values = []
        for distance in [0, 2, 3, 5, 8, 10, 12, 20]:
            bits = struct.unpack('<I', struct.pack('<f', float(distance * distance)))[0]
            values.append(b.call('embroidery_speed_for_distance_squared', bits))
        self.assertEqual(values[:2], [860, 860])
        self.assertEqual(values[-2:], [650, 650])
        self.assertEqual(values, sorted(values, reverse=True))

    def test_final_stitch_cycle_precedes_trim(self):
        b = self.b
        b.call('embroidery_stitch_complete')
        b.put32(0x40011008, 1 << 6)  # OUT1 enable
        b.events = [level(0), level(1), level(0)]
        b.call('embroidery_trim')
        self.assertEqual(b.polls, 103)
        self.assertTrue(all(pwm == 400 for pwm in b.pwm_history[3:]))
        self.assertEqual(b.get32(0x40012C34), 500)

    def motion(self, condition, speed=650):
        b = self.b
        b.call('plan_reset')
        b.mock_planner = True
        b.uc.mem_write(b.symbols['settings'], struct.pack('<4f', 100, 100, 100, 100))
        b.uc.mem_write(b.symbols['sys'] + 9, b'\x64')
        b.uc.mem_write(0x2000D000, struct.pack('<4f', 1, 2, 0, 0))
        b.uc.mem_write(0x2000D100, struct.pack('<ffB3x', 1000, speed, condition))
        b.call('mc_line', 0x2000D000, 0x2000D100)

    def test_stitch_enters_planner_only_after_needle_cycle(self):
        b = self.b
        b.events = [level(0), level(1), level(0)]
        self.motion(16)  # Spindle CW
        self.assertEqual(len(b.planned), 1)
        self.assertGreaterEqual(b.planned[0][0], 3)
        self.assertEqual(b.call('embroidery_hold_status'), 1)

    def test_jog_does_not_wait_for_a_stitch_or_drive_motor(self):
        b = self.b
        self.motion(4 | 16)  # No-feed-override marks jog, spindle CW
        self.assertEqual(len(b.planned), 1)
        condition = b.planned[0][1][8]
        self.assertEqual(condition & 48, 0)
        self.assertEqual(b.get32(0x40012C34), 500)

    def test_rapid_does_not_drive_motor(self):
        b = self.b
        self.motion(1 | 16)
        self.assertEqual(len(b.planned), 1)
        self.assertEqual(b.planned[0][1][8] & 48, 0)
        self.assertEqual(b.get32(0x40012C34), 500)


if __name__ == '__main__':
    unittest.main()
