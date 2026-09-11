"""px4_board pure helpers: params-file parsing, the byte-wise INT32 trap, shell command text."""

import importlib.util
import struct
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("px4_board", REPO / "scripts" / "px4_board.py")
board = importlib.util.module_from_spec(spec)
spec.loader.exec_module(board)

SAMPLE = """# tiltlab HITL set
# vehicle-id component-id name value type
1\t1\tCA_ROTOR0_AX\t0.25881904\t9
1\t1\tHIL_ACT_FUNC1\t101\t6
1\t1\tSYS_AUTOSTART\t1001\t6
"""


def test_parse_params_file_skips_comments_and_keeps_types():
    rows = board.parse_params_file(SAMPLE)
    assert rows == [("CA_ROTOR0_AX", 0.25881904, 9), ("HIL_ACT_FUNC1", 101.0, 6),
                    ("SYS_AUTOSTART", 1001.0, 6)]


def test_int32_travels_as_raw_bits_in_the_float_field():
    # 1120534528 is what the board showed for HIL_ACT_FUNC1 after a float write of 101.0: it is
    # the bit pattern of 101.0f read as an int32. Decoding a real INT32 must invert that.
    bits_of_101 = struct.unpack("<f", struct.pack("<i", 101))[0]
    assert board.decode_param_value(bits_of_101, board.INT32) == 101.0
    assert struct.unpack("<i", struct.pack("<f", 101.0))[0] == 1120534528
    assert board.decode_param_value(0.384, board.REAL32) == 0.384


def test_shell_commands_write_integers_as_integers_and_save():
    cmds = board.shell_commands(board.parse_params_file(SAMPLE))
    assert cmds[1] == "param set HIL_ACT_FUNC1 101"
    assert cmds[2] == "param set SYS_AUTOSTART 1001"
    assert cmds[0].startswith("param set CA_ROTOR0_AX 0.25881904")
    assert cmds[-1] == "param save"


def test_value_matches_tolerates_float32_rounding():
    assert board.value_matches(0.3840000033378601, 0.384)
    assert not board.value_matches(0.5, 0.384)
    assert not board.value_matches(None, 0.0)
