from opendbc.car.tesla.preap.constants import (
  get_hands_on_disengage_level,
  parse_hands_on_level_param,
)


def test_legacy_zero_decodes_to_default_two():
  assert get_hands_on_disengage_level(0) == 2
  assert get_hands_on_disengage_level(8) == 2


def test_explicit_encoded_levels():
  assert get_hands_on_disengage_level(1 << 8) == 1
  assert get_hands_on_disengage_level(2 << 8) == 2
  assert get_hands_on_disengage_level(3 << 8) == 3
  assert get_hands_on_disengage_level(8 | (1 << 8)) == 1
  assert get_hands_on_disengage_level(8 | (3 << 8)) == 3


def test_parse_persisted_level():
  assert parse_hands_on_level_param(None) == 2
  assert parse_hands_on_level_param("") == 2
  assert parse_hands_on_level_param(b"") == 2
  assert parse_hands_on_level_param("0") == 2
  assert parse_hands_on_level_param(0) == 2
  assert parse_hands_on_level_param(4) == 2
  assert parse_hands_on_level_param("foo") == 2
  assert parse_hands_on_level_param("1") == 1
  assert parse_hands_on_level_param(2) == 2
  assert parse_hands_on_level_param(b"3") == 3


def test_malformed_level_does_not_change_the_safety_threshold():
  for invalid in (b"3\xff", 3.9, True):
    assert parse_hands_on_level_param(invalid) == 2
