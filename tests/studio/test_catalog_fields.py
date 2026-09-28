"""The numeric limits sent to the page are the limits prepare() enforces."""

import unittest
from dbf_studio.analysis_bridge import FIELDS, SETTING_RANGES, field_ranges


class CatalogFields(unittest.TestCase):
    def test_ranges_cover_every_checked_setting(self):
        ranges = field_ranges()
        for key in [*FIELDS, *(key for key, _, _ in SETTING_RANGES), 'preflight']:
            self.assertIn(key, ranges)
            self.assertLess(ranges[key]['min'], ranges[key]['max'])
        self.assertTrue(ranges['segments']['integer'])
        self.assertFalse(ranges['speed']['integer'])
        self.assertEqual((ranges['speed']['min'], ranges['speed']['max']), FIELDS['speed'][1:])


if __name__ == '__main__':
    unittest.main()
