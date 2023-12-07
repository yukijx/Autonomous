from pytest import approx
from libs.utilities import Location

# threshold in degrees
BEARING_THRESHOLD = 1

# threshold in km
DISTANCE_THRESHOLD = 1

# coordinates of the EPF
EPF_LAT = 35.2103
EPF_LON = -97.4417

# coordinates of MDRS
MDRS_LAT = 38.4064
MDRS_LON = -110.7919

EXPECTED_DISTANCE = 1239.21  # Approximate distance in km

# 290 degrees is the approximate bearing from EPF to MDRS (calculated using https://www.movable-type.co.uk/scripts/latlong.html)
# but the function returns a bearing between -180 and 180 degrees so we need to subtract 360
EXPECTED_BEARING =  290 - 360  # Approximate bearing in degrees

def test_distance_to():
    actual_distance = Location.distance_to(EPF_LAT, EPF_LON, MDRS_LAT, MDRS_LON)
    assert approx(actual_distance, DISTANCE_THRESHOLD) == EXPECTED_DISTANCE

def test_calc_bearing():
    actual_bearing = Location.calc_bearing(EPF_LAT, EPF_LON, MDRS_LAT, MDRS_LON)
    assert approx(actual_bearing, abs=1) == EXPECTED_BEARING

def test_get_coordinates():
    # Test with known values
    actual_lat, actual_lon = Location.get_coordinates(EPF_LAT, EPF_LON, EXPECTED_DISTANCE, EXPECTED_BEARING)
    assert approx(actual_lat, 0.01) == MDRS_LAT
    assert approx(actual_lon, 0.01) == MDRS_LON