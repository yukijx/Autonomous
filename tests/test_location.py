import pytest
from pytest import approx
from libs.utilities import distance_to, calc_bearing, get_coordinates, calc_average_bearing

@pytest.fixture
def location_test():
    class LocationTest:
        # threshold in degrees
        BEARING_THRESHOLD = 1

        # threshold in km
        DISTANCE_THRESHOLD = 1

        # threshold in degrees
        # need to be pretty coarse since the other values
        # are only accurate to a few decimal places
        COORDINATE_THRESHOLD = 0.1

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

    return LocationTest()

def test_distance_to(location_test):
    epf_lat = location_test.EPF_LAT
    epf_lon = location_test.EPF_LON
    mdrs_lat = location_test.MDRS_LAT
    mdrs_lon = location_test.MDRS_LON
    expected_distance = location_test.EXPECTED_DISTANCE
    distance_threshold = location_test.DISTANCE_THRESHOLD
    actual_distance = distance_to(epf_lat, epf_lon, mdrs_lat, mdrs_lon)
    assert approx(actual_distance, distance_threshold) == expected_distance

def test_calc_bearing(location_test):
    epf_lat = location_test.EPF_LAT
    epf_lon = location_test.EPF_LON
    mdrs_lat = location_test.MDRS_LAT
    mdrs_lon = location_test.MDRS_LON
    expected_bearing = location_test.EXPECTED_BEARING
    bearing_threshold = location_test.BEARING_THRESHOLD
    actual_bearing = calc_bearing(epf_lat, epf_lon, mdrs_lat, mdrs_lon)
    assert approx(actual_bearing, bearing_threshold) == expected_bearing

def test_get_coordinates(location_test):
    epf_lat = location_test.EPF_LAT
    epf_lon = location_test.EPF_LON
    mdrs_lat = location_test.MDRS_LAT
    mdrs_lon = location_test.MDRS_LON
    expected_bearing = location_test.EXPECTED_BEARING
    expected_distance = location_test.EXPECTED_DISTANCE
    coordinate_threshold = location_test.COORDINATE_THRESHOLD
    # Test with known values
    actual_lat, actual_lon = get_coordinates(epf_lat, epf_lon, expected_distance, expected_bearing)
    assert approx(actual_lat, coordinate_threshold) == mdrs_lat
    assert approx(actual_lon, coordinate_threshold) == mdrs_lon

def test_calc_average_bearing():
    bearings_1 = [120, 140, 160, 180]
    expected_bearing_1 = 150
    actual_bearing_1 = calc_average_bearing(bearings_1)
    assert approx(actual_bearing_1, abs=1) == expected_bearing_1

    bearings_2 = [-120, -140, -160, -180]
    expected_bearing_2 = -150
    actual_bearing_2 = calc_average_bearing(bearings_2)
    assert approx(actual_bearing_2, abs=1) == expected_bearing_2

    bearings_3 = [140, 160, 180, -160]
    expected_bearing_3 = 170
    actual_bearing_3 = calc_average_bearing(bearings_3)
    assert approx(actual_bearing_3, abs=1) == expected_bearing_3

    bearings_4= [160, 180, -160, -140]
    expected_bearing_4 = -170
    actual_bearing_4 = calc_average_bearing(bearings_4)
    assert approx(actual_bearing_4, abs=1) == expected_bearing_4

    bearings_5 = [0, 20, 40, 60]
    expected_bearing_5 = 30
    actual_bearing_5 = calc_average_bearing(bearings_5)
    assert approx(actual_bearing_5, abs=1) == expected_bearing_5

    bearings_6 = [0, -20, -40, -60]
    expected_bearing_6 = -30
    actual_bearing_6 = calc_average_bearing(bearings_6)
    assert approx(actual_bearing_6, abs=1) == expected_bearing_6

    bearings_7 = [0, 20, 40, -40]
    expected_bearing_7 = 5
    actual_bearing_7 = calc_average_bearing(bearings_7)
    assert approx(actual_bearing_7, abs=1) == expected_bearing_7