#pragma once

struct GeoOrigin {
    double lat     = 0.0;
    double lon     = 0.0;
    double alt     = 0.0;
    double cos_lat = 1.0;

    GeoOrigin() = default;
    GeoOrigin(double lat, double lon, double alt)
        : lat(lat), lon(lon), alt(alt), cos_lat(1.0) {}
};
