//! This module contains code that works with the compact position format.
//!
//! Thanks to antirez! <https://github.com/antirez/dump1090>

pub fn cpr_nl_function(mut lat: f32) -> f32 {
    if lat < 0.0 {
        lat = -lat;
    }

    // Table is symmetric about the equator.
    if lat < 10.47047130 { return 59.0; }
    if lat < 14.82817437 { return 58.0; }
    if lat < 18.18626357 { return 57.0; }
    if lat < 21.02939493 { return 56.0; }
    if lat < 23.54504487 { return 55.0; }
    if lat < 25.82924707 { return 54.0; }
    if lat < 27.93898710 { return 53.0; }
    if lat < 29.91135686 { return 52.0; }
    if lat < 31.77209708 { return 51.0; }
    if lat < 33.53993436 { return 50.0; }
    if lat < 35.22899598 { return 49.0; }
    if lat < 36.85025108 { return 48.0; }
    if lat < 38.41241892 { return 47.0; }
    if lat < 39.92256684 { return 46.0; }
    if lat < 41.38651832 { return 45.0; }
    if lat < 42.80914012 { return 44.0; }
    if lat < 44.19454951 { return 43.0; }
    if lat < 45.54626723 { return 42.0; }
    if lat < 46.86733252 { return 41.0; }
    if lat < 48.16039128 { return 40.0; }
    if lat < 49.42776439 { return 39.0; }
    if lat < 50.67150166 { return 38.0; }
    if lat < 51.89342469 { return 37.0; }
    if lat < 53.09516153 { return 36.0; }
    if lat < 54.27817472 { return 35.0; }
    if lat < 55.44378444 { return 34.0; }
    if lat < 56.59318756 { return 33.0; }
    if lat < 57.72747354 { return 32.0; }
    if lat < 58.84763776 { return 31.0; }
    if lat < 59.95459277 { return 30.0; }
    if lat < 61.04917774 { return 29.0; }
    if lat < 62.13216659 { return 28.0; }
    if lat < 63.20427479 { return 27.0; }
    if lat < 64.26616523 { return 26.0; }
    if lat < 65.31845310 { return 25.0; }
    if lat < 66.36171008 { return 24.0; }
    if lat < 67.39646774 { return 23.0; }
    if lat < 68.42322022 { return 22.0; }
    if lat < 69.44242631 { return 21.0; }
    if lat < 70.45451075 { return 20.0; }
    if lat < 71.45986473 { return 19.0; }
    if lat < 72.45884545 { return 18.0; }
    if lat < 73.45177442 { return 17.0; }
    if lat < 74.43893416 { return 16.0; }
    if lat < 75.42056257 { return 15.0; }
    if lat < 76.39684391 { return 14.0; }
    if lat < 77.36789461 { return 13.0; }
    if lat < 78.33374083 { return 12.0; }
    if lat < 79.29428225 { return 11.0; }
    if lat < 80.24923213 { return 10.0; }
    if lat < 81.19801349 { return 9.0; }
    if lat < 82.13956981 { return 8.0; }
    if lat < 83.07199445 { return 7.0; }
    if lat < 83.99173563 { return 6.0; }
    if lat < 84.89166191 { return 5.0; }
    if lat < 85.75541621 { return 4.0; }
    if lat < 86.53536998 { return 3.0; }
    if lat < 87.00000000 { return 2.0; }
    1.0
}

pub fn cpr_mod_function(a: f32, b: f32) -> f32 {
    let res = a % b;
    if res < 0.0f32 {
        res + b
    } else {
        res
    }
}

pub fn cpr_n_function(lat: f32, isodd: f32) -> f32 {
    let nl = cpr_nl_function(lat) as f32 - isodd;
    if nl < 1.0 {
        1.0
    } else {
        nl
    }
}

pub fn cpr_dlon_function(lat: f32, isodd: f32) -> f32 {
    360.0 / cpr_n_function(lat, isodd)
}

/// Returns latitude and longitude or None if computation is not possible.
///
/// The `event` and `odd` format is (raw_latitude, raw_longitude, timestamp). The
/// timestamp ensures the calculation is done in the right order. The raw latitude
/// and longitude and straight from the decoding of the messages.
///
/// This is a copy of antirez's implementation.
pub fn decode_cpr(even: (u32, u32, u64), odd: (u32, u32, u64)) -> Option<(f32, f32)> {
    let air_dlat0: f32 = 360.0 / 60.0;
    let air_dlat1: f32 = 360.0 / 59.0;
    let lat0 = even.0 as f32;
    let lat1 = odd.0 as f32;
    let lon0 = even.1 as f32;
    let lon1 = odd.1 as f32;

    let j = (((59.0 * lat0 - 60.0 * lat1) / 131072.0) + 0.5).floor();
    let mut rlat0 = air_dlat0 * (cpr_mod_function(j, 60.0) + lat0 / 131072.0);
    let mut rlat1 = air_dlat1 * (cpr_mod_function(j, 59.0) + lat1 / 131072.0);  

    if rlat0 >= 270.0 {
        rlat0 -= 360.0;
    }

    if rlat1 >= 270.0 {
        rlat1 -= 360.0;
    }

    if cpr_nl_function(rlat0) != cpr_nl_function(rlat1) {
        return None;
    }

    if even.2 > odd.2 {
        let ni = cpr_n_function(rlat0, 0.0);
        let m = ((((lon0 * (cpr_nl_function(rlat0) - 1.0)) - (lon1 * cpr_nl_function(rlat0))) / 131072.0) + 0.5).floor();
        let mut lon = cpr_dlon_function(rlat0, 0.0) * (cpr_mod_function(m, ni) + lon0 / 131072.0);
        let lat = rlat0;
        if lon > 180.0 {
            lon -= 360.0;
        }
        Some((lat, lon))
    } else {
        let ni = cpr_n_function(rlat1, 1.0);
        let m = ((((lon0 * (cpr_nl_function(rlat1) - 1.0)) - (lon1 * cpr_nl_function(rlat1))) / 131072.0) + 0.5).floor();
        let mut lon = cpr_dlon_function(rlat1, 1.0) * (cpr_mod_function(m, ni) + lon1 / 131072.0);
        let lat = rlat1;
        if lon > 180.0 {
            lon -= 360.0;
        }        
        Some((lat, lon))
    }
}