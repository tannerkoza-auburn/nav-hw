function [Rbe_e] = llh2xyz(lat, lon, h)

    % Constants
    R_0 = 6378137.0; % Equatorial Radius (m)
    e = 0.0818191908425; % Eccentricity

    % Radius of Curvature
    R_E = R_0 / (sqrt(1-e^2*sin(lat)^2)); % Transverse Radius of Curvature (m)

    % LLH to ECEF
    x = (R_E + h)*cos(lat)*cos(lon);
    y = (R_E + h)*cos(lat)*sin(lon);
    z = ((1-e^2)*R_E + h)*sin(lat);

    Rbe_e = [x y z];

end