function llh = xyz2llh(x, y, z)

    % Constants
    R_0 = 6378137; % Equatorial Radius (m)
    e = 0.0818191908425; % Eccentricity

    % Initial Latitude
    lat = atan2(z, (1 - e^2)*vecnorm([x y]));
    lastLat = 0;
    diffLat = lat - lastLat;

    while abs(diffLat) > 1e-6

        % Radius of Curvature
        R_E = R_0 / sqrt(1-e^2*sin(lat)^2); % Transverse Radius of Curvature (m)

        % Height
        h = (vecnorm([x y])/cos(lat))- R_E;

        % Latitude
        lastLat = lat;
        lat = atan2(z, (1 - e^2*(R_E/(h+R_E)))*vecnorm([x y]));
        diffLat = lat - lastLat;

    end

    % Longitude 
    lon = atan2(y,x);
    llh = [lat lon h];

end