function pts = distortPts(pts, cc)
    %DISTORTPTS Map ideal (undistorted) pixel points onto the raw image.
    %
    % pts_raw = distortPts(pts, config.camcal)
    %
    % Inverse of undistortPts(), used when DRAWING model-derived overlays
    % (grid lines, projected world points) on top of the raw camera frame.
    % Fixed-point iteration on the fitted polynomial; converges in a few
    % steps for the small distortion of this lens. No-op when no calibration
    % is loaded. NaNs (used as line-break markers by the overlays) pass
    % through untouched.
    if nargin < 2 || ~isstruct(cc) || ~isfield(cc, 'available') || ~cc.available
        return;
    end
    dc = cc.dist_center(:)';
    target = (pts - dc) / cc.norm_f;   % undistorted, normalized
    d = target;
    for it = 1:8
        r2 = sum(d.^2, 2);
        d = target ./ (1 + cc.k1 * r2 + cc.k2 * r2.^2);
    end
    pts = dc + d * cc.norm_f;
end
