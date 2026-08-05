function pts = undistortPts(pts, cc)
    %UNDISTORTPTS Correct radial lens distortion on Nx2 clicked/detected pixels.
    %
    % pts_corrected = undistortPts(pts, config.camcal)
    %
    % cc is config.camcal from visionSystemConfig() (fitted by
    % calibrateOverheadCamera stage A). No-op when no calibration is loaded,
    % so callers can apply it unconditionally.
    %
    % Model: corrected = centre + (observed - centre) * (1 + k1*r^2 + k2*r^4),
    % r = |observed - centre| / norm_f. The polynomial is fitted directly in
    % the OBSERVED radius, so this direction is closed-form; distortPts()
    % (for drawing overlays on the raw image) is the iterative inverse.
    %
    % Valid ONLY in the fliplr'd pixel convention the whole pipeline uses.
    if nargin < 2 || ~isstruct(cc) || ~isfield(cc, 'available') || ~cc.available
        return;
    end
    dc = cc.dist_center(:)';
    d = (pts - dc) / cc.norm_f;
    r2 = sum(d.^2, 2);
    s = 1 + cc.k1 * r2 + cc.k2 * r2.^2;
    pts = dc + d .* s * cc.norm_f;
end
