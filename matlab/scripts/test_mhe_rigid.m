function out = test_mhe_rigid(opts)
%TEST_MHE_RIGID Synthetic validation of dune.MheRigid (no hardware needed).
%   Simulates a rigid two-tag rover driving an arc over the REAL anchor
%   layout, generates noisy ranges from each tag in turn (asynchronous, as the
%   token ring does), and checks the MHE recovers the true centre and heading.
%   This validates the parameterisation signs, the rigid distance constraint,
%   the gyro->heading fusion and the terrain pitch geometry.
%
%   out = test_mhe_rigid            % flat ground
%   out = test_mhe_rigid(pitch=0.15) % 8.6 deg nose-up slope

arguments
    opts.baseline (1,1) double = 0.50
    opts.pitch (1,1) double = 0        % constant rig pitch (rad)
    opts.rangeSigma (1,1) double = 0.03
    opts.gyroSigma (1,1) double = 0.01
    opts.dur (1,1) double = 14         % s
    opts.rate (1,1) double = 6         % total sweeps/s (both tags interleaved)
    opts.casadiPath string = ""
    opts.plot (1,1) logical = true
end

dune.addCasadi(opts.casadiPath);
rng(7);
A = dune.loadAnchors();
L = opts.baseline; L2 = L/2;

% ---- truth: constant-speed arc (a unicycle path) -------------------------
R = 1.0; ctr = [0.5, 0.6]; w = 0.30;          % rad/s yaw rate
n = round(opts.dur * opts.rate);
t = (0:n-1)' / opts.rate;
a = -0.6 + w * t;
Ctrue = ctr + R * [cos(a), sin(a)];
Psi   = a + pi/2;                              % tangent heading (CCW)
spdTrue = R * w;

% ---- per-node tag geometry (terrain-aware, mirrors MheRigid) -------------
lh = L2 * cos(opts.pitch);
mr = dune.MheRigid(A.pos);
mr.baseline = L; mr.horizon = 12; mr.tagZ = 0.24;
mr.build();

isFront = mod(0:n-1, 2)' == 0;                 % alternate front/rear sweeps
Pose = nan(n, 4);
for i = 1:n
    sgn = 1; if ~isFront(i), sgn = -1; end
    tagXY = Ctrue(i,:) + sgn * lh * [cos(Psi(i)), sin(Psi(i))];
    tagZ  = 0.24 + sgn * L2 * sin(opts.pitch);
    % true ranges + noise
    d3 = vecnorm([tagXY, tagZ] - A.pos, 2, 2);
    z = d3 + opts.rangeSigma * randn(numel(d3), 1);
    wts = ones(numel(d3), 1);
    % standalone raw fix for this tag (seed/guard input), solved at tagZ
    pHint = dune.multilaterate(A.pos, z, weights=wts, tagZ=tagZ);
    dt = 1 / opts.rate;
    omega = w + opts.gyroSigma * randn;
    [pose, ~] = mr.push(z, wts, isFront(i), dt, pHint, false, omega, opts.pitch);
    Pose(i,:) = pose;
end

ok = ~isnan(Pose(:,1));
cErr = vecnorm(Pose(ok,1:2) - Ctrue(ok,:), 2, 2);
yawErr = abs(wrapToPi(Pose(ok,3) - Psi(ok)));
spdErr = abs(Pose(ok,4) - spdTrue);

fprintf('\n=== MheRigid synthetic test (L=%.2f m, pitch=%.1f deg, sigma_r=%.0f mm) ===\n', ...
    L, rad2deg(opts.pitch), 1000*opts.rangeSigma);
fprintf('nodes %d (%d solved), guard trips %d\n', n, sum(ok), mr.nJumps);
fprintf('centre error : median %5.1f mm | p95 %5.1f mm | max %5.1f mm\n', ...
    1000*median(cErr), 1000*prctile(cErr,95), 1000*max(cErr));
fprintf('heading error: median %5.2f deg | p95 %5.2f deg | max %5.2f deg\n', ...
    rad2deg(median(yawErr)), rad2deg(prctile(yawErr,95)), rad2deg(max(yawErr)));
fprintf('speed  error : median %5.1f mm/s (true %.0f mm/s)\n', ...
    1000*median(spdErr), 1000*spdTrue);

% Baseline is exact by construction - confirm the implied tag separation.
[pf, prr] = mr.tagPositions(Pose(find(ok,1,'last'),:), opts.pitch);
fprintf('implied horizontal tag separation: %.4f m (expected L*cos(pitch) = %.4f)\n', ...
    norm(pf - prr), L*cos(opts.pitch));

out = struct('Ctrue', Ctrue, 'Psi', Psi, 'Pose', Pose, ...
             'cErr', cErr, 'yawErr', yawErr);

if opts.plot
    fig = figure('Visible','off','Position',[0 0 1100 480]);
    tiledlayout(1,2,'TileSpacing','compact');
    nexttile; hold on; grid on; axis equal;
    plot(A.pos(:,1), A.pos(:,2), 'k^', 'MarkerFaceColor','y');
    plot(Ctrue(:,1), Ctrue(:,2), 'k-', 'LineWidth', 1.4, 'DisplayName','truth');
    plot(Pose(ok,1), Pose(ok,2), 'r.-', 'MarkerSize', 6, 'DisplayName','MHE rigid');
    legend('Location','best'); title('centre'); xlabel('x (m)'); ylabel('y (m)');
    nexttile; hold on; grid on;
    plot(t(ok), rad2deg(wrapToPi(Psi(ok))), 'k-', 'DisplayName','truth');
    plot(t(ok), rad2deg(wrapToPi(Pose(ok,3))), 'r.-', 'DisplayName','MHE');
    xlabel('t (s)'); ylabel('heading (deg)'); legend('Location','best'); title('yaw');
    f = fullfile(dune.rootDir(), 'results', 'mhe_rigid_test.png');
    if ~isfolder(fileparts(f)), mkdir(fileparts(f)); end
    saveas(fig, f); close(fig);
    fprintf('figure: %s\n', f);
end
end
