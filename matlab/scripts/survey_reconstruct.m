function out = survey_reconstruct(captureFile)
%SURVEY_RECONSTRUCT Reconstruct anchor geometry from a self-survey and compare
%   to the clicked-Kinect anchors.json.
%
%   out = survey_reconstruct(captureFile)
%
%   The anchors range to each other over the air; those 10 pairwise distances
%   fix the constellation SHAPE with no camera involved. Classical MDS turns the
%   distances into positions, a rigid+reflection (NO-SCALE) fit lines them up
%   with anchors.json, and the residual is the disagreement.
%
%   Two comparisons:
%     RAW           - MDS(measured) vs Kinect. Delay-independent in geometry but
%                     dominated by the uncorrected antenna bias (~700 mm long),
%                     so it cannot indict anchors.json on its own.
%     DELAY-REMOVED - per-anchor bias b_i fit against Kinect (via survey_fit),
%                     subtracted, then re-reconstructed. The residual is the
%                     SHAPE-only disagreement.
%
%   What the residual can/cannot prove: the self-survey cannot pin absolute
%   geometry without the true delays - 5 additive bias DoF partially confound
%   with geometry perturbations. A LARGE residual is a genuine shape
%   disagreement (indicts the clicked geometry); a SMALL one only proves
%   consistency UP TO 5 free offsets (necessary, not sufficient, to validate
%   anchors.json).
%
%   No-scale is deliberate: a scaled fit would absorb the very global error we
%   are looking for.

arguments
    captureFile (1,1) string
end

A   = dune.loadAnchors();
ids = A.ids(:); n = numel(ids);
truth = A.pos(:,1:2);                      % clicked-Kinect ground truth (planar)

% --- parse to a full symmetric distance matrix ------------------------------
raw = readlines(captureFile);
D = nan(n);
for L = raw'
    t = split(L, ',');
    if numel(t) >= 6 && t(1) == "SURVEY" && t(2) == "v1"
        ia = find(ids == double(t(3)));    % address -> index, as survey_fit does
        ib = find(ids == double(t(4)));
        if isempty(ia) || isempty(ib), continue; end
        d = double(t(5)) / 1000;
        D(ia,ib) = d; D(ib,ia) = d;
    end
end
D(1:n+1:end) = 0;
missing = isnan(D) & ~eye(n);
if any(missing(:))
    [mi,mj] = find(triu(missing));
    pr = arrayfun(@(a,b) sprintf('A%d-A%d', ids(a), ids(b)), mi, mj, 'uni', 0);
    error('survey_reconstruct:incomplete', ...
          'Missing pair(s): %s. A weak/off anchor drops a pair.', strjoin(pr, ', '));
end

fprintf('\n=== survey_reconstruct: %s ===\n', captureFile);

% --- RAW: MDS on the measured ranges ----------------------------------------
[Xraw, evRaw] = mds2(D);
fprintf('\nMDS eigenvalues (raw): %s\n', mat2str(round(evRaw',1)));
fprintf('  3rd eigenvalue %.2f (near 0 = planar, internally consistent)\n', evRaw(3));
[XrawA, eRaw, Rr, tr, flipRaw] = alignRigidReflect(Xraw, truth);

% --- DELAY-REMOVED: per-anchor bias from survey_fit, then re-MDS ------------
% survey_fit fits measured_ij = kinect_ij + b_i + b_j against anchors.json and
% returns out.bias (per-anchor, m) and out.resid (per-pair). Reuse it verbatim.
sf = evalc_surveyfit(captureFile);
b  = sf.bias(:);                            % ordered by sf.ids
% reorder b to our index order (defensive; survey_fit uses the same loadAnchors)
bIdx = arrayfun(@(x) find(sf.ids == x), ids);
b = b(bIdx);

Dc = D;
for i = 1:n
    for j = i+1:n
        c = D(i,j) - (b(i) + b(j));
        Dc(i,j) = c; Dc(j,i) = c;
    end
end
Xcorr = mds2(Dc);
[XcorrA, eCorr] = alignRigidReflect(Xcorr, truth);

% --- report ------------------------------------------------------------------
fprintf('\nRAW alignment: rot %.1f deg, flip=%d, transl [%.2f %.2f] m\n', ...
        rad2deg(atan2(Rr(2,1),Rr(1,1))), flipRaw, tr);
fprintf('\n id     survey_raw (m)     delay_removed (m)    kinect (m)      raw_mm  corr_mm\n');
for i = 1:n
    fprintf(' A%d   [%+6.2f %+6.2f]   [%+6.2f %+6.2f]   [%+6.2f %+6.2f]  %6.0f  %6.0f\n', ...
        ids(i), XrawA(i,1), XrawA(i,2), XcorrA(i,1), XcorrA(i,2), ...
        truth(i,1), truth(i,2), 1000*eRaw(i), 1000*eCorr(i));
end
rmsRaw  = 1000*sqrt(mean(eRaw.^2));
rmsCorr = 1000*sqrt(mean(eCorr.^2));
fprintf('\n RAW           position RMS %.0f mm | max %.0f mm\n', rmsRaw, 1000*max(eRaw));
fprintf(' DELAY-REMOVED position RMS %.0f mm | max %.0f mm\n', rmsCorr, 1000*max(eCorr));

fprintf('\nper-anchor bias b_i removed (positive = board reads long):\n');
for i = 1:n, fprintf('   A%d  %+6.0f mm\n', ids(i), 1000*b(i)); end

% --- alignment-free pairwise-distance check ---------------------------------
fprintf('\nalignment-free pairwise error (survey vs kinect):\n');
de = [];
for i = 1:n
    for j = i+1:n
        dk = norm(truth(i,:) - truth(j,:));
        de(end+1) = D(i,j) - dk; %#ok<AGROW>
        fprintf('   A%d-A%d  survey %.3f  kinect %.3f  err %+6.0f mm\n', ...
                ids(i), ids(j), D(i,j), dk, 1000*(D(i,j)-dk));
    end
end
fprintf(' pairwise-distance RMS error: %.0f mm\n', 1000*sqrt(mean(de.^2)));

fprintf(['\nInterpretation: a LARGE delay-removed residual is a real shape ' ...
         'disagreement\n(indicts the clicked anchors.json). A SMALL one only ' ...
         'proves consistency up to\n5 additive offsets - necessary, not ' ...
         'sufficient, to validate the geometry.\n']);

% --- plot --------------------------------------------------------------------
fig = figure('Position',[60 60 760 680]); hold on; grid on; axis equal;
for i=1:n
    plot([truth(i,1) XrawA(i,1)],  [truth(i,2) XrawA(i,2)],  '-',  'Color',[.8 .8 .8]);
    plot([truth(i,1) XcorrA(i,1)], [truth(i,2) XcorrA(i,2)], '--', 'Color',[.7 .85 .7]);
end
h1 = plot(truth(:,1),  truth(:,2),  'ks', 'MarkerFaceColor','k', 'MarkerSize',9);
h2 = plot(XrawA(:,1),  XrawA(:,2),  'o',  'Color',[.85 .3 .3], 'MarkerSize',10, 'LineWidth',1.3);
h3 = plot(XcorrA(:,1), XcorrA(:,2), 'x',  'Color',[.2 .5 .2],  'MarkerSize',11, 'LineWidth',1.6);
text(truth(:,1)+0.06, truth(:,2), compose('A%d', ids));
legend([h1 h2 h3], {'Kinect (anchors.json)', ...
        sprintf('survey raw (RMS %.0f mm)', rmsRaw), ...
        sprintf('delay-removed (RMS %.0f mm)', rmsCorr)}, 'Location','best');
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Anchor self-survey vs Kinect - raw %.0f mm, shape %.0f mm', rmsRaw, rmsCorr));

f = replace(char(captureFile), '.txt', '_compare.png');
saveas(fig, f);
fprintf('\nfigure: %s\n', f);

out = struct('D', D, 'Draw_corrected', Dc, 'ids', ids, ...
             'Xraw', XrawA, 'Xcorr', XcorrA, 'truth', truth, ...
             'errRaw', eRaw, 'errCorr', eCorr, 'bias', b, ...
             'rmsRaw_mm', rmsRaw, 'rmsCorr_mm', rmsCorr, ...
             'pairErr_mm', 1000*de(:), 'figFile', f);
end

%% ── helpers ─────────────────────────────────────────────────────────────
function [X, ev] = mds2(D)
% Classical MDS -> planar embedding. Returns n x 2 coords and all eigenvalues.
n = size(D,1);
J = eye(n) - ones(n)/n;
B = -0.5 * J * (D.^2) * J;
[V,E] = eig((B+B')/2);
[ev, ord] = sort(diag(E), 'descend');
X = V(:,ord(1:2)) * diag(sqrt(max(ev(1:2), 0)));
end

function [Xa, err, R, t, didFlip] = alignRigidReflect(X, Y)
% Rigid + reflection Umeyama, NO SCALE. A UWB survey has unknown handedness, so
% try both flips and keep the better; scale is deliberately excluded so it
% cannot absorb a global size error.
best = inf; Xa = X; err = zeros(size(X,1),1); R = eye(2); t = [0 0]; didFlip = false;
for flip = [1 -1]
    Xf = X; Xf(:,2) = flip * Xf(:,2);
    mx = mean(Xf,1); my = mean(Y,1);
    [U,~,V] = svd((Xf-mx)' * (Y-my));
    Rr = V * diag([1, sign(det(V*U'))]) * U';
    tr = my - (Rr*mx')';
    Xar = (Rr*Xf')' + tr;
    e = vecnorm(Xar - Y, 2, 2);
    if sqrt(mean(e.^2)) < best
        best = sqrt(mean(e.^2));
        Xa = Xar; err = e; R = Rr; t = tr; didFlip = (flip == -1);
    end
end
end

function sf = evalc_surveyfit(captureFile)
% Call survey_fit for its per-anchor bias LS (it prints a table; suppress it).
evalc("sfout = survey_fit(captureFile);");
sf = sfout;
end
