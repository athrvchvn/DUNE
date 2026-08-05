function sketch_geometry()
%SKETCH_GEOMETRY Draw the anchor layout with all pairwise distances.
A = dune.loadAnchors();
n = numel(A.ids);
fig = figure('Visible','off', 'Position', [0 0 1000 640]);
hold on

% Testbed rectangle (anchor rectangle 6.3 x 3.1).
rectangle('Position', [0 0 6.3 3.1], 'EdgeColor', [0.3 0.3 0.3], 'LineStyle', '-');

% All pairwise links with distance labels.
for i = 1:n
    for j = i+1:n
        p = A.pos(i,1:2); q = A.pos(j,1:2);
        plot([p(1) q(1)], [p(2) q(2)], '--', 'Color', [0.55 0.55 0.85]);
        m = (p + q) / 2;
        d = norm(A.pos(i,:) - A.pos(j,:));
        text(m(1), m(2), sprintf('%.2f', d), 'FontSize', 9, ...
            'HorizontalAlignment', 'center', 'BackgroundColor', 'w', ...
            'Margin', 0.5, 'Color', [0.1 0.1 0.5]);
    end
end

% Anchors on top.
plot(A.pos(:,1), A.pos(:,2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 12);
for k = 1:n
    text(A.pos(k,1), A.pos(k,2) + 0.18, sprintf('A%d (%.2f, %.2f)', ...
        A.ids(k), A.pos(k,1), A.pos(k,2)), 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
end

axis equal; grid on
xlim([-0.9 7.2]); ylim([-0.7 3.9]);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Anchor geometry per anchors.json — all z = %.2f m (distances in m)', A.pos(1,3)));
outPng = fullfile(dune.rootDir(), 'results', 'anchor_geometry.png');
saveas(fig, outPng);
close(fig);
fprintf('Saved %s\n', outPng);
end
