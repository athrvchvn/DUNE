classdef LivePlotter < handle
    %LIVEPLOTTER  Live 2D/3D view of anchors + N tracked tags.
    %
    %   plotter = dune.LivePlotter(A);
    %   plotter.update(tagId, pos, info);      % pos = dim×1, info from dune.multilaterate
    %   plotter.update(tagId, pos, info, imu); % with IMU struct
    %
    % Each unique tagId gets its own color, trail, and marker on first call.
    % No tag IDs are pre-configured — the plotter scales to N tags.

    properties
        cfg
        fig
        ax
        hImuText        % uicontrol: IMU readout strip (shows most recent tag)
        trailLen = 200

        % Per-tag plot handles and data (containers.Map, int32 key)
        hTagMap          % tag_id  → marker handle
        hTrailMap        % tag_id  → trail line handle
        hEllipseMap      % tag_id  → covariance ellipse handle (2D only)
        trailDataMap     % tag_id  → dim × N matrix of trail points

        % Color palette — rows are RGB triples, cycled for new tags
        TAG_COLORS = [
            0.00  0.83  1.00   % cyan    (tag 1)
            0.90  0.22  0.28   % red     (tag 2)
            0.64  0.90  0.21   % lime    (tag 3)
            0.65  0.55  0.98   % violet  (tag 4)
            1.00  0.62  0.00   % orange  (tag 5)
            1.00  0.24  0.35   % rose    (tag 6)
        ]
    end

    methods
        function obj = LivePlotter(cfg)
            obj.cfg = cfg;
            obj.fig = figure('Name', 'UWB RTLS', 'NumberTitle', 'off');

            % Per-tag Maps
            obj.hTagMap      = containers.Map('KeyType','int32','ValueType','any');
            obj.hTrailMap    = containers.Map('KeyType','int32','ValueType','any');
            obj.hEllipseMap  = containers.Map('KeyType','int32','ValueType','any');
            obj.trailDataMap = containers.Map('KeyType','int32','ValueType','any');

            % IMU text strip at bottom
            imuPanelH = 0.12;
            obj.hImuText = uicontrol(obj.fig, ...
                'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0, 0, 1, imuPanelH], ...
                'String', 'IMU: no data', ...
                'HorizontalAlignment', 'left', ...
                'FontSize', 9, 'FontName', 'Monospaced', ...
                'BackgroundColor', [0.92 0.92 0.92], ...
                'ForegroundColor', [0.15 0.15 0.15]);

            % Main axes
            obj.ax = axes('Parent', obj.fig, ...
                'Position', [0.08, imuPanelH + 0.08, 0.86, 0.80]);
            hold(obj.ax, 'on'); grid(obj.ax, 'on');
            axis(obj.ax, 'equal');
            b = cfg.bounds;

            % Anchor markers (shared across all tags)
            if cfg.dim == 2
                plot(obj.ax, cfg.pos(:,1), cfg.pos(:,2), 'ks', ...
                    'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 9);
                for i = 1:numel(cfg.ids)
                    text(obj.ax, cfg.pos(i,1), cfg.pos(i,2), ...
                        sprintf('  A%d', cfg.ids(i)), 'FontWeight', 'bold');
                end
                xlim(obj.ax, b(1:2)); ylim(obj.ax, b(3:4));
                xlabel(obj.ax, 'x (m)'); ylabel(obj.ax, 'y (m)');
            else
                plot3(obj.ax, cfg.pos(:,1), cfg.pos(:,2), cfg.pos(:,3), ...
                    'ks', 'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 9);
                for i = 1:numel(cfg.ids)
                    text(obj.ax, cfg.pos(i,1), cfg.pos(i,2), cfg.pos(i,3), ...
                        sprintf('  A%d', cfg.ids(i)), 'FontWeight', 'bold');
                end
                xlim(obj.ax, b(1:2)); ylim(obj.ax, b(3:4)); zlim(obj.ax, b(5:6));
                xlabel(obj.ax, 'x (m)'); ylabel(obj.ax, 'y (m)'); zlabel(obj.ax, 'z (m)');
                view(obj.ax, 3);
            end
            title(obj.ax, 'UWB RTLS — waiting for data...');
        end

        function update(obj, tagId, pos, info, imu)
            if ~all(isfinite(pos)); return; end

            % ---- Create plot handles for a newly-discovered tag ----------
            if ~isKey(obj.hTagMap, tagId)
                nTags = numel(keys(obj.hTagMap));
                col = obj.TAG_COLORS(mod(nTags, size(obj.TAG_COLORS, 1)) + 1, :);

                if obj.cfg.dim == 2
                    obj.hTrailMap(tagId)   = plot(obj.ax, NaN, NaN, '-', 'Color', col);
                    obj.hTagMap(tagId)     = plot(obj.ax, NaN, NaN, 'o', 'MarkerSize', 11, ...
                        'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', ...
                        'DisplayName', sprintf('Tag 0x%02X', tagId));
                    obj.hEllipseMap(tagId) = plot(obj.ax, NaN, NaN, '-', 'Color', col * 0.7 + [0.3 0.3 0.3]);
                else
                    obj.hTrailMap(tagId)   = plot3(obj.ax, NaN, NaN, NaN, '-', 'Color', col);
                    obj.hTagMap(tagId)     = plot3(obj.ax, NaN, NaN, NaN, 'o', 'MarkerSize', 11, ...
                        'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', ...
                        'DisplayName', sprintf('Tag 0x%02X', tagId));
                end
                obj.trailDataMap(tagId) = zeros(obj.cfg.dim, 0);
                legend(obj.ax, 'Location', 'northwest');
                fprintf('[LivePlotter] New tag 0x%02X color=[%.2f %.2f %.2f]\n', tagId, col(1), col(2), col(3));
            end

            % ---- Update trail data for this tag --------------------------
            trail = [obj.trailDataMap(tagId), pos(:)];
            if size(trail, 2) > obj.trailLen
                trail = trail(:, end-obj.trailLen+1:end);
            end
            obj.trailDataMap(tagId) = trail;

            % ---- Refresh plot handles ------------------------------------
            if obj.cfg.dim == 2
                set(obj.hTagMap(tagId),   'XData', pos(1),      'YData', pos(2));
                set(obj.hTrailMap(tagId), 'XData', trail(1,:),  'YData', trail(2,:));
                if nargin >= 4 && isstruct(info) && all(isfinite(info.cov(:)))
                    [ex, ey] = obj.covEllipse(pos, info.cov);
                    set(obj.hEllipseMap(tagId), 'XData', ex, 'YData', ey);
                end
            else
                set(obj.hTagMap(tagId),   'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
                set(obj.hTrailMap(tagId), 'XData', trail(1,:), 'YData', trail(2,:), ...
                    'ZData', trail(3,:));
            end

            % Title shows summary across all tags
            nTags = numel(keys(obj.hTagMap));
            if nargin >= 4 && isstruct(info)
                title(obj.ax, sprintf('UWB RTLS — %d tag(s) | 0x%02X: %d anchors, rmse=%.3f m', ...
                    nTags, tagId, sum(info.used), info.rmse));
            end

            % IMU strip — update for whichever tag most recently sent IMU data
            if nargin >= 5 && ~isempty(imu) && isfield(imu, 'quat')
                q  = imu.quat;
                roll  = atan2d(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2));
                pitch = asind(max(-1, min(1, 2*(q(1)*q(3)-q(4)*q(2)))));
                yaw   = atan2d(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));
                a = imu.acc;
                obj.hImuText.String = sprintf( ...
                    'IMU [0x%02X]   Roll:%+7.1f\xB0   Pitch:%+7.1f\xB0   Yaw:%+7.1f\xB0      Ax:%+6.2f   Ay:%+6.2f   Az:%+6.2f m/s\xB2', ...
                    tagId, roll, pitch, yaw, a(1), a(2), a(3));
                obj.hImuText.ForegroundColor = [0.05 0.35 0.05];
            end

            drawnow limitrate;
        end
    end

    methods (Static, Access = private)
        function [ex, ey] = covEllipse(pos, C)
            C = C(1:2, 1:2);
            [V, D] = eig((C + C')/2);
            t = linspace(0, 2*pi, 40);
            a = 2 * sqrt(max(D(1,1), 0));
            b = 2 * sqrt(max(D(2,2), 0));
            xy = V * [a*cos(t); b*sin(t)];
            ex = pos(1) + xy(1,:);
            ey = pos(2) + xy(2,:);
        end
    end
end
