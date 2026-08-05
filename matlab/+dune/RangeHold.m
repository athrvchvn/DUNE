classdef RangeHold < handle
    %RANGEHOLD Last-known-range memory per anchor (geometry hold).
    %   The anchor POSITIONS are a fixed global configuration; what makes the
    %   fix jump is the anchor SUBSET changing between sweeps (scheduler
    %   skips, failed exchanges). RangeHold keeps each anchor's last good
    %   measurement and injects it into sweeps where that anchor is missing,
    %   so every solve sees the same full geometry:
    %     - while STILL: held at full weight up to maxAgeStill (the range is
    %       genuinely unchanged - the tag has not moved);
    %     - while MOVING: weight decays fast (exp(-(age/tau)^2)) and the
    %       entry is dropped after maxAge (a stale range would drag the fix).
    %
    %   rh = dune.RangeHold();
    %   [s2, nHeld] = rh.apply(sweep, still);   % sweep needs .thost
    %   Held entries carry qual = NaN and a wScale < = 1 that
    %   dune.solveSweep multiplies into the NLOS weights.

    properties
        maxAge      = 3    % s, hold cap while moving
        maxAgeStill = 15   % s, hold cap while still
        tau         = 1.0  % s, moving-decay time constant
        histN       = 5    % held value = median of the last histN samples
    end
    properties (SetAccess = private)
        ids = []; dist = []; rx = []; fp = []; t = [];
        histD = {}
    end

    methods
        function [s2, nHeld] = apply(obj, s, still)
            if nargin < 3, still = false; end
            SENT = -2147483648;
            % Update the cache from this sweep's real measurements
            for k = 1:numel(s.ids)
                if s.rx(k) == SENT || s.fp(k) == SENT, continue; end
                if ~isfinite(s.dist(k)) || s.dist(k) <= 0, continue; end
                j = find(obj.ids == s.ids(k), 1);
                if isempty(j)
                    obj.ids(end+1) = s.ids(k);
                    j = numel(obj.ids);
                    obj.histD{j} = [];
                end
                obj.histD{j} = [obj.histD{j}, s.dist(k)];
                if numel(obj.histD{j}) > obj.histN
                    obj.histD{j} = obj.histD{j}(end - obj.histN + 1 : end);
                end
                obj.dist(j) = median(obj.histD{j});   % noise-robust hold value
                obj.rx(j) = s.rx(k);
                obj.fp(j) = s.fp(k);
                obj.t(j) = s.thost;
            end
            % Inject held entries for anchors missing from this sweep
            s2 = s;
            s2.wScale = ones(1, numel(s.ids));
            nHeld = 0;
            cap = obj.maxAge;
            if still, cap = obj.maxAgeStill; end
            for j = 1:numel(obj.ids)
                if any(s.ids == obj.ids(j)), continue; end
                age = s.thost - obj.t(j);
                if age > cap, continue; end
                if still
                    w = 1;                       % parked: range is still true
                else
                    w = exp(-(age / obj.tau)^2); % moving: fade fast
                end
                if w < 0.05, continue; end
                s2.ids(end+1) = obj.ids(j);
                s2.dist(end+1) = obj.dist(j);
                s2.rx(end+1) = obj.rx(j);
                s2.fp(end+1) = obj.fp(j);
                s2.qual(end+1) = NaN;
                s2.wScale(end+1) = w;
                nHeld = nHeld + 1;
            end
        end
    end
end
