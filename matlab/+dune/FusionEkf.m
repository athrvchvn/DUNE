classdef FusionEkf < handle
    %FUSIONEKF Loosely/tightly-coupled 2D UWB + IMU EKF.
    %   Base state x = [px py vx vy bax bay]':
    %     p - world position (m), v - world velocity (m/s),
    %     ba - horizontal accelerometer bias in world frame (m/s^2).
    %   Optional (enableRangeBias): + one slowly-varying RANGE-BIAS state per
    %   anchor. With tightly-coupled range updates these remember each
    %   anchor's residual bias, so the position no longer jumps when the
    %   answering anchor SUBSET changes (scheduler skips, dropouts) - the
    %   common-mode bias stays in position (unobservable), but the
    %   per-anchor differences are learned and held.
    %
    %   Prediction: IMU-driven when world-frame linear acceleration is given,
    %   constant-velocity fallback otherwise. stillMode caps process noise so
    %   a parked tag can truly pin.
    %   Updates (all chi-squared gated): position fix, per-range
    %   (tightly-coupled), ZUPT.

    properties
        sigmaAccel   = 0.35   % m/s^2, accel white noise (IMU-driven predict)
        sigmaAccelCV = 0.8    % m/s^2, CV-fallback model uncertainty
        sigmaBiasRW  = 0.01   % m/s^2 / sqrt(s), accel-bias random walk
        posSigma     = 0.08   % m, fallback position-measurement sigma
        zuptSigma    = 0.05   % m/s, ZUPT pseudo-measurement sigma
        maxConsecReject = 10  % reinit from the fix after this many rejections
        stillMode = false     % driver-set stillness: caps process noise so
                              % the state can pin (ZUPT alone clamps velocity
                              % but CV Q keeps re-injecting position noise)
        sigmaAccelStill = 0.05
        rbSigmaInit  = 0.05   % m, initial per-anchor range-bias uncertainty
        rbSigmaRW    = 0.003  % m/sqrt(s), range-bias random walk
        stillInvert = false   % Phase-A3 noise-budget inversion: while STILL,
                              % the position is known constant but the ranges
                              % keep breathing, so invert the budget - position
                              % Q ~ 0 and the per-anchor bias RW opened - and
                              % the breathing is absorbed into the bias states
                              % instead of dragging the position.
        sigmaAccelStillInv = 0.005  % m/s^2, position Q when still+inverted
        rbSigmaRWStill     = 0.02   % m/sqrt(s), opened bias RW when still+inverted
        robust = true         % Huber-style M-estimation (Bitcraze-proven):
                              % marginal outliers are DOWN-WEIGHTED (R inflated
                              % so the effective NIS sits at the gate) instead
                              % of hard-rejected; only gross outliers beyond
                              % robustHardFactor x gate are rejected outright.
        robustHardFactor = 10
    end
    properties (SetAccess = private)
        x = zeros(6, 1)
        P = eye(6) * 1e4
        nRb = 0               % number of range-bias states (0 = disabled)
        initialized = false
        lastNis = NaN
        lastAccepted = true
        nRejected = 0
        nSoft = 0             % robust: updates applied with inflated R
        nReinit = 0
        consecReject = 0
    end

    methods
        function enableRangeBias(obj, M)
            % Add M per-anchor range-bias states (call before initialize;
            % ranges passed to updateRanges must use this same order).
            obj.nRb = M;
            obj.x = zeros(6 + M, 1);
            obj.P = eye(6 + M) * 1e4;
            obj.initialized = false;
        end

        function initialize(obj, pos)
            n = 6 + obj.nRb;
            obj.x = zeros(n, 1);
            obj.x(1:2) = pos(:);
            obj.P = zeros(n);
            obj.P(1:6, 1:6) = diag([0.25, 0.25, 1.0, 1.0, 0.01, 0.01]);
            if obj.nRb > 0
                obj.P(7:end, 7:end) = eye(obj.nRb) * obj.rbSigmaInit^2;
            end
            obj.initialized = true;
        end

        function p = pos(obj),  p = obj.x(1:2)'; end
        function v = vel(obj),  v = obj.x(3:4)'; end
        function b = bias(obj), b = obj.x(5:6)'; end
        function b = rangeBias(obj), b = obj.x(7:end)'; end

        function predict(obj, dt, aWorld)
            % predict(dt) or predict(dt, [])  -> CV fallback
            % predict(dt, [ax ay])            -> IMU-driven
            if ~obj.initialized || dt <= 0, return; end
            useImu = nargin >= 3 && numel(aWorld) == 2 && all(isfinite(aWorld));

            n = 6 + obj.nRb;
            I2 = eye(2); Z2 = zeros(2);
            F = eye(n);
            if useImu
                % p' = p + v dt + 1/2 (a - ba) dt^2 ; v' = v + (a - ba) dt
                F(1:6, 1:6) = [I2, dt*I2, -0.5*dt^2*I2;
                               Z2, I2,    -dt*I2;
                               Z2, Z2,     I2];
                u = aWorld(:);
                obj.x(1:2) = obj.x(1:2) + obj.x(3:4)*dt ...
                             + 0.5*(u - obj.x(5:6))*dt^2;
                obj.x(3:4) = obj.x(3:4) + (u - obj.x(5:6))*dt;
                sa = obj.sigmaAccel;
            else
                F(1, 3) = dt; F(2, 4) = dt;
                obj.x(1:2) = obj.x(1:2) + obj.x(3:4)*dt;
                sa = obj.sigmaAccelCV;
            end
            rbRW = obj.rbSigmaRW;
            if obj.stillMode
                if obj.stillInvert && obj.nRb > 0
                    sa = min(sa, obj.sigmaAccelStillInv);
                    rbRW = obj.rbSigmaRWStill;
                else
                    sa = min(sa, obj.sigmaAccelStill);
                end
            end
            % Accel-bias states are unobservable without IMU input - freeze.
            if useImu, sb = obj.sigmaBiasRW; else, sb = 0; end

            Q = zeros(n);
            Q(1:6, 1:6) = [sa^2*(dt^3/3)*I2, sa^2*(dt^2/2)*I2, Z2;
                           sa^2*(dt^2/2)*I2, sa^2*dt*I2,       Z2;
                           Z2,               Z2,               sb^2*dt*I2];
            if obj.nRb > 0
                Q(7:end, 7:end) = eye(obj.nRb) * rbRW^2 * dt;
            end
            obj.P = F * obj.P * F' + Q;
        end

        function ok = update(obj, z, H, R, zhat)
            % Generic gated EKF update (Joseph form). Gate: chi2(dof, 0.95).
            % zhat: predicted measurement for nonlinear h(x); default H*x.
            CHI2_95 = [3.841, 5.991, 7.815];   % dof 1..3
            if nargin < 5, zhat = H * obj.x; end
            y = z(:) - zhat(:);
            S = H * obj.P * H' + R;
            nis = y' * (S \ y);
            obj.lastNis = nis;
            gate = CHI2_95(min(numel(z), 3));
            if nis > gate && obj.robust && nis <= obj.robustHardFactor * gate
                % Huber: keep the measurement but inflate its covariance so
                % the effective NIS sits at the gate (bounded influence).
                R = R * (nis / gate);
                S = H * obj.P * H' + R;
                obj.nSoft = obj.nSoft + 1;
                ok = true;
            else
                ok = nis <= gate;
            end
            obj.lastAccepted = ok;
            if ~ok
                obj.nRejected = obj.nRejected + 1;
                return;
            end
            K = (obj.P * H') / S;
            obj.x = obj.x + K * y;
            IKH = eye(numel(obj.x)) - K * H;
            obj.P = IKH * obj.P * IKH' + K * R * K';
            obj.P = (obj.P + obj.P') / 2;
        end

        function ok = updatePosition(obj, pos, R)
            if ~obj.initialized
                obj.initialize(pos);
                ok = true;
                return;
            end
            if nargin < 3 || isempty(R) || any(~isfinite(R(:)))
                R = eye(2) * obj.posSigma^2;
            end
            H = [eye(2), zeros(2, 4 + obj.nRb)];
            ok = obj.update(pos, H, R);
            % Divergence recovery: a healthy filter should not reject a long
            % streak of fixes. Re-anchor on the measurements when it does.
            if ok
                obj.consecReject = 0;
            else
                obj.consecReject = obj.consecReject + 1;
                if obj.consecReject >= obj.maxConsecReject
                    obj.reinitFrom(pos);
                end
            end
        end

        function nAcc = updateRanges(obj, anchorPos, ranges, weights, tagZ, sigmaR)
            % Tightly-coupled sequential range updates, chi2(1)-gated each.
            % With range-bias states enabled, measurement model is
            %   z_k = ||p - a_k|| + b_k  -> per-anchor bias is learned while
            % the anchor answers and REMEMBERED while it is skipped.
            if nargin < 6, sigmaR = 0.05; end
            if nargin < 5, tagZ = 0.24; end
            if nargin < 4 || isempty(weights), weights = ones(size(ranges)); end
            nAcc = 0;
            if ~obj.initialized, return; end
            n = 6 + obj.nRb;
            for k = find(isfinite(ranges(:)') & weights(:)' > 0)
                dx = obj.x(1) - anchorPos(k, 1);
                dy = obj.x(2) - anchorPos(k, 2);
                dz2 = (anchorPos(k, 3) - tagZ)^2;
                pred = max(sqrt(dx^2 + dy^2 + dz2), 1e-6);
                H = zeros(1, n);
                H(1) = dx / pred;
                H(2) = dy / pred;
                zhat = pred;
                if obj.nRb > 0 && k <= obj.nRb
                    H(6 + k) = 1;
                    zhat = pred + obj.x(6 + k);
                end
                Rk = sigmaR^2 / weights(k);
                nAcc = nAcc + obj.update(ranges(k), H, Rk, zhat);
            end
            if nAcc > 0
                obj.consecReject = 0;
            else
                obj.consecReject = obj.consecReject + 1;
            end
        end

        function ok = updateZupt(obj)
            if ~obj.initialized, ok = false; return; end
            H = [zeros(2), eye(2), zeros(2, 2 + obj.nRb)];
            ok = obj.update([0; 0], H, eye(2) * obj.zuptSigma^2);
        end

        function reinitFrom(obj, pos)
            % Re-anchor pose but KEEP learned range biases (their memory is
            % the whole point); only their uncertainty is moderately reopened.
            rb = obj.x(7:end);
            obj.initialize(pos);
            if obj.nRb > 0
                obj.x(7:end) = rb;
                obj.P(7:end, 7:end) = eye(obj.nRb) * (obj.rbSigmaInit / 2)^2;
            end
            obj.nReinit = obj.nReinit + 1;
            obj.consecReject = 0;
        end
    end

    methods (Static)
        function aW = worldAccel(quat, acc)
            % Rotate BNO085 body-frame LINEAR acceleration (gravity-removed)
            % into the world frame; returns [ax ay] or [] if inputs invalid.
            % quat = [qw qx qy qz] (HostLink order).
            aW = [];
            if numel(quat) ~= 4 || numel(acc) ~= 3, return; end
            if any(~isfinite(quat)) || any(~isfinite(acc)), return; end
            nq = norm(quat);
            if nq < 0.5, return; end
            q = quat(:)' / nq;
            w = q(1); xq = q(2); yq = q(3); zq = q(4);
            Rq = [1-2*(yq^2+zq^2),   2*(xq*yq - zq*w), 2*(xq*zq + yq*w);
                  2*(xq*yq + zq*w), 1-2*(xq^2+zq^2),   2*(yq*zq - xq*w);
                  2*(xq*zq - yq*w),  2*(yq*zq + xq*w), 1-2*(xq^2+yq^2)];
            a3 = Rq * acc(:);
            aW = a3(1:2)';
        end
    end
end
