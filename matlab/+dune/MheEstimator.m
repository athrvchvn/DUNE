classdef MheEstimator < handle
    %MHEESTIMATOR Moving Horizon Estimator for 2D UWB positioning (CasADi/IPOPT).
    %   Experimental alternative to dune.FusionEkf. Keeps a sliding window of
    %   the last N sweeps and, each step, solves for the WHOLE trajectory over
    %   the window that best fits every anchor range (robust pseudo-Huber), a
    %   motion model, ZUPT when still, and an arrival cost tying the window
    %   start to the prior. Output is the newest window state (a filter).
    %
    %   Two motion models (obj.model):
    %     "cv"       state [px py vx vy]; constant-velocity, gyro rotates the
    %                velocity (coordinated turn). Holonomic - any direction.
    %     "unicycle" state [px py theta speed]; NON-HOLONOMIC car model: the
    %                gyro yaw rate drives heading (change-in-yaw, no absolute
    %                compass), velocity is FORCED along the heading (no sideways
    %                slip), speed is free. Right model for a front-steering car;
    %                the UWB anchors the absolute heading, the gyro smooths it.
    %
    %   Requires CasADi on the path (caller adds it):
    %     addpath('C:\Users\itisa\Downloads\casadi-3.7.0');
    %
    %   Usage:
    %     mhe = dune.MheEstimator(A.pos); mhe.model = "unicycle"; mhe.build();
    %     [p, v, info] = mhe.push(rangeCorr, weights, dt, rawFixHint, still, omega);
    %   push returns p = [NaN NaN] during warm-up (window not yet full).

    properties
        horizon    = 10       % N sweeps in the window (~2 s at 5 Hz)
        model      = "cv"      % "cv" | "unicycle"
        sigmaR     = 0.05      % range measurement sigma (m)
        sigmaAccel = 0.8       % process accel white-noise (m/s^2)
        sigmaTheta = 0.5       % unicycle: heading process noise vs gyro (rad/s)
        tagZ       = 0.24      % fixed tag antenna height (m)
        arrivalPos = 0.15      % arrival-cost sigma, position (m)
        arrivalVel = 0.6       % arrival-cost sigma, velocity/speed (m/s)
        arrivalTheta = 0.4     % unicycle: arrival-cost sigma, heading (rad)
        huberDelta = 0.15      % robust range-residual threshold (m)
        zuptSigma  = 0.03      % zero-velocity pseudo-meas sigma when still (m/s)
        maxJump    = 0.20      % m, jump-guard threshold vs the CV prediction
        maxIter    = 60
    end
    properties (SetAccess = private)
        A                      % M x 3 anchor positions
        M
        solver                 % casadi nlpsol Function
        built = false
        buf = {}               % ring of struct(z,w,dt,still,omega,pHint)
        Xw = []                % last window solution, 4N x 1 (model state space)
        prior = []             % arrival target (4x1, model state space)
        lastP = [NaN NaN]
        lastV = [0 0]
        nSolves = 0
        nJumps = 0
    end

    methods
        function obj = MheEstimator(anchorPos)
            obj.A = anchorPos;
            obj.M = size(anchorPos, 1);
        end

        function build(obj)
            % Construct the parametric NLP once; reused every push().
            import casadi.*
            N = obj.horizon; Ma = obj.M;
            Ax = obj.A(:,1); Ay = obj.A(:,2); dz = obj.A(:,3) - obj.tagZ;
            uni = obj.model == "unicycle";

            X     = MX.sym('X', 4, N);      % state per step (model-dependent)
            Z     = MX.sym('Z', Ma, N);
            W     = MX.sym('W', Ma, N);
            DT    = MX.sym('DT', N-1, 1);
            OMEGA = MX.sym('OMEGA', N-1, 1);
            STILL = MX.sym('STILL', N, 1);
            PRIOR = MX.sym('PRIOR', 4, 1);

            f = MX(0);
            d = obj.huberDelta; sR2 = obj.sigmaR^2; zs2 = obj.zuptSigma^2;
            for k = 1:N
                pred = sqrt((X(1,k) - Ax).^2 + (X(2,k) - Ay).^2 + dz.^2 + 1e-9);
                r = pred - Z(:,k);
                ph = d^2 * (sqrt(1 + (r ./ d).^2) - 1);   % pseudo-Huber
                f = f + sum1(W(:,k) .* ph) / sR2;
                if uni
                    f = f + STILL(k) * X(4,k)^2 / zs2;         % speed -> 0 when still
                else
                    f = f + STILL(k) * sum1(X(3:4,k).^2) / zs2;% |v| -> 0 when still
                end
            end

            sigA = obj.sigmaAccel; sTh = obj.sigmaTheta;
            for k = 1:N-1
                dt = DT(k);
                qp = 0.5 * sigA * dt^2 + 1e-6;
                qv = sigA * dt + 1e-6;
                if uni
                    % velocities implied by [theta, speed] (no sideways slip)
                    vk  = [X(4,k)  *cos(X(3,k));   X(4,k)  *sin(X(3,k))];
                    vk1 = [X(4,k+1)*cos(X(3,k+1)); X(4,k+1)*sin(X(3,k+1))];
                    dp  = X(1:2,k+1) - X(1:2,k) - 0.5*(vk + vk1)*dt;
                    dth = X(3,k+1) - X(3,k) - OMEGA(k)*dt;      % gyro drives heading
                    dsp = X(4,k+1) - X(4,k);                    % speed random walk
                    qth = sTh * dt + 1e-6;
                    f = f + sum1(dp.^2)/qp^2 + dth^2/qth^2 + dsp^2/qv^2;
                else
                    dp = X(1:2,k+1) - X(1:2,k) - 0.5*(X(3:4,k) + X(3:4,k+1))*dt;
                    th = OMEGA(k)*dt; cth = cos(th); sth = sin(th);
                    vrot = [cth*X(3,k) - sth*X(4,k); sth*X(3,k) + cth*X(4,k)];
                    dv = X(3:4,k+1) - vrot;
                    f = f + sum1(dp.^2)/qp^2 + sum1(dv.^2)/qv^2;
                end
            end

            f = f + sum1((X(1:2,1) - PRIOR(1:2)).^2) / obj.arrivalPos^2;
            if uni
                f = f + (X(3,1) - PRIOR(3))^2 / obj.arrivalTheta^2 ...
                      + (X(4,1) - PRIOR(4))^2 / obj.arrivalVel^2;
            else
                f = f + sum1((X(3:4,1) - PRIOR(3:4)).^2) / obj.arrivalVel^2;
            end

            P = [Z(:); W(:); DT; OMEGA; STILL; PRIOR];
            nlp = struct('x', X(:), 'f', f, 'p', P);
            opts = struct('print_time', false, 'ipopt', ...
                struct('print_level', 0, 'sb', 'yes', 'max_iter', obj.maxIter, ...
                       'tol', 1e-5, 'acceptable_tol', 1e-4, 'mu_strategy', 'adaptive'));
            obj.solver = nlpsol('mhe', 'ipopt', nlp, opts);
            obj.built = true;
        end

        function reset(obj)
            obj.buf = {}; obj.Xw = []; obj.prior = [];
            obj.lastP = [NaN NaN]; obj.lastV = [0 0];
        end

        function [p, v, info] = push(obj, z, w, dt, pHint, still, omega)
            % z,w: M x 1 corrected ranges / weights (NaN or w<=0 = absent).
            % dt:  seconds since the previous push. pHint: raw fix [1x2] (seeds
            % the optimiser + heading; may be NaN). still: logical -> ZUPT.
            % omega: gyro yaw rate (rad/s); drives the turn/heading model.
            if nargin < 5, pHint = [NaN NaN]; end
            if nargin < 6, still = false; end
            if nargin < 7 || ~isfinite(omega), omega = 0; end
            z = z(:); w = w(:);
            bad = ~isfinite(z) | ~isfinite(w) | w <= 0;
            z(bad) = 0; w(bad) = 0;
            obj.buf{end+1} = struct('z', z, 'w', w, 'dt', max(dt, 1e-3), ...
                'still', double(logical(still)), 'omega', omega, ...
                'pHint', pHint(:)');

            N = obj.horizon; uni = obj.model == "unicycle";
            info = struct('warmup', true, 'cost', NaN, 'iters', 0, 'jumped', false);
            p = [NaN NaN]; v = [NaN NaN];
            if numel(obj.buf) < N, return; end
            if numel(obj.buf) > N, obj.buf(1) = []; end
            if ~obj.built, obj.build(); end

            Zm = zeros(obj.M, N); Wm = zeros(obj.M, N);
            DTv = zeros(N-1, 1); OMv = zeros(N-1, 1); STv = zeros(N, 1);
            Ph = nan(N, 2); tw = zeros(N, 1);
            for k = 1:N
                Zm(:,k) = obj.buf{k}.z; Wm(:,k) = obj.buf{k}.w;
                STv(k) = obj.buf{k}.still; Ph(k,:) = obj.buf{k}.pHint;
                if k >= 2
                    DTv(k-1) = obj.buf{k}.dt;
                    OMv(k-1) = 0.5 * (obj.buf{k-1}.omega + obj.buf{k}.omega);
                    tw(k) = tw(k-1) + DTv(k-1);
                end
            end

            % Warm start + arrival prior (model-aware).
            if isempty(obj.Xw)
                [X0, pr] = obj.seedFirst(Ph, tw, uni);
            else
                [X0, pr] = obj.warmStart(DTv, OMv, uni);
            end
            Pv = [Zm(:); Wm(:); DTv; OMv; STv; pr];

            r = obj.solver('x0', X0, 'p', Pv);
            X = full(r.x);
            obj.nSolves = obj.nSolves + 1;
            Xm = reshape(X, 4, N);
            p = Xm(1:2,end)'; v = obj.stateVel(Xm(:,end), uni);
            st = obj.solver.stats();
            solveOk = ~isfield(st, 'success') || st.success;

            info.warmup = false; info.cost = full(r.f);
            if isfield(st, 'iter_count'), info.iters = st.iter_count; end

            % Jump-guard: reject an output that leaps from the CV prediction
            % (or a non-converged solve) -> fall back to the raw fix, reseed.
            if all(isfinite(obj.lastP)) && ...
               (~solveOk || norm(p - (obj.lastP + obj.lastV * dt)) > obj.maxJump)
                if all(isfinite(pHint)), p = pHint;
                else, p = obj.lastP + obj.lastV * dt; end
                v = obj.lastV;
                obj.reseedWindow(p, v, DTv, uni);
                obj.nJumps = obj.nJumps + 1;
                info.jumped = true;
            else
                obj.Xw = X;
                obj.prior = Xm(:,2);
            end
            obj.lastP = p; obj.lastV = v;
        end
    end

    methods (Access = private)
        function v = stateVel(~, xcol, uni)
            if uni, v = xcol(4) * [cos(xcol(3)), sin(xcol(3))];
            else,   v = xcol(3:4)'; end
        end

        function [X0, pr] = seedFirst(obj, Ph, tw, uni)
            % Seed the first full window from the buffered raw fixes: positions
            % where available, heading/speed from the average motion direction.
            fin = find(all(isfinite(Ph), 2));
            if numel(fin) >= 2
                vseed = (Ph(fin(end),:) - Ph(fin(1),:)) / max(tw(fin(end)) - tw(fin(1)), 1e-3);
                seedPos = Ph(fin(1),:);
            else
                vseed = [0 0];
                seedPos = mean(obj.A(:,1:2), 1);
                if ~isempty(fin), seedPos = Ph(fin(1),:); end
            end
            N = obj.horizon;
            Xm = zeros(4, N);
            for k = 1:N
                pk = Ph(k,:); if any(~isfinite(pk)), pk = seedPos; end
                Xm(1:2,k) = pk;
                if uni
                    Xm(3,k) = atan2(vseed(2), vseed(1));
                    Xm(4,k) = norm(vseed);
                else
                    Xm(3:4,k) = vseed;
                end
            end
            X0 = Xm(:); pr = Xm(:,1);        % first window: arrival ~ free fit
        end

        function [X0, pr] = warmStart(obj, DTv, OMv, uni)
            % Shift the previous solution forward one step and extrapolate the
            % new last node with the motion model.
            N = obj.horizon;
            Xp = reshape(obj.Xw, 4, N);
            last = Xp(:,end);
            if uni
                th = last(3) + OMv(end)*DTv(end);
                sp = last(4);
                last = [last(1:2) + sp*[cos(th); sin(th)]*DTv(end); th; sp];
            else
                last = last + [last(3:4)*DTv(end); 0; 0];
            end
            X0 = reshape([Xp(:,2:end), last], [], 1);
            pr = obj.prior;
        end

        function reseedWindow(obj, p, v, DTv, uni)
            % Rebuild the window as a clean constant-motion trajectory ending at
            % (p,v) so a rejected solve does not poison the next warm start.
            N = obj.horizon; dt = mean(DTv);
            Xm = zeros(4, N);
            th = atan2(v(2), v(1)); sp = norm(v);
            for k = 1:N
                Xm(1:2,k) = p(:) - v(:) * (N - k) * dt;
                if uni, Xm(3,k) = th; Xm(4,k) = sp;
                else,   Xm(3:4,k) = v(:); end
            end
            obj.Xw = Xm(:);
            obj.prior = Xm(:,2);
        end
    end
end
