classdef MheRigid < handle
    %MHERIGID Rigid dual-tag Moving Horizon Estimator (CasADi/IPOPT).
    %   Estimates ONE rigid body carrying TWO tags a fixed distance L apart,
    %   with the non-holonomic unicycle motion model. State per window node:
    %       x = [cx, cy, psi, speed]        (body centre, heading, forward speed)
    %   The two tag antennas are NOT solved separately - both are derived from
    %   the same pose, so the rig's physical facts are enforced STRUCTURALLY:
    %       front tag = c + (Lh/2)*[cos psi, sin psi]
    %       rear  tag = c - (Lh/2)*[cos psi, sin psi]
    %     * the inter-tag DISTANCE is exact by construction (not a penalty),
    %     * "both tags stationary, or both moving" is automatic (one speed
    %       state - the rigid link cannot let them disagree),
    %     * YAW is observed by the UWB geometry (which end is where) AND driven
    %       by the gyro yaw rate -> better than either source alone.
    %
    %   ASYNCHRONOUS BY DESIGN. The tags sweep at different instants (token
    %   ring), so each incoming sweep - from EITHER tag - becomes its own window
    %   node observing the body at its own timestamp, tagged with sgn = +1
    %   (front) or -1 (rear). No nearest-time pairing of the two streams.
    %
    %   TERRAIN (roll/pitch taken directly from the IMU, not estimated):
    %     * horizontal half-baseline Lh/2 = (L/2)*cos(pitch) - the projected
    %       separation shrinks when the rover pitches on a slope,
    %     * per-tag antenna height tz = tagZ +/- (L/2)*sin(pitch) - the front
    %       tag rises and the rear drops (or vice versa) when pitched.
    %   ROLL does not move the tags: the baseline lies along the body x-axis,
    %   which roll rotates about, so it is reported for attitude but does not
    %   enter the geometry. (It would matter only for a laterally-offset tag.)
    %
    %   Requires CasADi on the path:
    %     addpath('C:\Users\itisa\Downloads\casadi-3.7.0');
    %
    %   Usage:
    %     mr = dune.MheRigid(A.pos); mr.baseline = 0.50; mr.build();
    %     [pose, info] = mr.push(rangeCorr, weights, isFront, dt, ...
    %                            pHint, still, omega, pitch);
    %   pose = [cx cy psi speed]; NaN during warm-up (window not yet full).

    properties
        horizon    = 12        % nodes in the window (~2 s: 2 tags x ~3 Hz each)
        baseline   = 0.50      % L, antenna-centre to antenna-centre (m) - MEASURE IT
        sigmaR     = 0.05      % range measurement sigma (m)
        sigmaAccel = 0.8       % speed process noise (m/s^2)
        sigmaTheta = 0.5       % heading process noise vs the gyro (rad/s)
        tagZ       = 0.24      % nominal tag antenna height (m), level rig
        arrivalPos = 0.15      % arrival-cost sigma, centre (m)
        arrivalVel = 0.6       % arrival-cost sigma, speed (m/s)
        arrivalTheta = 0.4     % arrival-cost sigma, heading (rad)
        huberDelta = 0.15      % robust range-residual threshold (m)
        zuptSigma  = 0.03      % zero-speed pseudo-measurement sigma (m/s)
        maxJump    = 0.20      % jump-guard threshold on the centre (m)
        maxIter    = 80
    end
    properties (SetAccess = private)
        A, M
        solver
        built = false
        buf = {}
        Xw = []
        prior = []
        lastC = [NaN NaN]      % last centre output
        lastV = [0 0]          % last centre velocity (world)
        lastPsi = 0
        nSolves = 0
        nJumps = 0
    end

    methods
        function obj = MheRigid(anchorPos)
            obj.A = anchorPos;
            obj.M = size(anchorPos, 1);
        end

        function build(obj)
            import casadi.*
            N = obj.horizon; Ma = obj.M;
            Ax = obj.A(:,1); Ay = obj.A(:,2); Az = obj.A(:,3);

            X     = MX.sym('X', 4, N);       % [cx; cy; psi; speed]
            Z     = MX.sym('Z', Ma, N);      % ranges of the observing tag
            W     = MX.sym('W', Ma, N);      % weights (0 = anchor absent)
            SGN   = MX.sym('SGN', N, 1);     % +1 front tag, -1 rear tag
            LH    = MX.sym('LH', N, 1);      % horizontal HALF-baseline (m)
            TZ    = MX.sym('TZ', N, 1);      % observing tag's antenna height (m)
            DT    = MX.sym('DT', N-1, 1);
            OMEGA = MX.sym('OMEGA', N-1, 1); % gyro yaw rate (rad/s)
            STILL = MX.sym('STILL', N, 1);
            PRIOR = MX.sym('PRIOR', 4, 1);

            f = MX(0);
            d = obj.huberDelta; sR2 = obj.sigmaR^2; zs2 = obj.zuptSigma^2;
            for k = 1:N
                psi = X(3,k);
                off = SGN(k) * LH(k);                    % signed lever arm
                tx = X(1,k) + off * cos(psi);
                ty = X(2,k) + off * sin(psi);
                dz = TZ(k) - Az;                          % M x 1
                pred = sqrt((tx - Ax).^2 + (ty - Ay).^2 + dz.^2 + 1e-9);
                r = pred - Z(:,k);
                ph = d^2 * (sqrt(1 + (r ./ d).^2) - 1);   % pseudo-Huber
                f = f + sum1(W(:,k) .* ph) / sR2;
                f = f + STILL(k) * X(4,k)^2 / zs2;        % ZUPT on the body speed
            end

            sigA = obj.sigmaAccel; sTh = obj.sigmaTheta;
            for k = 1:N-1
                dt = DT(k);
                qp = 0.5 * sigA * dt^2 + 1e-6;
                qv = sigA * dt + 1e-6;
                qth = sTh * dt + 1e-6;
                % Unicycle: velocity is ALONG the heading (no sideways slip).
                vk  = [X(4,k)  *cos(X(3,k));   X(4,k)  *sin(X(3,k))];
                vk1 = [X(4,k+1)*cos(X(3,k+1)); X(4,k+1)*sin(X(3,k+1))];
                dp  = X(1:2,k+1) - X(1:2,k) - 0.5*(vk + vk1)*dt;  % trapezoidal
                dth = X(3,k+1) - X(3,k) - OMEGA(k)*dt;            % gyro drives psi
                dsp = X(4,k+1) - X(4,k);
                f = f + sum1(dp.^2)/qp^2 + dth^2/qth^2 + dsp^2/qv^2;
            end

            f = f + sum1((X(1:2,1) - PRIOR(1:2)).^2) / obj.arrivalPos^2 ...
                  + (X(3,1) - PRIOR(3))^2 / obj.arrivalTheta^2 ...
                  + (X(4,1) - PRIOR(4))^2 / obj.arrivalVel^2;

            P = [Z(:); W(:); SGN; LH; TZ; DT; OMEGA; STILL; PRIOR];
            nlp = struct('x', X(:), 'f', f, 'p', P);
            opts = struct('print_time', false, 'ipopt', ...
                struct('print_level', 0, 'sb', 'yes', 'max_iter', obj.maxIter, ...
                       'tol', 1e-5, 'acceptable_tol', 1e-4, 'mu_strategy', 'adaptive'));
            obj.solver = nlpsol('mheRigid', 'ipopt', nlp, opts);
            obj.built = true;
        end

        function reset(obj)
            obj.buf = {}; obj.Xw = []; obj.prior = [];
            obj.lastC = [NaN NaN]; obj.lastV = [0 0]; obj.lastPsi = 0;
        end

        function [pose, info] = push(obj, z, w, isFront, dt, pHint, still, omega, pitch)
            % z,w     : M x 1 corrected ranges / weights of the OBSERVING tag
            % isFront : true if this sweep came from the FRONT tag
            % dt      : seconds since the previous push (either tag)
            % pHint   : that tag's standalone raw fix [1x2] (seed/guard; may be NaN)
            % still   : logical, whole rig stationary -> ZUPT
            % omega   : gyro yaw rate (rad/s); hold last value on the no-IMU tag
            % pitch   : IMU pitch (rad) for the terrain geometry
            if nargin < 6, pHint = [NaN NaN]; end
            if nargin < 7, still = false; end
            if nargin < 8 || ~isfinite(omega), omega = 0; end
            if nargin < 9 || ~isfinite(pitch), pitch = 0; end
            z = z(:); w = w(:);
            bad = ~isfinite(z) | ~isfinite(w) | w <= 0;
            z(bad) = 0; w(bad) = 0;
            sgn = 1; if ~isFront, sgn = -1; end
            L2 = obj.baseline / 2;
            obj.buf{end+1} = struct('z', z, 'w', w, 'sgn', sgn, ...
                'lh', L2 * cos(pitch), ...                    % horizontal lever arm
                'tz', obj.tagZ + sgn * L2 * sin(pitch), ...   % this tag's height
                'dt', max(dt, 1e-3), 'still', double(logical(still)), ...
                'omega', omega, 'pHint', pHint(:)');

            N = obj.horizon;
            info = struct('warmup', true, 'cost', NaN, 'iters', 0, 'jumped', false);
            pose = [NaN NaN NaN NaN];
            if numel(obj.buf) < N, return; end
            if numel(obj.buf) > N, obj.buf(1) = []; end
            if ~obj.built, obj.build(); end

            Zm = zeros(obj.M, N); Wm = zeros(obj.M, N);
            SG = zeros(N,1); LHv = zeros(N,1); TZv = zeros(N,1); STv = zeros(N,1);
            DTv = zeros(N-1,1); OMv = zeros(N-1,1);
            for k = 1:N
                b = obj.buf{k};
                Zm(:,k) = b.z; Wm(:,k) = b.w;
                SG(k) = b.sgn; LHv(k) = b.lh; TZv(k) = b.tz; STv(k) = b.still;
                if k >= 2
                    DTv(k-1) = b.dt;
                    OMv(k-1) = 0.5 * (obj.buf{k-1}.omega + b.omega);
                end
            end

            if isempty(obj.Xw)
                [X0, pr] = obj.seedFirst();
            else
                Xp = reshape(obj.Xw, 4, N);
                last = Xp(:,end);
                th = last(3) + OMv(end)*DTv(end); sp = last(4);
                last = [last(1:2) + sp*[cos(th); sin(th)]*DTv(end); th; sp];
                X0 = reshape([Xp(:,2:end), last], [], 1);
                pr = obj.prior;
            end
            Pv = [Zm(:); Wm(:); SG; LHv; TZv; DTv; OMv; STv; pr];

            r = obj.solver('x0', X0, 'p', Pv);
            X = full(r.x);
            obj.nSolves = obj.nSolves + 1;
            Xm = reshape(X, 4, N);
            c = Xm(1:2,end)'; psi = Xm(3,end); spd = Xm(4,end);
            v = spd * [cos(psi), sin(psi)];
            st = obj.solver.stats();
            solveOk = ~isfield(st, 'success') || st.success;
            info.warmup = false; info.cost = full(r.f);
            if isfield(st, 'iter_count'), info.iters = st.iter_count; end

            % Jump-guard on the centre: reject a leap beyond the CV prediction
            % (or a non-converged solve); fall back to the measured geometry.
            if all(isfinite(obj.lastC)) && ...
               (~solveOk || norm(c - (obj.lastC + obj.lastV * dt)) > obj.maxJump)
                cFall = obj.lastC + obj.lastV * dt;
                if all(isfinite(pHint))
                    % the observing tag's own fix, de-levered back to the centre
                    cFall = pHint - sgn * obj.buf{end}.lh * [cos(obj.lastPsi), sin(obj.lastPsi)];
                end
                c = cFall; psi = obj.lastPsi; spd = norm(obj.lastV);
                v = obj.lastV;
                obj.reseedWindow(c, psi, spd, DTv);
                obj.nJumps = obj.nJumps + 1;
                info.jumped = true;
            else
                obj.Xw = X;
                obj.prior = Xm(:,2);
            end
            obj.lastC = c; obj.lastV = v; obj.lastPsi = psi;
            pose = [c, psi, spd];
        end

        function [pf, pr_] = tagPositions(obj, pose, pitch)
            %TAGPOSITIONS Front/rear tag xy implied by a pose (for display).
            if nargin < 3, pitch = 0; end
            lh = (obj.baseline/2) * cos(pitch);
            u = lh * [cos(pose(3)), sin(pose(3))];
            pf = pose(1:2) + u;
            pr_ = pose(1:2) - u;
        end
    end

    methods (Access = private)
        function [X0, pr] = seedFirst(obj)
            % Seed the centre/heading from the most recent front & rear raw
            % fixes in the window (heading = rear -> front).
            N = obj.horizon;
            pF = [NaN NaN]; pR = [NaN NaN];
            for k = N:-1:1
                b = obj.buf{k};
                if any(~isfinite(b.pHint)), continue; end
                if b.sgn > 0 && any(~isfinite(pF)), pF = b.pHint; end
                if b.sgn < 0 && any(~isfinite(pR)), pR = b.pHint; end
            end
            if all(isfinite([pF pR]))
                c0 = 0.5*(pF + pR);
                psi0 = atan2(pF(2) - pR(2), pF(1) - pR(1));
            elseif all(isfinite(pF))
                c0 = pF; psi0 = 0;
            elseif all(isfinite(pR))
                c0 = pR; psi0 = 0;
            else
                c0 = mean(obj.A(:,1:2), 1); psi0 = 0;
            end
            Xm = repmat([c0(:); psi0; 0], 1, N);
            X0 = Xm(:); pr = Xm(:,1);
        end

        function reseedWindow(obj, c, psi, spd, DTv)
            N = obj.horizon; dt = mean(DTv);
            v = spd * [cos(psi); sin(psi)];
            Xm = zeros(4, N);
            for k = 1:N
                Xm(1:2,k) = c(:) - v * (N - k) * dt;
                Xm(3,k) = psi; Xm(4,k) = spd;
            end
            obj.Xw = Xm(:);
            obj.prior = Xm(:,2);
        end
    end
end
