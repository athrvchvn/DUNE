function outFile = build_survey_anchors(surveyFile, opts)
%BUILD_SURVEY_ANCHORS Write an anchors.json from a self-survey (Kinect-free geometry).
%   outFile = build_survey_anchors("...survey_xxxx.txt")
%   build_survey_anchors(f, corrected=true)   % use delay-removed (green) instead
%
%   Reconstructs the anchor constellation from the survey's pairwise distances by
%   classical MDS. By default it writes the RAW MDS geometry (the "red circles" -
%   what the survey gives with no per-anchor correction). corrected=true writes
%   the delay-removed ("green") geometry.
%
%   The MDS frame is arbitrary (a set of distances has no absolute pose or
%   handedness), so the points are pose+reflection aligned onto the current
%   anchors.json purely as a convenient frame - this is NOT the same as using
%   Kinect for geometry: the rover solve and rover_run_report both re-fit the
%   UWB->world transform, so only the anchor SHAPE (survey-derived) affects
%   accuracy. Alignment fixes handedness (1 bit) and gives a readable frame.
%
%   Writes results/survey_runs/<survey>_anchors.json (loadAnchors format).

arguments
    surveyFile (1,1) string
    opts.corrected (1,1) logical = false   % false = raw red, true = delay-removed green
    opts.z (1,1) double = 0.24
end

A = dune.loadAnchors(); ids = A.ids(:); n = numel(ids); truth = A.pos(:,1:2);

% distance matrix
raw = readlines(surveyFile); D = nan(n);
for L = raw'
    t = split(L,',');
    if numel(t)>=6 && t(1)=="SURVEY" && t(2)=="v1"
        ia=find(ids==double(t(3))); ib=find(ids==double(t(4)));
        d=double(t(5))/1000; D(ia,ib)=d; D(ib,ia)=d;
    end
end
D(1:n+1:end)=0;
assert(~any(isnan(D(:))), 'incomplete survey - missing pair(s)');

% optional delay removal (per-anchor bias vs Kinect geometry, like survey_fit)
tag = 'raw';
if opts.corrected
    Dtrue = zeros(n); for i=1:n, for j=1:n, Dtrue(i,j)=norm(truth(i,:)-truth(j,:)); end, end
    M=[]; rhs=[];
    for i=1:n, for j=i+1:n
        r=zeros(1,n); r(i)=1; r(j)=1; M(end+1,:)=r; rhs(end+1,1)=D(i,j)-Dtrue(i,j); %#ok<AGROW>
    end, end
    b=M\rhs;
    for i=1:n, for j=i+1:n, c=D(i,j)-(b(i)+b(j)); D(i,j)=c; D(j,i)=c; end, end
    tag = 'corrected';
end

% classical MDS -> planar, then pose+reflection align (NO scale) to anchors.json
J=eye(n)-ones(n)/n; B=-0.5*J*(D.^2)*J; [V,E]=eig((B+B')/2);
[ev,ord]=sort(diag(E),'descend'); X=V(:,ord(1:2))*diag(sqrt(max(ev(1:2),0)));
best=inf; Xa=X;
for flip=[1 -1]
    Xf=X; Xf(:,2)=flip*Xf(:,2); mx=mean(Xf,1); my=mean(truth,1);
    [U,~,Vv]=svd((Xf-mx)'*(truth-my)); R=Vv*diag([1,sign(det(Vv*U'))])*U';
    Xar=(R*Xf')'+(my-(R*mx')'); e=sqrt(mean(vecnorm(Xar-truth,2,2).^2));
    if e<best, best=e; Xa=Xar; end
end
fprintf('survey geometry (%s): %d anchors, RMS vs Kinect %.0f mm\n', tag, n, 1000*best);

% write anchors.json
S.dim = 2;
S.bounds = [min(Xa(:,1))-0.5, max(Xa(:,1))+0.5, min(Xa(:,2))-0.5, max(Xa(:,2))+0.5, 0, 3];
anch = struct('id',{},'x',{},'y',{},'z',{});
for k=1:n
    anch(k) = struct('id',ids(k),'x',round(Xa(k,1),4),'y',round(Xa(k,2),4),'z',opts.z);
end
S.anchors = anch;
[~,base] = fileparts(surveyFile);
S.layout = sprintf('self_survey_%s_%s', tag, base);

outFile = fullfile(dune.rootDir(),'results','survey_runs', sprintf('%s_anchors_%s.json', base, tag));
fh=fopen(outFile,'w'); fwrite(fh, jsonencode(S,'PrettyPrint',true)); fclose(fh);
fprintf('wrote %s\n', outFile);
end
