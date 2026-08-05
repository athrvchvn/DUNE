function w = gapWeights(gapDb, threshDb, floorW)
%GAPWEIGHTS NLOS soft weights from the rx-fp power gap (dB).
%   w = dune.gapWeights(gap)  with gap in dB. Same rule as the previous
%   pipeline: unity weight up to the LOS threshold, then 10^(-excess/10),
%   floored so a suspect anchor is de-emphasised but never fully dropped
%   (the MAD gate in multilaterate handles hard outliers).
%   NaN gap (no diagnostics) -> weight 1.

arguments
    gapDb double
    threshDb (1,1) double = 3
    floorW (1,1) double = 0.05
end
excess = max(0, gapDb - threshDb);
w = max(floorW, 10 .^ (-excess / 10));
w(isnan(gapDb)) = 1;
end
