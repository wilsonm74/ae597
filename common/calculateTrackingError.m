function trackingError = calculateTrackingError(leaderPos, viewerState, viewerAxisBody)

% Leader position in global frame
leaderPos = leaderPos(:);

% Viewer position in global frame
viewerPos = viewerState(1:3);
viewerPos = viewerPos(:);

% Line of sight from viewer to leader
LOS = leaderPos - viewerPos;

if norm(LOS) < 1e-8
    trackingError = 0;
    return;
end

LOS = LOS / norm(LOS);

% Convert viewer body axis into global frame
Rbg = mathBody2Global(viewerState);

viewerDirGlobal = Rbg * viewerAxisBody(:);
viewerDirGlobal = viewerDirGlobal / norm(viewerDirGlobal);

% Pointing error
cosError = dot(viewerDirGlobal, LOS);

% Numerical protection
cosError = max(-1, min(1, cosError));

trackingError = acosd(cosError);

end