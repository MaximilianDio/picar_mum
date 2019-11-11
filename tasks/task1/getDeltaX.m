%% Messungenauigkeit berechnen nach Parthier s. 83

function deltaX = getDeltaX(Messreihe)
%% Messreihe ist Vektror mit Messungen einer Messreihe
    deltaXMax = max(Messreihe)-min(Messreihe);
    
    p = ceil(sqrt(length(Messreihe)));
    
    deltaX = deltaXMax/(p+1);
end