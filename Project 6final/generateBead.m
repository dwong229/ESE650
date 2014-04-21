function bead = generateBead(nbead,beadradii,limIJ)

% generate location of bead randomly
% at each iteration ensure bead distance is less than
% OUTPUT
% bead.center: [i,j]

imax = limIJ(1);
jmax = limIJ(2);


bead.radii = ones(nbead,1)*beadradii;
bead.stuck = false(nbead,1); % if bead is stuck, true.  Unknown or not stuck: false

beadsTooClose = true;

% limit range 
ijrange = limIJ - 2*[beadradii beadradii];


while beadsTooClose
    bead.center = bsxfun(@plus,floor(bsxfun(@times,rand(nbead,2),ijrange)),[beadradii beadradii]);
    distMat = dist(bead.center');
    distMat = distMat + eye(nbead)*(2*beadradii);
    
    if any(distMat(:)< 2*beadradii)
        beadsTooClose = true;
    else
        beadsTooClose = false;
    end
end