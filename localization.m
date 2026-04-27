function [located, estimatedPositions, locErrors, auxiliaryAnchors] = ...
    localization_main(blindNodes, auxiliaryAnchors, located, estimatedPositions, ...
    locErrors, commRange, minAnchorThreshold, RSSI0, n, noise_sigma, ...
    maxAcceptableError, beta, gamma, eta, xBounds, yBounds, zBounds)
% BLIND NODE LOCALIZATION 
%
% Implements:
%  RSSI-based distance estimation with Gaussian noise
%  Linearized localization equation system
%  Corrected distance with delta factor
%  Dynamic error correction for delta update
%  Upgrade threshold condition (tau = v * delta_t)
%  Exponential confidence function
%  Alpha parameter calculation
%  Weighted least squares solution
%
% Inputs:
%   blindNodes       - struct array with fields: id, position, velocity
%   auxiliaryAnchors - struct array with fields: id, position, velocity, confidence, delta
%   located          - boolean array (numBlindNodes x 1)
%   estimatedPositions - (numBlindNodes x 3) matrix
%   locErrors        - (numBlindNodes x 1) vector
%   commRange        - communication range (R)
%   minAnchorThreshold - minimum number of anchors required (k >= 3 for 3D)
%   RSSI0, n, noise_sigma - RSSI model parameters
%   maxAcceptableError - threshold tau (Eq.7)
%   beta, gamma      - confidence decay parameters (Eq.9)
%   eta              - learning rate for delta update (Eq.5)
%   xBounds, yBounds, zBounds - spatial boundaries [min, max]
%
% Outputs:
%   located, estimatedPositions, locErrors - updated
%   auxiliaryAnchors - updated with new delta values and upgraded nodes

numBlindNodes = length(blindNodes);

% === Ensure original anchor nodes have confidence = 1 ===
% ( For original anchor nodes, confidence is set to 1)
for i = 1:length(auxiliaryAnchors)
    if auxiliaryAnchors(i).confidence == 0
        auxiliaryAnchors(i).confidence = 1;
    end
end

% === Optional: RSSI filtering parameters ===
RSSI_min = -100;  % Minimum realistic RSSI value (dBm)
RSSI_max = -20;   % Maximum realistic RSSI value (dBm)

for i = 1:numBlindNodes
    if located(i)
        continue;
    end
    
    blindPos = blindNodes(i).position;
    blindVel = blindNodes(i).velocity;
    
    % === Get all anchors within communication range ===
    anchorPosAll = vertcat(auxiliaryAnchors.position);
    dists = vecnorm(anchorPosAll - blindPos, 2, 2);
    validMask = dists <= commRange;
    
    if sum(validMask) < minAnchorThreshold
        continue;  % Insufficient anchors, cannot localize
    end
    
    validAnchors = auxiliaryAnchors(validMask);
    validDists = dists(validMask);
    numValidAnchors = length(validAnchors);
    
    % ===RSSI-based distance estimation with Gaussian noise ===
    RSSI_measured = RSSI0 - 10*n*log10(validDists) + noise_sigma * randn(size(validDists));
    
    % === Optional: RSSI outlier filtering ===
    % Clamp RSSI to realistic range to avoid extreme distance estimates
    RSSI_measured = max(RSSI_min, min(RSSI_max, RSSI_measured));
    
    % Convert RSSI to estimated distance
    d_estimated = 10.^((RSSI0 - RSSI_measured) / (10*n));
    
    % Ensure positive distances
    d_estimated = max(d_estimated, 0.1);
    
    % ===  Dynamic error correction ===
    deltas = [validAnchors.delta]';
    d_corrected = d_estimated + deltas;
    
    % Ensure corrected distances are positive
    d_corrected = max(d_corrected, 0.1);
    
    % === Build localization equation system ===
    p = vertcat(validAnchors.position);
    k = size(p, 1);  % Number of anchors (k >= 3)
    
    A = zeros(k-1, 3);
    b = zeros(k-1, 1);
    
    for j = 2:k
        % Eq.(3) formulation: 2(p1 - pj)^T * p_hat = d_j^2 - d_1^2 + ||p1||^2 - ||pj||^2
        A(j-1, :) = 2 * (p(1, :) - p(j, :));
        b(j-1) = d_corrected(j)^2 - d_corrected(1)^2 + ...
                  (p(1,1)^2 + p(1,2)^2 + p(1,3)^2) - ...
                  (p(j,1)^2 + p(j,2)^2 + p(j,3)^2);
    end
    
    % ===  Weighted least squares with confidence ===
    confidences = [validAnchors.confidence]';
    
    % Weight matrix (excluding reference anchor)
    if numValidAnchors > 1
        W = diag(confidences(2:end));
    else
        W = 1;
    end
    
    %  Weighted least squares solution
    try
        estimatedPos = (A' * W * A) \ (A' * W * b);
    catch ME
        warning('Matrix singular for node %d, using OLS', i);
        estimatedPos = (A' * A) \ (A' * b);
    end
    
    % === Apply boundary constraints ===
    estimatedPos(1) = min(max(estimatedPos(1), xBounds(1)), xBounds(2));
    estimatedPos(2) = min(max(estimatedPos(2), yBounds(1)), yBounds(2));
    estimatedPos(3) = min(max(estimatedPos(3), zBounds(1)), zBounds(2));
    
    % === Compute localization error ===
    locError = norm(estimatedPos - blindPos);
    
    % ===Check if error is below threshold for upgrade ===
    if locError <= maxAcceptableError
        % Mark as located
        located(i) = true;
        estimatedPositions(i, :) = estimatedPos;
        locErrors(i) = locError;
        
        % ===  Calculate alpha for confidence ===
        % alpha = -ln(beta) / (gamma * tau)
        % where tau = maxAcceptableError
        alpha = -log(beta) / (gamma * max(maxAcceptableError, 0.1));
        
        % ===  Confidence value ===
        % C_i = exp(-alpha * e_i)
        nodeConfidence = exp(-alpha * locError);
        
        % === Update anchor delta values (error correction) ===
        for j = 1:numValidAnchors
            anchorID = validAnchors(j).id;
            anchorIdx = find([auxiliaryAnchors.id] == anchorID, 1);
            
            if ~isempty(anchorIdx)
                trueDist = validDists(j);
                oldDelta = auxiliaryAnchors(anchorIdx).delta;
                
                % Ranging error calculation
                rangingError = abs(trueDist - d_estimated(j));
                
                % Exponential moving average update
                auxiliaryAnchors(anchorIdx).delta = (1 - eta) * oldDelta + eta * rangingError;
            end
        end
        
        % === Upgrade blind node to auxiliary anchor ===
        newDirection = randn(1, 3);
        if norm(newDirection) > 0
            newDirection = newDirection / norm(newDirection);
        else
            newDirection = [1, 0, 0];
        end
        
        % Random speed between 70-100 m/s (consistent with blind node speeds)
        newSpeed = 70 + 30 * rand();
        newVelocity = newDirection * newSpeed;
        
        % Add to auxiliary anchor pool
        auxiliaryAnchors(end+1) = struct(...
            'id', blindNodes(i).id, ...
            'position', estimatedPos, ...
            'velocity', newVelocity, ...
            'confidence', nodeConfidence, ...
            'delta', 0);  % Initial delta = 0 as per paper
    end
end
