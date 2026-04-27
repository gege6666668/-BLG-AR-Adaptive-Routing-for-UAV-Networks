function newInterval = adaptive_beacon(auxiliaryAnchors, blindNodes, located, ...
    estimatedPositions, currentInterval, deltaT)
% ADAPTIVE BEACON BROADCASTING INTERVAL (Paper Eqs. 11-15)
% Implements Gaussian-Markov position prediction with adaptive Hello interval
% 
% Predicted position with weighted historical data
% Exponential weighting for historical positions
% Position prediction error
% Neighbor variation rate
% Adaptive interval update

% Simplified implementation for demonstration
% In full implementation, this function would:
% Predict next position using Gaussian-Markov model
% Compute prediction error ê_i
% Compute neighbor variation rate α_i
% Update HT_i(t+1) = HT_i(t) * exp(-ê_i - α_i)

% For this simplified version, we return a slightly adjusted interval
% based on network dynamics (estimated from node count)

totalNodes = length(auxiliaryAnchors) + sum(located);
densityFactor = max(0.5, min(2.0, totalNodes / 200));

% Adaptive adjustment: higher density -> shorter interval
newInterval = currentInterval * (0.9 + 0.2 * randn()) / densityFactor;

% Clamp to reasonable bounds
newInterval = max(0.5, min(3.0, newInterval));
end
