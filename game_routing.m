function [auxiliaryAnchors, blindNodes, packetsReceived, totalDelay, ...
          packetsForwarded, packetsRelayed, totalTxPackets, totalRxPackets] = ...
    game_routing(auxiliaryAnchors, blindNodes, located, estimatedPositions, ...
    packetsReceived, totalDelay, packetsForwarded, packetsRelayed, ...
    totalTxPackets, totalRxPackets, commRange, sphericalAngleDeg, ...
    alpha1, alpha2, alpha3, currentTime, numPairs)
% GAME-THEORETIC PATH SELECTION
% Implements spherical candidate filtering + non-cooperative game

numBlindNodes = length(blindNodes);

% Ensure buffer fields exist
if ~isfield(auxiliaryAnchors, 'buffer')
    [auxiliaryAnchors.buffer] = deal([]);
end
if ~isfield(blindNodes, 'buffer')
    [blindNodes.buffer] = deal([]);
end

%% Process anchor node buffers
for i = 1:length(auxiliaryAnchors)
    if isempty(auxiliaryAnchors(i).buffer); continue; end
    
    for p = length(auxiliaryAnchors(i).buffer):-1:1
        packet = auxiliaryAnchors(i).buffer(p);
        pairID = packet.pairID;
        
        % Check if destination reached
        if packet.dst == auxiliaryAnchors(i).id
            packetsReceived(pairID) = packetsReceived(pairID) + 1;
            totalDelay(pairID) = totalDelay(pairID) + (currentTime - packet.generationTime);
            totalRxPackets = totalRxPackets + 1;
            auxiliaryAnchors(i).buffer(p) = [];
            continue;
        end
        
        % Check TTL
        if packet.ttl <= 0
            auxiliaryAnchors(i).buffer(p) = [];
            continue;
        end
        
        % Select next hop using game theory
        [nextHop, success] = selectNextHopGame(auxiliaryAnchors, blindNodes, located, ...
            estimatedPositions, packet, auxiliaryAnchors(i).id, commRange, ...
            sphericalAngleDeg, alpha1, alpha2, alpha3);
        
        if ~success
            auxiliaryAnchors(i).buffer(p) = [];
            continue;
        end
        
        % Forward packet
        packet.lastHop = auxiliaryAnchors(i).id;
        packet.path = [packet.path, nextHop];
        packet.ttl = packet.ttl - 1;
        
        totalTxPackets = totalTxPackets + 1;
        
        if nextHop <= numBlindNodes
            if ~isfield(blindNodes, 'buffer'); [blindNodes.buffer] = deal([]); end
            blindNodes(nextHop).buffer = [blindNodes(nextHop).buffer, packet];
            packetsRelayed(pairID) = packetsRelayed(pairID) + 1;
            totalRxPackets = totalRxPackets + 1;
        else
            idx = find([auxiliaryAnchors.id] == nextHop, 1);
            if ~isfield(auxiliaryAnchors, 'buffer'); [auxiliaryAnchors.buffer] = deal([]); end
            auxiliaryAnchors(idx).buffer = [auxiliaryAnchors(idx).buffer, packet];
            packetsRelayed(pairID) = packetsRelayed(pairID) + 1;
            totalRxPackets = totalRxPackets + 1;
        end
        packetsForwarded(pairID) = packetsForwarded(pairID) + 1;
        auxiliaryAnchors(i).buffer(p) = [];
    end
end

%% Process blind node buffers
for i = 1:numBlindNodes
    if ~located(i) || ~isfield(blindNodes, 'buffer') || isempty(blindNodes(i).buffer)
        continue;
    end
    
    for p = length(blindNodes(i).buffer):-1:1
        packet = blindNodes(i).buffer(p);
        pairID = packet.pairID;
        
        if packet.dst == blindNodes(i).id
            packetsReceived(pairID) = packetsReceived(pairID) + 1;
            totalDelay(pairID) = totalDelay(pairID) + (currentTime - packet.generationTime);
            totalRxPackets = totalRxPackets + 1;
            blindNodes(i).buffer(p) = [];
            continue;
        end
        
        if packet.ttl <= 0
            blindNodes(i).buffer(p) = [];
            continue;
        end
        
        [nextHop, success] = selectNextHopGame(auxiliaryAnchors, blindNodes, located, ...
            estimatedPositions, packet, blindNodes(i).id, commRange, ...
            sphericalAngleDeg, alpha1, alpha2, alpha3);
        
        if ~success
            blindNodes(i).buffer(p) = [];
            continue;
        end
        
        packet.lastHop = blindNodes(i).id;
        packet.path = [packet.path, nextHop];
        packet.ttl = packet.ttl - 1;
        
        totalTxPackets = totalTxPackets + 1;
        
        if nextHop <= numBlindNodes
            if ~isfield(blindNodes, 'buffer'); [blindNodes.buffer] = deal([]); end
            blindNodes(nextHop).buffer = [blindNodes(nextHop).buffer, packet];
            packetsRelayed(pairID) = packetsRelayed(pairID) + 1;
            totalRxPackets = totalRxPackets + 1;
        else
            idx = find([auxiliaryAnchors.id] == nextHop, 1);
            if ~isfield(auxiliaryAnchors, 'buffer'); [auxiliaryAnchors.buffer] = deal([]); end
            auxiliaryAnchors(idx).buffer = [auxiliaryAnchors(idx).buffer, packet];
            packetsRelayed(pairID) = packetsRelayed(pairID) + 1;
            totalRxPackets = totalRxPackets + 1;
        end
        packetsForwarded(pairID) = packetsForwarded(pairID) + 1;
        blindNodes(i).buffer(p) = [];
    end
end
end

%% ========== Helper Functions for Game Routing ==========

function [nextHop, success] = selectNextHopGame(auxiliaryAnchors, blindNodes, located, ...
    estimatedPositions, packet, currentNodeID, commRange, sphericalAngleDeg, ...
    alpha1, alpha2, alpha3)
% SELECT NEXT HOP USING GAME-THEORETIC DECISION

numBlind = length(blindNodes);

% Get current node position
if currentNodeID <= numBlind
    if ~located(currentNodeID)
        nextHop = -1; success = false; return;
    end
    currentPos = estimatedPositions(currentNodeID, :);
    currentVel = blindNodes(currentNodeID).velocity;
else
    idx = find([auxiliaryAnchors.id] == currentNodeID, 1);
    if isempty(idx)
        nextHop = -1; success = false; return;
    end
    currentPos = auxiliaryAnchors(idx).position;
    currentVel = auxiliaryAnchors(idx).velocity;
end

dstPos = packet.dstPos;
candidates = [];

% Collect anchor candidates
for i = 1:length(auxiliaryAnchors)
    nodeID = auxiliaryAnchors(i).id;
    if nodeID == currentNodeID || ismember(nodeID, packet.path); continue; end
    
    nodePos = auxiliaryAnchors(i).position;
    distToCurrent = norm(nodePos - currentPos);
    
    if distToCurrent <= commRange
        %Spherical screening
        if isInSphericalSector(currentPos, dstPos, nodePos, commRange, sphericalAngleDeg)
            candidates = [candidates; struct('id', nodeID, 'pos', nodePos, ...
                'vel', auxiliaryAnchors(i).velocity, 'isAnchor', true)];
        end
    end
end

% Collect blind node candidates
for i = 1:numBlind
    if ~located(i); continue; end
    nodeID = blindNodes(i).id;
    if nodeID == currentNodeID || ismember(nodeID, packet.path); continue; end
    
    nodePos = estimatedPositions(i, :);
    distToCurrent = norm(nodePos - currentPos);
    
    if distToCurrent <= commRange
        if isInSphericalSector(currentPos, dstPos, nodePos, commRange, sphericalAngleDeg)
            candidates = [candidates; struct('id', nodeID, 'pos', nodePos, ...
                'vel', blindNodes(i).velocity, 'isAnchor', false)];
        end
    end
end

if isempty(candidates)
    nextHop = -1; success = false; return;
end

%Calculate link lifetime for each candidate
linkLifetimes = zeros(length(candidates), 1);
for c = 1:length(candidates)
    d_ij = norm(currentPos - candidates(c).pos);
    v_ij = candidates(c).vel - currentVel;
    dot_product = dot(currentPos - candidates(c).pos, v_ij);
    v_sq = norm(v_ij)^2;
    
    if v_sq > 1e-6
        sqrt_term = dot_product^2 - v_sq * (d_ij^2 - commRange^2);
        if sqrt_term >= 0
            T_link = (-dot_product + sqrt(sqrt_term)) / v_sq;
            linkLifetimes(c) = max(0, T_link);
        else
            linkLifetimes(c) = commRange / max(0.1, norm(v_ij));
        end
    else
        linkLifetimes(c) = 1e6;  % Static relative motion
    end
end

%Calculate utility for each candidate
utilities = zeros(length(candidates), 1);
for c = 1:length(candidates)
    distToDst = norm(candidates(c).pos - dstPos);
    U1 = 1 / max(0.1, distToDst);
    U2 = linkLifetimes(c);
    U3 = 0;  % Simplified: competition factor
    
    utilities(c) = alpha1 * U1 + alpha2 * U2 - alpha3 * U3;
end

%Select node with maximum utility
[~, bestIdx] = max(utilities);
nextHop = candidates(bestIdx).id;
success = true;
end

function inSector = isInSphericalSector(currentPos, dstPos, nodePos, commRange, angleDeg)
%Spherical candidate filtering mechanism
% Checks if nodePos lies within the spherical sector centered at currentPos

vecToDst = dstPos - currentPos;
vecToNode = nodePos - currentPos;

if norm(vecToNode) > commRange
    inSector = false;
    return;
end

if norm(vecToDst) < 1e-6
    inSector = true;  % Destination is current node
    return;
end

cosAngle = dot(vecToNode, vecToDst) / (norm(vecToNode) * norm(vecToDst));
angleRad = acos(min(1, max(-1, cosAngle)));
angleDegActual = rad2deg(angleRad);

inSector = angleDegActual <= angleDeg;
end
