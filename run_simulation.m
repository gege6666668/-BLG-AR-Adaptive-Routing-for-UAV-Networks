
---

### run_simulation.m 
%% BLG-AR Main Simulation Script
% Runs complete simulation: localization + adaptive beaconing + game routing

clc; clearvars; close all;

%% ========== 参数设置 ==========
% 网络参数
numBlindNodes = 160;
numAnchorNodes = 40;
communicationRange = 120;
xBounds = [0, 500]; yBounds = [0, 500]; zBounds = [100, 400];

% 定位算法参数（论文公式1-10）
RSSI0 = -40; n = 3;
noise_sigma = 1;
minAnchorThreshold = 3;
maxAcceptableError = 3.5;
beta = 0.5; gamma = 0.5;
eta = 0.1;

% 路由协议参数
sphericalAngle = 45;      % 球形筛选角度α
alpha1 = 0.4; alpha2 = 0.35; alpha3 = 0.25;

% 802.11ac协议参数
packetSize = 1024;        % Bytes
dataRate = 433e6;         % bps
P_tx = 0.5; P_rx = 0.3;   % W
T_pkt = packetSize*8 / dataRate;

% 仿真参数
deltaT = 0.5;
simTime = 200;

% 5组源-目的节点对
srcNodes = [1, 20, 35, 50, 75];
dstNodes = [101, 105, 110, 115, 120];
numPairs = length(srcNodes);

%% ========== 初始化节点 ==========
% 初始化锚节点
anchorIDs = num2cell((numBlindNodes+1):(numBlindNodes+numAnchorNodes))';
anchorPositions = [rand(numAnchorNodes,1)*diff(xBounds)+xBounds(1), ...
                   rand(numAnchorNodes,1)*diff(yBounds)+yBounds(1), ...
                   rand(numAnchorNodes,1)*diff(zBounds)+zBounds(1)];
anchorDirections = randn(numAnchorNodes,3);
anchorDirections = anchorDirections ./ vecnorm(anchorDirections,2,2);
anchorSpeeds = 70 + 30*rand(numAnchorNodes,1);
anchorVelocities = anchorDirections .* anchorSpeeds;

auxiliaryAnchors = struct(...
    'id', anchorIDs, ...
    'position', num2cell(anchorPositions,2), ...
    'velocity', num2cell(anchorVelocities,2), ...
    'confidence', num2cell(ones(numAnchorNodes,1)), ...
    'delta', num2cell(zeros(numAnchorNodes,1)));

% 初始化盲节点
blindIDs = num2cell(1:numBlindNodes)';
blindPositions = [rand(numBlindNodes,1)*diff(xBounds)+xBounds(1), ...
                  rand(numBlindNodes,1)*diff(yBounds)+yBounds(1), ...
                  rand(numBlindNodes,1)*diff(zBounds)+zBounds(1)];
blindDirections = randn(numBlindNodes,3);
blindDirections = blindDirections ./ vecnorm(blindDirections,2,2);
blindSpeeds = 70 + 30*rand(numBlindNodes,1);
blindVelocities = blindDirections .* blindSpeeds;

blindNodes = struct(...
    'id', blindIDs, ...
    'position', num2cell(blindPositions,2), ...
    'velocity', num2cell(blindVelocities,2));

%% ========== 初始化状态变量 ==========
located = false(numBlindNodes,1);
estimatedPositions = nan(numBlindNodes,3);
locErrors = nan(numBlindNodes,1);

% 性能指标（论文公式21-28）
packetsSent = zeros(1, numPairs);
packetsReceived = zeros(1, numPairs);
packetsForwarded = zeros(1, numPairs);
packetsRelayed = zeros(1, numPairs);
totalDelay = zeros(1, numPairs);

% 能量统计
totalTxPackets = 0;
totalRxPackets = 0;

packetID = 1;
lastHelloTime = 0;
helloInterval = 1.0;

%% ========== 主仿真循环 ==========
fprintf('========== BLG-AR Simulation Start ==========\n');

for currentTime = 0:deltaT:simTime
    round = currentTime/deltaT + 1;
    
    % 1. 更新节点位置
    [auxiliaryAnchors, blindNodes] = updatePositions(auxiliaryAnchors, blindNodes, ...
        located, xBounds, yBounds, zBounds, deltaT);
    
    % 2. 盲节点定位
    [located, estimatedPositions, locErrors, auxiliaryAnchors] = ...
        localization_main(blindNodes, auxiliaryAnchors, located, estimatedPositions, ...
        locErrors, communicationRange, minAnchorThreshold, RSSI0, n, noise_sigma, ...
        maxAcceptableError, beta, gamma, eta, xBounds, yBounds, zBounds);
    
    % 3. 自适应Becaon间隔
    if currentTime - lastHelloTime >= helloInterval
        helloInterval = adaptive_beacon(auxiliaryAnchors, blindNodes, located, ...
            estimatedPositions, helloInterval, deltaT);
        lastHelloTime = currentTime;
    end
    
    % 4. 生成数据包（每0.5秒）
    if mod(currentTime, 0.5) == 0
        for pairID = 1:numPairs
            src = srcNodes(pairID);
            dst = dstNodes(pairID);
            
            % 获取源节点位置
            if src <= numBlindNodes
                if ~located(src); continue; end
                srcPos = estimatedPositions(src,:);
            else
                idx = find([auxiliaryAnchors.id] == src, 1);
                if isempty(idx); continue; end
                srcPos = auxiliaryAnchors(idx).position;
            end
            
            % 获取目的节点位置
            if dst <= numBlindNodes
                if ~located(dst); continue; end
                dstPos = estimatedPositions(dst,:);
            else
                idx = find([auxiliaryAnchors.id] == dst, 1);
                if isempty(idx); continue; end
                dstPos = auxiliaryAnchors(idx).position;
            end
            
            % 创建数据包
            packet = struct(...
                'id', packetID, 'pairID', pairID, 'src', src, 'dst', dst, ...
                'srcPos', srcPos, 'dstPos', dstPos, 'generationTime', currentTime, ...
                'lastHop', src, 'ttl', 30, 'path', [src], 'size', packetSize);
            
            % 加入缓冲区
            if src <= numBlindNodes
                if ~isfield(blindNodes, 'buffer'); [blindNodes.buffer] = deal([]); end
                blindNodes(src).buffer = [blindNodes(src).buffer, packet];
            else
                idx = find([auxiliaryAnchors.id] == src, 1);
                if ~isfield(auxiliaryAnchors, 'buffer'); [auxiliaryAnchors.buffer] = deal([]); end
                auxiliaryAnchors(idx).buffer = [auxiliaryAnchors(idx).buffer, packet];
            end
            packetsSent(pairID) = packetsSent(pairID) + 1;
            packetID = packetID + 1;
        end
    end
    
    % 5. 数据包转发（调用game_routing模块）
    [auxiliaryAnchors, blindNodes, packetsReceived, totalDelay, ...
     packetsForwarded, packetsRelayed, totalTxPackets, totalRxPackets] = ...
        game_routing(auxiliaryAnchors, blindNodes, located, estimatedPositions, ...
        packetsReceived, totalDelay, packetsForwarded, packetsRelayed, ...
        totalTxPackets, totalRxPackets, communicationRange, sphericalAngle, ...
        alpha1, alpha2, alpha3, currentTime, numPairs);
    
    % 输出进度
    if mod(round, 50) == 0
        fprintf('Time: %.0fs | Located: %d/%d | Delivered: %d\n', ...
            currentTime, sum(located), numBlindNodes, sum(packetsReceived));
    end
end

%% ========== 性能评估==========
fprintf('\n========== BLG-AR Performance Evaluation ==========\n');

pairResults = zeros(numPairs, 5);  % [PDR, AvDelay, Throughput, RO, ~]

for pairID = 1:numPairs
    % Packet Delivery Ratio
    PDR = packetsReceived(pairID) / max(1, packetsSent(pairID));
    
    % Average End-to-End Delay
    if packetsReceived(pairID) > 0
        AvDelay = totalDelay(pairID) / packetsReceived(pairID);
    else
        AvDelay = 0;
    end
    
    %Throughput (Kbps)
    totalBytes = packetsReceived(pairID) * packetSize;
    Throughput = (totalBytes * 8) / simTime / 1000;
    
    %Routing Overhead
    N_S = max(1, packetsReceived(pairID));
    RO = (packetsForwarded(pairID) + packetsRelayed(pairID)) / N_S;
    
    pairResults(pairID, :) = [PDR, AvDelay, Throughput, RO, 0];
    
    fprintf('Pair %d (Node%d→%d): PDR=%.1f%%, Delay=%.3fs, Throughput=%.2fKbps, RO=%.2f\n', ...
        pairID, srcNodes(pairID), dstNodes(pairID), PDR*100, AvDelay, Throughput, RO);
end

% 平均性能
fprintf('\n========== Average Performance (5 Pairs) ==========\n');
fprintf('Avg PDR: %.2f%% (Eq.21)\n', mean(pairResults(:,1))*100);
fprintf('Avg Delay: %.4f s (Eq.22)\n', mean(pairResults(:,2)));
fprintf('Avg Throughput: %.2f Kbps (Eq.23)\n', mean(pairResults(:,3)));
fprintf('Avg Routing Overhead: %.2f (Eq.24)\n', mean(pairResults(:,4)));

% 能量消耗
E_tx = totalTxPackets * P_tx * T_pkt;   % 公式26
E_rx = totalRxPackets * P_rx * T_pkt;   % 公式27
E_total = E_tx + E_rx;                   % 公式25

fprintf('\n========== Energy Consumption (Eq.25-28) ==========\n');
fprintf('Total Tx Packets N_tx: %d\n', totalTxPackets);
fprintf('Total Rx Packets N_rx: %d\n', totalRxPackets);
fprintf('Packet Transmission Time T_pkt: %.6f s (Eq.28)\n', T_pkt);
fprintf('Tx Energy E_tx: %.2f J (Eq.26)\n', E_tx);
fprintf('Rx Energy E_rx: %.2f J (Eq.27)\n', E_rx);
fprintf('Total Energy E_total: %.2f J (Eq.25)\n', E_total);

% 绘图
figDir = 'results/figures';
if ~exist(figDir, 'dir'); mkdir(figDir); end

figure('Position', [100,100,1200,400]);
subplot(1,3,1);
valid = ~isnan(locErrors);
histogram(locErrors(valid), 20);
xlabel('Localization Error (m)'); ylabel('Node Count');
title('Blind Node Localization Error');

subplot(1,3,2);
bar(1:numPairs, pairResults(:,1)*100);
xlabel('Communication Pair'); ylabel('PDR (%)');
title('Packet Delivery Ratio (Eq.21)'); ylim([0 100]); grid on;

subplot(1,3,3);
bar(1:numPairs, pairResults(:,2));
xlabel('Communication Pair'); ylabel('Delay (s)');
title('End-to-End Delay (Eq.22)'); grid on;

saveas(gcf, fullfile(figDir, 'BLGAR_results.png'));
fprintf('\nFigures saved to: %s\n', fullfile(figDir, 'BLGAR_results.png'));
fprintf('\n========== BLG-AR Simulation Complete ==========\n');

%% ========== 辅助函数 ==========
function [aux, blind] = updatePositions(aux, blind, located, xB, yB, zB, dt)
    bounds = {xB, yB, zB};
    for i = 1:length(aux)
        aux(i) = moveNode(aux(i), bounds, dt);
    end
    for i = 1:length(blind)
        if ~located(i)
            blind(i) = moveNode(blind(i), bounds, dt);
        end
    end
end

function node = moveNode(node, bounds, dt)
    newPos = node.position + node.velocity * dt;
    for dim = 1:3
        if newPos(dim) < bounds{dim}(1) || newPos(dim) > bounds{dim}(2)
            node.velocity(dim) = -node.velocity(dim);
            newPos(dim) = min(max(newPos(dim), bounds{dim}(1)), bounds{dim}(2));
        end
    end
    node.position = newPos;
end
