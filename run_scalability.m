% run_scalability_RS.m
clc; clear; close all;

%% ------------------------------------------------------------------------
%  Paths and basic settings
% -------------------------------------------------------------------------
projectRoot = '/home/ibrahim/projects/reeds-shepp-marching';
configFile  = fullfile(projectRoot, 'config', 'settings.config');
scaledDir   = fullfile(projectRoot, 'scaled_maps');
destDir     = fullfile(projectRoot, 'scalability');

if ~exist(destDir, 'dir')
    mkdir(destDir);
end

executablePath = ['sudo ' fullfile(projectRoot, 'RS_marching')];

% Sizes to test (must match generated maps)
target_sizes = 100:100:2000;   % 100, 200, ..., 2000

% Number of repetitions per size
numRepeats = 5;

% Base problem (for 100x100)
N0        = 100;       % base grid size
r_base    = 5;         % base turning radius in config
x_s_base  = 10; y_s_base  = 10;
x_g_base  = 80; y_g_base  = 80;
theta_s   = 90; theta_g   = 90;   % degrees

%% ------------------------------------------------------------------------
%  Storage for all runs
% -------------------------------------------------------------------------
numTestsTotal = numel(target_sizes) * numRepeats;
results = table('Size', [numTestsTotal 5], ...
    'VariableTypes', {'double','double','double','double','double'}, ...
    'VariableNames', {'N','numCells','repeatIdx','marchTime_us','queryTime_us'});

row = 0;

%% Progress bar
hWait = waitbar(0, 'Running scalability experiments...');

%% ------------------------------------------------------------------------
%  Main loops: over sizes and repeats
% -------------------------------------------------------------------------
for idxN = 1:numel(target_sizes)
    N = target_sizes(idxN);
    scale = N / N0;

    % Scaled radius and frontline
    r_scaled   = r_base * scale;
    x_s_scaled = round(x_s_base * scale);
    y_s_scaled = round(y_s_base * scale);
    x_g_scaled = round(x_g_base * scale);
    y_g_scaled = round(y_g_base * scale);
    
    fprintf('\n=== Grid size: %dx%d, radius = %.3f ===\n', N, N, r_scaled);

    % Update config text once per size (then only rewrite it each repeat)
    cfgTextOriginal = fileread(configFile);

    for rIdx = 1:numRepeats
        % --- update config text for this size ---
        cfgText = cfgTextOriginal;

        imgPathN = fullfile(scaledDir, sprintf('map_%dx%d.png', N, N));

        % imagePath=...  (only that line)
        cfgText = regexprep(cfgText, 'imagePath=[^\r\n]*', ...
            ['imagePath=' imgPathN]);

        % radius=...  (only that line)
        cfgText = regexprep(cfgText, 'radius=[^\r\n]*', ...
            sprintf('radius=%g', r_scaled));

        % initialFrontline={...}
        frontlineStr = sprintf('initialFrontline={%d,%d,%d,%d,%d,%d}', ...
            x_s_scaled, y_s_scaled, theta_s, ...
            x_g_scaled, y_g_scaled, theta_g);
        cfgText = regexprep(cfgText, 'initialFrontline=\{[^\}]*\}', frontlineStr);

        % Write updated config
        fid = fopen(configFile, 'w');
        if fid == -1
            error('Could not open config file for writing: %s', configFile);
        end
        fwrite(fid, cfgText);
        fclose(fid);

        % --- run executable and capture console output ---
        fprintf('  Run %d/%d for %dx%d...\n', rIdx, numRepeats, N, N);
        [status, cmdout] = system(executablePath);

        if status ~= 0
            warning('Executable failed for size %dx%d, repeat %d. Status: %d', ...
                    N, N, rIdx, status);
        end

        % Optionally save raw console output for this run
        outLogPath = fullfile(destDir, ...
            sprintf('log_N%04d_rep%02d.txt', N, rIdx));
        fid = fopen(outLogPath, 'w');
        if fid ~= -1
            fwrite(fid, cmdout);
            fclose(fid);
        end

        % --- parse timings from cmdout ---
        % "Marching execution time in us: 2964us"
        marchTok = regexp(cmdout, ...
            'Marching execution time in us:\s*([0-9\.]+)us', ...
            'tokens', 'once');
        % "Forward simulation time in us: 354us"
        queryTok = regexp(cmdout, ...
            'Forward simulation time in us:\s*([0-9\.]+)us', ...
            'tokens', 'once');

        if isempty(marchTok)
            warning('Could not parse marching time for N=%d, repeat=%d', N, rIdx);
            march_us = NaN;
        else
            march_us = str2double(marchTok{1});
        end

        if isempty(queryTok)
            warning('Could not parse query time for N=%d, repeat=%d', N, rIdx);
            query_us = NaN;
        else
            query_us = str2double(queryTok{1});
        end

        % --- store this run ---
        row = row + 1;
        results.N(row)           = N;
        results.numCells(row)    = N^2;
        results.repeatIdx(row)   = rIdx;
        results.marchTime_us(row)= march_us;
        results.queryTime_us(row)= query_us;

        % Update progress bar
        waitbar(row/numTestsTotal, hWait, ...
            sprintf('Running scalability experiments... (%d/%d)', ...
                    row, numTestsTotal));
    end
end

close(hWait);

%% ------------------------------------------------------------------------
%  Save all raw results
% -------------------------------------------------------------------------
results = results(1:row, :);  % in case of early stop
writetable(results, fullfile(destDir, 'scalability_results_RS_all_runs.csv'));
save(fullfile(destDir, 'scalability_results_RS_all_runs.mat'), 'results');

disp('All runs completed and raw results saved.');

%% ------------------------------------------------------------------------
%  Compute averages per grid size
% -------------------------------------------------------------------------
uniqueNs  = unique(results.N);
numSizes  = numel(uniqueNs);
meanMarch = zeros(numSizes,1);
stdMarch  = zeros(numSizes,1);
meanQuery = zeros(numSizes,1);
stdQuery  = zeros(numSizes,1);
numCells  = uniqueNs.^2;

for i = 1:numSizes
    Ni   = uniqueNs(i);
    mask = results.N == Ni;
    marchVals = results.marchTime_us(mask);
    queryVals = results.queryTime_us(mask);

    meanMarch(i) = mean(marchVals, 'omitnan');
    stdMarch(i)  = std(marchVals,  'omitnan');
    meanQuery(i) = mean(queryVals, 'omitnan');
    stdQuery(i)  = std(queryVals,  'omitnan');
end

avgResults = table(uniqueNs, numCells, meanMarch, stdMarch, meanQuery, stdQuery, ...
    'VariableNames', {'N','numCells','meanMarch_us','stdMarch_us','meanQuery_us','stdQuery_us'});

writetable(avgResults, fullfile(destDir, 'scalability_results_RS_averaged.csv'));
save(fullfile(destDir, 'scalability_results_RS_averaged.mat'), 'avgResults');

%% ------------------------------------------------------------------------
%  Helper: style for journal-quality plots
% -------------------------------------------------------------------------
function style_axes()
    ax = gca;
    ax.FontName = 'Times';
    ax.FontSize = 18;
    ax.FontWeight = 'bold';
    ax.LineWidth = 1.8;
    box on;
end

%% ------------------------------------------------------------------------
%  Plot 1: log–log marching time vs pixels (averaged)
% -------------------------------------------------------------------------
figure;
loglog(avgResults.numCells, avgResults.meanMarch_us*1e-6, '-o', ...
    'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % s
hold on; grid on;

% Fit slope in log–log space
valid = ~isnan(avgResults.meanMarch_us);
p = polyfit(log10(avgResults.numCells(valid)), ...
            log10(avgResults.meanMarch_us(valid)), 1);
fitLine = 10.^polyval(p, log10(avgResults.numCells(valid))) * 1e-6;
loglog(avgResults.numCells(valid), fitLine, '--', 'LineWidth', 2);

xlabel('Number of grid cells $n$','Interpreter','latex');
ylabel('Marching time [s]','Interpreter','latex');
title('Scaling of Reeds--Shepp marching time','Interpreter','latex');

style_axes();

lgd = legend('Average over 5 runs', ...
             sprintf('Linear fit (slope = %.3f)', p(1)), ...
             'Location','NorthWest');
set(lgd, 'Interpreter','latex', 'FontSize',16, 'FontWeight','bold', ...
    'Box','on', 'LineWidth',1.5);

%% ------------------------------------------------------------------------
%  Plot 2: log–log query time vs pixels (averaged)
% -------------------------------------------------------------------------
figure;
loglog(avgResults.numCells, avgResults.meanQuery_us*1e-6, '-s', ...
    'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % s
hold on; grid on;

xlabel('Number of grid cells $n$','Interpreter','latex');
ylabel('Forward simulation (query) time [s]','Interpreter','latex');
title('Scaling of Reeds--Shepp path query time','Interpreter','latex');

style_axes();

lgd2 = legend('Average over 5 runs', 'Location','NorthWest');
set(lgd2, 'Interpreter','latex', 'FontSize',16, 'FontWeight','bold', ...
    'Box','on', 'LineWidth',1.5);
