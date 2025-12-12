% run_RS_benchmark_set.m
%
% Run the RS_marching solver for a set of start/goal configurations
% on the waffle_inflated map, save logs + primitives.json, and parse
% timing and distance information into a table.

clc; clear; close all;

%% ------------------------------------------------------------------------
%  Paths and basic settings
% -------------------------------------------------------------------------
projectRoot  = '/home/ibrahim/projects/reeds-shepp-marching';
configFile   = fullfile(projectRoot, 'config', 'settings.config');
imgPath      = fullfile(projectRoot, 'images', 'waffle_inflated.jpeg');
destDir      = fullfile(projectRoot, 'bench_RS_vs_RRT');  % <- change if desired
if ~exist(destDir, 'dir')
    mkdir(destDir);
end

executablePath = ['sudo ' fullfile(projectRoot, 'RS_marching')];

% Turning radius to use (must match your earlier experiments)
r_value = 10;  % adjust if needed

%% ------------------------------------------------------------------------
%  Define start/goal configurations
%  Each row: [xs, ys, thetas_deg, xg, yg, thetag_deg]
% -------------------------------------------------------------------------
frontlines = [
    200, 100,   0, 190, 500,   0;
    100, 470, 45.345, 336, 735, -132.435;
    421, 27, 15, 249, 417, -90;
    21, 281, 90, 522, 450, 0;
    238, 598, 0, 265, 30, 90;
    50, 50, 90, 150, 515, 90;
    65, 600, 179.324, 230, 375, 45;
    150, 150, 0, 335, 160, 0;
    20, 310, 90, 270, 50, 180;
    40, 600, 0, 525, 440, 180;
];

numRuns = size(frontlines, 1);

%% ------------------------------------------------------------------------
%  Storage for all runs
% -------------------------------------------------------------------------
results = table('Size', [numRuns 12], ...
    'VariableTypes', { ...
        'double','double','double','double','double','double', ... % xs, ys, thetas, xg, yg, thetag
        'double','double','double','double','double','double'  ... % marchTime_us, queryTime_us, p1, p2, minColl, totalDist
    }, ...
    'VariableNames', { ...
        'xs','ys','thetas_deg','xg','yg','thetag_deg', ...
        'marchTime_us','queryTime_us','p1','p2','minCollisions','totalDistance' ...
    });

%% ------------------------------------------------------------------------
%  Main loop over all start/goal configurations
% -------------------------------------------------------------------------
for k = 1:numRuns
    xs   = frontlines(k,1);
    ys   = frontlines(k,2);
    th_s = frontlines(k,3);
    xg   = frontlines(k,4);
    yg   = frontlines(k,5);
    th_g = frontlines(k,6);

    fprintf('\n=== Run %d/%d ===\n', k, numRuns);
    fprintf('Start: (%d,%d,%d), Goal: (%d,%d,%d)\n', xs,ys,th_s, xg,yg,th_g);

    % --- load original config text ---
    cfgTextOriginal = fileread(configFile);
    cfgText = cfgTextOriginal;

    % 1) Set imagePath to waffle_inflated.jpeg
    cfgText = regexprep(cfgText, 'imagePath=[^\r\n]*', ...
        ['imagePath=' imgPath]);

    % 2) Set radius
    cfgText = regexprep(cfgText, 'radius=[^\r\n]*', ...
        sprintf('radius=%g', r_value));

    % 3) Set initialFrontline
    frontlineStr = sprintf('initialFrontline={%d,%d,%d,%d,%d,%d}', ...
        xs, ys, th_s, xg, yg, th_g);
    cfgText = regexprep(cfgText, 'initialFrontline=\{[^\}]*\}', frontlineStr);

    % --- write updated config file ---
    fid = fopen(configFile, 'w');
    if fid == -1
        error('Could not open config file for writing: %s', configFile);
    end
    fwrite(fid, cfgText);
    fclose(fid);

    % --- run executable and capture console output ---
    fprintf('  Running RS_marching...\n');
    [status, cmdout] = system(executablePath);

    if status ~= 0
        warning('RS_marching failed for run %d (status=%d).', k, status);
    end

    % --- save raw console output ---
    logFile = fullfile(destDir, sprintf('RS_log_run%03d.txt', k));
    fid = fopen(logFile, 'w');
    if fid ~= -1
        fwrite(fid, cmdout);
        fclose(fid);
    else
        warning('Could not save log file for run %d.', k);
    end

    % --- copy primitives.json with unique name (if it exists) ---
    primSrc = fullfile(projectRoot, 'primitives.json');
    if exist(primSrc, 'file')
        primDst = fullfile(destDir, sprintf('primitives_run%03d.json', k));
        copyfile(primSrc, primDst);
    else
        warning('primitives.json not found after run %d.', k);
    end

    % --- parse timings from cmdout ---
    % "Marching execution time in us: 57518us"
    marchTok = regexp(cmdout, ...
        'Marching execution time in us:\s*([0-9\.]+)us', ...
        'tokens','once');

    % "Forward simulation time in us: 366us"
    queryTok = regexp(cmdout, ...
        'Forward simulation time in us:\s*([0-9\.]+)us', ...
        'tokens','once');

    % "p1: 4"  "p2: 1"
    p1Tok = regexp(cmdout, 'p1:\s*([0-9\.\-]+)', 'tokens','once');
    p2Tok = regexp(cmdout, 'p2:\s*([0-9\.\-]+)', 'tokens','once');

    % "Total distance: 532.029"
    distTok = regexp(cmdout, 'Total distance:\s*([0-9\.\-]+)', 'tokens','once');

    if isempty(marchTok)
        warning('Could not parse marching time for run %d.', k);
        march_us = NaN;
    else
        march_us = str2double(marchTok{1});
    end

    if isempty(queryTok)
        warning('Could not parse query time for run %d.', k);
        query_us = NaN;
    else
        query_us = str2double(queryTok{1});
    end

    if isempty(p1Tok)
        p1 = NaN;
    else
        p1 = str2double(p1Tok{1});
    end

    if isempty(p2Tok)
        p2 = NaN;
    else
        p2 = str2double(p2Tok{1});
    end

    if isempty(distTok)
        totalDist = NaN;
    else
        totalDist = str2double(distTok{1});
    end

    minCollisions = min(p1, p2);

    % --- store in results table ---
    results.xs(k)           = xs;
    results.ys(k)           = ys;
    results.thetas_deg(k)   = th_s;
    results.xg(k)           = xg;
    results.yg(k)           = yg;
    results.thetag_deg(k)   = th_g;

    results.marchTime_us(k) = march_us;
    results.queryTime_us(k) = query_us;
    results.p1(k)           = p1;
    results.p2(k)           = p2;
    results.minCollisions(k)= minCollisions;
    results.totalDistance(k)= totalDist;

    fprintf('  Parsed: march=%g us, query=%g us, p1=%g, p2=%g, dist=%g\n', ...
        march_us, query_us, p1, p2, totalDist);
end

%% ------------------------------------------------------------------------
%  Save structured results
% -------------------------------------------------------------------------
csvPath = fullfile(destDir, 'RS_benchmark_results.csv');
matPath = fullfile(destDir, 'RS_benchmark_results.mat');

writetable(results, csvPath);
save(matPath, 'results');

fprintf('\nAll runs completed. Results saved to:\n  %s\n  %s\n', csvPath, matPath);
