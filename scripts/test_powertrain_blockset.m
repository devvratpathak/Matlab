%% Test Powertrain Blockset Availability
fprintf('Testing Powertrain Blockset HEV P2 Reference Application...\n');
hev_p2_func = which('autoblkHevP2Start');
if ~isempty(hev_p2_func)
    fprintf('SUCCESS: autoblkHevP2Start found at:\n  %s\n', hev_p2_func);
else
    fprintf('NOTE: autoblkHevP2Start not found in default path.\n');
end

% Also test Virtual Vehicle Composer
vvc = which('virtualVehicleComposer');
if ~isempty(vvc)
    fprintf('SUCCESS: Virtual Vehicle Composer found at:\n  %s\n', vvc);
end

% Test Simscape Battery & Driveline
simscape_ver = ver('simscape');
if ~isempty(simscape_ver)
    fprintf('SUCCESS: Simscape version %s loaded.\n', simscape_ver.Version);
end
