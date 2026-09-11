%% Check MATLAB and Toolbox Environment
v = ver;
fprintf('=======================================================\n');
fprintf('     MATLAB R2026a INSTALLED TOOLBOX AUDIT             \n');
fprintf('=======================================================\n');
for k = 1:length(v)
    fprintf('- %-35s (v%s)\n', v(k).Name, v(k).Version);
end
fprintf('=======================================================\n');
