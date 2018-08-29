function results = VerifyModel(spinpath, gccpath, promelafile, neverclaims)

% Generate the pan file first
spincommand = {spinpath};
spinoptions = {'-a'};
filecommand = {promelafile};
[status, result] = system(strjoin([spincommand, spinoptions, filecommand],' '));

% Compile one file for safety and one for liveness verification
gcccommand = {gccpath};
safefile = 'pan_safe';
livefile = 'pan_live';
commonoptions = { ...
    '-DMEMLIM=3000', ...
    '-DXUSAFE', ...
    '-DCOLLAPSE', ...
    '-O2', ...
    '-w', ...
    'pan.c'};
safetyoptions = { ...
    '-DSAFETY', ...
    '-o',safefile};
livenessoptions = { ...
    '-DNOFAIR', ...
    '-o',livefile};
    
[status, result] = system(strjoin([gcccommand, commonoptions, safetyoptions],' '));
[status, result] = system(strjoin([gcccommand, commonoptions, livenessoptions],' '));

% Run the verification
safepan = {['.\' safefile]};
livepan = {['.\' livefile]};
commonoptions = { ...
    '-m100000', ...
    '-c1', ...
    '-N'};
livenessoptions = { ...
    '-a'};

results = cell(size(neverclaims,1),3);

errorregex = 'State-vector \d+ byte, depth reached \d+, errors: (\d+)';

for i = 1:size(neverclaims, 1)
    name = neverclaims(i,1);
    type = neverclaims{i,2};
    disp(['Starting verification of ', name{1}])
    if strcmpi(type,'liveness')
        command = strjoin([livepan, livenessoptions, commonoptions, name], ' ');
    elseif strcmpi(type,'safety')
        command = strjoin([safepan, commonoptions, name], ' ');
    else
        command = '';
    end
    [status, results{i,3}] = system(command);
    
    results{i,1} = name{1};
    results{i,2} = type;
    
    errors = regexp(results{i,3}, errorregex, 'tokens');
    
    disp(['Finished verification of ', name{1}, '. Errors: ', errors{1}{1}])
    disp(' ')
    
end

end

