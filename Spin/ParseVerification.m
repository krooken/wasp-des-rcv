function results = ParseVerification(verificationOutput)

results = cell(size(verificationOutput,1),8);

depthregex = 'State-vector \d+ byte, depth reached (\d+), errors: \d+';
statesregex = '(\d+) states, stored';
transitionsregex = '(\d+(\.\d+e\+\d+)?) transitions \(= (stored|visited)\+matched\)';
memoryregex = '((\d|\.)+)\Wtotal actual memory usage';
timeregex = 'pan: elapsed time ((\d|\.)+) seconds';
errorregex = 'State-vector \d+ byte, depth reached \d+, errors: (\d+)';

for i = 1:size(verificationOutput, 1)
    name = verificationOutput(i,1);
    type = verificationOutput{i,2};
    result = verificationOutput{i,3};
    
    results{i,1} = name{1};
    results{i,2} = type;
    
    depth = regexp(result, depthregex, 'tokens');
    results{i,3} = depth{1}{1};
    
    states = regexp(result, statesregex, 'tokens');
    results{i,4} = states{1}{1};
    
    transition = regexp(result, transitionsregex, 'tokens');
    results{i,5} = transition{1}{1};
    
    memory = regexp(result, memoryregex, 'tokens');
    results{i,6} = memory{1}{1};
    
    time = regexp(result, timeregex, 'tokens');
    results{i,7} = time{1}{1};
    
    errors = regexp(result, errorregex, 'tokens');
    results{i,8} = errors{1}{1};
    
end

end