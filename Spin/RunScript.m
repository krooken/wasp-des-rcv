
spinpath = 'C:\jspin\bin\spin.exe';
gccpath = 'C:\cygwin\bin\gcc.exe';
promelafile = 'SupervisoryPathController.pml';
neverclaims = { ...
    'missionComplete', 'Liveness'; ...
    'stopInTheEnd', 'Liveness'; ...
    'allPathsKnown', 'Safety'; ...
    'driveOnlyOnPaths', 'Safety'; ...
    'safeStop', 'Safety'; ...
    'unsafeStop', 'Safety'; ...
    'failure', 'Liveness'; ...
    'failToReachGoal', 'Safety'};

results = VerifyModel(spinpath, gccpath, promelafile, neverclaims);

parsedResults = ParseVerification(results);

latexCode = LatexifyResult(parsedResults);

fileh = fopen('table.txt', 'w');
fprintf(fileh, '%s', latexCode);
[~] = fclose(fileh);