params;


if logTrajectories == 1
    % Create a new folder with unique timestamp
    currTime = strrep(string(datetime), ':', '-');
    mkdir('Results', currTime)
    dataFolder = strcat('/Users/nshubham/all_ws/matlab_ws/DataGeneration-HumanRobot/Results/', currTime, '/');
    
    % Write Params
    parm = evalc('type params');
    
    filename = strcat(dataFolder, 'params.txt');   
    fid = fopen(filename,'w');    
    fprintf(fid,'%s',parm);          
    fclose(fid);
end




% Simulate N times and write data

for i=1:numSimulations
    disp(i)
    simulate(i, dataFolder);
    
end