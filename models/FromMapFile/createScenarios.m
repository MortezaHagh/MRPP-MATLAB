function scenarios = createScenarios(path)

    % path = 'Berlin_1_256-random-1.scen';

    fid = fopen(path);
    fgetl(fid);

    N = 1000;

    iLine = 1;
    scenarios = zeros(N, 4);
    tline = fgetl(fid);

    while ischar(tline)
        temp = split(tline);
        temp = temp(5:8);
        temp = str2double(temp)';

        scenarios(iLine, :) = temp;
        tline = fgetl(fid);
        iLine = iLine + 1;
    end

    fclose(fid);

end
