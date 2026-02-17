function [pos, grip, numSegments] = loadDemos_segmented(filename)
    %% 1. Setup and File Reading
    delimiter = '\t'; % Check if your file is actually tab-separated. Based on your paste, it might be spaces.
    startRow = 2;
    
    % Format string: 16 columns of floats. 
    % We end with %[^\n\r] to discard any extra trailing whitespace characters.
    formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

    fileID = fopen(filename,'r');
    
    % Read the data
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, ...
                'MultipleDelimsAsOne', true, ... % Added this to handle variable spacing
                'EmptyValue' ,NaN, ...
                'HeaderLines' ,startRow-1, ...
                'ReturnOnError', false);
    fclose(fileID);

    %% 2. Synchronization
    % Find the first index where Column 16 (first_packet) is non-zero
    idx = find(dataArray{1,16}, 1, 'first');
    
    % If no sync signal is found, default to the start (safety check)
    if isempty(idx)
        idx = 1;
    end

    %% 3. Extract Raw Synchronized Data
    % Get the raw vectors starting from 'idx' to the end
    rawX = dataArray{1,5}(idx:end);
    rawY = dataArray{1,6}(idx:end);
    rawZ = dataArray{1,7}(idx:end);
    
    % Get Gripper data (Column 15). Rounding ensures 0.000000 isn't treated differently from 0
    rawGr = round(dataArray{1,15}(idx:end)); 
    
    % Combine into a temporary matrix
    fullPos = [rawX, rawY, rawZ];
    
    %% 4. Segmentation Logic
    % Find indices where the gripper state changes. 
    % diff returns non-zero at index i if element i and i+1 are different.
    changePoints = find(diff(rawGr) ~= 0);
    
    % Calculate start and end indices for each segment
    segStarts = [1; changePoints + 1];
    segEnds   = [changePoints; length(rawGr)];
    
    % Pre-allocate outputs
    numSegments = length(segStarts);
    pos = cell(numSegments, 1);   % Cell array to store the coordinate matrices
    grip = zeros(numSegments, 1); % Vector to store the gripper state (0 or 1)
    
    % Loop through segments and slice the data
    for i = 1:numSegments
        s = segStarts(i);
        e = segEnds(i);
        
        % Store the position chunk for this segment
        pos{i} = fullPos(s:e, :);
        
        % Store the gripper state for this segment
        grip(i) = rawGr(s);
    end
end