% Read the Excel file
[num, txt, raw] = xlsread('Event_dataset.xlsx');

% Get the number of rows in the data
[numRows, ~] = size(raw);

% Initialize a column for the new labels
newLabels = zeros(numRows-1, 1);

% Process each row (excluding the header)
for i = 2:numRows
    % Get values from 6th and 7th columns
    val6 = raw{i, 6};
    val8 = raw{i, 8};
    
    % Assign labels based on conditions
    % Modify these conditions according to your specific requirements
    if val6 == 1 && val8 == 0
        newLabels(i-1) = 1;
    elseif val6 == -1 && val8 == 0
        newLabels(i-1) = 2;
    elseif val6 == 1 && val8 == 1
        newLabels(i-1) = 3;
        
    elseif val6 == -1 && val8 == 1
        newLabels(i-1) = 4;
    elseif val6 == 1 && val8 == -1
        newLabels(i-1) = 5;
    elseif val6 == -1 && val8 == -1
        newLabels(i-1) = 6;
    else
        newLabels(i-1) = 0;
    end
end

% Add the new labels to the raw cell array
raw(2:end, end+1) = num2cell(newLabels);

% Add a header for the new column
raw{1, end} = 'New Labels';

% Write the updated data back to Excel
xlswrite('Event_dataset.xlsx', raw);

disp('Process completed. New labels added to the Excel file.');