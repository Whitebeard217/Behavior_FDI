% Read the Excel file
[num, txt, raw] = xlsread('Event_dataset.xlsx');

% Get the number of rows in the data
[numRows, ~] = size(raw);

% Initialize a column for the new labels
newLabels = zeros(numRows-1, 1);

% Process each row (excluding the header)
for i = 2:numRows
    % Get values from 6th and 7th columns
    val5 = raw{i, 5};
    if isnumeric(val5) && val5 > 0.09
        newLabels(i-1) = 1;

    elseif isnumeric(val5) && val5 < -0.09
        newLabels(i-1) = -1;
        %disp('Negative found')
    else
        newLabels(i-1) = 0;
    end
end
raw(2:end, end+1) = num2cell(newLabels);

% Add a header for the new column
raw{1, end} = 'New Labels';

% Write the updated data back to Excel
xlswrite('Event_dataset.xlsx', raw);

disp('Process completed. New labels added to the Excel file.');