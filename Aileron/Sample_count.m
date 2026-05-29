% Read the Excel file
[num, txt, raw] = xlsread('Event_dataset.xlsx');
count0 = 0;
count1 = 0;
count2 = 0;
count3 = 0;
count4 = 0;
count5 = 0;
count6 = 0;
% Process each row (excluding the header)
for i = 2:numRows
    % Get values from 6th and 7th columns
    val9 = raw{i, 9};
    if isnumeric(val9) && val9 == 0
        count0 = count0+1;

    elseif isnumeric(val9) && val9 == 1
        count1 = count1+1;
        %disp('Negative found')
    elseif isnumeric(val9) && val9 == 2
        count2 = count2+1;
    elseif isnumeric(val9) && val9 == 3
        count3 = count3+1;
    elseif isnumeric(val9) && val9 == 4
        count4 = count4+1;
    elseif isnumeric(val9) && val9 == 5
        count5 = count5+1;
    elseif isnumeric(val9) && val9 == 6
        count6 = count6+1;
    end
end
count0
count1
count2
count3
count4
count5
count6