function randVals = genValsInRange(min, max, total, cols)
%This function generates random values in the given range
%params: min = lower val, max = upper val, total = total numbers generated
% cols = number of columns (use this for vectors etc)
randVals = (max - min).*rand(total,cols) + min;
end
