function [labels] = generateLabels(auv)
%% generate labels for the plot from auv's id

labels = strings(auv.n + 1,1);
for i = 1 : auv.n
    labels(i) = string(auv.id(i));
end
labels(end) = 'R';
end