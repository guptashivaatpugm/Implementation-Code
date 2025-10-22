% Define method names
methods = {'DSA', 'TLBO', 'TDF-IMC', 'IMC-ADRC', 'IMC', 'IMC-PID', 'Proposed'};

% Define performance metric values
IAE  = [4.266e107, 7.238e145, 5.38e144, 9.36e6, 72.54468, 72.37669, 2.29653];
ITAE = [2.1074e110, 3.5731e148, 2.657e147, 4.20e6, 16817.89, 1697.08, 9.03];
ISE  = [2.252e214, 5.5683e290, 3.38e288, 1.22e6, 13.99757, 14.79607, 0.9983];
ITSE = [1.121e217, 2.767e293, 1.677e291, 5.76e6, 3604.396, 3835.056, 1.23];

metrics = {IAE, ITAE, ISE, ITSE};
titles = {'IAE Comparison', 'ITAE Comparison', 'ISE Comparison', 'ITSE Comparison'};
ylabels = {'IAE', 'ITAE', 'ISE', 'ITSE'};

for i = 1:4
    figure;
    bar(metrics{i});
    set(gca, 'YScale', 'log'); % Use logarithmic scale
    set(gca, 'XTickLabel', methods, 'FontName', 'Times New Roman', ...
        'FontSize', 14, 'FontWeight', 'bold');
    title(titles{i}, 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
    ylabel(ylabels{i}, 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
    xtickangle(45);
end
