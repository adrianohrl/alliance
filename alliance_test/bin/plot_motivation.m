function plot_motivation(robot_id, task_id, filename)

	if nargin < 2
		error('Please, enter at least the desired robot_id and the desired task_id.');
	end;
	if nargin == 2 || ~exist('filename') || isempty(filename)
		filename = ['../bag/' robot_id '-' task_id '-motivation.csv']; 
	end;
	if exist(filename, 'file') ~= 2
		error(['The input filename does not exist: ' filename ' .']);
	end;
	try
		csv_file = csvread(filename, 1, 0);
	catch
		disp(['No data available in:' filename]);
		return;
	end;
	t = csv_file(:, 1);
 	t = 1e-9 * (t - min(t) * ones(size(t)));
	fid = fopen(filename);
	csv_file_header = fgetl(fid);
	fclose(fid);
	new_filename = strrep(filename, '.csv', '-new.csv');
	fid = fopen(new_filename, 'w');
	fprintf(fid, '%s\n', csv_file_header);
	fclose(fid);
	csvwrite(new_filename, compress_motivation([t csv_file(:, 2 : end)]), '-append');
	ylabels = {'Impatience'; 'Acquiescent'; 'Suppressed'; 'Resetted'; 'Applicable'; 'Motivation'; 'Threshold'; 'Active'};
	for i = 1 : rows(ylabels)
		graphes.(ylabels{i}) = i + 1;
	end;
	ylabels = {{'Impatience'}; {'Acquiescent'}; {'Resetted'}; {'Motivation'; 'Threshold'}};
	n = rows(ylabels);
	figure;
	for i = 1 : n
		subplot(n, 1, i);
		hold on;
		for j = 1 : rows(ylabels{i})
			plot(t, csv_file(:, graphes.(ylabels{i}{1})));
		end;
		ylabel(ylabels{i}{1});
		grid on;
	end;
	xlabel('t [s]');
	subplot(n, 1, 1);
	title(['Motivation of the ' robot_id '/' task_id ' behaviour set']);
end
