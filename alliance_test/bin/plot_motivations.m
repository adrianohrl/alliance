function plot_motivations(robot_id, tasks_id, filename)

	if nargin < 2
		error('Please, enter at least the desired robot_id and the desired tasks_id.');
	end;
	figure;
	n = rows(tasks_id);
	motivation_index = 7;
	threshold_index = 8;
	for j = 1 : n
		task_id = tasks_id{j}{1};
		if nargin == 2 || ~exist('filename') || isempty(filename)
			filename = ['../bag/' robot_id '-' task_id '-motivation-new.csv'];
		end;
		if exist(filename, 'file') ~= 2
   disp([filename ' does not exist.']);
			continue;
		end;
		try
			csv_file = csvread(filename, 1, 0);
			if rows(csv_file) == 0
				continue;
			end;
		catch e
			continue;
		end;
		t = csv_file(:, 1);
		subplot(n, 1, j);
		hold on;
		plot(t, csv_file(:, motivation_index), 'b');
		plot(t, csv_file(:, threshold_index), '--r');
		ylabel([robot_id '/' task_id]);
		grid on;
	end;	
	xlabel('t [s]');
	subplot(n, 1, 1);
	title(['Motivations of the ' robot_id]);
end
