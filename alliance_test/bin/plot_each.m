clc;
clear all;
close all;
robots_id = {{'robot1'}; {'robot2'}; {'robot3'}};
tasks_id = {{'wander'}; {'border_protection'}; {'report'}};
for i = 1 : rows(robots_id)
  for j = 1 : rows(tasks_id)
    try
      plot_motivation(robots_id{i}{1}, tasks_id{j}{1});
    catch e
      disp(e.message);
    end;
  end;
end;
