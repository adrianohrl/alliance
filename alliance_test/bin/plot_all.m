clc;
clear all;
close all;

robots_id = {{'robot1'}; {'robot2'}; {'robot3'}; {'robot4'}; {'robot5'}};
tasks_id = {{'wander'}; {'border_protection'}; {'report'}};
for i = 1 : rows(robots_id)
  try
    plot_motivations(robots_id{i}{1}, tasks_id);
  catch e
    disp(e.message);
  end;
end;
