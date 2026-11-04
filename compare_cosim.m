%% Simulink / Gazebo comparison: the steps used in our co-simulation tests
% Run after a simulation, while 'out' is in the MATLAB workspace.
% The original model logs out.simu_states, out.gaz_states, and out.gz_time.
% These steps collect our timestamp checks, interpolation, plots, and errors
% in one script. They do not modify the model or shift either time axis.

%% Select the experiment being compared
% 'pitch': four separate theta/q/pn/h figures. 'vertical': separate h/w figures.
test_case = 'pitch';
% Our final pitch comparison used positive timestamps with no extra cutoff.
% For the later vertical run, change this explicitly to 0.060 s after inspecting
% raw startup samples. This is a documented exclusion, not automatic filtering.
comparison_start = 0;

%% Read raw logged signals
% .Time is Simulink logging time; gz_time.Data is Gazebo MESSAGE time.
t_sim = double(out.simu_states.Time(:));
x_sim = double(squeeze(out.simu_states.Data));
t_gazebo_log = double(out.gaz_states.Time(:));
t_gazebo = double(out.gz_time.Data(:));
x_gazebo = double(squeeze(out.gaz_states.Data));
% Timeseries can store vectors as N-by-12 or 12-by-N. Use rows as samples.
if ~isequal(size(x_sim),[numel(t_sim) 12]) && ...
        isequal(size(x_sim),[12 numel(t_sim)])
    x_sim = x_sim.';
end
if ~isequal(size(x_gazebo),[numel(t_gazebo) 12]) && ...
        isequal(size(x_gazebo),[12 numel(t_gazebo)])
    x_gazebo = x_gazebo.';
end
assert(isequal(size(x_sim),[numel(t_sim) 12]),'Unexpected Simulink state size.');
assert(isequal(size(x_gazebo),[numel(t_gazebo) 12]),'Unexpected Gazebo state size.');
assert(numel(t_gazebo_log)==numel(t_gazebo),'State/stamp logging lengths differ.');
assert(all(isfinite(t_sim)) && all(diff(t_sim)>0),'Invalid local time axis.');
assert(all(isfinite(x_sim(:))),'Nonfinite analytical state.');

%% Inspect acquisition timing BEFORE sorting or removing observations
finite_stamps = t_gazebo(isfinite(t_gazebo));
resets = sum(diff(finite_stamps)<0);
positive = isfinite(t_gazebo) & t_gazebo>0;
fprintf('Final Simulink time: %.6f s\n',t_sim(end));
assert(any(positive),'No positive Gazebo timestamps received.');
fprintf('Final positive Gazebo timestamp: %.6f s\n', ...
    t_gazebo(find(positive,1,'last')));
fprintf('Gazebo timestamp resets: %d\n',resets);
fprintf('Distinct positive Gazebo timestamps: %d\n',numel(unique(t_gazebo(positive))));
assert(resets==0,'A timestamp reset occurred; separate runs before comparing.');

%% Preserve raw data, including startup spikes and held messages
% Use a new filename for each experiment when saving. Do not save live ROS
% node/publisher/subscriber objects as a substitute for the code/configuration.
% Example, uncomment with your chosen new filename:
% save('pitch_run_01_numeric.mat','t_sim','x_sim','t_gazebo_log', ...
%      't_gazebo','x_gazebo','-v7');

%% Retain distinct measurements inside the common time interval
% Zero stamps are uninitialized subscriber outputs. Repeated stamps are held
% measurements and should not receive additional weight in error calculations.
eligible = positive & all(isfinite(x_gazebo),2) & ...
    t_gazebo>=t_sim(1) & t_gazebo<=t_sim(end);
source_rows = find(eligible);
[t_unique,first] = unique(t_gazebo(eligible),'stable');
assert(numel(t_unique)>=2,'Too few measurements in the common time interval.');
gz_unique = x_gazebo(source_rows(first),:);
% Interpolate analytical states at actual Gazebo timestamps. No extrapolation,
% fitted delay, clock scaling, or smoothing is applied.
sim_at_gz = interp1(t_sim,x_sim,t_unique,'linear');

%% Show early raw observations so a startup anomaly remains visible
n = min(10,numel(t_unique));
disp(table(t_unique(1:n),gz_unique(1:n,3),sim_at_gz(1:n,6),gz_unique(1:n,6), ...
    'VariableNames',{'GazeboTime','Gazebo_h','Simulink_w','Gazebo_w'}));
keep = t_unique>=comparison_start-1e-9;
assert(nnz(keep)>=2,'Too few observations after the selected comparison start.');
t_compare = t_unique(keep);
sim_compare = sim_at_gz(keep,:);
gz_compare = gz_unique(keep,:);
fprintf('Retained interval: %.6f to %.6f s; N=%d\n', ...
    t_compare(1),t_compare(end),numel(t_compare));

%% Select state components: [pn pe h u v w phi theta psi p q r]
switch lower(test_case)
    case 'pitch'
        columns = [8 11 1 3];
        names = {'Pitch_angle_rad';'Pitch_rate_rad_per_s'; ...
            'North_position_m';'Altitude_m'};
        titles = {'Pitch angle','Body pitch rate','North position','Altitude'};
        ylabels = {'\theta (rad)','q (rad/s)','p_n (m)','h (m)'};
    case 'vertical'
        columns = [3 6];
        names = {'Altitude_h';'Vertical_velocity_w'};
        titles = {'Altitude','Body vertical velocity'};
        ylabels = {'h (m)','w (m/s)'};
    otherwise
        error('Set test_case to pitch or vertical.');
end

%% Calculate raw and retained errors with the same sign convention
% Error = Gazebo state minus interpolated analytical state. RMSE gives each
% distinct retained timestamp equal weight; MaxAbsoluteError is the worst |e|.
e_raw = gz_unique(:,columns)-sim_at_gz(:,columns);
e = gz_compare(:,columns)-sim_compare(:,columns);
Raw_RMSE = sqrt(mean(e_raw.^2,1)).';
Raw_MaxAbsoluteError = max(abs(e_raw),[],1).';
raw_errors = table(Raw_RMSE,Raw_MaxAbsoluteError,'RowNames',names);
RMSE = sqrt(mean(e.^2,1)).';
MaxAbsoluteError = max(abs(e),[],1).';
errors = table(RMSE,MaxAbsoluteError,'RowNames',names);
disp('Raw common-interval errors before the stated startup exclusion:');
disp(raw_errors);
disp('Retained-interval errors for the report:');
disp(errors);

%% Draw each comparison in its OWN figure, as in our final report plots
for k = 1:numel(columns)
    c = columns(k);
    figure('Color','w');
    plot(t_compare,sim_compare(:,c),'b-','LineWidth',1.5); hold on;
    plot(t_compare,gz_compare(:,c),'r--','LineWidth',1.5);
    grid on;
    xlabel('Simulation time (s)'); ylabel(ylabels{k});
    title([titles{k} ': Simulink vs Gazebo']);
    legend('Simulink','Gazebo','Location','best');
    set(gca,'FontSize',12); xlim([t_compare(1) t_compare(end)]);
end
% For the report, save the desired figure through MATLAB's figure Save/Export
% controls. Source data and initial conditions must match the figure caption.
