%%%%% Author: Anna Borsuk. %%%%%

% establish connection with the robot
a = arduino('COM9');

% switch pause option on
pause on;

% Sensors matrix:
%
%       US  F
% US0F0 0   0   -> sensorState 1
% US0F1 0   1   -> sensorState 2
% US1F0 1   0   -> sensorState 3
% US1F1 1   1   -> sensorState 4
%
% US - UltraSonic reading
% F - Front Flex sensors reading

Sensors = [0 0; 0 1; 1 0; 1 1];

% States matrix:
% SensorStates/Actions
%
%       F   L   R   
% US0F0 1   2   3   
% US0F1 4   5   6   
% US1F0 7   8   9  
% US1F1 10  11  12  
%
% F - Forward action    -> action 1
% L - Left action       -> action 2
% R - Right action      -> action 3

S = [1 2 3; 4 5 6; 7 8 9; 10 11 12];

numberOfStates = 12;
numberOfActions = 3;

% Reward matrix:
% prevSensorStates/currSensorStates
%
%       US0F0   US0F1   US1F0   US1F1
% US0F0 0       0       0       0
% US0F1 10      -10     0       -20
% US1F0 30      10      -10     -20
% US1F1 20      10      10      -20

% R = [0 0 0 0; 10 -10 0 -20; 30 10 -10 -20; 20 10 10 -20];


% Reward matrix:
% prevSensorStates/currSensorStates + Action taken
%
%       US0F0+F US0F0+L US0F0+R US0F1+F US0F1+L US0F1+R US1F0+F US1F0+L US1F0+R US1F1+F US1F1+L US1F1+R
% US0F0 20      0       0       -10     -10     -10     20      0       0       0       0       0
% US0F1 0       10      10      -10     -10     -10     0       10      10      -10     0       0      
% US1F0 0       30      30      -20     0       0       0       30      30      -30     0       0
% US1F1 0       20      20      -10     10      10      -10     20      20      -30     -10     -10

R = [20 0 0 -10 -10 -10 20 0 0 0 0 0; 0 10 10 -10 -10 -10 0 10 10 -10 0 0; 0 30 30 -20 0 0 0 30 30 -30 0 0; 0 20 20 -10 10 10 -10 20 20 -30 -10 -10];

% Reward matrix:
% prevSensorStates/Action taken
%
%       F   L   R
% US0F0 10  0   0
% US0F1 -10 10  10      
% US1F0 -10 20  20
% US1F1 -10 10  10

% R = [10 0 0; -10 10 10; -10 20 20; -10 10 10];

% Reward matrix:
% prevStates/Action taken
%
%       F   L   R
% 1     10  0   0
% 2     10  0   0    
% 3     10  0   0
% 4     -10 10  10
% 5     0   10  0
% 6     0   0   10
% 7     -20 10  10
% 8     -20 20  0
% 9     -20 0   20
% 10    -20 10  10
% 11    -20 20  0
% 12    -20 0   20

% R = [10 0 0; 10 0 0; 10 0 0; -10 10 10; 0 10 0; 0 0 10; -20 10 10; -20 20 0; -20 0 20; -20 10 10; -20 20 0; -20 0 20];


% Q matrix:
% States/avaliable action
%
%       F   L   R
% 1     0   0   0
% 2     0   0   0
% 3     0   0   0
% 4     0   0   0
% ...
% 12    0   0   0

Q = zeros(numberOfStates,numberOfActions,'double');

%%%%% initialise backup queue:
backupQueue = zeros(3,2,'int8');
backupQueue(1:3,1) = -1;

%%%%% initialise (learning) variables and constants:
numberOfEpisodes = 30;
numberOfTrials = 100;

alpha = 0.9;
gamma = 0.8;
epsilon = 0.2;

longTermReturn = 0;

%%%%% initialise plot matrices:
PlotData = zeros(numberOfEpisodes,1,'double');
ObstaclesPlot = zeros(numberOfEpisodes,1,'double');

%%%%% initialise diary file (eg. 09-Mar-2013_17:56:33.txt):
time = datestr(clock);
filename = strrep(time, ' ', '_');
filename = strrep(filename, ':', '-');
ext = '.txt';
file = strcat(filename, ext);

fid = fopen(file, 'w');

%%%%% set initial action:
action = 1;

%%%%% get initial state and initial sensor state:
[iniSensorState,iniState] = getState(a,Sensors,action,S);

%%%%% save to diary:
fprintf(fid, 'iniSensorState: ');
fprintf(fid, '%d \n', iniSensorState);
fprintf(fid, 'iniState: ');
fprintf(fid, '%d \n\n', iniState);

%%%%% print to screen:
fprintf('iniSensorState: ');
fprintf('%d \n', iniSensorState);
fprintf('iniState: ');
fprintf('%d \n\n', iniState);

%%%%% initial state becomes state and initial sensor state becomes sensor state:
state = iniState;
sensorState = iniSensorState;

%%%%% start episode:

for e=1:numberOfEpisodes
    
    %%%%% reset avoidance counters:
    detectedObstacles = 0;
    avoidedObstacles = 0;
    notAvoidedObstacles = 0;
    
    %%%%% reset longTermReturnSum:
    longTermReturnSum = 0;
    
    %%%%% save to diary:
    fprintf(fid, '### START EPISODE ### \n\n');
    
    fprintf(fid, 'Episode number: ');
    fprintf(fid, '%d \n\n', e);
    
    %%%%% print to screen:
    fprintf('Episode number: ');
    fprintf('%d \n\n', e);

    %%%%% start trial:
    
    for n=1:numberOfTrials
        
        %%%%% save to diary:
        fprintf(fid, '--- START TRIAL --- \n\n');
        
        fprintf(fid, 'Trial number: ');
        fprintf(fid, '%d \n\n', n);
        
        %%%%% print to screen:
        fprintf('Trial number: ');
        fprintf('%d \n\n', n);
        
        %%%%% save state in 1st queue slot:
        backupQueue(1,1) = state;
        
        %%%%% save to diary:
        fprintf(fid, 'State: ');
        fprintf(fid, '%d \n\n', state);
        
        %%%%% print to screen:
        fprintf('State: ');
        fprintf('%d \n\n', state);
        
        %%%%% save to diary:
        fprintf(fid, 'Q matrix:\n\n');        
        for i=1:numberOfStates
            fprintf(fid,'%f\t%f\t%f\n\n', Q(i,:));
        end  
        
        %%%%% epsilon-greedy policy:        
        rng('shuffle');
        random = rand(1);
        if (random < epsilon)
            % choose random action:
            action = randi(numberOfActions,1,1);
            
            %%%%% save to diary:
            fprintf(fid, 'Random action selected: ');
            fprintf(fid, '%d \n\n', action);
            
            %%%%% print to screen:
            fprintf('Random action selected: ');
            fprintf('%d \n\n', action);
        else
            % choose greedy action:
            greedyActions = zeros(numberOfActions,1,'int8');
            counter = 0;
            
            % get maxActionValue:
            maxActionValue = maxQAction(a,Q,state,numberOfActions);
            
            % remember actions where their value = maxActionValue:
            for i=1:numberOfActions
                if (Q(state,i) == maxActionValue)
                    counter = counter+1;
                    greedyActions(counter) = i;
                end
            end
            
            % if there is only one such action, choose it (randi(1,1,1) always gives 1); 
            % otherwise choose randomly between actions with the same max value:
            randomGreedyAction = randi(counter,1,1);
            action = greedyActions(randomGreedyAction);
            
            %%%%% save to diary:
            fprintf(fid, 'Greedy action selected: ');
            fprintf(fid, '%d \n\n', action);
            
            %%%%% print to screen:
            fprintf('Greedy action selected: ');
            fprintf('%d \n\n', action);
        end
        
        %%%%% perform selected action:        
        if (action == 1)
            %%%%% print to screen:
            fprintf('Forward Start \n');
            a.moveRobot('f',5);
            fprintf('Moved Forward \n');
            
            %%%%% save to diary:
            fprintf(fid, 'Moved Forward \n\n');
            %pause;
        elseif (action == 2)
            %%%%% print to screen:
            fprintf('Left Start \n');
            a.moveRobot('l',5);
            fprintf('Moved Left \n');
            
            %%%%% save to diary:
            fprintf(fid, 'Moved Left \n\n');
            %pause;
        elseif (action == 3)
            %%%%% print to screen:
            fprintf('Right Start \n');
            a.moveRobot('r',5);
            fprintf('Moved Right \n');
            
            %%%%% save to diary:
            fprintf(fid, 'Moved Right \n\n');
            %pause;
        end
        
        %%%%% now state becomes previous state and sensor state becomes previous sensor state:        
        prevState = state;
        prevSensorState = sensorState;
        
        %%%%% get current state:        
        [currSensorState,currState] = getState(a,Sensors,action,S);
        
        %%%%% save to diary:
        fprintf(fid, 'prevState: ');
        fprintf(fid, '%d \n', prevState);
        fprintf(fid, 'currState: ');
        fprintf(fid, '%d \n\n', currState);
        
        %%%%% print to screen:
        fprintf('prevState: ');
        fprintf('%d \n', prevState);
        fprintf('currState: ');
        fprintf('%d \n\n', currState);
        
        %%%%% get maximum current state Q action value:        
        %     maxCurrQAction = 0;
        %     for i=1:4
        %         maxCurrQAction = max([maxCurrQAction Q(currState,i)]);
        %     end
        
        maxCurrQAction = maxQAction(a,Q,currState,numberOfActions);
        
        %%%%% save to diary:
        fprintf(fid, 'maxCurrQAction: ');
        fprintf(fid, '%d \n\n', maxCurrQAction);
        
        %%%%% print to screen:
        fprintf('maxCurrQAction: ');
        fprintf('%d \n\n', maxCurrQAction);
        
        %%%%% check if robot saw the obstacle:        
        if (prevSensorState == 3)
            detectedObstacles = detectedObstacles + 1;
        end
        
        %%%%% check if robot saw the obstacle and bumped into it regardless:        
        if (prevSensorState == 3) && (currSensorState == 4)
            notAvoidedObstacles = notAvoidedObstacles + 1;
        end
        
        %%%%% check if robot avoided obstacle:        
        if (prevSensorState == 3) && (currSensorState == 1)
            avoidedObstacles = avoidedObstacles + 1;
        end
        
        %%%%% get the current state reward:        
        reward = R(prevSensorState, 3*(currSensorState-1)+action);  %%% for prevSensorState/currSensorState + action taken R matrix
        %reward = R(prevState, action);                             %%% for prevState/action taken R matrix
        %reward = R(prevSensorState,currSensorState);               %%% for prevSensorState/currSensorState R matrix
        %reward = R(prevSensorState, action);                       %%% for prevSensorState/action taken R matrix
        
        %%%%% save reward in backup queue:
        backupQueue(1,2) = reward;
        
        %%%%% save to diary:
        fprintf(fid, 'prevSensorState: ');
        fprintf(fid, '%d \n', prevSensorState);
        fprintf(fid, 'currSensorState: ');
        fprintf(fid, '%d \n\n', currSensorState);
        fprintf(fid, 'reward: ');
        fprintf(fid, '%d \n\n', reward);
        
        %%%%% print to screen:
        fprintf('reward: ');
        fprintf('%d \n\n', reward);
        
        %%%%% save to diary:
        fprintf(fid, 'Q(prevState,action) before update: ');
        fprintf(fid, '%f \n', Q(prevState,action));
        
        %%%%% print to screen:
        fprintf('Q(prevState,action) before update: ');
        fprintf('%f \n', Q(prevState,action));
        
        %%%%% update Q value of previous state after action taken:        
        Q(prevState,action) = Q(prevState,action) + alpha * (reward + gamma * maxCurrQAction - Q(prevState,action));
        
        %%%%% save to diary:
        fprintf(fid, 'Q(prevState,action) after update: ');
        fprintf(fid, '%f \n\n', Q(prevState,action));
        
        %%%%% print to screen:
        fprintf('Q(prevState,action) after update: ');
        fprintf('%f \n\n', Q(prevState,action));
        
        %%%%% save to diary:
        fprintf(fid, 'Q matrix after update: \n\n');
        for i=1:numberOfStates
            fprintf(fid,'%f\t%f\t%f\n\n', Q(i,:));
        end
        
        %%%%% print to screen:
        fprintf('Q matrix after update: \n\n');
        for i=1:numberOfStates
            fprintf('%f\t%f\t%f\n\n', Q(i,:));
        end
        
        %%%%% calculate long term return:        
        longTermReturn = reward + gamma * (reward + gamma * maxCurrQAction);        
        longTermReturnSum = longTermReturnSum + longTermReturn;
        
        %%%%% save to diary:
        fprintf(fid, 'longTermReturn: ');
        fprintf(fid, '%f \n\n', longTermReturn);
        
        % PlotData((e-1)*numberOfTrials + n) = longTermReturn;
        
        %%%%% do the backup:        
        if (backupQueue(2,1) ~= -1)
            state2 = backupQueue(2,1);                          % get state - act as previous state
            action2 = mod(backupQueue(1,1), numberOfActions);   % calculate action - act as action taken
            if action2 == 0
                action2 = numberOfActions;
            end
            maxCurrQAction2 = maxQAction(a,Q,backupQueue(1,1),numberOfActions);   % calculate maxQAction based on the following state - act as current state max Q action value
            reward2 = backupQueue(2,2);
            
            %%%%% save to diary:
            fprintf(fid, 'backupQueue(2,1) state2: ');
            fprintf(fid, '%d \n', state2);
            fprintf(fid, 'backupQueue(2,1) action2: ');
            fprintf(fid, '%d \n', action2);
            fprintf(fid, 'backupQueue(2,1) maxCurrQAction2: ');
            fprintf(fid, '%f \n', maxCurrQAction2);
            fprintf(fid, 'backupQueue(2,2) reward2: ');
            fprintf(fid, '%d \n', reward2);
            fprintf(fid, 'Q(state2,action2) before update: ');
            fprintf(fid, '%f \n', Q(state2,action2));
            
            %%%%% update the Q value using decay value:
            Q(state2,action2) = Q(state2,action2) + (0.5) * alpha * (reward2 + gamma * maxCurrQAction2 - Q(state2,action2));
            
            %%%%% save to diary:
            fprintf(fid, 'Q(state2,action2) after update: ');
            fprintf(fid, '%f \n\n', Q(state2,action2));
        end
        
        if (backupQueue(3,1) ~= -1)
            state3 = backupQueue(3,1);                          % get state - act as previous state
            action3 = mod(backupQueue(2,1), numberOfActions);   % calculate action - act as action taken
            if action3 == 0
                action3 = numberOfActions;
            end
            maxCurrQAction3 = maxQAction(a,Q,backupQueue(2,1),numberOfActions);   % calculate maxQAction based on the following state - act as current state max Q action value
            reward3 = backupQueue(3,2);
            
            %%%%% save to diary:
            fprintf(fid, 'backupQueue(3,1) state3: ');
            fprintf(fid, '%d \n', state3);
            fprintf(fid, 'backupQueue(3,1) action3: ');
            fprintf(fid, '%d \n', action3);
            fprintf(fid, 'backupQueue(3,1) maxCurrQAction3: ');
            fprintf(fid, '%f \n', maxCurrQAction3);
            fprintf(fid, 'backupQueue(3,2) reward3: ');
            fprintf(fid, '%d \n', reward3);
            fprintf(fid, 'Q(state3,action3) before update: ');
            fprintf(fid, '%f \n', Q(state3,action3));
            
            %%%%% update the Q value using decay value:
            Q(state3,action3) = Q(state3,action3) + (0.25) * alpha * (reward3 + gamma * maxCurrQAction3 - Q(state3,action3));
            
            %%%%% save to diary:
            fprintf(fid, 'Q(state3,action3) after update: ');
            fprintf(fid, '%f \n\n', Q(state3,action3));
        end
        
        % update backupQueue states:        
        backupQueue(3,1) = backupQueue(2,1);
        backupQueue(2,1) = backupQueue(1,1);
        backupQueue(1,1) = currState;
        
        % update backupQueue rewards:        
        backupQueue(3,2) = backupQueue(2,2);
        backupQueue(2,2) = backupQueue(1,2);
        backupQueue(1,2) = reward;
        
        %%%%% now current state becomes state and current sensor state becomes sensor state:        
        state = currState;
        sensorState = currSensorState;
        
        %%%%% save to diary:
        fprintf(fid, '--- END TRIAL --- \n\n');
        
    end
    
    %%%%% save data for plotting:    
    longTermReturnAvarage = longTermReturnSum / numberOfTrials;    
    PlotData(e) = longTermReturnAvarage;
    
    %%%%% save to diary:
    fprintf(fid, 'longTermReturnAvarage: ');
    fprintf(fid, '%f \n\n', longTermReturnAvarage);
    
    %%%%% print to screen:
    fprintf('longTermReturnAvarage: ');
    fprintf('%f \n\n', longTermReturnAvarage);
    
    %%%%% save obstacles data for plotting:    
    obstaclesRate = avoidedObstacles / detectedObstacles;    
    ObstaclesPlot(e) = obstaclesRate;
    
    %%%%% save to diary:
    fprintf(fid, 'detectedObstacles: ');
    fprintf(fid, '%d \n', detectedObstacles);
    fprintf(fid, 'avoidedObstacles: ');
    fprintf(fid, '%d \n', avoidedObstacles);
    fprintf(fid, 'notAvoidedObstacles: ');
    fprintf(fid, '%d \n', notAvoidedObstacles);
    fprintf(fid, 'obstaclesRate: ');
    fprintf(fid, '%f \n\n', obstaclesRate);
    
    fprintf(fid, '### END EPISODE ### \n\n');    
    
    fprintf('End Episode number: ');
    fprintf('%d \n\n', e);
    
    %%%%% pause to move robot back to start position:
    pause;
    
end

fclose(fid);

%%%%% save final Q matrix as .mat file (eg. 09-Mar-2013_17:56:33_Q.mat):
qname = '_Q';
qstring = strcat(filename, qname);
qext = '.mat';
qfile = strcat(qstring, qext);
save(qfile, 'Q');

%%%%% save PlotData as .mat file (eg. 09-Mar-2013_17:56:33_PlotData.mat):
pdname = '_PlotData';
pdstring = strcat(filename, pdname);
pdext = '.mat';
pdfile = strcat(pdstring, pdext);
save(pdfile, 'PlotData');

%%%%% plot data:
strArray = java_array('java.lang.String', numberOfEpisodes);

for s=1:numberOfEpisodes+1
	strArray(s) = java.lang.String(int2str(s-1));
end

xTickLabel = cell(strArray);

hold off;

figure = subplot(2,1,1);
plot(PlotData);

%set(gca,'XTick',0:numberOfTrials:numberOfEpisodes*numberOfTrials);
set(gca,'XTick',0:1:numberOfEpisodes);
set(gca,'XTickLabel',xTickLabel);
set(gca,'XGrid','on');

hold on;

%for p=1:numberOfEpisodes
%	plot(p*numberOfTrials, PlotData(p*numberOfTrials), 'ro');
%	annot = sprintf(' %f', PlotData(p*numberOfTrials));
%	text(p*numberOfTrials, PlotData(p*numberOfTrials), annot);
%end

title('Q-Learning - epsilon-greedy policy');
xlabel(['Number of episodes (' int2str(numberOfTrials) ' trials per episode)']);
ylabel('Long-term Return');

%%%%% plot obstacle data:
figure = subplot(2,1,2);
plot(ObstaclesPlot);

set(gca,'XTick',0:1:numberOfEpisodes);
set(gca,'XTickLabel',xTickLabel);
set(gca,'XGrid','on');

hold on;

title(['alpha: ' num2str(alpha) ' gamma: ' num2str(gamma) ' epsilon: ' num2str(epsilon)]);
xlabel(['Number of episodes (' int2str(numberOfTrials) ' trials per episode)']);
ylabel('Avoided Obstacles / Detected Obstacles');

%%%%% save plot as .fig file (eg. 09-Mar-2013_17:56:33_Plot.fig):
opname = '_Plot';
opfile = strcat(filename, opname);
hgsave(figure,opfile);

%%%%% save plot as .jpg file (eg. 09-Mar-2013_17:56:33_PlotJPG.jpg):
ojpgname = '_PlotJPG';
ojpgfile = strcat(filename, ojpgname);
saveas(figure,ojpgfile,'jpg')

pause off;


