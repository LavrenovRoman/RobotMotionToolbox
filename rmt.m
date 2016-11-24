%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing 
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function ret = rmt(action,~)

% robot motion toolbox

% actions:

%  ini:  figure and controls


if nargin<1,
    action='ini';
end;


thisfile='rmt';

switch action

    %================
    % INITIALIZATION
    %================
    case 'ini'  % Initialize figure and controls

        figpri=figure( ...
            'Name','Robot Motion Toolbox', ...
            'Position',[35 50 1100 600], ...
            'NumberTitle','off', ...
            'MenuBar','none',...
            'ToolBar','auto',...
            'Visible','off'...
            );
        ret = figpri;

        set(figpri, 'Visible','on');
        rmt('ini_UserData');

        %default values
        data.initial = [2 2];
        data.final = [5 5];
        data.orientation = 0;
        temp(1) = 0.8;
        temp(2) = 0.5;
        temp(3) = 0.2;
        temp(4) = 0.01;
        data.pi_tuning = temp; 
        data.epsilonvoronoi = 0.2;
        set(gcf,'UserData',data);      
        
        
        set(gcf,'UserData',data);

    %MOBILE ROBOT TOOLBOX
        %uicontrol( ...
        %    'Style','text', ...
        %    'Units','normalized', ...
        %    'BackgroundColor',[0.80 0.80 0.80], ...
        %    'ListboxTop',0, ...
        %    'HorizontalAlignment','center',...
        %    'FontName','Helvetica',...
        %    'FontSize',16,...
        %    'FontUnits','points',...
        %    'FontWeight','normal',...
        %    'Position',[0.02    0.96   0.28    0.02], ...
        %    'String','MOBILE ROBOT TOOLBOX');

        %axes 1 --> workspace
        handle_axes = axes('position',[0.35    0.37    0.63    0.566]);
        data.frame_limits = [0 20 0 10]; %
        data.handle_env = handle_axes;
        set(data.handle_env,'xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');


        %axes 2 --> orientation
        handle_axes = axes('position',[0.35    0.048    0.19    0.2477]);
        data.handle_ori = handle_axes;
        hold on;%new
        set(data.handle_ori,'xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
        
        %axes 3 --> linear velocity
        handle_axes = axes('position',[0.573    0.048    0.19    0.2477]);
        data.handle_vel = handle_axes;
        hold on;
        set(data.handle_vel,'xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
        
        %axes 4 --> steering angle
        handle_axes = axes('position',[0.79    0.048    0.19    0.2477],'Tag','ang');
        data.handle_ang = handle_axes;
        hold on;
        set(data.handle_ang,'xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
        
        
        a = uimenu('Label','File');
        uimenu(a,'Label','&Open','Callback',strcat(thisfile,'(''open'')'));
        uimenu(a,'Label','&Save','Callback',strcat(thisfile,'(''save'')'),'Separator','on');
        
        a = uimenu('Label','Setup');
        uimenu(a,'Label','&Environment limits','Callback',strcat(thisfile,'(''environment_limits'')'));        
        uimenu(a,'Label','&Robot initial and final position','Callback',strcat(thisfile,'(''robot_initial'')'), 'Separator', 'on');
        uimenu(a,'Label','&Epsilon Voronoi','Callback',strcat(thisfile,'(''EpsilonVoronoi'')'), 'Separator','on');
        uimenu(a,'Label','&PI tuning parameters','Callback',strcat(thisfile,'(''PI_tuning'')'), 'Separator','on');
        
        
        a = uimenu('Label','Help');
        uimenu(a,'Label','&About RMTool','Callback',strcat(thisfile,'(''help_menu'')'));                
        
        %frame path planning
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.01    0.70   0.1575   0.29], ...
            'BackgroundColor',[0.70 0.70 0.70]);
        
        %frame criterias
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.175    0.70   0.1575   0.29], ...
            'BackgroundColor',[0.70 0.70 0.70]);
        
        %path planning approach
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015   0.88   0.15    0.0952], ...
            'String','Path Planning Approach:');
        
        %criterias of optimization
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.18   0.88   0.15    0.0952], ...
            'String','Criterias of optimization:');
        
        %length of path
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.18   0.84   0.15    0.0952], ...
            'String','length of path');
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.18   0.805   0.15    0.0952], ...
            'String','0');
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.305   0.805   0.025    0.0952], ...
            'String','100');
        uicontrol( ...
            'Style','slider', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'Position',[0.18   0.845   0.15    0.025], ...
            'Min',0, ...
            'Max',100, ...
            'Value', 50, ...
            'Tag', 'crit_length', ...
            'SliderStep', [1/100 5/100], ...
            'Callback',strcat(thisfile,'(''crit_length'')'));
        
        
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.015   0.85   0.15    0.0952], ...
            'Tag', 'pathpoints', ...
            'CallBack',strcat(thisfile,'(''change_planning_approach'')'), ...
            'String','Cell Decomposition|Visibility Graph|Voronoi Diagram|Manual points'); % |variable step saving data');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.015   0.81   0.15    0.0952], ...
            'Tag', 'cells', ...
            'CallBack',strcat(thisfile,'(''change_cell_type'')'), ...
            'String','Triangular cell|Rectangular cell|Polytopal cell|Trapezoidal cell'); % |variable step saving data');

        %intermediate trajectory points
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.76    0.15    0.0952], ...
            'String','Intermediate points:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.015    0.73    0.15    0.0952], ...
            'Tag', 'waypoints', ...
            'String','Middle points|Norm 1|Norm 2|Norm Inf.'); % |variable step saving data');
        
        %car-type
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.015    0.68    0.15    0.0952], ...
            'Tag', 'car_type', ...
            'String','Car-like|Differential-drive'); % |variable step saving data');
        
        
        %environment button
        
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.08    0.14    0.1517    0.05], ...
            'CallBack',strcat(thisfile,'(''run_environment'')'), ...
            'String','Environment');
        
        %path planning button
        
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.08    0.08    0.1517    0.05], ...
            'Tag','path_planning_button',...
            'CallBack',strcat(thisfile,'(''run_path_planning'')'), ...
            'String','Path Planning');

        %control button
        
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.08    0.02    0.1517    0.05], ...
            'CallBack',strcat(thisfile,'(''run_control'')'), ...
            'String','Motion Control');

        
    %    %radiobutton triangular cell
    %    uicontrol( ...
    %        'Style','radiobutton', ...
    %        'Units','normalized', ...
    %        'BackgroundColor',[0.80 0.80 0.80], ...
    %        'ListboxTop',0, ...
    %        'Position',[0.0303    0.76    0.0983    0.0416], ...
    %        'Tag', 'triang', ...
    %        'CallBack',strcat(thisfile,'(''triangular'')'), ...
    %        'Value',1,...
    %        'String','Triangular cell');

    %    %radiobutton polytopal cell
    %    uicontrol( ...
    %        'Style','radiobutton', ...
    %        'Units','normalized', ...
    %        'BackgroundColor',[0.80 0.80 0.80], ...
    %        'ListboxTop',0, ...
    %        'Position',[0.16    0.76    0.0983    0.0416], ...
    %        'Tag', 'poly', ...
    %        'CallBack',strcat(thisfile,'(''polytopal'')'), ...
    %        'Value',0,...
    %        'String','Polytopal cell');

    %    %radiobutton rectangular cell
    %    uicontrol( ...
    %        'Style','radiobutton', ...
    %        'Units','normalized', ...
    %        'BackgroundColor',[0.80 0.80 0.80], ...
    %        'ListboxTop',0, ...
    %        'Position',[0.0303    0.73    0.1161    0.0416], ...
    %        'Tag', 'rect', ...
    %        'CallBack',strcat(thisfile,'(''rectangle'')'), ...
    %        'Value',0,...
    %        'String','Rectangular cell');

        
    %    %radiobutton trapezoidal cell
    %    uicontrol( ...
    %        'Style','radiobutton', ...
    %        'Units','normalized', ...
    %        'BackgroundColor',[0.80 0.80 0.80], ...
    %        'ListboxTop',0, ...
    %        'Position',[0.16    0.73    0.1140    0.0416], ...
    %        'Tag', 'trapez', ...
    %        'CallBack',strcat(thisfile,'(''trapez'')'), ...
    %        'Value',0,...
    %        'String','Trapezoidal cell');

        %Max linear velocity
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.665    0.15    0.0235], ...%[0.0303    0.53    0.1349    0.0235], ...
            'String','Max linear velocity (m/s)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.635    0.15    0.0334],...%[0.0303    0.4866    0.1349    0.0434], ...
            'CallBack',strcat(thisfile,'(''max_lin_vel_changed'')'), ...
            'Tag', 'linvel', ...
            'String','1');

        %Max angular velocity
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.605    0.15    0.0235], ...
            'String','Max angular velocity (rad/s)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.575    0.15    0.0334], ...
            'CallBack',strcat(thisfile,'(''max_ang_vel_changed'')'), ...
            'Tag', 'angvel', ...
            'String','1');
        
        %Max steering angle
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.545    0.15    0.0235], ...
            'String','Max steering angle (deg)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.515    0.15    0.0334], ...
            'Tag', 'steering', ...
            'CallBack',strcat(thisfile,'(''max_steering_changed'')'), ...
            'String','30');

        %Wheel radius
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.485    0.15    0.0235], ...
            'String','Wheel radius (m)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.455    0.15    0.0334], ...
            'Tag', 'wheel', ...
            'CallBack',strcat(thisfile,'(''wheel_radius_changed'')'), ...
            'String','0.05');

        %Wheelbase
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.425    0.15    0.0235], ...
            'String','Wheel Base (m)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.395    0.15    0.0334], ...
            'Tag', 'wheelbase', ...
            'CallBack',strcat(thisfile,'(''wheel_base_changed'')'), ...
            'String','0.25');

        
        %Sampling period
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.365    0.15    0.0235], ...
            'String','Sampling period (seconds)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.335    0.15    0.0334], ...
            'Tag', 'sampling', ...
            'CallBack',strcat(thisfile,'(''sampling_changed'')'), ...
            'String','0.1');

        %lookahead distance
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.305    0.15    0.0235], ...
            'String','Lookahead Distance (int value)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.015    0.275    0.15    0.0334], ...
            'Tag', 'lookahead', ...
            'CallBack',strcat(thisfile,'(''lookahead_changed'')'), ...
            'String','10');

        %motion controller
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.015    0.245    0.15    0.0235], ...
            'String','Motion Controller:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.015    0.215    0.15    0.0334], ...
            'Tag', 'controller', ...
            'String','Pure-Pursuit|PI'); % |variable step saving data');


        
        %radiobutton car-like
      %  uicontrol( ...
      %      'Style','radiobutton', ...
      %      'Units','normalized', ...
      %      'BackgroundColor',[0.80 0.80 0.80], ...
      %      'ListboxTop',0, ...
      %      'Position',[0.0513    0.2033    0.0983    0.0416], ...
      %      'Tag', 'robotCar', ...
      %      'CallBack',strcat(thisfile,'(''robot_car'')'), ...
      %      'Value',1,...
      %      'String','Car-like');

        %radiobutton Differential-drive
      %  uicontrol( ...
      %      'Style','radiobutton', ...
      %      'Units','normalized', ...
      %      'BackgroundColor',[0.80 0.80 0.80], ...
      %      'ListboxTop',1, ...
      %      'Position',[0.18    0.2033    0.1203    0.0416], ...
      %      'Tag', 'robotDifferential', ...
      %      'CallBack',strcat(thisfile,'(''robot_differential'')'), ...
      %      'String','Differential-drive');

        %text: Trajectory-Workspace
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.6    0.955    0.1349    0.0235], ...
            'String','Trajectory / Workspace');
        %text: Orientation
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.42    0.31   0.1349    0.0235], ...
            'String','Orientation [deg]');
        %text: Orientation - time
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.43    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        %text: Angular/linear velocity
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.61    0.31   0.1349    0.0235], ...
            'String','Linear velocity [m/s]');
        %text: Angular/linear velocity - time
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.65    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        %text: Steering Angle
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.84    0.31   0.1349    0.0235], ...
            'String','Steering angle [deg]');
        %text: Steering Angle - time
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.87    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        
        set(figpri, 'Visible','on');
        set(gcf,'UserData',data);
        
        
    %case 'robot_car'
    %    set(findobj(gcf,'Tag','robotCar'),'Value',1);
    %    set(findobj(gcf,'Tag','robotDifferential'),'Value',0);
    %case 'robot_differential'
    %    set(findobj(gcf,'Tag','robotCar'),'Value',0);
    %    set(findobj(gcf,'Tag','robotDifferential'),'Value',1);
    
    %case 'triangular'
    %    set(findobj(gcf,'Tag','triang'),'Value',1);
    %    set(findobj(gcf,'Tag','poly'),'Value',0);
    %    set(findobj(gcf,'Tag','rect'),'Value',0);
    %    set(findobj(gcf,'Tag','trapez'),'Value',0);
    %case 'polytopal'
    %    set(findobj(gcf,'Tag','triang'),'Value',0);
    %    set(findobj(gcf,'Tag','poly'),'Value',1);
    %    set(findobj(gcf,'Tag','rect'),'Value',0);
    %    set(findobj(gcf,'Tag','trapez'),'Value',0);
    %case 'rectangle'
    %    set(findobj(gcf,'Tag','triang'),'Value',0);
    %    set(findobj(gcf,'Tag','poly'),'Value',0);
    %    set(findobj(gcf,'Tag','rect'),'Value',1);
    %    set(findobj(gcf,'Tag','trapez'),'Value',0);
    %case 'trapez'
    %    set(findobj(gcf,'Tag','triang'),'Value',0);
    %    set(findobj(gcf,'Tag','poly'),'Value',0);
    %    set(findobj(gcf,'Tag','rect'),'Value',0);
    %    set(findobj(gcf,'Tag','trapez'),'Value',1);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %               RUN ENVIRONMENT
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    case 'run_environment'
        planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
        data = get(gcf,'UserData');
        
        switch planning_approach
            case 1 %cell decomposition
                %obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                prompt = {'Number of obstacles:','Obstacle size:'};
                dlg_title = 'Obstacles';
                num_lines = 1;
                defaultans = {'5','2'};
                input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
                obs_no = char(input_user(1));
                obs_size = char(input_user(2));
                if isempty(obs_no)
                    return;
                end
                try 
                    obs_no = str2double(obs_no);
                    obs_size = str2double(obs_size);
                catch
                    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
                    rmt('run_environment');
                    return;
                end
                %if ((obs_no <= 0) || (obs_no >= 6) || (obs_no ~= round(obs_no)))
                %    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
                %    rmt('run_environment');
                %    return;
                %end                
                data = get(gcf,'UserData');
                limits = data.frame_limits;
                %we clean the workspace figure
                cla(data.handle_env);
                set(data.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');

                %we clean the orientation figure
                cla(data.handle_ori);
                set(data.handle_ori,'XGrid','on','YGrid','on','Visible','off');
                %we clean the velocities figure
                cla(data.handle_vel);
                set(data.handle_vel,'XGrid','on','YGrid','on','Visible','off');
                %we clean the steering angle figure
                cla(data.handle_ang);
                set(data.handle_ang,'XGrid','on','YGrid','on','Visible','off');

                
                disp('Cell decomposition...');
                [objects,initial_point,final_point] = rmt_define_regions(data.handle_env,obs_no,limits(1),limits(2),limits(3),limits(4));
                set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');

                data.Nobstacles = obs_no;
                data.obstacles = objects;
                data.initial=initial_point;
                data.final=final_point;
                data.orientation=data.orientation;
                set(gcf,'UserData',data);
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
             
            case 2 %visibility graph
                disp('Visibility graph...');
                obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                if isempty(obs_no)
                    return;
                end
                try 
                    obs_no = str2double(obs_no);
                catch
                    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
                    rmt('run_environment');
                    return;
                end
                %if ((obs_no <= 0) || (obs_no >= 5) || (obs_no ~= round(obs_no)))
                %    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 4!'),'Robot Motion Toolbox','modal'));
                %    rmt('run_environment');
                %    return;
                %end
                
                
                data = get(gcf,'UserData');
                limits = data.frame_limits;
                %we clean the workspace figure
                cla(data.handle_env);
                set(data.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');

                %we clean the orientation figure
                cla(data.handle_ori);
                set(data.handle_ori,'XGrid','on','YGrid','on','Visible','off');
                %we clean the velocities figure
                cla(data.handle_vel);
                set(data.handle_vel,'XGrid','on','YGrid','on','Visible','off');
                %we clean the steering angle figure
                cla(data.handle_ang);
                set(data.handle_ang,'XGrid','on','YGrid','on','Visible','off');
               
                [map, obstacles,initial_point,final_point] = rmt_build_grid_map2(data.handle_env,limits,obs_no);                
                set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
                
                %data.map = map;
                data.Nobstacles = obs_no;
                data.obstacles = obstacles;
                data.initial=initial_point;
                data.final=final_point;
                data.orientation=data.orientation;
                set(gcf,'UserData',data);
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            case 3 %Voronoi
                disp('Voronoi diagram...');
                obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                if isempty(obs_no)
                    return;
                end
                try 
                    obs_no = str2double(obs_no);
                catch
                    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
                    rmt('run_environment');
                    return;
                end
                %if ((obs_no <= 0) || (obs_no >= 5) || (obs_no ~= round(obs_no)))
                %    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 4!'),'Robot Motion Toolbox','modal'));
                %    rmt('run_environment');
                %    return;
                %end                               
                data = get(gcf,'UserData');
                limits = data.frame_limits;
                %we clean the workspace figure
                cla(data.handle_env);
                set(data.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');

                %we clean the orientation figure
                cla(data.handle_ori);
                set(data.handle_ori,'XGrid','on','YGrid','on','Visible','off');
                %we clean the velocities figure
                cla(data.handle_vel);
                set(data.handle_vel,'XGrid','on','YGrid','on','Visible','off');
                %we clean the steering angle figure
                cla(data.handle_ang);
                set(data.handle_ang,'XGrid','on','YGrid','on','Visible','off');
                wheelbase_var = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                %[map, obstacles,initial_point,final_point] = rmt_build_grid_map2(data.handle_env,limits,obs_no);                
                [initial_point, final_point, X_Total_points,Y_Total_points, ...
                    All_cells_Number, Cell_start, X1] = rmt_obstacle_draw(data.handle_env,obs_no,limits,wheelbase_var*0.5,data.epsilonvoronoi);
                set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
                %saving data
                data.Nobstacles = obs_no;
                data.initial=initial_point;
                data.final=final_point;
                data.X_Total_points = X_Total_points;                
                data.Y_Total_points = Y_Total_points;
                data.All_cells_Number = All_cells_Number;
                data.Cell_start = Cell_start;
                data.X1 = X1;
                data.orientation=data.orientation;
                danger_no = char(inputdlg('Number of dangers:','Robot Motion Toolbox',1,{'0'}));
                try 
                    danger_no = str2num(danger_no);
                catch
                    uiwait(errordlg(sprintf('\nNumber of dangers should be a natural number between 0 and 5!'),'Robot Motion Toolbox','modal'));
                    rmt('run_environment');
                    return;
                end
                danger_position = zeros(danger_no, 2);
                if danger_no > 0     
                    danger_halfsize = 0.15;
                    uiwait(msgbox(sprintf('\nFor defining a danger:\n\t - left-click = pick a danger position\n'),'Robot Motion Toolbox','modal'));
                    number = 0;
                    while number < danger_no
                        [x,y,but]=ginput(1);
                        if but==1
                            number = number + 1;
                            danger_position(number, 1) = x;
                            danger_position(number, 2) = y;
                            plot(danger_position(number, 1),danger_position(number, 2),'xw','Markersize',13, 'LineWidth', 5, 'Color', 'r');
                            dx(1, number) = x;
                            dy(1, number) = y;
                        end;
                    end;
                    out_poly = 1;
                    for a=2:length(X1)
                        ox = (X1{a}(:,1))';
                        oy = (X1{a}(:,2))';
                        inpoly(a-1,:) = inpolygon(dx, dy, ox, oy);
                    end;
                    for a=1:size(inpoly,2)
                        lgth = length(data.X1);
                        if isempty(find(inpoly(:,a), 1))
                            point = [danger_position(a,1) danger_position(a,2)];
                            near_obstacle = 0;
                            for b=2:length(X1)
                                ox = (X1{b}(:,1));
                                oy = (X1{b}(:,2));
                                for c=1:size(ox,1)
                                    if c~=size(ox,1)
                                        edge = [ox(c) oy(c) ox(c+1) oy(c+1)];
                                    else
                                        edge = [ox(c) oy(c) ox(1) oy(1)];
                                    end
                                    dist = distancePointEdge(point, edge);
                                    if dist < danger_halfsize*4;
                                        near_obstacle = 1;
                                        break;
                                    end;
                                end;
                                if near_obstacle==1
                                    break;
                                end;
                            end
                            if near_obstacle==0
                                X1_new(1,1) = point(1,1)-danger_halfsize;
                                X1_new(1,2) = point(1,2)-danger_halfsize;
                                X1_new(2,1) = point(1,1)+danger_halfsize;
                                X1_new(2,2) = point(1,2)-danger_halfsize;
                                X1_new(3,1) = point(1,1)+danger_halfsize;
                                X1_new(3,2) = point(1,2)+danger_halfsize;
                                X1_new(4,1) = point(1,1)-danger_halfsize;
                                X1_new(4,2) = point(1,2)+danger_halfsize;
                                data.X1{lgth+1} = X1_new;
                                data.Nobstacles = data.Nobstacles + 1;
                            end;
                        end;
                    end;
                end;
                for a=2:length(data.X1)
                    poly_obstacles(a-1) = {data.X1{a}};
                end
                data.obstacles = poly_obstacles;
                data.danger_positions = danger_position;
                set(gcf,'UserData',data);
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            case 4 %manual points
                cla(data.handle_env);
                axes(data.handle_env);
                cla(data.handle_ori);
                axes(data.handle_ori);
                cla(data.handle_vel);
                axes(data.handle_vel);
                cla(data.handle_ang);
                axes(data.handle_ang);                
                traj_ini = [0 0];
                data = get(gcf,'UserData');
                sampling_period = eval(get(findobj(gcf,'Tag','sampling'),'String'));                 
                %axes(data.handle_env);                
                traj = rmt_get_waypoints(data.handle_env,data.frame_limits,sampling_period,traj_ini);
                %cla(data.handle_env);
                data.initial = [traj(1,1) traj(2,1)];                
                data.final = [traj(1,end) traj(2,end)];
                data.trajectory = traj;
                plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
                plot(data.trajectory(1,:),data.trajectory(2,:),'r','LineWidth',3);
                plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
                grid on;
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','off');
        end%switch  
        set(gcf,'UserData',data);%to save data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %               RUN PATH PLANNING
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    case 'run_path_planning'
        planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
        data = get(gcf,'UserData');
        
        cla(data.handle_ori);
        cla(data.handle_vel);
        cla(data.handle_ang);
        if ~isfield(data,'Nobstacles')
            return;
        end
        for i=1:data.Nobstacles
            aux = data.obstacles{i};            
            [as bs] = size(data.obstacles{1});
            if(as > bs)
                aux = aux';
            end
            obstaclesCD(i) = {aux};
        end
        
        switch planning_approach
            case 1%cell decomposition
            %    if (get(findobj(gcf,'Tag','triang'),'Value') == 1)
            %        [C,adj,mid_X,mid_Y,com_F]=rmt_triangular_decomposition(obstaclesCD,data.frame_limits);   %triangular decomposition
            %    elseif (get(findobj(gcf,'Tag','rect'),'Value') == 1)
            %        [C,adj,mid_X,mid_Y,com_F]=rmt_rectangular_decomposition(obstaclesCD,data.frame_limits);   %rectangular decomposition
            %    elseif (get(findobj(gcf,'Tag','poly'),'Value') == 1)
            %        [C,adj,mid_X,mid_Y,com_F]=rmt_polytopal_decomposition(obstaclesCD,data.frame_limits);   %polytopal decomposition
            %    elseif (get(findobj(gcf,'Tag','trapez'),'Value') == 1)
            %        [C,adj,mid_X,mid_Y,com_F]=rmt_trapezoidal_decomposition(obstaclesCD,data.frame_limits);   %trapezoidal decomposition
            %    end
                cell_type = get(findobj(gcf,'Tag','cells'),'Value');
                switch cell_type
                    case 1%triang cell
                        [C,adj,mid_X,mid_Y,com_F]=rmt_triangular_decomposition(obstaclesCD,data.frame_limits);   %triangular decomposition
                    case 2%rect cell
                        [C,adj,mid_X,mid_Y,com_F]=rmt_rectangular_decomposition(obstaclesCD,data.frame_limits);   %rectangular decomposition
                    case 3%poly cell 
                        [C,adj,mid_X,mid_Y,com_F]=rmt_polytopal_decomposition(obstaclesCD,data.frame_limits);   %polytopal decomposition
                    case 4%trapez cell
                        [C,adj,mid_X,mid_Y,com_F]=rmt_trapezoidal_decomposition(obstaclesCD,data.frame_limits);   %trapezoidal decomposition
                end
                cla(data.handle_env);
                axes(data.handle_env);
                rmt_plot_environment(obstaclesCD,data.frame_limits,C);
                plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
                plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
                set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                grid on;
                
                if (get(findobj(gcf,'Tag','waypoints'),'Value') == 1) %middle points
                    [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                        data.initial(1),data.initial(2),data.final(1),data.final(2),obstaclesCD);
                    plot(traj(1,:),traj(2,:),'r','LineWidth',2);
                    data.trajectory = traj;
                    fprintf('\nTravelled distance via middle points: %g.\n',travel_dist);
                elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 2)  %norm 1
                    %function 1-norm
                    safe_dist=0.1;  %should be smaller than half of shortest traversed segment
                    [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                        data.initial(1),data.initial(2),data.final(1),data.final(2),obstaclesCD);                    
                    start_p = [data.initial(1) data.initial(2)];
                    goal_p = [data.final(1) data.final(2)];
                    [traj_norm_one, dist_norm_one] = rmt_optimize_traj_norm_one(com_F,...
                        path_cells,start_p,goal_p,safe_dist);  %via LP
                    plot(traj_norm_one(1,:),traj_norm_one(2,:),'r','LineWidth',2);
                    data.trajectory = traj_norm_one;
                    fprintf('\nTravelled distance via Norm-1: %g.\n',dist_norm_one);
                elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 3)  %norm 2
                    safe_dist=0.1;  %should be smaller than half of shortest traversed segment
                    [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                        data.initial(1),data.initial(2),data.final(1),data.final(2),obstaclesCD);                    
                    start_p = [data.initial(1) data.initial(2)];
                    goal_p = [data.final(1) data.final(2)];
                    [traj_norm_2_sq, dist_norm_2_sq] = rmt_optimize_traj_norm_two_sq(com_F,path_cells,start_p,goal_p,safe_dist);  %via QP
                    data.trajectory = traj_norm_2_sq;                    
                    plot(traj_norm_2_sq(1,:),traj_norm_2_sq(2,:),'r','LineWidth',2);                    
                    fprintf('\nTravelled distance via Norm-2: %g.\n',dist_norm_2_sq);
                 elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 4)  %norm inf.
                    safe_dist=0.1;  %should be smaller than half of shortest traversed segment
                    [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                        data.initial(1),data.initial(2),data.final(1),data.final(2),obstaclesCD);                    
                    start_p = [data.initial(1) data.initial(2)];
                    goal_p = [data.final(1) data.final(2)];
                    [traj_norm_inf, dist_norm_inf] = rmt_optimize_traj_norm_inf(com_F,path_cells,start_p,goal_p,safe_dist);  %via LP
                    data.trajectory = traj_norm_inf;                    
                    plot(traj_norm_inf(1,:),traj_norm_inf(2,:),'r','LineWidth',2);                       
                    fprintf('\nTravelled distance via Norm-Inf.: %g.\n',dist_norm_inf);
                end
                set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
            case 2%visibility graph
                input_variables = zeros(8,1);
                input_variables(1) = data.frame_limits(1);
                input_variables(2) = data.frame_limits(2);
                input_variables(3) = data.frame_limits(3);
                input_variables(4) = data.frame_limits(4);                
                input_variables(5) = length(data.obstacles);
                input_variables(6) = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                input_variables(7) = data.initial(1);
                input_variables(8) = data.initial(2);
                input_variables(9) = data.final(1);
                input_variables(10) = data.final(2);
                cla(data.handle_env);%new
                axes(data.handle_env);
                rmt_plot_environment(data.obstacles,data.frame_limits);                
                plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
                plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
                grid on;                                               
                %traj = rmt_visibility_graph_new2(data.handle_env,input_variables,data.map,data.obstacles);
                traj = rmt_vgraph2(data.handle_env,input_variables,data.obstacles);
                %plot(traj(1,:),traj(2,:),'--r','LineWidth',2);%new
                %set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                %[trajectory] = rmt_vgraph2(handle,input_variables,seq_obstacles)
                data.trajectory = traj';  
                set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
            case 3%voronoi    
                cla(data.handle_env);%new
                axes(data.handle_env);
                rmt_plot_environment(data.obstacles,data.frame_limits);                
                plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
                plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
                for i=1:size(data.danger_positions,1)
                    plot(data.danger_positions(i, 1),data.danger_positions(i, 2),'xw','Markersize',13, 'LineWidth', 5, 'Color', 'r');
                end;
                grid on; 
                limits = data.frame_limits;
                whe = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                [X_Total_points,Y_Total_points, ...
                    All_cells_Number, Cell_start, X1] = rmt_voronoi_epsi(data.handle_env,data.Nobstacles,limits,whe*0.5,...
                    data.epsilonvoronoi,data.obstacles);                                
                data.X_Total_points = X_Total_points;                
                data.Y_Total_points = Y_Total_points;
                data.All_cells_Number = All_cells_Number;
                data.Cell_start = Cell_start;
                data.X1 = X1;
                [trajDV, Vertex_Cord_DV, PathWithoutCurve, CostWithoutCurve, ...
                    VertWithoutCurve, Edges, Verts] = rmt_get_voronoi(data.handle_env, ...
                    data.frame_limits, (data.Nobstacles+1), data.initial, ...
                    data.final,data.X_Total_points, data.Y_Total_points, ...
                    data.All_cells_Number, data.Cell_start, data.X1, 1);
                %traj = rmt_visibility_graph(data.handle_env,input_variables,data.map,data.obstacles);                                
                data.trajectory = trajDV';
                
                data.Vertex_Cord_DV = Vertex_Cord_DV;
                data.DVPaths = PathWithoutCurve;
                data.DVCosts = CostWithoutCurve;
                data.DVVerts = VertWithoutCurve;
                data.Edges = Edges;
                data.Verts = Verts;
                
                data.Homotopies = rmt_create_dv_homotopies(data.Vertex_Cord_DV, data.DVPaths, ...
                    data.DVCosts, data.DVVerts, (data.Nobstacles+1), data.X1);
                                
                % 1st - criterii_length;
                % 2nd - criterii_curve;
                criterias = [0.5, 0.5];
                
                rmt_iterations_with_creterias(data, (data.Nobstacles+1), data.X1, ...
                    criterias);
                
                %shortest_path
                input_variables = zeros(8,1);
                input_variables(1) = data.frame_limits(1);
                input_variables(2) = data.frame_limits(2);
                input_variables(3) = data.frame_limits(3);
                input_variables(4) = data.frame_limits(4);                
                input_variables(5) = length(data.obstacles);
                input_variables(6) = data.initial(1);
                input_variables(7) = data.initial(2);
                input_variables(8) = data.final(1);
                input_variables(9) = data.final(2);
                [trajVG, Vertex_Cord_VG, PathWithoutPoint, CostWithoutPoint] = rmt_shortest_path(input_variables,data.obstacles, 0);
                
                data.Vertex_Cord_VG = Vertex_Cord_VG;
                data.VGPaths = PathWithoutPoint;
                data.VGCosts = CostWithoutPoint;
                
                data.homotopies = rmt_create_homotopies(data.Vertex_Cord_DV, data.DVPaths, data.Vertex_Cord_VG, data.VGPaths, (data.Nobstacles+1), data.X1);
                
                
                if size(data.danger_positions,1) > 0
                    minDistFromDangers = zeros(size(data.DVPaths,1),1);
                    maxDistFromDangers = zeros(size(data.DVPaths,1),1);
                    procDontViewDangers = zeros(size(data.DVPaths,1),1);
                    sumDistFromDangers = zeros(size(data.DVPaths,1),1);
                    for i=1:size(data.DVPaths,1)
                        path_i = rmt_get_voronoi_path(Vertex_Cord_DV, data.DVCosts(i), data.DVPaths{i,1}, data.initial, data.final, 0.25);
                        minDistFromDangers(i,1) = 10000000;
                        pointsViewDanger = zeros(size(path_i,1),1);
                        for j=1:size(path_i,1);
                            minDist = 10000000;
                            for k=1:size(data.danger_positions,1)
                                dist = sqrt((data.danger_positions(k,1)-path_i(j,1))^2 + (data.danger_positions(k,2)-path_i(j,2))^2);
                                
                                %if pointsViewDanger(j,1)==0
                                findIntersect = 0;
                                line = [path_i(j,1) path_i(j,2) data.danger_positions(k,1) data.danger_positions(k,2)];
                                for o=2:(data.Nobstacles+1)
                                    for r=1:length(data.X1{o})
                                       a=r;
                                       if(r==length(data.X1{o}))
                                           b=1;
                                       else
                                           b=r+1;
                                       end
                                       line_edge = [data.X1{o}(a,1) data.X1{o}(a,2) data.X1{o}(b,1) data.X1{o}(b,2)];
                                       intersection_point = intersectEdges(line, line_edge);
                                       if ~isnan(intersection_point(1,1)) || ~isnan(intersection_point(1,2))
                                           findIntersect = 1;
                                           break;
                                       end
                                    end;
                                    if findIntersect==1
                                        break;
                                    end;
                                end;
                                if findIntersect==0
                                    pointsViewDanger(j,1) = pointsViewDanger(j,1) + 1;
                                    sumDistFromDangers(i,1) = sumDistFromDangers(i,1) + dist;
                                end;
                                %end;
                                
                                if (dist<minDist)
                                    minDist = dist;
                                end;
                            end;
                            if (minDist<minDistFromDangers(i,1))
                                minDistFromDangers(i,1) = minDist;
                            end;
                            if (minDist>maxDistFromDangers(i,1))
                                maxDistFromDangers(i,1) = minDist;
                            end;
                        end;
                        procDontViewDangers(i,1) = sum(sum(pointsViewDanger==0))/size(path_i,1);
                    end;
                    %[procDV findDV]=max(procDontViewDangers);
                    [procDV findDV]=max(sumDistFromDangers);
                    pathDV = data.DVPaths{findDV,1}; 
                    findVG = find(data.homotopies(findDV, :) == 1, 1);
                    pathVG = data.VGPaths{findVG,1};
                else
                    [costVG findVG]=min(data.VGCosts);
                    pathVG = data.VGPaths{findVG,1};
                    findDV = find(data.homotopies(:, findVG) == 1, 1);
                    if ~isempty(findDV)
                        pathDV = data.DVPaths{findDV,1};
                    else
                        [costDV MinCost]=min(data.DVCosts);
                        pathDV = data.DVPaths{MinCost,1};
                        findVG = find(data.homotopies(MinCost, :) == 1, 1);
                        if ~isempty(findVG)
                            pathVG = data.VGPaths{findVG,1};
                        end
                    end
                end;

                data.coord_pathDV = rmt_get_voronoi_path(data.Vertex_Cord_DV, data.DVCosts(findDV), pathDV, data.initial, data.final, 0.1);
                                
                %{            
                path_size = 2*data.DVCosts(findDV)/eps;

                coord_pathDV = zeros(ceil(path_size), 2);
                coord_pathDV(1,:) = [data.initial(1) data.initial(2)];
                number = 1;
                for i=1:length(pathDV)
                    dist = norm([data.Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1) data.Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2)]);
                    dict(1,:) = [(data.Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1))/dist (data.Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2))/dist];
                    while (dist>eps)
                        number = number + 1;
                        coord_pathDV(number, :) = [eps*dict(1,1)+coord_pathDV(number-1,1) eps*dict(1,2)+coord_pathDV(number-1,2)];
                        dist = norm([data.Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1) data.Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2)]);
                    end
                    number = number + 1;
                    coord_pathDV(number, :) = [data.Vertex_Cord_DV(pathDV(i),1) data.Vertex_Cord_DV(pathDV(i),2)];
                end
                dist = norm([data.final(1)-coord_pathDV(number,1) data.final(2)-coord_pathDV(number,2)]);
                dict(1,:) = [(data.final(1)-coord_pathDV(number,1))/dist (data.final(2)-coord_pathDV(number,2))/dist];
                while (dist>eps)
                    number = number + 1;
                    coord_pathDV(number, :) = [eps*dict(1,1)+coord_pathDV(number-1,1) eps*dict(1,2)+coord_pathDV(number-1,2)];
                    dist = norm([data.final(1)-coord_pathDV(number,1) data.final(2)-coord_pathDV(number,2)]);
                end
                number = number + 1;
                coord_pathDV(number,:) = [data.final(1) data.final(2)];
                number = number + 1;
                for i=number:size(coord_pathDV,1)
                    coord_pathDV(number,:) = [];
                end;
                %}

                data.coord_pathVG = rmt_get_visibility_shortest_path(data.Vertex_Cord_VG, pathVG, data.VGCosts(findVG), size(data.coord_pathDV,1));

                %{
                step_size_VG = costVG/(size(coord_pathDV,1)-1);
                coord_pathVG = zeros(ceil(path_size), 2);
                coord_pathVG(1,:) = [data.initial(1) data.initial(2)];
                number = 1;
                for i=2:length(pathVG)
                    dist = norm([data.Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1) data.Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2)]);
                    dict(1,:) = [(data.Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1))/dist (data.Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2))/dist];
                    while (dist>2*step_size_VG)
                        number = number + 1;
                        coord_pathVG(number, :) = [step_size_VG*dict(1,1)+coord_pathVG(number-1,1) step_size_VG*dict(1,2)+coord_pathVG(number-1,2)];
                        dist = norm([data.Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1) data.Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2)]);
                    end
                    number = number + 1;
                    coord_pathVG(number, :) = [data.Vertex_Cord_VG{1,pathVG(i)}(1,1) data.Vertex_Cord_VG{1,pathVG(i)}(2,1)];
                    if (i~=length(pathVG))
                        number = number + 1;
                        dist = 2*step_size_VG - dist;
                        distNext = norm([data.Vertex_Cord_VG{1,pathVG(i+1)}(1,1)-coord_pathVG(number,1) data.Vertex_Cord_VG{1,pathVG(i+1)}(2,1)-coord_pathVG(number,2)]);
                        dict(1,:) = [(data.Vertex_Cord_VG{1,pathVG(i+1)}(1,1)-coord_pathVG(number,1))/distNext (data.Vertex_Cord_VG{1,pathVG(i+1)}(2,1)-coord_pathVG(number,2))/distNext];
                        coord_pathVG(number, :) = [dist*dict(1,1)+coord_pathVG(number-1,1) dist*dict(1,2)+coord_pathVG(number-1,2)];
                    end;
                end;
                number = number + 1;
                for i=number:size(coord_pathVG,1)
                    coord_pathVG(number,:) = [];
                end;
                %}
                
        end
        set(gcf,'UserData',data);
        rmt('crit_length');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %               RUN CONTROL
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'run_control'
        data = get(gcf,'UserData');
        sampling_period = eval(get(findobj(gcf,'Tag','sampling'),'String'));
        max_ang_vel = eval(get(findobj(gcf,'Tag','angvel'),'String'));
        max_linear_vel = eval(get(findobj(gcf,'Tag','linvel'),'String'));
        max_steering = eval(get(findobj(gcf,'Tag','steering'),'String'));
        wheel_radius = eval(get(findobj(gcf,'Tag','wheel'),'String'));
        wheel_base = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
        lookahead_distance = eval(get(findobj(gcf,'Tag','lookahead'),'String'));

      %  if (get(findobj(gcf,'Tag','robotCar'),'Value') == 1)
      %      robotType = 1; %car-like robot
      %  elseif (get(findobj(gcf,'Tag','robotDifferential'),'Value') == 1)
      %      robotType = 2; %differential robot
      %  end
        robotType = get(findobj(gcf,'Tag','car_type'),'Value'); % 1=car-like 2=differential
        
        data = get(gcf,'UserData');
        xini = data.initial(1);
        yini = data.initial(2);        
        if ~isfield(data,'trajectory')
            return;
        end
        ref_trajectory = data.trajectory;
        threshold_goal = 0.15;%distance to consider the goal has been reached.
        input_variables = [sampling_period, xini, yini, wheel_radius,...
            wheel_base,max_linear_vel,...
            max_steering, lookahead_distance,robotType,threshold_goal,...
            max_ang_vel,data.frame_limits(1),data.frame_limits(2),...
            data.frame_limits(3) data.frame_limits(4)];                             
        if(get(findobj(gcf,'Tag','controller'),'Value') == 1)
            %function pure-pursuit
            fprintf('\nPure pursuit controller is running...');
            [array_time, array_alpha, array_v, array_pos] = rmt_pure_pursuit(input_variables,ref_trajectory,data.orientation,data.obstacles,data.Nobstacles);           
        else
            %PI control
            fprintf('\nPI controller is running...');
            theta_init = data.orientation;%improvement permit to the user to select the initial orientation of the robot
            dstar = 0.01;
            threshold_goal = 0.2;%distance to consider the goal has been reached.
            input_variables = [sampling_period, xini, yini, theta_init,wheel_radius,...
            wheel_base,max_linear_vel,max_steering, dstar,robotType,threshold_goal,...
            max_ang_vel,lookahead_distance,data.frame_limits(1),data.frame_limits(2),...
            data.frame_limits(3) data.frame_limits(4)];   
            pi_tuning = data.pi_tuning;
            [array_time, array_alpha, array_v, array_pos] = rmt_pi_controller(input_variables,ref_trajectory,pi_tuning,data.obstacles,data.Nobstacles);
            %array_time = var_output(:,1);
            %array_alpha = var_output(:,2)';
            %array_v = var_output(:,3);
            %array_pos = [var_output(:,1), var_output(:,2), var_output(:,3)];
            array_pos = array_pos;
        end
        
        %drawing the result
        color = hsv(5);
        color = color(randperm(5),:); 
        cha = floor(rand(1)*5+1);
        %trajectory
        plot(data.handle_env,array_pos(1,:),array_pos(2,:),'Color','k','LineWidth',2);        
        set(data.handle_env,'XGrid','on','YGrid','on');

        %orientation
        plot(data.handle_ori,array_time,rad2deg(array_pos(3,:)),'Color','k','LineWidth',2);
        hold on;
        xmax = max(array_time);
        ymax = max(rad2deg(array_pos(3,:)));
        ymin = min(rad2deg(array_pos(3,:)));
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_ori,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');                        
        %set(data.handle_ori,'XGrid','on','YGrid','on');
        
        %velocities
        plot(data.handle_vel,array_time,array_v,'Color','k','LineWidth',2);        
        hold on;
        xmax = max(array_time);
        ymax = max(array_v)+1;
        ymin = min(array_v)-1;
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_vel,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');                                
        %set(data.handle_vel,'XGrid','on','YGrid','on');

        %steering angle
        plot(data.handle_ang,array_time,rad2deg(array_alpha),'Color','k','LineWidth',2);                
        hold on;
        xmax = max(array_time);
        ymax = max(rad2deg(array_alpha));
        ymin = min(rad2deg(array_alpha));
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_ang,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');                                        
        %set(data.handle_ang,'XGrid','on','YGrid','on');
        
    case 'max_lin_vel_changed'
        input_val = char(get(findobj(gcf,'Tag','linvel'),'String'));
        todoOK = rmt_detect_error(input_val,0.1,10);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 10!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','linvel'),'String','1');
        end
    case 'max_ang_vel_changed'
        input_val = char(get(findobj(gcf,'Tag','angvel'),'String'));
        todoOK = rmt_detect_error(input_val,0.1,5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 5!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','angvel'),'String','1');
        end
    case 'max_steering_changed'
        input_val = char(get(findobj(gcf,'Tag','steering'),'String'));
        todoOK = rmt_detect_error(input_val,10,60);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 10 and 60!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','steering'),'String','30');
        end
    case 'wheel_radius_changed'
        input_val = char(get(findobj(gcf,'Tag','wheel'),'String'));
        todoOK = rmt_detect_error(input_val,0.01,1.5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.01 and 1.5!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','wheel'),'String','0.05');
        end
    case 'wheel_base_changed'
        input_val = char(get(findobj(gcf,'Tag','wheelbase'),'String'));
        todoOK = rmt_detect_error(input_val,0.0,2.5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 2.5!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','wheelbase'),'String','0.25');
        end
    case 'sampling_changed'
        input_val = char(get(findobj(gcf,'Tag','sampling'),'String'));
        todoOK = rmt_detect_error(input_val,0.01,2);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.01 and 2!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','sampling'),'String','0.1');
        end
    case 'lookahead_changed'
        input_val = char(get(findobj(gcf,'Tag','lookahead'),'String'));
        todoOK = rmt_detect_error(input_val,1,20);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 1 and 20!'),'Robot Motion Toolbox','modal')); 
            set(findobj(gcf,'Tag','lookahead'),'String','10');
        end
    case 'change_planning_approach'
        planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
        if (planning_approach == 1)
        %    set(findobj(gcf,'Tag','triang'),'Enable','on');
        %    set(findobj(gcf,'Tag','rect'),'Enable','on');
        %    set(findobj(gcf,'Tag','poly'),'Enable','on');
        %    set(findobj(gcf,'Tag','trapez'),'Enable','on');
            set(findobj(gcf,'Tag','waypoints'),'Enable','on');
            set(findobj(gcf,'Tag','cells'),'Enable','on');
        else
        %    set(findobj(gcf,'Tag','triang'),'Enable','off');
        %    set(findobj(gcf,'Tag','rect'),'Enable','off');
        %    set(findobj(gcf,'Tag','poly'),'Enable','off');
        %    set(findobj(gcf,'Tag','trapez'),'Enable','off');
            set(findobj(gcf,'Tag','waypoints'),'Enable','off');
            set(findobj(gcf,'Tag','cells'),'Enable','off');
        end
    case 'save'
        [filename, pathname] = uiputfile('*.rmt', 'Save Workspace as');
                
        if (~isequal(filename,0) && ~isequal(pathname,0))
            hgsave(fullfile(pathname, filename));
        end
    case 'open'
        [file, path1] = uigetfile({'*.rmt'}, 'Load');
        file = char(file);
        path1 = char(path1);
        file2=fullfile(path1,file);
        if ~isempty(strfind(file,'.rmt'))
            delete(gcf);
            hgload(file2);
        end
    case 'environment_limits'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('x min:'),...
            sprintf('x max:'),...
            sprintf('y min:')...
            sprintf('y max: \n\t')},'Robot Motion Toolbox',...
            [1,1,1,1],{num2str(data.frame_limits(1)),num2str(data.frame_limits(2)),...
            num2str(data.frame_limits(3)),num2str(data.frame_limits(4))});
        try
            temp(1) = eval(answer{1});
            temp(2) = eval(answer{2});
            temp(3) = eval(answer{3});
            temp(4) = eval(answer{4});
        catch 
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        if ((temp(1) >= temp(2)) || (temp(3)>= temp(4)) || (temp(1)~=round(temp(1))) ...
                || (temp(2)~=round(temp(2))) || (temp(3)~=round(temp(3))) || (temp(4)~=round(temp(4))))
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.frame_limits = temp;
        set(data.handle_env,'xlim',[temp(1) temp(2)],'ylim',[temp(3) temp(4)],'XGrid','on','YGrid','on');
        set(gcf,'UserData',data);
    case 'robot_initial'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Initial point:'),...
            sprintf('Final point:'),...
            sprintf('Initial orientation (deg):')},'Robot Motion Toolbox',...
            [1,1,1],{mat2str(data.initial,3),mat2str(data.final,3),...
            num2str(data.orientation,3)});
        try
            initial = eval(answer{1});
            final = eval(answer{2});
            ini_ori = eval(answer{3});
        catch 
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        if (length(initial) ~= 2 || length(final) ~= 2 )
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.initial = initial;
        data.final = final;
        data.orientation = ini_ori;
        set(gcf,'UserData',data);
        %rmt('run_path_planning');
           
    case 'PI_tuning'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Kv:'),...
            sprintf('Ki:'),...
            sprintf('Kh:')...
            sprintf('dstar: \n\t')},'Robot Motion Toolbox',...
            [1,1,1,1],{num2str(data.pi_tuning(1)),num2str(data.pi_tuning(2)),...
            num2str(data.pi_tuning(3)),num2str(data.pi_tuning(4))});
        try
            temp(1) = eval(answer{1});
            temp(2) = eval(answer{2});
            temp(3) = eval(answer{3});
            temp(4) = eval(answer{4});
        catch 
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.pi_tuning = temp;        
        set(gcf,'UserData',data);    
         
    case 'EpsilonVoronoi'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Epsilon:')},'Robot Motion Toolbox',...
            [1],{num2str(data.epsilonvoronoi)});
        try
            temp(1) = eval(answer{1});
        catch 
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.epsilonvoronoi = temp;        
        set(gcf,'UserData',data);   
        
    case 'help_menu'
        set(gcf,'Units','pixel');
        dim = get(gcf,'Position');
        set(gcf,'Units','normalized');
        figure('WindowStyle','modal','Units','pixel','Menubar','None','NumberTitle','off','Name',...
            'About RMTool','Visible','on','Position',[dim(1)+(dim(3)-700)/2 dim(2)+(dim(4)-414)/2 700 414],'Resize','off');%,...
        axes('Position',[0 0 1 1]);
        about23 = imread('rmt_data.txt','bmp');
        %load PNTuse about23; %  524 x  886 
        imshow(about23);
        axis off;
        
    case 'crit_length'
        data = get(gcf,'UserData');
        n = get(findobj(gcf,'Tag','crit_length'),'Value');
        
        if length(data.coord_pathDV)>0 && length(data.coord_pathVG)>0
        
            n_percent = n/100;
            cla(data.handle_env);%new
            axes(data.handle_env);
            rmt_plot_environment(data.obstacles,data.frame_limits);                
            plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
            plot(data.final(1),  data.final(2),  'pw','Markersize',13, 'Color', 'b');
            for i=1:size(data.danger_positions,1)
                plot(data.danger_positions(i, 1),data.danger_positions(i, 2),'xw','Markersize',13, 'LineWidth', 5, 'Color', 'r');
            end;
            path_DV_VG = zeros(size(data.coord_pathDV, 1),2);
            if n==0
                path_DV_VG=data.coord_pathDV;
            elseif n==100
                path_DV_VG=data.coord_pathVG;
            else
                for i=1:size(data.coord_pathDV, 1)
                    path_DV_VG(i,1) = (1-n_percent)*data.coord_pathDV(i,1) + n_percent*data.coord_pathVG(i,1);
                    path_DV_VG(i,2) = (1-n_percent)*data.coord_pathDV(i,2) + n_percent*data.coord_pathVG(i,2);
                end;
            end;
            
            for i=1:size(path_DV_VG, 1)-1
                x=[path_DV_VG(i,1) path_DV_VG(i+1,1)];
                y=[path_DV_VG(i,2) path_DV_VG(i+1,2)];
                plot(x,y,'-','color','r','LineWidth',2);
            end;     
            
            data.trajectory = path_DV_VG';
            set(gcf,'UserData',data);%to save data
        end;
        
end;    % switch 
