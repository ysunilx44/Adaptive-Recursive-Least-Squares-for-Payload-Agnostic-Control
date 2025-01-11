classdef PENDULUM < handle
    
    
    
    properties(GetAccess = 'private', SetAccess = 'private')
        m_Haxle                     %handle
        m_Hbase
        m_Hmass
        m_Hrod
        m_Hrodbase
        
        m_Hfig
        m_Haxis
        
        m_Vaxle
        m_Vbase
        m_Vmass
        m_Vrod
        m_Vrodbase
        
        m_WS                        %x length
        
        m_Mode                      %Velocity or Force Control
        
        m_Mp                        %Mass of pendulum
        m_Mc                        %Mass of cart
        m_L                         %Length of pendulum
        m_b                         %Viscous damping coef.

        m_Hrod2            % Handle for the second rod
        m_Hmass2           % Handle for the second mass
        m_Vrod2            % Vertices for the second rod
        m_Vmass2           % Vertices for the second mass
        m_L2                 % Length of the second pendulum
        m_Mp2                % Mass of the second pendulum
        m_b2
        
        m_Q                         %q1 = theta, q2 = theta_dot, q3 = x, q4 = xdot
    end
    
    properties(GetAccess = 'public', SetAccess = 'public')
        U = 0 ;                     %control input.
    end
    
    properties(GetAccess = 'public', SetAccess = 'private')
        Q  = 0;                     
        L  = 0;
        Mp = 0;
        Mc = 0;
        b  = 0;
        L2  = 0;               % Length of the second pendulum
        Mp2  = 0;              % Mass of the second pendulum
        b2 = 0;
    end
    
    methods
        function set.U(obj,val)
            obj.U   = val;
            obj.UPDATE_OTHER_DATA;
        end
        
        function Q = get.Q(obj)
            Q = obj.m_Q;
        end
    
        function L = get.L(obj)
            L = obj.m_L;
        end
        
        function Mp = get.Mp(obj)
            Mp = obj.m_Mp;
        end
        
        function Mc = get.Mc(obj)
            Mc = obj.m_Mc;
        end
        
        function b = get.b(obj)
            b = obj.m_b;
        end
        function L2 = get.L2(obj)
            L2 = obj.m_L2;
        end
        function Mp2 = get.Mp2(obj)
            Mp2 = obj.m_Mp2;
        end
        function b2 = get.b2(obj)
            b2 = obj.m_b2;
        end
    end
    
    methods(Access = 'public')
        function obj = PENDULUM(varargin)
            
            %default
            obj.m_WS        = 3;
            obj.m_Q         = [0 0 0 0 0 0];
            
            obj.m_Hfig      = gcf;
            obj.m_Haxis     = gca;
            
            obj.m_Mp        = 1;
            obj.m_Mc        = 1;
            obj.m_L         = 1;
            obj.m_b         = 1;
            obj.m_L2        = 1;            % Length of the second pendulum
            obj.m_Mp2       = 1;            % Mass of the second pendulum
            obj.m_b2        = 1;
            
            obj.m_Mode      = 'Force'; %'Velocity'
            
            %specified
            if ~isempty(varargin)
                for k = 1:2:length(varargin)
                    tmp_str = varargin{k};
                    switch tmp_str
                        case 'Mode'
                            obj.m_Mode          = varargin{k+1};
                        case 'Damping'
                            obj.m_b             = varargin{k+1};
                        case 'MassCart'
                            obj.m_Mc        	= varargin{k+1};
                        case 'MassPendulum'
                            obj.m_Mp        	= varargin{k+1};
                        case 'PendulumLength'
                            obj.m_L            	= varargin{k+1};
                        case 'SecondPendulumLength'   % New case for second pendulum length
                            obj.m_L2 = varargin{k+1};
                        case 'SecondPendulumMass'     % New case for second pendulum mass
                            obj.m_Mp2 = varargin{k+1};
                        case 'SecondPendulumDamping'  % New case for second pendulum damping
                            obj.m_b2 = varargin{k+1};
                        case 'WorkspaceLength'
                            obj.m_WS        	= varargin{k+1};
                        case 'InitialStates'
                            obj.m_Q             = varargin{k+1};
                            obj.m_Q             = obj.m_Q(:);
                        case 'Fig'
                            obj.m_Hfig          = varargin{k+1};
                        case 'Axis'
                            obj.m_Haxis         = varargin{k+1};
                        otherwise
                            errordlg([tmp_str,' is not a valid parameter name.'],'Syntax Error');
                            return
                    end
                end
            end
            
            %initialize robot
            obj.INITIALIZE
        end
    end
    methods(Access = 'private')
        function INITIALIZE(obj)
            %COLORS
            c_runway 	= [162 20  20 ]/255;
            c_base  	= [190 122 66 ]/255;
            c_axle  	= [142 152 187]/255;
            c_rodbase   = [190 122 66 ]/255;
            c_rod       = [180   180 130]/255;
            c_mass      = [100 100 100]/255;
            c_floor     = [100 100 100]/255;
            
            %INITIALIZE THE FIGURE
            figure(obj.m_Hfig)
            axes(obj.m_Haxis)
            cla
            set(gcf,'RendererMode','manual','Renderer','OpenGL')
            
            %GET THE PARTS
%                         cd 'Pend_Model'
%                         [F_base, V_base, ~]             = STL2MAT('base.stl',1);
%                         [F_axle, V_axle, ~]				= STL2MAT('axle.stl',1);
%                         [F_rodbase, V_rodbase, ~]		= STL2MAT('rodbase.stl',1);
%                         [F_rod, V_rod, ~]               = STL2MAT('rod.stl',1);
%                         [F_mass, V_mass, ~]             = STL2MAT('mass.stl',1);
%                         [F_runway, V_runway, ~]         = STL2MAT('runway.stl',1);
%                         [F_floor, V_floor, ~]           = STL2MAT('floor.stl',1);
%                         cd ..
%             
%                         save PARTS F_base V_base F_axle V_axle F_rodbase V_rodbase F_rod ...
%                         V_rod F_mass V_mass F_runway V_runway F_floor V_floor
%             
            load PARTS2
            
            
            %DRAW THE PARTS AS PATCH OBJECTS WITH HANDLES
            WS   = obj.m_WS;
            del  = 0.09;
            %runway
            V_runway(:,1)  = V_runway(:,1)*WS;
            V_runway(:,3)  = V_runway(:,3) - del;
            patch('faces', F_runway, 'Vertices' ,V_runway,'EdgeColor','none','FaceC',c_runway,...
                'AmbientStrength',.6,'DiffuseStrength',.9,'SpecularStrength',.3,'SpecularExponent',30);
            
            %base
            V_base(:,3)  = V_base(:,3) - del;
            obj.m_Hbase = patch('faces',F_base, 'Vertices' ,V_base,'EdgeColor','none','FaceC',c_base,...
                'AmbientStrength',.3,'DiffuseStrength',.6,'SpecularStrength',.8,'SpecularExponent',10);
            obj.m_Vbase = V_base;
            
            %axle
            V_axle(:,3)  = V_axle(:,3) - del;
            obj.m_Haxle = patch( 'faces',F_axle, 'Vertices' ,V_axle,'EdgeColor','none','FaceC',c_axle,...
                'AmbientStrength',.3,'DiffuseStrength',.6,'SpecularStrength',.8,'SpecularExponent',10);
            obj.m_Vaxle = V_axle;
            
            %rodbase
            V_rodbase(:,3)  = V_rodbase(:,3) - del;
            obj.m_Hrodbase = patch( 'faces',F_rodbase, 'Vertices' ,V_rodbase,'EdgeColor','none','FaceC',c_rodbase,...
                'AmbientStrength',.3,'DiffuseStrength',.6,'SpecularStrength',.8,'SpecularExponent',10);
            obj.m_Vrodbase = V_rodbase;
            
            %rod
            V_rod(:,3)  = V_rod(:,3) * obj.m_L;
            obj.m_Hrod = patch( 'faces',F_rod, 'Vertices' ,V_rod,'EdgeColor','none','FaceC',c_rod,...
                'AmbientStrength',.3,'DiffuseStrength',.6,'SpecularStrength',.8,'SpecularExponent',10);
            obj.m_Vrod = V_rod;
            
            %mass
            V_mass(:,3)  = V_mass2(:,3) + obj.m_L;
            obj.m_Hmass = patch( 'faces',F_mass, 'Vertices' ,V_mass,'EdgeColor','none','FaceC',c_mass,...
                'AmbientStrength',.3,'DiffuseStrength',.6,'SpecularStrength',.8,'SpecularExponent',10);
            obj.m_Vmass = V_mass;
            
            % Second Rod
            V_rod2(:,3) = V_rod2(:,3) * obj.m_L2;  % Scale the rod vertices by the length of the second pendulum
            obj.m_Hrod2 = patch('faces', F_rod, 'Vertices', V_rod2, 'EdgeColor', 'none', 'FaceC', c_rod, ...
                'AmbientStrength', .3, 'DiffuseStrength', .6, 'SpecularStrength', .8, 'SpecularExponent', 10);
            obj.m_Vrod2 = V_rod2;
            
            % Second Mass
            V_mass2(:,3) = V_mass2(:,3) + obj.m_L2;  % Position the mass at the end of the second rod
            obj.m_Hmass2 = patch('faces', F_mass, 'Vertices', V_mass2, 'EdgeColor', 'none', 'FaceC', c_mass, ...
                'AmbientStrength', .3, 'DiffuseStrength', .6, 'SpecularStrength', .8, 'SpecularExponent', 10);
            obj.m_Vmass2 = V_mass2;


            %floor
            V_floor(:,1:2) = V_floor(:,1:2)*0.5/0.2;
            V_floor(:,3)   = V_floor(:,3) - 3;
            for i = 1:round(WS/.500)
                for j = 1:round(1/.500)
                    floor(i,j) = patch('faces', F_floor, 'Vertices' ,V_floor,'EdgeColor','none',...
                        'FaceC',c_floor,'FaceAlpha',1,'Visible','off','AmbientStrength',.5,...
                        'DiffuseStrength',.3,'SpecularStrength',.1,'SpecularExponent',50);
                    tmp         = get(floor(i,j),'Vertices');
                    tmp(:,1)    = tmp(:,1) + (i-1)*.500 + -WS/2;
                    tmp(:,2)    = tmp(:,2) + (j-1)*.500 + -0.5;
                    set(floor(i,j),'Vertices',tmp,'Visible','on')
                end
            end
            
            
            %MAKE AXIS LOOK NICE
            axis equal
            axis([-obj.m_WS/2-0.1 obj.m_WS/2+0.1 -0.6 0.6 -3 1])
            
            light
            lighting gouraud
            
            %perspective
            camproj('perspective')
            view(20,32)
        end
        
        function UPDATE_OTHER_DATA(obj)
            persistent THEN u_old
            
            if isempty(THEN)
                THEN    = obj.GETTIME;
                u_old   = 0;
            end
            
            %KEEP TRACK OF TIME
            now     = obj.GETTIME;
            del_t   = now - THEN;
            THEN    = now;
            del_t = 0.05;
            
            %CALCULATE THE NEW STATES               
            T       = [0 del_t];
            Q       = obj.m_Q;
            L       = obj.m_L;
            Mp      = obj.m_Mp;
            Mc      = obj.m_Mc;
            b       = obj.m_b;
            L2      = obj.m_L2;
            Mp2     = obj.m_Mp2;
            b2      = obj.m_b2;
            Mode    = obj.m_Mode;
            

            if del_t ~= 0
                
                switch Mode
                    case 'Velocity'
                        u       = (obj.U - u_old)/del_t;
                        u_old   = obj.U;
                    otherwise
                        u = obj.U;
                end
                
                [~, Q] = ode45(@(T,Q)dequations(T,Q,u,L,Mp,Mc,b,L2, Mp2, b2, Mode),T,Q);                     
                Q = Q(end,:);
                
                while Q(1) > 2*pi
                    Q(1) = Q(1) - 2*pi;
                end
                
                while Q(1) < -2*pi
                    Q(1) = Q(1) + 2*pi;
                end
                obj.m_Q = Q(:);
            end
       
            %NOW UPDATE THE PATCH OBJECTS
            obj.DRAW
        end
        
 
        
        function DRAW(obj)
            
            POS = obj.m_Q(5);
            
            %CART
            V       = obj.m_Vbase;
            V(:,1)  = V(:,1) + POS;
            set(obj.m_Hbase, 'Vertices',V)
            
            %AXLE
            V       = obj.m_Vaxle;
            V(:,1)  = V(:,1) + POS;
            set(obj.m_Haxle, 'Vertices',V)
            theta1 = obj.m_Q(1) + pi;
            %ROD BASE
            V       = obj.m_Vrodbase;
            V       = [obj.Ry(-theta1) * V']';
            V(:,1)  = V(:,1) + POS; 
            set(obj.m_Hrodbase, 'Vertices',V)
            
            %ROD
            V       = obj.m_Vrod;
            V       = [obj.Ry(-theta1) * V']';
            V(:,1)  = V(:,1) + POS;
            x_off = V(:, 1);
            set(obj.m_Hrod, 'Vertices',V)
            
            %MASS
            V       = obj.m_Vmass;
            V       = [obj.Ry(-theta1) * V']';
            V(:,1)  = V(:,1) + POS;
            set(obj.m_Hmass, 'Vertices',V)
            
            % Second Pendulum
            % Calculate the rotation and translation for the second rod
            
            theta2 = obj.m_Q(2) + pi; % Assume theta2 is the angle of the second pendulum
            V = obj.m_Vrod2;
            V = [obj.Ry(-theta2) * V']';
            % Offset position based on the first pendulum’s length and angle
            x_offset = POS - obj.m_L * sin(theta1);
            z_offset = obj.m_L * cos(theta1);

            V(:,1) = V(:,1) + x_offset;  % Translate the rod
            V(:,3) = V(:,3) + z_offset;
            set(obj.m_Hrod2, 'Vertices', V);
        
            % Update the position of the second mass
            V = obj.m_Vmass2;
            V = [obj.Ry(-theta2) * V']';
            V(:,1) = V(:,1) + x_offset;
            V(:,3) = V(:,3) + z_offset;  % Translate the mass
            set(obj.m_Hmass2, 'Vertices', V);
        end
            
            
      
        
        function R = Ry(obj, angle)
            R = [cos(angle)      0       sin(angle)      ;
                 0               1       0               ;
                 -sin(angle)     0       cos(angle)      ];
        end
        
        function out = GETTIME(obj)
            tmp = clock;
            out = 3600*tmp(4) + 60*tmp(5)+tmp(6);
        end
     
    end
    
    
    
end