function t = accelerationSimulation(vehiclefile, dx)

    %% Loading vehicle
    
    % filename
    
    %% Simulation settings
    
    % date and time in simulation name
    use_date_time_in_name = false ;
    % time step
    dt = 1E-3 ;
    % maximum simulation time for memory preallocation
    t_max = 60 ;
    % acceleration sensitivity for drag limitation
    ax_sens = 0.05 ; % [m/s2]
    % speed traps
    speed_trap = [50;100;150;200;250;300;350]/3.6 ;
    % track data
    bank = 0 ;
    incl = 0 ;
    
    dx = 1; % [m]
    
    x_max = 75;
    
    %% Vehicle data preprocessing
    
    % loading file
    veh = load(vehiclefile) ;
    % mass
    M = veh.M ;
    % gravity constant
    g = 9.81 ;
    % longitudinal tyre coefficients
    dmx = veh.factor_grip*veh.sens_x ;
    mux = veh.factor_grip*veh.mu_x ;
    Nx = veh.mu_x_M*g ;
    % normal load on all wheels
    Wz = M*g*cosd(bank)*cosd(incl) ;
    % induced weight from banking and inclination
    Wy = M*g*sind(bank) ;
    Wx = M*g*sind(incl) ;
    % ratios
    rf = veh.ratio_final ;
    rg = veh.ratio_gearbox ;
    rp = veh.ratio_primary ;
    % tyre radius
    Rt = veh.tyre_radius ;
    % drivetrain efficiency
    np = veh.n_primary ;
    ng = veh.n_gearbox ;
    nf = veh.n_final ;
    % engine curves
    rpm_curve = [0;veh.en_speed_curve] ;
    torque_curve = veh.factor_power*[veh.en_torque_curve(1);veh.en_torque_curve] ;
    % shift points
    shift_points = table2array(veh.shifting(:,1)) ;
    shift_points = [shift_points;veh.en_speed_curve(end)] ;
    
    %% Acceleration preprocessing
    
    % memory preallocation
    N = t_max/dt ;
    T = -ones(N,1) ;
    X = -ones(N,1) ;
    V = -ones(N,1) ;
    A = -ones(N,1) ;
    RPM = -ones(N,1) ;
    TPS = -ones(N,1) ;
    BPS = -ones(N,1) ;
    GEAR = -ones(N,1) ;
    MODE = -ones(N,1) ;
    % initial time
    t = 0 ;
    t_start = 0 ;
    % initial distance
    x = 0 ;
    x_start = 0 ;
    % initial velocity
    v = 0 ;
    % initial accelerartion
    a = 0 ;
    % initial gears
    gear = 1 ;
    gear_prev = 1 ;
    % shifting condition
    shifting = false ;
    % initial rpm
    rpm = 0 ;
    % initial tps
    tps = 0 ;
    % initial bps
    bps = 0 ;
    % initial trap number
    trap_number = 1 ;
    % speed trap checking condition
    check_speed_traps = true ;
    % iteration number
    i = 1 ;
    
    %% HUD display
    
    %  folder
    [folder_status,folder_msg] = mkdir('OpenDRAGSims') ;
    % diary
    if use_date_time_in_name
        date_time = "_"+datestr(now,'yyyy_mm_dd')+"_"+datestr(now,'HH_MM_SS') ; %#ok<UNRCH>
    else
        date_time = "" ;
    end
    simname = "OpenDRAGSims/OpenDRAG_"+veh.name+date_time ;
    delete(simname+".log") ;
    diary(simname+".log") ;
    % HUD
    
    
    %% Acceleration
    
    while x < x_max
        % saving values
        MODE(i) = 1 ;
        T(i) = t ;
        X(i) = x ;
        V(i) = v ;
        A(i) = a ;
        RPM(i) = rpm ;
        TPS(i) = tps ;
        BPS(i) = 0 ;
        GEAR(i) = gear ;
        if check_speed_traps
            % checking if current speed is above trap speed
            if v>=speed_trap(trap_number)
                % fprintf('%s%3d %3d%s ','Speed Trap #',trap_number,round(speed_trap(trap_number)*3.6),'km/h')
                % hud(v,a,rpm,gear,t,x,t_start,x_start)
                % next speed trap
                trap_number = trap_number+1 ;
                % checking if speed traps are completed
                if trap_number>length(speed_trap)
                    check_speed_traps = false ;
                end
            end
        end
        % aero forces
        Aero_Df = 1/2*veh.rho*veh.factor_Cl*veh.Cl*veh.A*v^2 ;
        Aero_Dr = 1/2*veh.rho*veh.factor_Cd*veh.Cd*veh.A*v^2 ;
        % rolling resistance
        Roll_Dr = veh.Cr*(-Aero_Df+Wz) ;
        % normal load on driven wheels
        Wd = (veh.factor_drive*Wz+(-veh.factor_aero*Aero_Df))/veh.driven_wheels ;
        % drag acceleration
        ax_drag = (Aero_Dr+Roll_Dr+Wx)/M ;
        % rpm calculation
        if gear==0 % shifting gears
            rpm = rf*rg(gear_prev)*rp*v/Rt*60/2/pi ;
            rpm_shift = shift_points(gear_prev) ;
        else % gear change finished
            rpm = rf*rg(gear)*rp*v/Rt*60/2/pi ;
            rpm_shift = shift_points(gear) ;
        end
        % checking for gearshifts
        if rpm>=rpm_shift && ~shifting % need to change gears
            if gear==veh.nog % maximum gear number
                % HUD
                
                hud(v,a,rpm,gear,t,x,t_start,x_start)
                
            else % higher gear available
                % shifting condition
                shifting = true ;
                % shift initialisation time
                t_shift = t ;
                % zeroing  engine acceleration
                ax = 0 ;
                % saving previous gear
                gear_prev = gear ;
                % setting gear to neutral for duration of gearshift
                gear = 0 ;
            end
        elseif shifting % currently shifting gears
            % zeroing  engine acceleration
            ax = 0 ;
            % checking if gearshift duration has passed
            if t-t_shift>veh.shift_time
                % HUD
          
                % shifting condition
                shifting = false ;
                % next gear
                gear = gear_prev+1 ;
            end
        else % no gearshift
            % max long acc available from tyres
            ax_tyre_max_acc = 1/M*(mux+dmx*(Nx-Wd))*Wd*veh.driven_wheels ;
            % getting power limit from engine
            engine_torque = interp1(rpm_curve,torque_curve,rpm) ;
            wheel_torque = engine_torque*rf*rg(gear)*rp*nf*ng*np ;
            ax_power_limit = 1/M*wheel_torque/Rt ;
            % final long acc
            ax = min([ax_power_limit,ax_tyre_max_acc]) ;
        end
        % tps
        tps = ax/ax_power_limit ;
        % longitudinal acceleration
        if x >= 75
            break;
        end
        a = ax+ax_drag ;
        % new position
        dt = (-v + sqrt(v^2 + 2*a*dx)) / a; % Time taken to cover dx at acceleration a
        v = v + a*dt; % New velocity after covering dx
        t = t + dt; % Update the total time
        x = x + dx; % Update the position by dx
        % next iteration
        i = i+1 ;
    end
    i_acc = i ; % saving acceleration index
    % average acceleration
    a_acc_ave = v/t ;
    % disp(['Average acceleration:    ',num2str(a_acc_ave/9.81,'%6.3f'),' [G]'])
    % disp(['Peak acceleration   :    ',num2str(max(A)/9.81,'%6.3f'),' [G]'])
    hud(v,a,rpm,gear,t,x,t_start,x_start)
    % acceleration timer
    
    %% Results compression
    
    % getting values to delete
    to_delete = T==-1 ;
    % deleting values
    T(to_delete) = [] ;
    X(to_delete) = [] ;
    V(to_delete) = [] ;
    A(to_delete) = [] ;
    RPM(to_delete) = [] ;
    TPS(to_delete) = [] ;
    BPS(to_delete) = [] ;
    GEAR(to_delete) = [] ;
    MODE(to_delete) = [] ;
    
    %% HUD function
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [] = hud(v,a,rpm,gear,t,x,t_start,x_start)
    %     disp(['          Speed         : ',num2str(v*3.6),' km/h'])
    % %     disp(['          Acceleration  : ',num2str(g/9.81),' G'])
    % %     disp(['          RPM           : ',num2str(rpm)])
    % %     disp(['          Gear          : ',num2str(gear)])
    %     disp(['          Time          : ',num2str(t-t_start),' s'])
    %     disp(['          Distance      : ',num2str(x-x_start),' m'])
    %     disp(['          Total Time    : ',num2str(t),' s'])
    %     disp(['          Total Distance: ',num2str(x),' m'])
        % fprintf('%7.2f\t%7.2f\t%7d\t%7d\t%7.2f\t%7.2f\t%7.2f\t%7.2f\n',v*3.6,a/9.81,round(rpm),gear,t,x,t-t_start,x-x_start) ;
    end
end
