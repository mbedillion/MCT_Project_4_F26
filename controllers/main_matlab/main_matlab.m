function main_matlab
    wb_console_print('Project 4 MATLAB controller starting...', WB_STDOUT);
    controllerPath = fileparts(mfilename('fullpath'));
    proofPath = fullfile(controllerPath, 'simulation_proof.log');
    trajectory = getTrajectory(fullfile(controllerPath, 'buggyTrace.csv'));
    timestep = wb_robot_get_basic_time_step();
    initializeDriver();
    calllib('libDriver', 'wbu_driver_set_dipped_beams', true);
    calllib('libDriver', 'wbu_driver_set_gear', 1);
    controller = your_controller(trajectory);
    controller.startSensors(timestep);
    tuning = strcmp(getenv('MATLAB_TUNING'), '1');
    X=[]; Y=[]; delta=[]; xdot=[]; ydot=[]; psi=[]; psidot=[]; force=[]; error=[];
    passedMiddle = false; finished = false; step = 0;
    while calllib('libDriver', 'wbu_driver_step') ~= -1
        step = step + 1;
        [xg,yg,vx,vy,yaw,yawRate,F,steer] = controller.update(timestep);
        steer = clamp(steer, -pi/6, pi/6); F = clamp(F, 0, 16000);
        calllib('libDriver','wbu_driver_set_throttle',clamp(F/5000,0,1));
        calllib('libDriver','wbu_driver_set_steering_angle',-steer);
        [distance,index] = closestNode(xg,yg,trajectory);
        if mod(step,250)==0
            message=sprintf('Step %d: waypoint %d/%d, error %.3f m',step,index-1,size(trajectory,1)-1,distance);
            wb_console_print(message,WB_STDOUT); appendProof(proofPath,message);
        end
        passedMiddle = passedMiddle || abs(index-size(trajectory,1)/2)<100;
        if passedMiddle && index >= size(trajectory,1)-50
            wb_console_print('Destination reached! :)',WB_STDOUT); finished=true; break;
        end
        X(end+1,1)=xg; Y(end+1,1)=yg; delta(end+1,1)=steer;
        xdot(end+1,1)=vx; ydot(end+1,1)=vy; psi(end+1,1)=yaw;
        psidot(end+1,1)=yawRate; force(end+1,1)=F; error(end+1,1)=distance;
    end
    calllib('libDriver','wbu_driver_set_cruising_speed',0);
    calllib('libDriver','wbu_driver_set_steering_angle',0);
    if finished
        [grade,scores]=evaluation(error,trajectory,X,Y,delta);
        summary=sprintf(['Grade: %.2f/100.0; time: %.3f s; average error: %.4f m; ' ...
            'maximum error: %.4f m; average steering change: %.6f rad/timestep'], ...
            grade,length(X)*timestep*0.001,mean(error),max(error),mean(abs(diff(delta))));
        appendProof(proofPath,summary);
        showResult(trajectory,timestep,X,Y,delta,xdot,ydot,force,psi,psidot,error,grade,scores);
    elseif tuning
        appendProof(proofPath,'Simulation ended without reaching destination.');
    end
    calllib('libDriver','wbu_driver_cleanup'); unloadlibrary('libDriver');
end

function initializeDriver
    if libisloaded('libDriver'), unloadlibrary('libDriver'); end
    root=getenv('WEBOTS_HOME');
    if isempty(root), error('WEBOTS_HOME is not set. Start this controller from Webots.'); end
    if ismac
        root=fullfile(root,'Contents'); driverName='libdriver.dylib';
    elseif isunix
        driverName='libdriver.so';
    elseif ispc
        driverName='driver.dll';
    else
        error('Unsupported operating system for the Webots vehicle driver.');
    end
    driverLibrary=fullfile(root,'lib','controller',driverName);
    driverHeader=fullfile(root,'include','controller','c','webots','vehicle','driver.h');
    controllerIncludes=fullfile(root,'include','controller','c');
    if ~isfile(driverLibrary), error('Webots vehicle driver library not found: %s',driverLibrary); end
    if ~isfile(driverHeader), error('Webots vehicle driver header not found: %s',driverHeader); end
    loadlibrary(driverLibrary,driverHeader,'alias','libDriver','includepath',controllerIncludes);
    calllib('libDriver','wbu_driver_init');
end

function appendProof(path,message)
    file=fopen(path,'a'); if file~=-1, fprintf(file,'%s\n',message); fclose(file); end
end
