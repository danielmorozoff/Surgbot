%!!!!DEPRECATED!!!!
%executor v0.1 - matlab class controlling execution of NI DAQ commands
%unit (NIDAQ USB 6009/6008)
%Dan Morozoff - 9/11/14
function [profile,cluster, pool,job,task] = executor(type,cluster)
  num_workers = 1;
% load / create cluster
    if isempty(cluster)
        profile = parallel.defaultClusterProfile;
        cluster = parcluster(profile);
    end
    pool = parpool(cluster,num_workers);
% make job
    job= createJob(cluster);
    if strcmp(type,'output')
        disp('Making output job');
        task = createTask(job,@eval,0, {'auto=Autodrill(''output'');auto.sendDAQProbePulseTimed(100,100)'});
    elseif strcmp(type,'input')
        disp('Making input job');
         task = createTask(job,@eval,0, {'auto=Autodrill(''input'');auto.acquireDAQProbePulse(''background'',16000,1000)'});
    end
    submit(job);
end
%% Deprecated Code
% create batch job - need the class
%     batch('auto=Autodrill(''output'');auto.sendDAQProbePulseTimed(100,100)');
    %Need to fire an event for the acquisition to start
%     auto= Autodrill('input');
%         auto.acquireDAQProbePulse('background',16000,10,1000);