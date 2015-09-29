%Autodrill_Parallel v0.1 - matlab class controlling parallel singal
%generation and recording on NI-DAQ board
%unit (NIDAQ USB 6009/6008)
%Dan Morozoff - 11/11/14
classdef Autodrill_Parallel < handle
    properties
      num_workers = 1;  
      %Currently created at instantiation
      profile;
      cluster;
      pool;
      
      signal_job;
      listening_job;
      signal_task;
      listening_task;
      
      time_jobSubmit;
      
      auto_output;
    end
    events
        ClearClass
    end
     methods (Static)
      function handleEvnt(src,evtdata)
         if src.ClearClass
            disp('ClearClass is true')  % Respond to true ToggleState here
            whos;
%             clear(obj)
         end
      end
   end
    methods (Access = public)
        function obj = Autodrill_Parallel(auto)
            % create cluster & pool
             obj.profile = parallel.defaultClusterProfile;
             obj.cluster = parcluster(obj.profile);
             
             obj.pool = parpool(obj.cluster,obj.num_workers); 
             
             ajob =  obj.mJob('output',auto);
             
%               obj.mJob('input');
             
%                 global auto;
%                 auto=Autodrill('output');
%                 obj.auto_output=Autodrill('output');
 
            
              obj.auto_output = auto; 
              assignin('base','auto',auto)
             
             obj.signal_task = createTask(obj.signal_job ,@evalin,0,{'base',' j = getCurrentJob;auto = j.JobData; auto.output_s= daq.createSession(''ni'');auto.output_s.addAnalogOutputChannel(''Dev1'',auto.signal_outputChannel,''Voltage'');auto.sendDAQProbePulseTimed(100,100)'},'CaptureDiary', true);
                 
%              obj.signal_task = createTask(obj.signal_job ,@evalin,0, {'base','test=''mytest'';disp(''test'''},'CaptureDiary', true);
%              obj.signal_task.InputArguments = {};
             
%              obj.listening_task = createTask(obj.listening_job,@eval,0, {'auto=Autodrill(''input'');auto.acquireDAQProbePulse(''background'',16000,1000)'});
             
            
        end
        %% Wrapper functions- access points
        function pulse_mouse(obj)
           obj.time_jobSubmit = tic;
            
           submit(obj.signal_job);
            
%            obj.signal_job =  batch('batchCode',0);
            toc(obj.time_jobSubmit)
        end
        
        function listen_mouse(obj)
           submit(obj.listening_job);
        end
        %% Main execution functions
        function job = mJob(obj,type,data)
          if strcmp(type,'output')
            
              disp('Making output job');
            obj.signal_job= createJob(obj.cluster);
            obj.signal_job.AutoAttachFiles = false;
        
          elseif strcmp(type,'input')            
              disp('Making input job');
              obj.listening_job = createJob(obj.cluster);
          end
          
          obj.signal_job.JobData = data;
          job= obj.signal_job;
        end
        function cancelJob(~,job)
           cancel(job); 
           %Reload jobs
           obj.signal_job= obj.mJob('output');
           obj.makeJob('input');
        end
        function cancelTask(~,task)
            cancel(task);
        end
        function killPool(obj)
            delete(obj.pool);
        end
        function closeAndClear(obj)
           notify(obj,'ClearClass'); % Broadcast notice of event 
        end
    end
end
