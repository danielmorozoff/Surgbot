%Autodrill v0.1.8 - matlab class controlling sutter controllers (MP285). NI
%DAQ I/O. Arduino Yun-BK Precision Power
%Dan Morozoff - 12/12/14
%This version maintains an in sync in out signal/read generator
classdef Autodrill_v0_18 < handle

    properties
    %ChannelS
    %fancy boards
    in_out_s;
    %shitty boards
    in_s;
    out_s;
    
    %COM PORTS
    sutter_port = 'COM5';
    ardui_port = 'COM4';
    %Channels
    signal_inputChannel = 'ai0';
    signal_outputChannel = 'ao1';
    
    %Settings for session
    rate = 10000; %Set to 16 KHz scans/s
    %Number of cyles to probe fxn
    pulse_cycles = 0;
    %Arduino device to control BK PRECISION power unit. These vary in
    %their max current outputs. We used 1902 (15A)
    ardui;
    voltage_reg_pin = 5;%Pin to adjust voltage
    current_reg_pin = 9;%Pin to adjust current max.
    
    %Sutter device
    sutter;
    %DRILL STATE
    skull_pierced = false;
    
    z_step_size = 50; %in micrometers
    % Sutter scaling factor
    %scales to the device r from mm to micrometers
    scaling_factor = 1000;
    %Path storage
    path; %[x y z ang]
    path_r;
    %Milling
    spline_obj;
    spline_pnts;
    milling_offset = 75;
    
    %Fourier filter
    %Frequencies that will be displayed/potentially measured to
    %boost speed.
    freq_top = 110;
    freq_bot = 90;
    
        %Mouse FFT Params
        mouse_fft_background_signal = 46.0962; % shitty comp in vivarium 1Mohm resist
        %0.7085; % This is for a gain of x100 avg over 2mins of measurements. this is measured empirically.
        mouse_fft_max_signal = 86.0633; % shitty comp in vivarium- actual brain
        %266.6654; %this represents a closed loop across the NiDAQ
        mouse_fft_attenuation_factor = .6;
        
        %Measured signals
        lifetime_logged_integral_of_fft = [];
        current_measurement_log_of_fft = [];
    
    %Through-hole properties
    holes_to_make = [];
       
    %Parallel resources
    parallel_cluster;
    worker_pool;
    current_signal_worker_job; %This is reserved for the current signal generating worker
    
    %Listener Handles
    DATA_AVAILABLE;
    
    end    
    methods (Access = public)
        %%Constructor...
        function obj = Autodrill_v0_18(NiDAQ_board,remote)
            %add libraries to path
            addpath('dependencies');
            addpath('dependencies/Sutter MP285');
            addpath('dependencies/ArduinoIO');
            
            %Initialize Arduino IO
            try
                obj.ardui = arduino(obj.ardui_port);
                obj.ardui.pinMode(obj.voltage_reg_pin,'output');
                obj.ardui.pinMode(obj.current_reg_pin,'output');
            catch err
                delete(instrfind({'Port'},{obj.ardui_port}));
            end
            %Initialize Sutter
            try
           
            obj.sutter = sutterMP285(obj.sutter_port);
                moveTo(obj.sutter,[0 0 0]);
            catch err
              delete(instrfind({'Port'},{obj.sutter_port}));
            end

            %Initialize DAQa.
            daq_devices = daq.getDevices;
            device= daq_devices(1).ID;
            %only ao0 and ao1 are outputs on the USB6008/6009 boards.
            
            %Create NI-DAQ session object and set the in/out channels
            %!!!Still need to queue data for out channel
            
            if(strcmp(NiDAQ_board,'fancy'))
                obj.in_out_s= daq.createSession('ni');
                obj.in_out_s.Rate = obj.rate;
                obj.in_out_s.addAnalogInputChannel(device,obj.signal_inputChannel,'Voltage');
                obj.in_out_s.addAnalogOutputChannel(device,obj.signal_outputChannel,'Voltage');
                
            elseif strcmp(NiDAQ_board,'shitty')
                
                    obj.out_s= daq.createSession('ni');
                    obj.out_s.addAnalogOutputChannel(device,obj.signal_outputChannel,'Voltage');
                
                    obj.in_s = daq.createSession('ni');
                	obj.in_s.addAnalogInputChannel(device,obj.signal_inputChannel,'Voltage');
                    obj.in_s.Rate = obj.rate;
               
            end
            %Setup parallel compute
            if remote
                poolobj = gcp('nocreate');
                if(isempty(poolobj))
                    obj = obj.createParallelProcess(1);
                else
                    %Can't figure out how to get the current cluster obj --
                    %don't really need it
                    obj.worker_pool = poolobj;
                end
            end
            
            %Initialize the MP 285 device -- this opens the serial object now
            %stored in sutter which extends the serial class.
%              sutter=sutterMP285(dev_loc);
%              sutter.verbose=1;
        end
        %% A set of functions to store and make through holes-- basically waypoints
        %[x y z] in micrometers
        function storeHole(obj)
            pos = getPosition(obj.sutter)';
             cprintf([0 1 0],'Storing %g  um...\n',pos);
            obj.holes_to_make  = [obj.holes_to_make ; pos];
        end
        
        function make_marked_holes(obj, speed)
            figure;
            hold on;
            plot3(0,0,0,'gx','MarkerSize',50);    
            grid on;
            drawnow
            cnt  = 1 ;
            obj.setDrillVelocity(speed);
           while  cnt <= size(obj.holes_to_make,1)
               
               x = obj.holes_to_make(cnt,1);
               y = obj.holes_to_make(cnt,2);
               z = obj.holes_to_make(cnt,3)-.2;
               
                   cprintf([0 1 0],'Move To %g %g %g um...\n',x ,y ,z);
                   moveTo(obj.sutter,[x y z]);

                   plot3(x/obj.scaling_factor,y/obj.scaling_factor,-1*z/obj.scaling_factor,'r.','MarkerSize',30);   
                   drawnow 
                   
                   %Drill
                   [pierced,pierced_z]= obj.stepDrillZ(x, y, z);
                    plot3(x/obj.scaling_factor,y/obj.scaling_factor,-1*pierced_z/obj.scaling_factor,'b.','MarkerSize',20);    
                    drawnow;
                    
               cnt = cnt+1;
               
           end
           obj.drillControl('off',0,0);
        end
        %% Primary control loop -- to make craniotomy
        function obj=make_craniotomy(obj,d,distance_between_pnts)
%             obj.setDrillVelocity(400) %in micrometers/s
            obj.resetToOrigin();
            obj.path = [];
            r = d/2;
            obj.path_r = r;

            %Add a step_size that is proportional to the linead distance between
            %hole centers
            radial_step_size = 2*asin(distance_between_pnts/(2*r));
            figure;
            hold on;
            plot3(0,0,0,'gx','MarkerSize',50);    
            grid on;
            drawnow
            
            obj.setDrillVelocity(20);
            
            t0 = tic;
            for ang=0:radial_step_size:2*pi
                z_start= -1 ; %This can be modififed in order to evaluate how to detect the surface of the bone
%                   z =randi([-2 -1],1,1);
%                 Scale to the device
               [x,y] = obj.convertRadialToXY(ang);
                z = obj.scaling_factor*z_start;
                
             
               if(ang<2*pi)
                   cprintf([0 1 0],'Move To %g %g %g um...\n',x ,y ,z);
                   moveTo(obj.sutter,[x y z]);

                   plot3(x/obj.scaling_factor,y/obj.scaling_factor,-1*z/obj.scaling_factor,'r.','MarkerSize',30);   
                   drawnow 
                   
                   %Drill
                   [pierced,pierced_z]= obj.stepDrillZ(x, y, z);
                    plot3(x/obj.scaling_factor,y/obj.scaling_factor,-1*pierced_z/obj.scaling_factor,'b.','MarkerSize',20);    
                    drawnow;
               end
                        
                if(abs(x)<(10^-6)) x = 0;end
                if(abs(y)<(10^-6)) y = 0;end    
                cprintf([0 1 .5], 'STORING PATH: %d %d %d \n', x,y,pierced_z);
                
                if(ang==2*pi)
                    obj.path = [obj.path; x y  obj.path(1,3) ang];
                else
                    obj.path = [obj.path; x y  pierced_z ang];
                end
                
                %Retract drill prior to move
                moveTo(obj.sutter,[x,y,z]);
                
            end
            toc(t0);
            cprintf([1 0 0],'Path: %d \n', obj.path);
            
            %Interpolate and retrace
            obj=obj.splineInterpolateZDrill;
%             obj=obj.splineInterpolateZDrill_2;
            
            %Mill the interpolated values.
            obj.setDrillVelocity(10);
            obj.mill();
            
            obj.resetToOrigin();
            disp(obj.path)
        end
        %% Drill functions
        %Drill on or off/ voltage/current
        function obj = drillControl(obj,state,voltage,current)
            if(strcmp(state,'on'))
                    obj.setDrillCurrent(current);
                    obj.setDrillVoltage(voltage);
            else
                    obj.setDrillCurrent(0);
                    obj.setDrillVoltage(0);
            end
        end
        %Milling mode of the drill
        function mill(obj)
            %pull back drill 
            x0= obj.spline_pnts(1,1);
            y0 = obj.spline_pnts(1,2);
            moveTo(obj.sutter,[x0 y0 -5000]);
            %turn drill on -- set speed (adjusted empirically)
            obj.drillControl('on',25,2);
            %Begin milling move.
           for(i=1:1:length(obj.spline_pnts))
                x= obj.spline_pnts(i,1);
                y = obj.spline_pnts(i,2);
                z = obj.spline_pnts(i,3)-obj.milling_offset;
                moveTo(obj.sutter,[x y z]);
                hold on;
                plot3(x/1000,y/1000,-1*z/1000,'c.','MarkerSize',5);
                drawnow;
           end
           obj.drillControl('off',0,0);
        end
        %Interpolation functions for the drill milling mode
        function obj = splineInterpolateZDrill(obj)
            path_to_user = obj.path(1:(end),1:3)';
            cprintf([1 0 0 ], 'Interpolating...');
            path_to_user

            sp = cscvn(path_to_user(:,[1:end 1]));
            obj.spline_obj = sp;
            %Points for the spline fit
            points=fnplt(sp);
            obj.spline_pnts = points'; %transpose them in order to keep the same sorting style...
        end
        function obj = splineInterpolateZDrill_2(obj)
            cprintf([1 0 0 ], 'Interpolating...');
            for(i=1:1:length(obj.path))
                if(i<length(obj.path))
                    path_to_user = obj.path(i:i+1,1:3)';
                    
                    cprintf([1 0 0 ],'spline btwn:\n');
                        path_to_user
                    
                    sp = cscvn(path_to_user(:,[1:end 1]));
%                   Store spline if need be later...
%                     obj.spline_obj = sp;
                    %Points for the spline fit
                    points=fnplt(sp);
                    obj.spline_pnts = [obj.spline_pnts; points'];
                end
            end
        end
        %Step drill in z to do depth holes 
        function [pierced,curZ] = stepDrillZ(obj,x_in,y_in,z_in)
            pierced = 0;
            cnt = 0;
            obj.drillControl('on',28,2);

            while(~pierced)
                curZ = (z_in+(obj.z_step_size)*cnt);
                fprintf(1,'Drilling to depth:  %d um \n',curZ);
                moveTo(obj.sutter,[x_in y_in curZ]); 
               %DEV
%                 if(curZ>=1000* 1)
%                     obj.drillControl('off',0,0);
%                     pierced = 1;
%                 end
%                 !!!THIS IS WHERE THE MAIN MEASUREMENT IN THE LOOP SHOULD OCCUR!!!
%                 %mesaure Signal -.75 seconds provides approx 3
%                 measurements.

                    obj.acquireDAQProbePulse('foreground',5000,1000,.75); 
                if(obj.skull_pierced)  
                    pierced = 1;
                    obj.skull_pierced = false;
                else 

                end
                cnt = cnt+1;
            end
            cprintf([1 .5 0],'***Pierced @ %d *** \n', curZ);
        end
        %Currently max voltage is 25V, but I measured it to 32V.
        % the calibration factor for voltage and current require them to be
        % set to the powersupply maxes.
        function setDrillVoltage(obj,vIn)
            if(vIn>25)
                vIn = 25;
            end
            %convert to arduino scale
            vToSet = int8(vIn*255/60);
            fprintf('Drill voltage set to %d \n',vIn);
            obj.ardui.analogWrite(obj.voltage_reg_pin,vToSet);
        end
        %%The fuse on the foredom is at 3.1A, but the max draw is 2A
        function setDrillCurrent(obj,iIn)
            if(iIn>2)
                iIn = 2;
            end
            iToSet = int8(iIn*255/15);
            fprintf('Drill current set to %d \n',iToSet);
            obj.ardui.analogWrite(obj.current_reg_pin,iToSet);
        end
        %More basic functionality of SutterMP285
        function resetToOrigin(obj)
            moveTo(obj.sutter,[0 0 0]);
        end
        %Since it makes more sense to set a velocity based on um/sec...
        function setDrillVelocity(obj,vel)
            [stepMult, ~,~] = getStatus(obj.sutter);
            velToSet = vel* stepMult; % the first unit is the multiplier.
            %The scale factor simply multiplies the speed -  so I set it to 1 
            
            setVelocity(obj.sutter,velToSet,1);
            getStatus(obj.sutter);
        end
        %% NI-DAQ in/out functionality
        %%%%For lower end board USB 6008/6009
        function obj= createParallelProcess(obj,num_workers)
            %Create background process           
             if isempty(obj.parallel_cluster)
                disp('Creating cluster...');
                profile = parallel.defaultClusterProfile;
                obj.parallel_cluster = parcluster(profile);
             end
                obj.worker_pool = parpool(obj.parallel_cluster,num_workers);
        end
        %Send Daq parallel job process
        function [job,task]=sendDaqOnParallel(obj)
             job = createJob(obj.parallel_cluster);

             task = createTask(job,@eval,0, {'auto=Autodrill_v0_16(''shitty'',0);auto.sendDaqProbePulse(100,1000)'});
             obj.current_signal_worker_job = job;
             
             submit(job);
%              wait(job);
%              out = fetchOutputs(job)
        end
        %This methods currently only supports sine waves...
        function sendDaqProbePulse(obj, freq, cycles)
             tic;
                compRes = toc;
                resolution = 2.5; %no idea why but this coefficient sets the loop
                amplitude = .0100;
                freq = freq+40;%correction factor for timed loop
                  x = 0:resolution:2*pi*cycles;
%                 x = 0:(1/freq):2*pi*cycles;
                fxn = amplitude*sin(freq*x)+amplitude;
                cnt =1;
                cycle_cnt =0;
%                 figure;
%                 hold on;
%                 ylim([-2.5 7.5]);
%                 col = jet();
                while cycle_cnt<=cycles   
                   outputSingleScan(obj.out_s, fxn(cnt));
%                    plot(x(cnt),fxn(cnt),'-ko','MarkerSize',5) ; %drops the execution loop to 60hz @ 2.5 coef
%                    drawnow;
                     cnt = cnt+1;
                    if (cnt > length(x)) 
                        cnt = 1;
                        cycle_cnt = cycle_cnt+1;
                    end  
                end  
                %Reset output
                outputSingleScan(obj.out_s,0); % 0 ouput
        end
       
        %Note the duration here is in seconds and max rate for the current
        %daq is 48000 (48Khz)
         function acquireDAQProbePulse(obj,acquisition_type, rate, data_update_size,time_to_sample)
           
           if (~obj.in_s.IsDone) 
             obj.in_s.Rate = rate;
                obj.in_s.DurationInSeconds = time_to_sample+.5; % added .5 secs to ensure proper obj cleanup on board
%                 obj.input_s.IsContinuous = true;
                obj.in_s.NotifyWhenDataAvailableExceeds = data_update_size;
           end
           %clear current recording of data
           obj.current_measurement_log_of_fft = [];
           
                dateobj = cdfepoch(now);
                startTime_seconds=todatenum(dateobj)/1000;
                start_time_obj = tic;
                   
                %Event handler -- event is called DataAvailable
                lh = addlistener(obj.in_s,'DataAvailable', @(src,event)analyzeTrigger(obj,event,rate,start_time_obj,startTime_seconds,time_to_sample));
                
                obj.DATA_AVAILABLE = lh ;
                
                if strcmp(acquisition_type,'foreground')
                     cprintf([0 1 0],'***RECORDING FOREGROUND*** \n');
                     [data,time] = obj.in_s.startForeground;
%                     obj.analyzeCurrentSignal(data,rate);
                elseif strcmp(acquisition_type,'background')
                      cprintf([0 1 0],'***RECORDING BACKGROUND*** \n');
                      obj.in_s.startBackground();
%                     while  obj.in_s.IsLogging
% %                         cprintf([0 1 0],'***RECORDING*** \n');
%                           pause(duration/10);
%                     end
                    %%Done here
    %                 obj.input_s.wait() %if you want to block the execution thread.
                end
                cprintf([1 0 0],'***FINISHED RECORDING*** \n');
         end
        
        %%%%FOR HIGH END BOARDs > USB 6356
        %This prepares the output buffer to send the signal it does note
        %execute it.
        function queueSignal(obj,fxn, frequency,duration)
            if(isempty(duration)) 
                duration = obj.rate;
            end
            x = linspace(0,2*pi,duration); %final is the number of points and should match the sampling rate
            if(isempty(frequency)) 
                frequency = obj.rate;
            end
            if strcmp(fxn,'sine')
                fxn = 2.5*sin(frequency*x)+2.5;
            end
            
            obj.in_out_s.queueOutputData(fxn);
        end
       %!!NOT TESTED!!%
        %Function that actually executes the signal and listens to it
        %param sets whether this is done in the foreground or in the
        %background
        function [capturedData,time]=sendReceiveSignal(obj,param)
            
            %Event handler -- event is called DataAvailable
                lh = addlistener(obj.in_out_s,'DataAvailable', @(src,event)analyzeTrigger(obj,event));
                
            if strcmp(param,'foreground')
                %This ensures simoultaneous start of send receive
                %operations-- this blocks the thread execution
               [capturedData,time]= obj.in_out_s.startForeground();
            else
               [capturedData,time]= obj.in_out_s.startBackground();
            end
        end
        %Delete the current worker job
        function delete_current_job(obj)
           delete(obj.current_signal_worker_job);
        end
         %% Connection and disconnection of all devices
         function disconnectDevices(obj,cond)
             fclose(obj.sutter);
             delete(instrfind({'Port'},{obj.ardui_port}));
             delete(instrfind({'Port'},{obj.sutter_port}));
             obj.sutter = [];
             obj.ardui = [];
%              obj.Autodrill(obj,cond);
         end
         function obj = connectDevices(obj)
             obj.sutter = sutterMP285(obj.sutter_port);
             obj.ardui = arduino(obj.ardui_port);
         end
         
    end
    methods (Access = private)
        %Conversion from radial parameters to xy coords
        function [x,y]=convertRadialToXY(obj, ang)
            xp=obj.path_r*cos(ang);
            yp=obj.path_r*sin(ang);

            x = obj.scaling_factor*xp;
            y = obj.scaling_factor*yp;
        end
        function stop_drill = analyzeTrigger(obj,event,sampling_rate,start_time_obj, start_time_seconds,time_to_sample)
%           time = event.TimeStamps;
            data = event.Data;
            
            curTime = start_time_seconds+toc(start_time_obj);
            
            fprintf('Startime- %d\n', start_time_seconds);  
            fprintf('Curtime- %d\n', curTime);
              fprintf('time to sample- %d\n', time_to_sample);
           fprintf('max- %d\n', start_time_seconds+time_to_sample);
          
             stop_drill = false;
            if (curTime < (start_time_seconds+time_to_sample) && ~stop_drill)
                stop_drill = obj.analyzeCurrentSignal(data,sampling_rate);
            else
                %time of probe expired -- 
                
                %Get rid of event listener
                    %stop acquisition  -- access to nidaq
                    stop(obj.in_s);
                    %delete event listener
                    delete(obj.DATA_AVAILABLE);
                    disp('Session stopped')
            end
            if(stop_drill)
               %delete event listener 
               obj.skull_pierced = true;
               cprintf([1 .5 .5], '****PIERECED****\n');
            end
        end
        
        function stop_drill = analyzeCurrentSignal(obj,data,rate)
            hold on;
%             plot(time,data);
%             axis([(max(time)-.1) (max(time)+.1) -.015 (max(data)+0.005)]);
            
            drawnow;
            
            Fs = rate;
            Nsamps = length(data);
            %Flattop window
            window = flattopwin(Nsamps);  
            
            windowed_data = data.*window;
            
            d_fft= abs(fft(windowed_data));
            
%             d_fft_power = (conv(d_fft,conj(d_fft))/Nsamps^2);
           
            bot_lim_ind = floor((length(data)*obj.freq_bot/Fs)-1);
            top_lim_ind = ceil((length(data)*obj.freq_top/Fs)+1);
            
            f = Fs*(1:Nsamps)/Nsamps;

            subsample_fft = d_fft(bot_lim_ind:top_lim_ind);
            subsample_f = f(bot_lim_ind:top_lim_ind);
%             subsample_power = d_fft_power(bot_lim_ind:top_lim_ind);
%             d_fft_log_sub = 20*log(susbample_fft/rms(susbample_fft));
            
%             plot(subsample_f,subsample_fft, 'r')
%             xlabel('Frequency (Hz)')
%             ylabel('Amplitude')
%             
%             persistent fft_maxes ;
%             max_val_fft = max(subsample_fft);
%             fft_maxes = [fft_maxes, max_val_fft];

            %integrate signal in subsample zone
            integral_of_fft = trapz(subsample_fft);
            obj.lifetime_logged_integral_of_fft = [obj.lifetime_logged_integral_of_fft; integral_of_fft ];
            obj.current_measurement_log_of_fft = [obj.current_measurement_log_of_fft; integral_of_fft];
            
            cprintf([0 1 0], 'Integral of subsample_fft: %d \n', integral_of_fft);
            %Evaluation of pierced skull. Greater than background and .6 of the maximum signal 
            if (integral_of_fft > obj.mouse_fft_background_signal &&  integral_of_fft >= obj.mouse_fft_attenuation_factor*obj.mouse_fft_max_signal)
                stop_drill = true;
            else
                stop_drill = false;
            end

        end
    end
end
