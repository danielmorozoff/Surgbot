%Autodrill v0.1 - matlab class controlling sutter controllers (MP285). NI DAQ I/O
%unit (NIDAQ USB 6009/6008)
%Dan Morozoff - 9/11/14
classdef Autodrill
    properties
        classType = 'input'
        %data acquisition session
        output_s;
        input_s;
        dev_s;
        %COM PORTS
        sutter_port = 'COM5';
        ardui_port = 'COM9';
        %Channels
        signal_inputChannel = 'ai0';
        signal_outputChannel = 'ao1';
        dev_signal_outputChannel = 'ao0';
        %Settings for session
        rate = 16000; %Set to 16 KHz 
        %Number of cyles to probe fxn
        pulse_cycles = 0;
        %Arduino device to control BK PRECISION power unit. These vary in
        %their max current outputs. We used 1902 (15A)
        ardui;
        voltage_reg_pin = 5;%Pin to adjust voltage
        current_reg_pin = 9;%Pin to adjust current max.
        %Sutter device
        sutter;
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
        
        %Fourier filter
       %Frequencies that will be displayed/potentially measured to
        %boost speed.
        freq_top = 120;
        freq_bot = 80;
    end
    
    methods (Access = public)
        %Constructor...
        function obj = Autodrill(param) 
            
           obj.classType = param;
            %add libraries to path
            addpath('C:\Users\dmorozoff\Desktop\NeuroCode\SurgicalBot\dev');
            addpath('dependencies');
            addpath('dependencies/Sutter MP285');
            addpath('dependencies/ArduinoIO');
            %Initialize Arduino IO
%             try
%                 obj.ardui = arduino(obj.ardui_port);
%                 obj.ardui.pinMode(obj.voltage_reg_pin,'output');
%                 obj.ardui.pinMode(obj.current_reg_pin,'output');
%             catch err
%                 delete(instrfind({'Port'},{obj.ardui_port}));
%             end
            %Initialize Sutter
%             try
%            
%             obj.sutter = sutterMP285(obj.sutter_port);
%                 moveTo(obj.sutter,[0 0 0]);
%             catch err
%               delete(instrfind({'Port'},{obj.sutter_port}));
%             end
            %Initialize DAQ
            daq_devices = daq.getDevices;
            device= daq_devices(1).ID;
            %only ao0 and ao1 are outputs on the USB6009 board.
               if(strcmp(param,'output'))
                    obj.output_s.Rate = obj.rate;
                    obj.output_s= daq.createSession('ni');
                    obj.output_s.addAnalogOutputChannel(device,obj.signal_outputChannel,'Voltage');
               end
               if(strcmp(param,'input'))
                    obj.input_s= daq.createSession('ni');
                    obj.input_s.Rate = obj.rate;   
                    obj.input_s.addAnalogInputChannel(device, obj.signal_inputChannel, 'Voltage');
                    
               end
               
            %%%%%%%%
            %DEV CODE
            obj.dev_s = daq.createSession('ni');
                obj.dev_s.Rate = obj.rate;
                if(strcmp(param,'output'))
                    obj.dev_s.addAnalogOutputChannel(device,obj.dev_signal_outputChannel,'Voltage');
                end
            %%%%%%%%
            
            %Initialize the MP 285 device -- this opens the serial object now
            %stored in sutter which extends the serial class.
%              sutter=sutterMP285(dev_loc);
%              sutter.verbose=1;
        end
        %% Primary control loop 
        function obj=control_loop(obj,d,distance_between_pnts)
            obj.setDrillVelocity(400) %in micrometers/s
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
            
            obj.setDrillVelocity(40);
            
            t0 = tic;
            for ang=0:radial_step_size:2*pi
                z_start= -4 ; %This can be modififed in order to evaluate how to detect the surface of the bone
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
            obj.mill();
            
            obj.resetToOrigin();
            disp(obj.path)
        end
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
                z = obj.spline_pnts(i,3);
                moveTo(obj.sutter,[x y z]);
                hold on;
                plot3(x/1000,y/1000,-1*z/1000,'c.','MarkerSize',5);
                drawnow;
           end
           obj.drillControl('off',0,0);
        end
        %% Drill on or off/ voltage/current
        function obj = drillControl(obj,state,voltage,current)
            if(strcmp(state,'on'))
                    obj.setDrillCurrent(current);
                    obj.setDrillVoltage(voltage);
            else
                    obj.setDrillCurrent(0);
                    obj.setDrillVoltage(0);
            end
        end
        function obj = splineInterpolateZDrill(obj)
            path_to_user = obj.path(1:(end),1:3)';
            cprintf([1 0 0 ], 'Interpolating...');
            path_to_user
            
            sp = cscvn(path_to_user(:,[1:end 1]));
            obj.spline_obj = sp;
            %Points for the spline fit
            points=fnplt(sp);
            obj.spline_pnts = points'; %transpose them in order to keep the same sorting style...
%             hold on;
%             plot3(obj.spline_pnts(:,1),obj.spline_pnts(:,2),obj.spline_pnts(:,3),'ro','MarkerSize',10);
            
%             %Cubic spline is of the form C(1)*X^(K-1) + C(2)*X^(K-2) + ... + C(K-1)*X + C(K)
%             for(i=1:1:2) %spline_obj.pieces
%                 if(i<sp.pieces)
%                     break_pnt = sp.breaks(i:i+1); % limits of the piece
%                 end
%                  piecewise_x = linspace(break_pnt(1),break_pnt(2),100);
%                  yyp3=ppval(sp,piecewise_x);
%                  plot(piecewise_x,yyp3,'b-','linewidth',2);
%             end
%             
%             for row = 1:size(obj.path,1)
%                 %Move to intial point
%                 x = obj.path(row,1);
%                 y = obj.path(row,2);
%                 z = obj.path(row,3);
% %                 moveTo(obj.sutter,[x y z]);
% 
%                 if(row<size(obj.path,1))
%                     %pick point on interval between the two- since it's linear this doesn't matter. 
%                      x2= obj.path((row+1),1);
%                      y2= obj.path((row+1),2);
%                       z2= obj.path((row+1),3);
%                 elseif(row==size(obj.path,1))
%                      x2= obj.path((1),1);
%                      y2= obj.path((1),2);
%                      z2= obj.path((1),3);
%                 end
                
%                 interp_step =.1;
%                 if(x2<x)
%                    x_interv = fliplr(x2:interp_step:x);
%                 else
%                    x_interv = x:interp_step:x2;
%                 end
%                 if(y2<y)
%                    y_interv = fliplr(y2:interp_step:y);
%                 else
%                    y_interv = y:interp_step:y2;
%                 end
%                  if(z2<z)
%                    z_interv = fliplr(z2:interp_step:z);
%                 else
%                    z_interv = z:interp_step:z2;
%                 end
%                 
                
%                 if(strcmp(interp_type,'spline') || strcmp(interp_type,'nearest') || strcmp(interp_type,'cubic') || strcmp(interp_type,'linear'))
%                 else
%                     interp_type= 'spline';
%                 end
                
%                obj.path
%                disp('-----------------');
%                x_interv
%                y_interv
%               cprintf([0 1 .5], 'Interpolating %d %d %d %d \n', x,y,x2,y2);
%               z_interp=interp1(obj.path(:,3),[obj.path(:,1),obj.path(:,2)],z_interv,interp_type) % spits out z interpolation
%                z_interp = interp2(obj.path,x_interv,y_interv,interp_type);  
%                hold on;
%                z_interp
%                plot3(x_interv,y_interv,z_interp, 'ro','MarkerSize', 10);
%                   x_med = (x2-x)/2;
%                   y_med = (y2-y)/2;
%                 vec_x = [x x2];
% %                 vec_y = [y y2];
%                 vec_z = [z z2];
%                  z_inter = interp1(vec_x,vec_z, x_med,'spline');
%                  
%                  cprintf([1 0 0],'Interpolated Point: [ %d %d %d ] \n', x2,y2,z_inter);
%                  moveTo(obj.sutter,[x2 y2 z_inter]);
%                  plot3(x_med/obj.scaling_factor,y/obj.scaling_factor,z_inter/obj.scaling_factor,'go','MarkerSize',10);
 
%             end
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
        %% Step drill in z to do depth holes 
        function [pierced,curZ] = stepDrillZ(obj,x_in,y_in,z_in)
            pierced = 0;
            cnt = 0;
            obj.drillControl('on',25,2);
            while(~pierced)
                curZ = (z_in+(obj.z_step_size)*cnt);
                fprintf(1,'Drilling to depth:  %d um \n',curZ);
                moveTo(obj.sutter,[x_in y_in curZ]); 
               
                if(curZ>=1000* 3.2)
                    obj.drillControl('off',0,0);
                    pierced = 1;
                end
                cnt = cnt+1;
%                 %mesaure Signal
%                 if(met)
%                     pierced = 1;
%                 else 
% 
%                 end
            end
            cprintf([1 .5 0],'***Pierced @ %d *** \n', curZ);
        end
        %%
        %%%%Functionality for the MH170 Foredom Drill
        %%Currently max voltage is 15V, but I measured it to 32V.\
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
        %%%%More basic functionality of SutterMP285
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
        %%
        %%Send pulse from the daq board
        function sendDAQProbePulse(obj,fxn,freq,resolution,cycles)
            if(strcmp(obj.classType,'output'))
                %Since the output is V > 0  -- MAX OUTPUT FOR NI-6009 is 150hz
                if (strcmp(fxn,'sine'))
                   x = 0:(1/obj.rate):2*pi;
                   fxn = 2.5*sin(100*obj.rate*x*4)+2.5;
                end
                for i=1:length(fxn);
                   outputSingleScan(obj.output_s,fxn(i));
%                    outputSingleScan(obj.dev_s,fxn(i))
                end
                if(obj.pulse_cycles<cycles)
                    obj.pulse_cycles= obj.pulse_cycles+1; 
                    obj.sendDAQProbePulse(fxn,freq,resolution,cycles);
                end
                outputSingleScan(obj.output_s,0); % 0 ouput
            end
        end
        %This function is used by crappy DAQ chips that don't have a set
        %out freq so we rely on a timed loop to control it
        function sendDAQProbePulseTimed(obj,freq,cycles)
            if(strcmp(obj.classType,'output'))
                tic;
                compRes = toc;
                resolution = 2.5; %no idea why but this coefficient sets the loop
                amplitude = .0100;
                freq = freq+5;%correction factor for timed loop
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
                   outputSingleScan(obj.output_s, fxn(cnt));
%                    plot(x(cnt),fxn(cnt),'-ko','MarkerSize',5) ; %drops the execution loop to 60hz @ 2.5 coef
%                    drawnow;
                     cnt = cnt+1;
                    if (cnt > length(x)) 
                        cnt = 1;
                        cycle_cnt = cycle_cnt+1;
                    end  
                end
                %changes execution frequency...
%                 if(obj.pulse_cycles<cycles)
%                     disp('cycle')
%                     obj.pulse_cycles= obj.pulse_cycles+1;
% %                     obj.pulse_color_count = obj.pulse_color_count+1;
%                     obj.sendDAQProbePulseTimed(freq,cycles);
%                 end
                %Reset output
                outputSingleScan(obj.output_s,0); % 0 ouput
            end
        end
        %Note the duration here is in seconds and max rate for the current
        %daq is 48000 (48Khz)
         function [data,time,lh]=acquireDAQProbePulse(obj,acquisition_type, rate,duration, data_update_size)
            if(strcmp(obj.classType,'input'))
                obj.input_s.Rate = rate;
                obj.input_s.DurationInSeconds = duration;
%                 obj.input_s.IsContinuous = true;
                obj.input_s.NotifyWhenDataAvailableExceeds = data_update_size;
                %Event handler -- event is called DataAvailable
                lh = addlistener(obj.input_s,'DataAvailable', @(src,event)analyzeTrigger(obj,src,event,rate,data_update_size));
                if strcmp(acquisition_type,'foreground')
                     cprintf([0 1 0],'***RECORDING*** \n');
                     [data,time] = obj.input_s.startForeground;
%                     obj.analyzeCurrentSignal(data,rate);
                elseif strcmp(acquisition_type,'background')
                    obj.input_s.startBackground();
                    while  obj.input_s.IsLogging
%                         cprintf([0 1 0],'***RECORDING*** \n');
                          pause(duration/10);
                    end
                    %%Done here
    %                 obj.input_s.wait() %if you want to block the execution thread.
                end
                cprintf([1 0 0],'***FINISHED RECORDING*** \n');
            end
         end
         %% Connection and disconnection
         
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
        function [x,y]=convertRadialToXY(obj, ang)
            xp=obj.path_r*cos(ang);
            yp=obj.path_r*sin(ang);

            x = obj.scaling_factor*xp;
            y = obj.scaling_factor*yp;
        end
        function analyzeTrigger(obj,src,event,sampling_rate,data_size)
            time = event.TimeStamps;
            data = event.Data;
                       
            obj.analyzeCurrentSignal(data,sampling_rate);
        end
        function analyzeCurrentSignal(obj,data,rate)
        
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
           
            bot_lim_ind = floor((length(data)*obj.freq_bot/Fs)+1);
            top_lim_ind = ceil((length(data)*obj.freq_top/Fs)+1);
            
            f = Fs*(1:Nsamps)/Nsamps;

            subsample_fft = d_fft(bot_lim_ind:top_lim_ind);
            subsample_f = f(bot_lim_ind:top_lim_ind);
%             subsample_power = d_fft_power(bot_lim_ind:top_lim_ind);
%             d_fft_log_sub = 20*log(susbample_fft/rms(susbample_fft));
            
            plot(subsample_f,subsample_fft, 'r')
            xlabel('Frequency (Hz)')
            ylabel('Amplitude')
            
            persistent fft_maxes ;
            max_val_fft = max(subsample_fft);
            fft_maxes = [fft_maxes, max_val_fft];
%             disp(fft_maxes);
            
%             disp('-----------')
%             fprintf('Max: %d \n',max_val_fft)
%             fprintf('Mean-Max: %d \n',mean(fft_maxes))
%             fprintf('SD-Max: %d \n',std(fft_maxes))
%             disp('-----------')
            
            %integrate the area.
            
% %             amplitude = abs(fft_data);
%             [X Y]=meshgrid(freq,amplitude);
%             Z=griddata(freq,amplitude,amplitude, X,Y);
% %             plot(freq, amplitude);
%             axis([50 1.5*max(freq) 0 6000]);
%            surf_data = horzcat(freq',amplitude);
%            surf(freq,amplitude,freq,'CDataMapping','direct','EdgeColor','none');
%            colorbar;
%            view(2);

%             axis([0 2*freq 0 max(fft_data)]);
            %Logic to handle start/stop of robot based on signal parameters
        end
        
        %@Deprecated
        %%Timers 
        function timerControl(obj,duration)
            fprintf('Setting timer duration: %d\n',duration)
            obj.signal_acquisition_timer.StartFcn = @obj.signal_acquisition_timer_startFxn;
            obj.signal_acquisition_timer.StopFcn  = @obj.signal_acquisition_timer_stopFxn;
            obj.signal_acquisition_timer.TimerFcn = @obj.signal_acquisition_timer_mainFxn;
            obj.signal_acquisition_timer.ExecutionMode = 'fixedRate';
            obj.signal_acquisition_timer.Period = 1;
            
            start(obj.signal_acquisition_timer);
        end
        
        function signal_acquisition_timer_startFxn(obj,mTimer,~)
            disp('Signal acquisiotn timer start')
        
        end
        function signal_acquisition_timer_mainFxn(obj,mTimer,~)
            disp('Main Timer')
        end
         function signal_acquisition_timer_stopFxn(obj,mTimer,~)
            cprintf([1 0  0],'Signal acquisiotn timer stopped \n')   
            stop(obj.signal_acquisition_timer);
            delete(obj.signal_acquisition_timer)
            
        end
    end
end